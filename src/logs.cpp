#include "btick/logs.hpp"

#include <chrono>
#include <condition_variable>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <nlohmann/json.hpp>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

#include "btick/events.hpp"
#include "btick/monitor.hpp"
#include "btick/tree_serializer.hpp"

namespace btick {

namespace {

std::string get_iso8601_utc_timestamp() {
  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  std::time_t timer = std::chrono::system_clock::to_time_t(now);
  std::tm bt{};
  gmtime_r(&timer, &bt);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &bt);
  std::ostringstream ss;
  ss << buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "Z";
  return ss.str();
}

std::string generate_session_id() {
  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  return "session_" + std::to_string(ms);
}

std::string format_timestamp_filename_suffix() {
  auto now = std::chrono::system_clock::now();
  std::time_t timer = std::chrono::system_clock::to_time_t(now);
  std::tm bt{};
  localtime_r(&timer, &bt);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &bt);
  return std::string(buf);
}

std::string resolve_log_file_path(const std::string &input_path, bool timestamp_filename) {
  if (!timestamp_filename || input_path.empty()) {
    return input_path;
  }

  std::filesystem::path p(input_path);
  std::filesystem::path parent = p.parent_path();
  std::string stem = p.stem().string();
  std::string ext = p.extension().string();

  std::string ts = format_timestamp_filename_suffix();

  std::string base_filename = stem.empty() ? (ts + ext) : (stem + "_" + ts + ext);
  std::filesystem::path candidate =
      parent.empty() ? std::filesystem::path(base_filename) : parent / base_filename;

  if (!std::filesystem::exists(candidate)) {
    return candidate.string();
  }

  // Handle collisions deterministically: _1, _2, etc.
  for (int counter = 1; counter < 10000; ++counter) {
    std::string collision_filename = stem.empty()
                                         ? (ts + "_" + std::to_string(counter) + ext)
                                         : (stem + "_" + ts + "_" + std::to_string(counter) + ext);
    std::filesystem::path next_candidate =
        parent.empty() ? std::filesystem::path(collision_filename) : parent / collision_filename;
    if (!std::filesystem::exists(next_candidate)) {
      return next_candidate.string();
    }
  }

  return candidate.string();
}

}  // namespace

class Logs::Impl {
 public:
  Impl(Monitor &monitor, const std::string &tree_id, const std::string &file_path,
       bool timestamp_filename, std::chrono::milliseconds flush_interval)
      : tree_id_(tree_id.empty() ? monitor.tree_id() : tree_id),
        file_path_(resolve_log_file_path(file_path, timestamp_filename)),
        flush_interval_(flush_interval),
        session_id_(generate_session_id()),
        recorded_at_(get_iso8601_utc_timestamp()) {
    auto instance = monitor.get_tree_instance(tree_id_);
    if (instance) {
      tree_instance_weak_ = instance;
      std::string initial_json_str = instance->to_json_tree_created();
      if (!initial_json_str.empty()) {
        try {
          initial_tree_json_ = nlohmann::json::parse(initial_json_str);
        } catch (...) {
          initial_tree_json_ = nlohmann::json::object();
        }
      }
      listener_handle_ =
          instance->add_event_listener([this](const NodeStatusEvent &ev) { handle_event(ev); });
    }

    if (flush_interval_.count() > 0) {
      flush_thread_ = std::thread([this]() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stopped_) {
          cv_flush_.wait_for(lock, flush_interval_, [this]() { return stopped_.load(); });
          if (dirty_) {
            persist_to_disk_locked();
          }
        }
      });
    }
  }

  ~Impl() {
    stop();
    flush();
  }

  const std::string &file_path() const noexcept { return file_path_; }

  void flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    persist_to_disk_locked();
  }

  void stop() {
    if (stopped_.exchange(true)) {
      return;
    }

    if (listener_handle_ != 0) {
      if (auto instance = tree_instance_weak_.lock()) {
        instance->remove_event_listener(listener_handle_);
      }
      listener_handle_ = 0;
    }

    cv_flush_.notify_all();
    if (flush_thread_.joinable()) {
      flush_thread_.join();
    }
  }

 private:
  void handle_event(const NodeStatusEvent &ev) {
    if (stopped_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    uint64_t seq = ++event_counter_;
    nlohmann::json transition = to_json_transition(ev, seq);
    events_.push_back(std::move(transition));
    dirty_ = true;
  }

  void persist_to_disk_locked() {
    if (initial_tree_json_.empty()) {
      if (auto instance = tree_instance_weak_.lock()) {
        std::string tree_json_str = instance->to_json_tree_created();
        if (!tree_json_str.empty()) {
          try {
            initial_tree_json_ = nlohmann::json::parse(tree_json_str);
          } catch (...) {
          }
        }
      }
    }

    nlohmann::json tree_obj =
        initial_tree_json_.empty() ? nlohmann::json::object() : initial_tree_json_;
    nlohmann::json tree_array = nlohmann::json::array();
    if (!tree_obj.empty()) {
      tree_array.push_back(tree_obj);
    }

    std::string tree_name = tree_obj.value("tree_name", tree_id_);
    if (tree_name.empty()) {
      tree_name = "MainTree";
    }

    // Build recorded node executions and snapshots from event sequence
    nlohmann::json node_executions = nlohmann::json::array();
    nlohmann::json snapshots = nlohmann::json::array();
    nlohmann::json current_tree = tree_obj;

    struct ActiveExec {
      std::string tree_id;
      std::string node_id;
      uint16_t uid{0};
      std::string status;
      double start_timestamp{0.0};
      uint32_t tick_count{0};
      size_t start_event_index{0};
    };
    std::unordered_map<std::string, ActiveExec> active_map;

    for (size_t idx = 0; idx < events_.size(); ++idx) {
      const auto &ev = events_[idx];
      std::string tid = ev.value("tree_id", "");
      uint16_t uid = ev.value("uid", 0);
      std::string nid = ev.value("node_id", "");
      std::string status = ev.value("status", "");
      double ts = ev.value("timestamp", 0.0);
      uint32_t tc = ev.value("tick_count", 0);
      double dur = ev.value("duration", 0.0);

      // 1. Maintain node executions
      std::string key = tid + "::" + std::to_string(uid) + "::" + nid;
      if (status == "RUNNING") {
        if (active_map.find(key) == active_map.end()) {
          active_map[key] = ActiveExec{tid, nid, uid, status, ts, tc, idx};
        }
      } else if (status == "SUCCESS" || status == "FAILURE" || status == "IDLE") {
        auto it = active_map.find(key);
        if (it != active_map.end()) {
          nlohmann::json exec = {
              {"tree_id", it->second.tree_id},
              {"node_id", it->second.node_id},
              {"uid", it->second.uid},
              {"status", status},
              {"start_timestamp", it->second.start_timestamp},
              {"end_timestamp", ts},
              {"duration", dur > 0.0 ? dur : (ts - it->second.start_timestamp) * 1000.0},
              {"tick_count", tc > 0 ? tc : it->second.tick_count},
              {"start_event_index", it->second.start_event_index},
              {"end_event_index", idx}};
          node_executions.push_back(std::move(exec));
          active_map.erase(it);
        }
      }

      // 2. Track tree state for snapshots
      if (current_tree.contains("nodes") && current_tree["nodes"].is_array()) {
        for (auto &node : current_tree["nodes"]) {
          uint16_t node_uid = node.value("uid", 0);
          std::string node_id = node.value("id", "");
          if ((uid > 0 && node_uid == uid) || (!nid.empty() && node_id == nid)) {
            if (status == "NODE_PORT_UPDATE") {
              if (ev.contains("output_ports") && ev["output_ports"].is_array()) {
                for (const auto &port : ev["output_ports"]) {
                  std::string pname = port.value("port_name", "");
                  std::string pval = port.value("value", "");
                  std::string pmode = port.value("mode", "Literal");
                  if (pmode == "Blackboard" ||
                      (pval.size() >= 2 && pval.front() == '{' && pval.back() == '}')) {
                    node["ports"][pname] = pval;
                    if (node.contains("attributes") && node["attributes"].is_object()) {
                      node["attributes"].erase(pname);
                    }
                  } else {
                    node["attributes"][pname] = pval;
                    if (node.contains("ports") && node["ports"].is_object()) {
                      node["ports"].erase(pname);
                    }
                  }
                  if (node.contains("port_details") && node["port_details"].is_array()) {
                    for (auto &pd : node["port_details"]) {
                      if (pd.value("name", "") == pname) {
                        pd["value"] = pval;
                        pd["mode"] = pmode;
                        break;
                      }
                    }
                  }
                }
              }
            } else if (status != "BLACKBOARD_UPDATE" && status != "PAUSED_AT_BREAKPOINT" &&
                       status != "RESUMED") {
              node["status"] = status;
            }
            break;
          }
        }
      }

      if (ev.contains("blackboard") && ev["blackboard"].is_object()) {
        if (!current_tree.contains("blackboard") || !current_tree["blackboard"].is_object()) {
          current_tree["blackboard"] = nlohmann::json::object();
        }
        for (auto &[bk, bv] : ev["blackboard"].items()) {
          current_tree["blackboard"][bk] = bv;
        }
      }

      // Capture snapshot at NODE_PORT_UPDATE or periodic keyframe boundary (every 250 events)
      if (status == "NODE_PORT_UPDATE" || (idx > 0 && idx % 250 == 0)) {
        nlohmann::json sn = {{"eventIndex", idx},
                             {"timestamp", ts},
                             {"trees", nlohmann::json::array({current_tree})}};
        snapshots.push_back(std::move(sn));
      }
    }

    nlohmann::json root = {{"id", session_id_},
                           {"name", tree_name},
                           {"recorded_at", recorded_at_},
                           {"initial_trees", tree_array},
                           {"trees", tree_array},
                           {"events", events_},
                           {"snapshots", std::move(snapshots)},
                           {"node_executions", std::move(node_executions)}};

    namespace fs = std::filesystem;
    fs::path target_path(file_path_);
    if (target_path.has_parent_path()) {
      std::error_code ec;
      fs::create_directories(target_path.parent_path(), ec);
    }

    fs::path temp_path = target_path;
    temp_path += ".tmp";

    {
      std::ofstream out(temp_path);
      if (out.is_open()) {
        out << root.dump(2);
        out.close();
        std::error_code ec;
        fs::rename(temp_path, target_path, ec);
        if (ec) {
          std::ofstream direct_out(target_path);
          direct_out << root.dump(2);
        }
      } else {
        std::ofstream direct_out(target_path);
        direct_out << root.dump(2);
      }
    }
    dirty_ = false;
  }

  std::weak_ptr<TreeInstance> tree_instance_weak_;
  std::string tree_id_;
  std::string file_path_;
  std::chrono::milliseconds flush_interval_{2000};
  std::string session_id_;
  std::string recorded_at_;
  TreeInstance::ListenerHandle listener_handle_{0};
  nlohmann::json initial_tree_json_{nlohmann::json::object()};

  mutable std::mutex mutex_;
  std::condition_variable cv_flush_;
  std::thread flush_thread_;
  std::atomic<bool> stopped_{false};
  std::atomic<bool> dirty_{false};

  std::vector<nlohmann::json> events_;
  uint64_t event_counter_{0};
};

Logs::Logs(Monitor &monitor, const std::string &tree_id, const std::string &file_path,
           bool timestamp_filename, std::chrono::milliseconds flush_interval)
    : impl_(std::make_unique<Impl>(monitor, tree_id, file_path, timestamp_filename,
                                   flush_interval)) {}

Logs::Logs(Monitor &monitor, const std::string &tree_id, const std::string &file_path,
           std::chrono::milliseconds flush_interval)
    : impl_(std::make_unique<Impl>(monitor, tree_id, file_path, false, flush_interval)) {}

Logs::Logs(Monitor &monitor, const std::string &file_path, bool timestamp_filename,
           std::chrono::milliseconds flush_interval)
    : impl_(std::make_unique<Impl>(monitor, monitor.tree_id(), file_path, timestamp_filename,
                                   flush_interval)) {}

Logs::Logs(Monitor &monitor, const std::string &file_path, std::chrono::milliseconds flush_interval)
    : impl_(std::make_unique<Impl>(monitor, monitor.tree_id(), file_path, false, flush_interval)) {}

Logs::~Logs() = default;

void Logs::flush() {
  if (impl_) {
    impl_->flush();
  }
}

void Logs::stop() {
  if (impl_) {
    impl_->stop();
  }
}

const std::string &Logs::file_path() const noexcept {
  static const std::string empty_str;
  return impl_ ? impl_->file_path() : empty_str;
}

Logs::Logs(Logs &&) noexcept = default;
Logs &Logs::operator=(Logs &&) noexcept = default;

}  // namespace btick
