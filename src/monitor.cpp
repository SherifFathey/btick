#include "btick/monitor.hpp"

#include <charconv>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "btick/tree_serializer.hpp"

namespace btick {

static std::string generate_observer_id(const std::string &name) {
  static std::atomic<uint64_t> counter{0};
  static std::random_device rd;
  static std::mt19937_64 generator(rd());
  static std::mutex generator_mutex;
  uint64_t entropy = 0;
  {
    std::lock_guard<std::mutex> lock(generator_mutex);
    entropy = generator();
  }
  std::ostringstream suffix;
  suffix << std::hex << std::setfill('0') << std::setw(16) << entropy << std::setw(16)
         << counter.fetch_add(1, std::memory_order_relaxed);
  return name + "_" + suffix.str();
}

static bool starts_with(std::string_view value, std::string_view prefix) {
  return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

static bool ends_with(std::string_view value, std::string_view suffix) {
  return value.size() >= suffix.size() && value.substr(value.size() - suffix.size()) == suffix;
}

static std::string field_as_string(const nlohmann::json &object, const std::string &key) {
  const auto field = object.find(key);
  if (field == object.end() || field->is_null()) {
    return {};
  }
  if (field->is_string()) {
    return field->get<std::string>();
  }
  return field->dump();
}

static bool parse_uid_field(const nlohmann::json &object, uint16_t &uid) {
  const auto field = object.find("uid");
  if (field == object.end()) {
    return false;
  }

  uint64_t parsed_uid = 0;
  if (field->is_string()) {
    const std::string text = field->get<std::string>();
    const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed_uid);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
      return false;
    }
  } else if (field->is_number_unsigned()) {
    parsed_uid = field->get<uint64_t>();
  } else if (field->is_number_integer()) {
    const auto signed_uid = field->get<int64_t>();
    if (signed_uid < 0) {
      return false;
    }
    parsed_uid = static_cast<uint64_t>(signed_uid);
  } else {
    return false;
  }
  if (parsed_uid > std::numeric_limits<uint16_t>::max()) {
    return false;
  }
  uid = static_cast<uint16_t>(parsed_uid);
  return true;
}

static bool parse_delay_field(const nlohmann::json &object, int &delay_ms) {
  const auto field = object.find("delay_ms");
  if (field == object.end()) {
    delay_ms = 0;
    return true;
  }

  int parsed_delay = 0;
  if (field->is_string()) {
    const std::string text = field->get<std::string>();
    const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed_delay);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
      return false;
    }
  } else if (field->is_number_unsigned()) {
    const uint64_t value = field->get<uint64_t>();
    if (value > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
      return false;
    }
    parsed_delay = static_cast<int>(value);
  } else if (field->is_number_integer()) {
    const int64_t value = field->get<int64_t>();
    if (value < 0 || value > std::numeric_limits<int>::max()) {
      return false;
    }
    parsed_delay = static_cast<int>(value);
  } else {
    return false;
  }
  if (parsed_delay < 0) {
    return false;
  }
  delay_ms = parsed_delay;
  return true;
}

static bool parse_enabled_field(const nlohmann::json &object, bool &enabled) {
  const auto field = object.find("enabled");
  if (field == object.end()) {
    return false;
  }
  if (field->is_boolean()) {
    enabled = field->get<bool>();
    return true;
  }
  if (field->is_number_unsigned()) {
    const uint64_t value = field->get<uint64_t>();
    if (value == 0 || value == 1) {
      enabled = value == 1;
      return true;
    }
    return false;
  }
  if (field->is_number_integer()) {
    const int64_t value = field->get<int64_t>();
    if (value == 0 || value == 1) {
      enabled = value == 1;
      return true;
    }
    return false;
  }
  if (field->is_string()) {
    const auto value = field->get<std::string>();
    if (value == "true" || value == "1") {
      enabled = true;
      return true;
    }
    if (value == "false" || value == "0") {
      enabled = false;
      return true;
    }
  }
  return false;
}

static std::string derive_origin(const rclcpp::Node *node) {
  if (!node) {
    return {};
  }
  const std::string node_name = node->get_name();
  constexpr std::string_view prefix = "bt_navigator_";
  constexpr std::string_view suffix = "_rclcpp_node";
  if (!starts_with(node_name, prefix) || !ends_with(node_name, suffix) ||
      node_name.size() <= prefix.size() + suffix.size()) {
    return {};
  }

  return node_name.substr(prefix.size(), node_name.size() - prefix.size() - suffix.size());
}

Monitor::Monitor(rclcpp::Node *node, const BT::Tree &tree, const std::string &tree_name,
                 const std::string &xml_path, const std::string &discovery_topic,
                 const std::string &transitions_topic) {
  init(node, tree, tree_name, xml_path, discovery_topic, transitions_topic);
}

Monitor::Monitor(rclcpp::Node::SharedPtr node, const BT::Tree &tree, const std::string &tree_name,
                 const std::string &xml_path, const std::string &discovery_topic,
                 const std::string &transitions_topic) {
  init(node.get(), tree, tree_name, xml_path, discovery_topic, transitions_topic);
}

void Monitor::add_tree(rclcpp::Node *node, const BT::Tree &tree, const std::string &tree_name,
                       const std::string &xml_path, const std::string &discovery_topic,
                       const std::string &transitions_topic) {
  detached_ = false;
  init(node, tree, tree_name, xml_path, discovery_topic, transitions_topic);
}

void Monitor::add_tree(rclcpp::Node::SharedPtr node, const BT::Tree &tree,
                       const std::string &tree_name, const std::string &xml_path,
                       const std::string &discovery_topic, const std::string &transitions_topic) {
  add_tree(node.get(), tree, tree_name, xml_path, discovery_topic, transitions_topic);
}

bool Monitor::remove_tree(const std::string &tree_id) {
  std::shared_ptr<TreeInstance> target;
  {
    std::lock_guard<std::mutex> lock(instances_mutex_);
    auto it = instances_.find(tree_id);
    if (it == instances_.end()) {
      return false;
    }
    target = it->second;
    instances_.erase(it);
    if (instance_ && instance_->tree_id() == tree_id) {
      instance_ = instances_.empty() ? nullptr : instances_.begin()->second;
    }
  }

  if (target) {
    target->set_status(TreeStatus::DESTROYED);
    RCLCPP_INFO(logger_, "[BTick] Disconnected: tree='%s'", target->tree_name().c_str());
    if (discovery_pub_) {
      std_msgs::msg::String msg;
      msg.data = nlohmann::json{{"type", "tree_destroyed"}, {"tree_id", tree_id}}.dump();
      discovery_pub_->publish(msg);
    }
    return true;
  }
  return false;
}

void Monitor::init(rclcpp::Node *node, const BT::Tree &tree, const std::string &tree_name,
                   const std::string &xml_path, const std::string &discovery_topic,
                   const std::string &transitions_topic) {
  if (!node) {
    throw std::invalid_argument("[BTick] Monitor received a null rclcpp::Node pointer");
  }

  logger_ = node->get_logger();
  std::string id;
  std::string display_name = tree_name;
  {
    std::lock_guard<std::mutex> lock(instances_mutex_);
    do {
      id = generate_observer_id(tree_name);
    } while (instances_.count(id) != 0);

    size_t same_name_count = 0;
    for (const auto &entry : instances_) {
      if (entry.second->tree_name() == tree_name) {
        ++same_name_count;
      }
    }
    if (same_name_count > 0) {
      display_name += " #" + std::to_string(same_name_count + 1);
    }
  }
  auto new_instance = std::make_shared<TreeInstance>(
      tree, tree_name, xml_path, id, display_name, derive_origin(node), node->get_name(), logger_);

  // Transient Local QoS for Tree Discovery
  if (!discovery_pub_) {
    auto discovery_qos = rclcpp::QoS(20).transient_local().reliable();
    discovery_pub_ = node->create_publisher<std_msgs::msg::String>(discovery_topic, discovery_qos);

    // Reliable QoS for high-frequency transitions
    auto transition_qos = rclcpp::QoS(100).reliable();
    transition_pub_ =
        node->create_publisher<std_msgs::msg::String>(transitions_topic, transition_qos);
  }

  {
    std::lock_guard<std::mutex> lock(instances_mutex_);
    instances_.emplace(id, new_instance);
    instance_ = new_instance;
  }

  // Dedicated background executor for command subscriber so
  // breakpoints/resume/step work even if user's main thread is paused on
  // cv_breakpoint_.wait() inside tree.tickOnce()
  if (!cmd_node_) {
    const std::string cmd_node_name = std::string("btick_cmd_") + node->get_name() + "_" + id;
    try {
      rclcpp::NodeOptions cmd_node_options;
      cmd_node_options.use_global_arguments(false);

      cmd_node_ = std::make_shared<rclcpp::Node>(cmd_node_name, cmd_node_options);

      cmd_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      cmd_executor_->add_node(cmd_node_);
    } catch (const std::exception &ex) {
      const std::string original_error = ex.what();
      const bool node_name_conflict =
          original_error.find("Address already in use") != std::string::npos ||
          original_error.find("already in use") != std::string::npos ||
          original_error.find("duplicate") != std::string::npos;
      if (discovery_pub_) {
        std_msgs::msg::String error_msg;
        error_msg.data = nlohmann::json{
            {"type", "initialization_error"},
            {"source", "btick"},
            {"protocol", "btick"},
            {"tree_id", new_instance->tree_id()},
            {"tree_name", tree_name},
            {"session_id", "btick_" + id},
            {"ros2_node_name", cmd_node_name},
            {"error_category",
             node_name_conflict ? "ros2_node_name_conflict" : "ros2_node_creation"},
            {"ros2_node_name_conflict", node_name_conflict},
            {"error",
             original_error}}.dump();
        discovery_pub_->publish(error_msg);
      }
      RCLCPP_ERROR(logger_,
                   "[BTick] ❌ Failed to initialize Monitor\n"
                   "[BTick] Application/node: '%s'\n"
                   "[BTick] Tree: '%s'\n"
                   "[BTick] ROS2 node name: '%s'\n"
                   "[BTick] %s\n"
                   "[BTick] %s\n"
                   "[BTick] Original error: %s",
                   node->get_name(), tree_name.c_str(), cmd_node_name.c_str(),
                   node_name_conflict ? "[BTick] ❌ ROS2 node name is already in use by "
                                        "another running node."
                                      : "[BTick] ❌ ROS2 command node creation failed.",
                   node_name_conflict ? "[BTick] This is a ROS2 node-name conflict, not "
                                        "a Groot2/BTick TCP port conflict."
                                      : "[BTick] This failure occurred during ROS2 node "
                                        "creation, not Groot2/BTick TCP transport.",
                   original_error.c_str());

      throw std::runtime_error(
          std::string("[BTick][") +
          (node_name_conflict ? "ros2_node_name_conflict" : "ros2_node_creation") +
          "] Failed to create ROS2 command node '" + cmd_node_name + "' for tree '" + tree_name +
          (node_name_conflict ? "'. This is a ROS2 node-name conflict, "
                                "not a Groot2/BTick TCP port conflict. "
                              : "'. ROS2 node creation failed. ") +
          "Original error: " + original_error);
    }

    command_response_pub_ = cmd_node_->create_publisher<std_msgs::msg::String>(
        "/btick/command_out", rclcpp::QoS(20).reliable());

    command_sub_ = cmd_node_->create_subscription<std_msgs::msg::String>(
        "/btick/command_in", rclcpp::QoS(20).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (detached_ || !msg) return;

          std::string action;
          std::string request_id;
          std::string target_tree_id;
          std::shared_ptr<TreeInstance> target_instance;

          const auto publish_command_error = [&](const std::string &error,
                                                 const std::string &err_tree_id = "") {
            if (!command_response_pub_) {
              return;
            }
            nlohmann::json response = {
                {"type", "command_error"}, {"action", action}, {"error", error}};
            if (!request_id.empty()) {
              response["request_id"] = request_id;
            }
            const std::string tid =
                !err_tree_id.empty()
                    ? err_tree_id
                    : (target_instance ? target_instance->tree_id() : target_tree_id);
            if (!tid.empty()) {
              response["tree_id"] = tid;
            }
            std_msgs::msg::String response_msg;
            response_msg.data = response.dump();
            command_response_pub_->publish(response_msg);
          };

          const auto publish_command_ack = [&](const std::string &target_tid) {
            if (!command_response_pub_) {
              return;
            }
            nlohmann::json ack = {{"type", "command_ack"},
                                  {"action", action},
                                  {"tree_id", target_tid},
                                  {"success", true}};
            if (!request_id.empty()) {
              ack["request_id"] = request_id;
            }
            std_msgs::msg::String ack_msg;
            ack_msg.data = ack.dump();
            command_response_pub_->publish(ack_msg);
          };

          try {
            const nlohmann::json command = nlohmann::json::parse(msg->data);
            if (!command.is_object()) {
              throw std::invalid_argument("Command JSON must be an object");
            }
            action = field_as_string(command, "action");
            request_id = field_as_string(command, "request_id");
            auto get_field = [&command](const std::string &key) {
              return field_as_string(command, key);
            };

            target_tree_id = get_field("tree_id");

            if (target_tree_id == "ALL") {
              if (action != "clear_all_breakpoints" && action != "clear_all_faults") {
                publish_command_error("ALL is not supported for this action", "ALL");
                return;
              }
              {
                std::lock_guard<std::mutex> lock(instances_mutex_);
                for (const auto &entry : instances_) {
                  if (action == "clear_all_breakpoints") {
                    entry.second->clear_all_breakpoints();
                  } else {
                    entry.second->clear_all_faults();
                  }
                }
              }
              publish_command_ack("ALL");
              return;
            }

            if (target_tree_id.empty() || target_tree_id == "MainTree" ||
                target_tree_id == "tree") {
              publish_command_error("tree_id is required", target_tree_id);
              return;
            }

            {
              std::lock_guard<std::mutex> lock(instances_mutex_);
              auto it = instances_.find(target_tree_id);
              if (it != instances_.end()) {
                target_instance = it->second;
              }
            }

            if (!target_instance) {
              RCLCPP_DEBUG(
                  logger_,
                  "[BTick] Target tree '%s' not found on this monitor, ignoring command '%s'",
                  target_tree_id.c_str(), action.c_str());
              return;
            }

            RCLCPP_DEBUG(logger_, "[BTick] Command received: action='%s' tree='%s' paused_uid=%u",
                         action.c_str(), target_instance->tree_id().c_str(),
                         target_instance->paused_at_node_uid());

            if (action == "set_breakpoint") {
              uint16_t uid = 0;
              bool enabled = false;
              if (!parse_uid_field(command, uid)) {
                publish_command_error("Invalid uid: expected an integer in [0, 65535]");
              } else if (!parse_enabled_field(command, enabled)) {
                publish_command_error("Invalid enabled value: expected a boolean or 0/1");
              } else {
                target_instance->set_breakpoint(uid, enabled);
                publish_command_ack(target_instance->tree_id());
              }
            } else if (action == "clear_breakpoint") {
              uint16_t uid = 0;
              if (!parse_uid_field(command, uid)) {
                publish_command_error("Invalid uid: expected an integer in [0, 65535]");
              } else {
                target_instance->set_breakpoint(uid, false);
                publish_command_ack(target_instance->tree_id());
              }
            } else if (action == "clear_all_breakpoints") {
              target_instance->clear_all_breakpoints();
              publish_command_ack(target_instance->tree_id());
            } else if (action == "resume" || action == "continue") {
              std::string forced_status_str = get_field("forced_status");
              if (forced_status_str == "SUCCESS") {
                target_instance->force_success();
              } else if (forced_status_str == "FAILURE") {
                target_instance->force_failure();
              } else {
                target_instance->resume();
              }
              publish_command_ack(target_instance->tree_id());
            } else if (action == "force_success") {
              target_instance->force_success();
              publish_command_ack(target_instance->tree_id());
            } else if (action == "force_failure") {
              target_instance->force_failure();
              publish_command_ack(target_instance->tree_id());
            } else if (action == "step") {
              target_instance->step();
              publish_command_ack(target_instance->tree_id());
            } else if (action == "inject_fault" || action == "set_fault") {
              uint16_t uid = 0;
              std::string mode = get_field("mode");
              int delay_ms = 0;
              if (!parse_uid_field(command, uid)) {
                publish_command_error("Invalid uid: expected an integer in [0, 65535]");
              } else if (mode.empty()) {
                publish_command_error("Fault mode cannot be empty");
              } else if (!parse_delay_field(command, delay_ms)) {
                publish_command_error("Invalid delay_ms: expected a non-negative integer");
              } else {
                target_instance->set_fault_injection(uid, mode, delay_ms);
                publish_command_ack(target_instance->tree_id());
              }
            } else if (action == "clear_fault") {
              uint16_t uid = 0;
              if (!parse_uid_field(command, uid)) {
                publish_command_error("Invalid uid: expected an integer in [0, 65535]");
              } else {
                target_instance->clear_fault_injection(uid);
                publish_command_ack(target_instance->tree_id());
              }
            } else if (action == "clear_all_faults") {
              target_instance->clear_all_faults();
              publish_command_ack(target_instance->tree_id());
            } else if (action == "set_blackboard") {
              std::string request_id = get_field("request_id");
              std::string key = get_field("key");
              std::string val_str = get_field("value");
              std::string type_str = get_field("value_type");
              if (type_str.empty()) {
                type_str = get_field("type");
              }

              RCLCPP_DEBUG(logger_,
                           "[BTick] Blackboard command received: "
                           "request_id='%s' tree='%s' key='%s'",
                           request_id.c_str(), target_instance->tree_id().c_str(), key.c_str());

              std::string error_msg;
              std::string actual_val_json;
              bool ok = target_instance->set_blackboard_value(key, val_str, type_str, error_msg,
                                                              actual_val_json);

              if (ok) {
                RCLCPP_INFO(logger_,
                            "[BTick] Blackboard updated: tree='%s' key='%s' "
                            "value='%s' type='%s'",
                            target_instance->tree_id().c_str(), key.c_str(), val_str.c_str(),
                            type_str.c_str());
              } else {
                RCLCPP_WARN(logger_,
                            "[BTick] Blackboard update failed: tree='%s' "
                            "key='%s' reason='%s'",
                            target_instance->tree_id().c_str(), key.c_str(), error_msg.c_str());
              }

              if (command_response_pub_) {
                nlohmann::json response = {{"type", "blackboard_update_response"},
                                           {"source", "ros2"},
                                           {"protocol", "ros2"},
                                           {"tree_id", target_instance->tree_id()},
                                           {"key", key},
                                           {"value_type", type_str},
                                           {"success", ok}};
                if (!request_id.empty()) {
                  response["request_id"] = request_id;
                }
                if (ok) {
                  response["value"] = nlohmann::json::parse(actual_val_json);
                } else {
                  response["error"] = error_msg;
                }
                std_msgs::msg::String resp_msg;
                resp_msg.data = response.dump();
                command_response_pub_->publish(resp_msg);
                RCLCPP_DEBUG(logger_,
                             "[BTick] Blackboard response published: "
                             "request_id='%s' ok=%s",
                             request_id.c_str(), ok ? "true" : "false");
              }
            } else if (action == "set_node_port") {
              std::string request_id = get_field("request_id");
              std::string uid_str = get_field("uid");
              std::string port_name = get_field("port_name");
              std::string val_str = get_field("value");
              std::string mode_str = get_field("mode");

              uint16_t uid = 0;
              const bool uid_is_valid = parse_uid_field(command, uid);

              std::string error_msg;
              std::string actual_val;
              std::string actual_mode;
              const bool ok = uid_is_valid && target_instance->set_node_port_value(
                                                  uid, port_name, val_str, mode_str, error_msg,
                                                  actual_val, actual_mode);
              if (!uid_is_valid) {
                error_msg = "Invalid uid: expected an integer in [0, 65535]";
              }

              if (ok) {
                RCLCPP_INFO(logger_,
                            "[BTick] Node port updated: tree='%s' "
                            "port='%s' value='%s' mode='%s'",
                            target_instance->tree_id().c_str(), port_name.c_str(),
                            actual_val.c_str(), actual_mode.c_str());
              } else {
                RCLCPP_WARN(logger_,
                            "[BTick] Node port update failed: tree='%s' "
                            "port='%s' reason='%s'",
                            target_instance->tree_id().c_str(), port_name.c_str(),
                            error_msg.c_str());
              }

              if (command_response_pub_) {
                nlohmann::json response = {{"type", "set_node_port_response"},
                                           {"source", "ros2"},
                                           {"protocol", "ros2"},
                                           {"tree_id", target_instance->tree_id()},
                                           {"uid", uid},
                                           {"port_name", port_name},
                                           {"success", ok}};
                if (!request_id.empty()) {
                  response["request_id"] = request_id;
                }
                if (ok) {
                  response["value"] = actual_val;
                  response["mode"] = actual_mode;
                } else {
                  response["error"] = error_msg;
                }
                std_msgs::msg::String resp_msg;
                resp_msg.data = response.dump();
                command_response_pub_->publish(resp_msg);
              }
            } else if (action == "get_tree" || action == "discover" || action == "request_tree") {
              if (discovery_pub_) {
                std_msgs::msg::String msg;
                msg.data = target_instance->to_json_tree_created();
                discovery_pub_->publish(msg);
              }
              publish_command_ack(target_instance->tree_id());
            }
          } catch (const std::exception &e) {
            RCLCPP_WARN(logger_, "[BTick] Error processing command: %s", e.what());
            publish_command_error(e.what());
          }
        });

    auto last_full_publish = std::chrono::steady_clock::now();
    auto burst_count = std::make_shared<uint32_t>(0);
    std::function<void()> discovery_callback = [this, last_full_publish, burst_count]() mutable {
      if (detached_ || !discovery_pub_) {
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      // Keep full snapshots within consumer liveness windows without
      // flooding transient-local QoS history.
      if (*burst_count >= 10 && now - last_full_publish < std::chrono::seconds(1)) {
        return;
      }
      last_full_publish = now;
      if (*burst_count < 10) {
        ++*burst_count;
      }

      std::vector<std::shared_ptr<TreeInstance>> instances;
      {
        std::lock_guard<std::mutex> lock(instances_mutex_);
        for (const auto &entry : instances_) {
          instances.push_back(entry.second);
        }
      }
      for (const auto &instance : instances) {
        std_msgs::msg::String discovery_msg;
        discovery_msg.data = instance->to_json_tree_created();
        discovery_pub_->publish(discovery_msg);
      }
    };
    discovery_timer_ =
        cmd_node_->create_wall_timer(std::chrono::milliseconds(30), discovery_callback);

    cmd_thread_ = std::thread([this]() {
      if (cmd_executor_) {
        cmd_executor_->spin();
      }
    });
  }

  // Hook into TreeInstance status change callback
  new_instance->add_event_listener([this](const NodeStatusEvent &ev) {
    if (detached_) return;

    // 1. Publish to high-speed JSON transition topic
    if (transition_pub_) {
      const nlohmann::json transition = node_status_event_to_json(ev);
      std_msgs::msg::String msg;
      msg.data = transition.dump();
      transition_pub_->publish(msg);
    }
  });

  // Publish initial tree discovery packet
  std_msgs::msg::String disc_msg;
  disc_msg.data = new_instance->to_json_tree_created();
  discovery_pub_->publish(disc_msg);

  RCLCPP_INFO(logger_, "[BTick] Connected: tree='%s'", display_name.c_str());
}

Monitor::~Monitor() { detach(); }

void Monitor::detach() {
  if (detached_) return;
  detached_ = true;

  if (discovery_timer_) {
    discovery_timer_->cancel();
    discovery_timer_.reset();
  }
  if (cmd_executor_) {
    cmd_executor_->cancel();
  }
  if (cmd_thread_.joinable()) {
    cmd_thread_.join();
  }
  if (cmd_node_) {
    if (cmd_executor_) {
      cmd_executor_->remove_node(cmd_node_);
    }
    cmd_node_.reset();
  }
  cmd_executor_.reset();

  {
    std::lock_guard<std::mutex> lock(instances_mutex_);
    for (const auto &entry : instances_) {
      entry.second->set_status(TreeStatus::DESTROYED);
      RCLCPP_INFO(logger_, "[BTick] Disconnected: tree='%s'", entry.second->tree_name().c_str());
      if (discovery_pub_) {
        std_msgs::msg::String msg;
        msg.data = nlohmann::json{{"type", "tree_destroyed"}, {"tree_id", entry.first}}.dump();
        discovery_pub_->publish(msg);
      }
    }
    instances_.clear();
    instance_.reset();
  }
}

Monitor::ListenerHandle Monitor::add_event_listener(const std::string &tree_id,
                                                    EventListener listener) {
  std::lock_guard<std::mutex> lock(instances_mutex_);
  auto it = instances_.find(tree_id);
  if (it != instances_.end()) {
    return it->second->add_event_listener(std::move(listener));
  }
  if (tree_id.empty() && instance_) {
    return instance_->add_event_listener(std::move(listener));
  }
  return 0;
}

Monitor::ListenerHandle Monitor::add_event_listener(EventListener listener) {
  std::lock_guard<std::mutex> lock(instances_mutex_);
  return instance_ ? instance_->add_event_listener(std::move(listener)) : 0;
}

bool Monitor::remove_event_listener(const std::string &tree_id, ListenerHandle handle) {
  std::lock_guard<std::mutex> lock(instances_mutex_);
  auto it = instances_.find(tree_id);
  if (it != instances_.end()) {
    return it->second->remove_event_listener(handle);
  }
  if (tree_id.empty() && instance_) {
    return instance_->remove_event_listener(handle);
  }
  return false;
}

bool Monitor::remove_event_listener(ListenerHandle handle) {
  std::lock_guard<std::mutex> lock(instances_mutex_);
  return instance_ ? instance_->remove_event_listener(handle) : false;
}

std::string Monitor::get_tree_created_json(const std::string &tree_id) const {
  std::lock_guard<std::mutex> lock(instances_mutex_);
  if (!tree_id.empty()) {
    auto it = instances_.find(tree_id);
    if (it != instances_.end()) {
      return it->second->to_json_tree_created();
    }
  }
  return instance_ ? instance_->to_json_tree_created() : std::string{};
}

std::shared_ptr<TreeInstance> Monitor::get_tree_instance(const std::string &tree_id) const {
  std::lock_guard<std::mutex> lock(instances_mutex_);
  if (!tree_id.empty()) {
    auto it = instances_.find(tree_id);
    if (it != instances_.end()) {
      return it->second;
    }
  }
  return instance_;
}

std::string Monitor::tree_id() const {
  std::lock_guard<std::mutex> lock(instances_mutex_);
  return instance_ ? instance_->tree_id() : std::string{};
}

std::string Monitor::tree_name() const {
  std::lock_guard<std::mutex> lock(instances_mutex_);
  return instance_ ? instance_->tree_name() : std::string{};
}

}  // namespace btick
