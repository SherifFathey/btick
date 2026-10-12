#include "btick/tree_instance.hpp"

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/json_export.h>
#include <unistd.h>

#include <algorithm>
#include <behaviortree_cpp/scripting/script_parser.hpp>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <thread>
#include <unordered_set>

#include "btick/json_utils.hpp"
#include "btick/port_utils.hpp"
#include "btick/tree_serializer.hpp"

namespace btick {

namespace {
struct TreeNodeAccessor : public BT::TreeNode {
  using BT::TreeNode::config;
  using BT::TreeNode::modifyPortsRemapping;
  using BT::TreeNode::postConditionsScripts;
  using BT::TreeNode::preConditionsScripts;
  using BT::TreeNode::setStatus;
};
}  // namespace

static double current_time_seconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return static_cast<double>(nanoseconds) / 1'000'000'000.0;
}

static std::string normalize_port_type(std::string type_name) {
  if (type_name.find("basic_string") != std::string::npos || type_name == "std::string") {
    return "string";
  }
  return type_name.empty() ? "string" : type_name;
}

static std::string port_mode(const std::string &value) {
  return value.size() >= 2 && value.front() == '{' && value.back() == '}' ? "Blackboard"
                                                                          : "Literal";
}

static std::string port_direction(BT::PortDirection direction) {
  switch (direction) {
    case BT::PortDirection::INPUT:
      return "Input";
    case BT::PortDirection::OUTPUT:
      return "Output";
    case BT::PortDirection::INOUT:
      return "Inout";
    default:
      return "Unknown";
  }
}

static bool is_known_port_direction(const std::string &direction) {
  return direction == "Input" || direction == "Output" || direction == "Inout";
}

static bool parse_double_value(const std::string &text, double &value) {
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value, std::chars_format::general);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
}

static std::shared_ptr<BT::Blackboard::Entry> find_blackboard_entry(
    const BT::TreeNode &node, const std::vector<BT::Blackboard::Ptr> &blackboards,
    const std::string &key) {
  if (node.config().blackboard) {
    auto entry = node.config().blackboard->getEntry(key);
    if (entry) {
      return entry;
    }
  }
  for (const auto &blackboard : blackboards) {
    if (blackboard) {
      auto entry = blackboard->getEntry(key);
      if (entry) {
        return entry;
      }
    }
  }
  return {};
}

struct BlackboardValueSnapshot {
  uint64_t sequence_id{0};
  std::optional<std::string> value;
};

static BlackboardValueSnapshot snapshot_blackboard_entry(
    const std::shared_ptr<BT::Blackboard::Entry> &entry) {
  if (!entry) {
    return {};
  }

  BlackboardValueSnapshot snapshot;
  std::unique_lock<std::mutex> lock(entry->entry_mutex);
  snapshot.sequence_id = entry->sequence_id;
  if (entry->value.isString()) {
    if (auto value = entry->value.tryCast<std::string>()) {
      snapshot.value = value.value();
    }
  } else if (entry->value.isNumber()) {
    if (auto value = entry->value.tryCast<double>()) {
      snapshot.value = format_json_number(value.value());
    }
  } else if (entry->value.isType<bool>()) {
    if (auto value = entry->value.tryCast<bool>()) {
      snapshot.value = value.value() ? "true" : "false";
    }
  } else {
    snapshot.value = entry->info.typeName();
  }
  return snapshot;
}

static std::string combined_config_direction(const BT::TreeNode &node,
                                             const std::string &port_name) {
  const bool is_input = node.config().input_ports.count(port_name) != 0;
  const bool is_output = node.config().output_ports.count(port_name) != 0;
  if (is_input && is_output) {
    return "Inout";
  }
  if (is_input) {
    return "Input";
  }
  if (is_output) {
    return "Output";
  }
  return "Unknown";
}

BTPortDetail TreeInstance::resolve_port(const BT::TreeNode &node, const std::string &port_name,
                                        const BTPortDetail *xml_port) const {
  BTPortDetail resolved;
  resolved.name = port_name;

  const auto &config = node.config();
  const BT::PortInfo *manifest_port = nullptr;
  if (config.manifest) {
    auto it = config.manifest->ports.find(port_name);
    if (it != config.manifest->ports.end()) {
      manifest_port = &it->second;
      resolved.direction = port_direction(manifest_port->direction());
      resolved.type = normalize_port_type(manifest_port->typeName());
      resolved.default_value = manifest_port->defaultValueString();
      resolved.description = manifest_port->description();
    }
  }

  if (!is_known_port_direction(resolved.direction)) {
    resolved.direction = combined_config_direction(node, port_name);
  }
  if (!is_known_port_direction(resolved.direction) && xml_port &&
      is_known_port_direction(xml_port->direction)) {
    resolved.direction = xml_port->direction;
  }
  if (!is_known_port_direction(resolved.direction)) {
    resolved.direction = "Unknown";
  }

  if (resolved.type == "string" && xml_port && !xml_port->type.empty() &&
      (!manifest_port || manifest_port->typeName().empty())) {
    resolved.type = xml_port->type;
  }
  if (resolved.default_value.empty() && xml_port) {
    resolved.default_value = xml_port->default_value;
  }
  if (resolved.description.empty() && xml_port) {
    resolved.description = xml_port->description;
  }

  auto input_it = config.input_ports.find(port_name);
  auto output_it = config.output_ports.find(port_name);
  if (input_it != config.input_ports.end()) {
    resolved.value = input_it->second;
  } else if (output_it != config.output_ports.end()) {
    resolved.value = output_it->second;
  } else if (xml_port && !xml_port->value.empty()) {
    resolved.value = xml_port->value;
  } else {
    resolved.value = resolved.default_value;
  }

  if (resolved.type == "string" && !resolved.value.empty() &&
      (!manifest_port || manifest_port->typeName().empty()) &&
      (!xml_port || xml_port->type.empty())) {
    resolved.type = infer_port_type(resolved.value);
  }
  resolved.mode = port_mode(resolved.value);
  return resolved;
}

TreeInstance::TreeInstance(const BT::Tree &tree, const std::string &tree_name,
                           const std::string &xml_path, const std::string &tree_id,
                           const std::string &display_name, const std::string &origin,
                           const std::string &node_name, const rclcpp::Logger &logger)
    : StatusChangeLogger(tree.rootNode()),
      tree_name_(tree_name),
      display_name_(display_name.empty() ? tree_name : display_name),
      origin_(origin),
      xml_path_(xml_path),
      node_name_(node_name),
      logger_(logger) {
  pid_ = static_cast<int>(::getpid());
  char host_buf[256];
  if (::gethostname(host_buf, sizeof(host_buf)) == 0) {
    hostname_ = host_buf;
  }
  if (!tree_id.empty()) {
    tree_id_ = tree_id;
  } else {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(1000, 9999);
    tree_id_ = tree_name + "_" + std::to_string(dis(gen));
  }

  // Load XML tree model if xml_path is valid for port / attribute enrichment
  BTTreeModel xml_model;
  if (!xml_path_.empty()) {
    std::ifstream f(xml_path_);
    if (f.good()) {
      xml_model = TreeParser::parse_xml_file(xml_path_);
    }
  }
  std::unordered_map<std::string, std::string> xml_path_to_id;
  std::function<void(const std::string &, const std::string &)> index_xml_paths =
      [&](const std::string &node_id, const std::string &path) {
        const auto node_it = xml_model.nodes.find(node_id);
        if (node_it == xml_model.nodes.end()) {
          return;
        }
        xml_path_to_id[path] = node_id;
        for (size_t index = 0; index < node_it->second.children_ids.size(); ++index) {
          index_xml_paths(node_it->second.children_ids[index], path + "/" + std::to_string(index));
        }
      };
  if (!xml_model.root_node_id.empty()) {
    index_xml_paths(xml_model.root_node_id, "");
  }

  // Build tree model directly from live instantiated BT::Tree structure to
  // guarantee 100% UID alignment
  if (tree.rootNode()) {
    root_bb_ = const_cast<BT::Tree &>(tree).rootBlackboard();
    if (root_bb_) {
      blackboards_.push_back(root_bb_);
    }
    for (const auto &st : tree.subtrees) {
      if (st && st->blackboard) {
        bool exists = false;
        for (const auto &bb : blackboards_) {
          if (bb == st->blackboard) {
            exists = true;
            break;
          }
        }
        if (!exists) {
          blackboards_.push_back(st->blackboard);
        }
      }
    }

    model_.tree_id = tree_name_;
    model_.tree_name = tree_name_;
    model_.raw_xml = xml_model.raw_xml;

    std::unordered_set<std::string> ambiguous_runtime_names;
    std::function<void(const BT::TreeNode *, const std::string &, const std::string &)> traverse =
        [&](const BT::TreeNode *n, const std::string &parent_id, const std::string &path) {
          if (!n) return;
          uint16_t uid = n->UID();
          std::string id = n->name() + "_" + std::to_string(uid);
          BTNodeModel m;
          m.id = id;
          m.uid = uid;
          m.name = n->name();
          m.type = n->registrationName().empty() ? toStr(n->type()) : n->registrationName();

          if (n->type() == BT::NodeType::CONTROL)
            m.category = "Control";
          else if (n->type() == BT::NodeType::ACTION)
            m.category = "Action";
          else if (n->type() == BT::NodeType::CONDITION)
            m.category = "Condition";
          else if (n->type() == BT::NodeType::DECORATOR)
            m.category = "Decorator";
          else if (n->type() == BT::NodeType::SUBTREE)
            m.category = "SubTree";
          else
            m.category = TreeParser::determine_category(m.type);

          m.parent_id = parent_id;
          m.status = toStr(n->status());

          // 1. Extract exact runtime ports and attributes directly from
          // TreeNode config
          for (const auto &p : n->config().input_ports) {
            if (p.first == "ID" || (!p.first.empty() && p.first.front() == '_')) {
              continue;
            }
            std::string val = p.second;
            if (!val.empty() && val.front() == '{' && val.back() == '}') {
              m.ports[p.first] = val;
            } else {
              m.attributes[p.first] = val;
            }
          }
          for (const auto &p : n->config().output_ports) {
            if (p.first == "ID" || (!p.first.empty() && p.first.front() == '_')) {
              continue;
            }
            std::string val = p.second;
            if (!val.empty() && val.front() == '{' && val.back() == '}') {
              m.ports[p.first] = val;
            } else {
              m.attributes[p.first] = val;
            }
          }

          std::string xml_id;
          const auto path_it = xml_path_to_id.find(path);
          if (path_it != xml_path_to_id.end()) {
            xml_id = path_it->second;
          } else if (xml_model.name_to_id.count(m.name)) {
            xml_id = xml_model.name_to_id.at(m.name);
          }
          const BTNodeModel *xml_node = !xml_id.empty() && xml_model.nodes.count(xml_id)
                                            ? &xml_model.nodes.at(xml_id)
                                            : nullptr;

          // Resolve each port exactly once. Runtime metadata is primary; XML
          // details are only used to fill fields absent from the live node.
          std::set<std::string> port_names;
          for (const auto &p : n->config().input_ports) {
            if (p.first != "ID" && (p.first.empty() || p.first.front() != '_')) {
              port_names.insert(p.first);
            }
          }
          for (const auto &p : n->config().output_ports) {
            if (p.first != "ID" && (p.first.empty() || p.first.front() != '_')) {
              port_names.insert(p.first);
            }
          }
          if (n->config().manifest) {
            for (const auto &p : n->config().manifest->ports) {
              if (p.first != "ID" && (p.first.empty() || p.first.front() != '_')) {
                port_names.insert(p.first);
              }
            }
          }
          if (xml_node) {
            for (const auto &p : xml_node->port_details) port_names.insert(p.name);
            for (const auto &p : xml_node->ports) m.ports.emplace(p.first, p.second);
            for (const auto &p : xml_node->attributes) m.attributes.emplace(p.first, p.second);
            for (const auto &p : xml_node->meta) m.meta.emplace(p.first, p.second);
          }
          for (const auto &port_name : port_names) {
            const BTPortDetail *xml_port = nullptr;
            if (xml_node) {
              for (const auto &candidate : xml_node->port_details) {
                if (candidate.name == port_name) {
                  xml_port = &candidate;
                  break;
                }
              }
            }
            m.port_details.push_back(resolve_port(*n, port_name, xml_port));
          }

          if (parent_id.empty() && model_.root_node_id.empty()) {
            model_.root_node_id = id;
          }

          model_.nodes[id] = m;
          model_.uid_to_id[uid] = id;
          if (!m.name.empty()) {
            if (!ambiguous_runtime_names.count(m.name)) {
              if (model_.name_to_id.count(m.name)) {
                model_.name_to_id.erase(m.name);
                ambiguous_runtime_names.insert(m.name);
              } else {
                model_.name_to_id[m.name] = id;
              }
            }
          }
          uid_to_node_[uid] = const_cast<BT::TreeNode *>(n);

          if (!parent_id.empty() && model_.nodes.count(parent_id)) {
            model_.nodes[parent_id].children_ids.push_back(id);
          }

          if (auto control = dynamic_cast<const BT::ControlNode *>(n)) {
            size_t child_index = 0;
            for (const auto &child : control->children()) {
              traverse(child, id, path + "/" + std::to_string(child_index++));
            }
          } else if (auto decorator = dynamic_cast<const BT::DecoratorNode *>(n)) {
            if (decorator->child()) {
              traverse(decorator->child(), id, path + "/0");
            }
          }
        };
    traverse(tree.rootNode(), "", "");
    root_node_ = const_cast<BT::TreeNode *>(tree.rootNode());
    register_hooks_recursive(root_node_);
  } else if (!xml_model.nodes.empty()) {
    model_ = xml_model;
  }

  status_ = TreeStatus::CREATED;
}

TreeInstance::~TreeInstance() {
  set_status(TreeStatus::DESTROYED);
  if (root_node_) {
    clear_hooks_recursive(root_node_);
    root_node_ = nullptr;
  }
  wait_for_hooks();
  uid_to_node_.clear();
  {
    std::lock_guard<std::mutex> lock(mutex_port_queue_);
    queued_port_updates_.clear();
  }
}

TreeInstance::HookExecutionGuard::HookExecutionGuard(TreeInstance &owner) : owner_(owner) {
  std::lock_guard<std::mutex> lock(owner_.mutex_hooks_);
  ++owner_.active_hooks_;
}

TreeInstance::HookExecutionGuard::~HookExecutionGuard() {
  {
    std::lock_guard<std::mutex> lock(owner_.mutex_hooks_);
    --owner_.active_hooks_;
  }
  owner_.cv_hooks_.notify_all();
}

void TreeInstance::wait_for_hooks() {
  std::unique_lock<std::mutex> lock(mutex_hooks_);
  cv_hooks_.wait(lock, [this]() { return active_hooks_ == 0; });
}

void TreeInstance::clear_hooks_recursive(BT::TreeNode *n) {
  if (!n) return;
  auto &scripts = static_cast<TreeNodeAccessor *>(n)->preConditionsScripts();
  auto &post_scripts = static_cast<TreeNodeAccessor *>(n)->postConditionsScripts();
  {
    std::lock_guard<std::mutex> lk(mutex_original_scripts_);
    auto fail_it = original_failure_if_scripts_.find(n->UID());
    if (fail_it != original_failure_if_scripts_.end()) {
      scripts[size_t(BT::PreCond::FAILURE_IF)] = fail_it->second;
      original_failure_if_scripts_.erase(fail_it);
    }
    auto succ_it = original_success_if_scripts_.find(n->UID());
    if (succ_it != original_success_if_scripts_.end()) {
      scripts[size_t(BT::PreCond::SUCCESS_IF)] = succ_it->second;
      original_success_if_scripts_.erase(succ_it);
    }
    auto skip_it = original_skip_if_scripts_.find(n->UID());
    if (skip_it != original_skip_if_scripts_.end()) {
      scripts[size_t(BT::PreCond::SKIP_IF)] = skip_it->second;
      original_skip_if_scripts_.erase(skip_it);
    }
    auto while_it = original_while_true_scripts_.find(n->UID());
    if (while_it != original_while_true_scripts_.end()) {
      scripts[size_t(BT::PreCond::WHILE_TRUE)] = while_it->second;
      original_while_true_scripts_.erase(while_it);
    }
    auto always_it = original_always_post_scripts_.find(n->UID());
    if (always_it != original_always_post_scripts_.end()) {
      post_scripts[size_t(BT::PostCond::ALWAYS)] = always_it->second;
      original_always_post_scripts_.erase(always_it);
    }
  }

  if (auto control = dynamic_cast<BT::ControlNode *>(n)) {
    for (auto &child : control->children()) {
      clear_hooks_recursive(const_cast<BT::TreeNode *>(child));
    }
  } else if (auto decorator = dynamic_cast<BT::DecoratorNode *>(n)) {
    if (decorator->child()) {
      clear_hooks_recursive(const_cast<BT::TreeNode *>(decorator->child()));
    }
  }
}

void TreeInstance::register_hooks_recursive(BT::TreeNode *n) {
  if (!n) return;

  struct PortTickSnapshot {
    bool output_baseline_initialized{false};
    std::vector<BTPortDetail> input_ports;
    std::vector<BTPortDetail> output_ports;
    std::map<std::string, std::pair<std::string, uint64_t>> output_sequences;
    std::map<std::string, std::pair<std::string, std::string>> input_values;
  };

  auto port_snapshot = std::make_shared<PortTickSnapshot>();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto node_model_id = model_.uid_to_id.find(n->UID());
    if (node_model_id != model_.uid_to_id.end()) {
      const auto node_model = model_.nodes.find(node_model_id->second);
      if (node_model != model_.nodes.end()) {
        for (const auto &detail : node_model->second.port_details) {
          if (detail.direction == "Input" || detail.direction == "Inout") {
            port_snapshot->input_ports.push_back(detail);
          }
          if (detail.direction == "Output" || detail.direction == "Inout") {
            port_snapshot->output_ports.push_back(detail);
          }
        }
      }
    }
  }

  auto &precondition_scripts = static_cast<TreeNodeAccessor *>(n)->preConditionsScripts();
  const auto failure_if_index = size_t(BT::PreCond::FAILURE_IF);
  const auto success_if_index = size_t(BT::PreCond::SUCCESS_IF);
  const auto skip_if_index = size_t(BT::PreCond::SKIP_IF);
  const auto while_true_index = size_t(BT::PreCond::WHILE_TRUE);

  auto &post_scripts = static_cast<TreeNodeAccessor *>(n)->postConditionsScripts();
  const auto always_index = size_t(BT::PostCond::ALWAYS);

  {
    std::lock_guard<std::mutex> lk(mutex_original_scripts_);
    original_failure_if_scripts_[n->UID()] = precondition_scripts[failure_if_index];
    original_success_if_scripts_[n->UID()] = precondition_scripts[success_if_index];
    original_skip_if_scripts_[n->UID()] = precondition_scripts[skip_if_index];
    original_while_true_scripts_[n->UID()] = precondition_scripts[while_true_index];
    original_always_post_scripts_[n->UID()] = post_scripts[always_index];
  }

  precondition_scripts[failure_if_index] =
      [this, node_ptr = n, port_snapshot](BT::Ast::Environment &environment) -> BT::Any {
    HookExecutionGuard hook_guard(*this);
    if (status_ == TreeStatus::DESTROYED) {
      return BT::Any(false);
    }

    uint16_t uid = node_ptr->UID();

    // 0. Drain and apply any queued live port updates for this node on the BT ticking thread
    {
      std::lock_guard<std::mutex> lock(mutex_port_queue_);
      auto it = queued_port_updates_.find(uid);
      if (it != queued_port_updates_.end()) {
        for (const auto &upd : it->second) {
          apply_port_to_node(node_ptr, upd.port_name, upd.value);
        }
        queued_port_updates_.erase(it);
      }
    }

    // Capture input port values and baseline output sequence IDs before tick
    const auto &node_config = static_cast<TreeNodeAccessor *>(node_ptr)->config();
    port_snapshot->input_values.clear();
    for (const auto &port : port_snapshot->input_ports) {
      auto configured_input = node_config.input_ports.find(port.name);
      const std::string value =
          configured_input == node_config.input_ports.end() ? port.value : configured_input->second;
      if (port_mode(value) == "Blackboard") {
        if (value.size() >= 3 && value.front() == '{' && value.back() == '}') {
          const std::string key = value.substr(1, value.size() - 2);
          const auto entry = find_blackboard_entry(*node_ptr, blackboards_, key);
          const auto snapshot = snapshot_blackboard_entry(entry);
          if (snapshot.value) {
            port_snapshot->input_values[port.name] = {key, *snapshot.value};
          }
        }
      } else {
        port_snapshot->input_values[port.name] = {"", value};
      }
    }

    port_snapshot->output_sequences.clear();
    for (const auto &port : port_snapshot->output_ports) {
      auto configured_output = node_config.output_ports.find(port.name);
      if (configured_output == node_config.output_ports.end()) {
        continue;
      }
      const std::string &mapping = configured_output->second;
      if (mapping.size() >= 3 && mapping.front() == '{' && mapping.back() == '}') {
        const std::string key = mapping.substr(1, mapping.size() - 2);
        const auto entry = find_blackboard_entry(*node_ptr, blackboards_, key);
        const auto snapshot = snapshot_blackboard_entry(entry);
        port_snapshot->output_sequences[port.name] = {key, snapshot.sequence_id};
      }
    }
    port_snapshot->output_baseline_initialized = true;

    PreDecision decision = PreDecision::NONE;

    // 1. Check Interactive Breakpoints & Single-Stepping
    {
      std::unique_lock<std::mutex> lock(mutex_breakpoint_);
      bool is_user_bp = !breakpoints_.empty() && breakpoints_.count(uid);
      bool is_bypassed = (uid == resume_bypassed_uid_ &&
                          node_ptr->status() == BT::NodeStatus::RUNNING && !single_step_mode_);

      bool is_leaf_node = (dynamic_cast<BT::ActionNodeBase *>(node_ptr) != nullptr ||
                           dynamic_cast<BT::ConditionNode *>(node_ptr) != nullptr);
      bool is_step_target = single_step_mode_ && (uid != step_source_uid_) && is_leaf_node;

      if ((is_user_bp || is_step_target) && !is_bypassed) {
        paused_at_node_uid_ = uid;
        resume_requested_ = false;
        step_requested_ = false;
        single_step_mode_ = false;
        step_source_uid_ = 0;

        NodeStatusEvent ev_paused;
        ev_paused.tree_id = tree_id_;
        ev_paused.tree_name = tree_name_;
        ev_paused.node_id = node_ptr->name() + "_" + std::to_string(uid);
        ev_paused.node_name = node_ptr->name();
        ev_paused.uid = uid;
        ev_paused.prev_status = toStr(node_ptr->status());
        ev_paused.current_status = "PAUSED_AT_BREAKPOINT";
        ev_paused.timestamp = current_time_seconds();
        ev_paused.blackboard_json = get_blackboard_json(
            static_cast<TreeNodeAccessor *>(node_ptr)->config().blackboard.get());
        emit_event(ev_paused);

        RCLCPP_INFO(logger_, "[BTick] Execution paused: tree='%s' node='%s'", tree_id_.c_str(),
                    node_ptr->name().c_str());

        // Wait until user makes decision: Continue, Force Success, Force
        // Failure, or Step
        cv_breakpoint_.wait(lock, [this]() {
          return resume_requested_ || step_requested_ || status_ == TreeStatus::DESTROYED;
        });

        paused_at_node_uid_ = 0;
        resume_requested_ = false;
        step_requested_ = false;
        BT::NodeStatus decision_status = forced_return_status_;
        forced_return_status_ = BT::NodeStatus::IDLE;

        // Release lock before sending event callback to prevent any deadlock
        lock.unlock();

        NodeStatusEvent ev_resumed;
        ev_resumed.tree_id = tree_id_;
        ev_resumed.tree_name = tree_name_;
        ev_resumed.node_id = node_ptr->name() + "_" + std::to_string(uid);
        ev_resumed.node_name = node_ptr->name();
        ev_resumed.uid = uid;
        ev_resumed.prev_status = "PAUSED_AT_BREAKPOINT";
        ev_resumed.current_status = (decision_status == BT::NodeStatus::SUCCESS)   ? "SUCCESS"
                                    : (decision_status == BT::NodeStatus::FAILURE) ? "FAILURE"
                                                                                   : "RESUMED";
        ev_resumed.timestamp = current_time_seconds();
        emit_event(ev_resumed);

        if (decision_status == BT::NodeStatus::SUCCESS) {
          decision = PreDecision::FORCED_SUCCESS;
        } else if (decision_status == BT::NodeStatus::FAILURE) {
          decision = PreDecision::FORCED_FAILURE;
        }
      }
    }

    // 2. Check Fault Injection Rules if not already forced by interactive decision
    if (decision == PreDecision::NONE) {
      std::unique_lock<std::mutex> lock(mutex_faults_);
      auto it = fault_rules_.find(uid);
      if (it != fault_rules_.end()) {
        FaultInjectionRule rule = it->second;
        if (rule.mode == "DELAY" && rule.delay_ms > 0) {
          RCLCPP_INFO(logger_, "[BTick] Delay started: node='%s' delay=%dms",
                      node_ptr->name().c_str(), rule.delay_ms);
          NodeStatusEvent event;
          event.tree_id = tree_id_;
          event.tree_name = tree_name_;
          event.node_id = node_ptr->name() + "_" + std::to_string(uid);
          event.node_name = node_ptr->name();
          event.uid = uid;
          event.prev_status = toStr(node_ptr->status());
          event.current_status = "DELAY_WAITING";
          event.duration = static_cast<double>(rule.delay_ms);
          event.timestamp = current_time_seconds();
          event.blackboard_json = get_blackboard_json(
              static_cast<TreeNodeAccessor *>(node_ptr)->config().blackboard.get());
          lock.unlock();
          emit_event(event);
          lock.lock();

          cv_fault_.wait_for(lock, std::chrono::milliseconds(rule.delay_ms), [this, uid]() {
            const auto current = fault_rules_.find(uid);
            return current == fault_rules_.end() || current->second.mode != "DELAY" ||
                   status_ == TreeStatus::DESTROYED;
          });

          const auto current = fault_rules_.find(uid);
          const bool has_current_rule = current != fault_rules_.end();
          if (has_current_rule) {
            rule = current->second;
          }
          lock.unlock();
          event.prev_status = "DELAY_WAITING";
          event.current_status = "DELAY_FINISHED";
          event.duration = 0.0;
          event.timestamp = current_time_seconds();
          emit_event(event);
          lock.lock();
          if (!has_current_rule) {
            rule.mode.clear();
          }
        }

        if (rule.mode == "FORCE_FAILURE" || rule.mode == "FAILURE") {
          decision = PreDecision::FORCED_FAILURE;
        } else if (rule.mode == "FORCE_SUCCESS" || rule.mode == "SUCCESS") {
          decision = PreDecision::FORCED_SUCCESS;
        } else if (rule.mode == "FORCE_SKIPPED" || rule.mode == "SKIPPED") {
          decision = PreDecision::FORCED_SKIPPED;
        } else if (rule.mode == "FORCE_RUNNING" || rule.mode == "RUNNING") {
          RCLCPP_INFO(logger_, "[BTick] Fault active: node='%s' mode='FORCE_RUNNING'",
                      node_ptr->name().c_str());
          NodeStatusEvent event;
          event.tree_id = tree_id_;
          event.tree_name = tree_name_;
          event.node_id = node_ptr->name() + "_" + std::to_string(uid);
          event.node_name = node_ptr->name();
          event.uid = uid;
          event.prev_status = toStr(node_ptr->status());
          event.current_status = "RUNNING";
          event.timestamp = current_time_seconds();
          event.blackboard_json = get_blackboard_json(
              static_cast<TreeNodeAccessor *>(node_ptr)->config().blackboard.get());
          lock.unlock();
          emit_event(event);
          lock.lock();

          cv_fault_.wait(lock, [this, uid]() {
            const auto current = fault_rules_.find(uid);
            return current == fault_rules_.end() || current->second.mode != "FORCE_RUNNING" ||
                   status_ == TreeStatus::DESTROYED;
          });
        }
      }
    }

    if (decision != PreDecision::NONE) {
      std::lock_guard<std::mutex> lk(mutex_pre_decision_);
      pending_pre_decision_[uid] = decision;
    }

    if (decision == PreDecision::FORCED_FAILURE) {
      std::lock_guard<std::mutex> lk(mutex_pre_decision_);
      pending_pre_decision_.erase(uid);
      return BT::Any(true);
    }

    BT::ScriptFunction original;
    {
      std::lock_guard<std::mutex> lk(mutex_original_scripts_);
      auto it = original_failure_if_scripts_.find(uid);
      if (it != original_failure_if_scripts_.end()) {
        original = it->second;
      }
    }
    return original ? original(environment) : BT::Any(false);
  };

  precondition_scripts[success_if_index] =
      [this, node_ptr = n](BT::Ast::Environment &environment) -> BT::Any {
    HookExecutionGuard hook_guard(*this);
    uint16_t uid = node_ptr->UID();
    bool is_forced_success = false;
    {
      std::lock_guard<std::mutex> lk(mutex_pre_decision_);
      auto it = pending_pre_decision_.find(uid);
      if (it != pending_pre_decision_.end() && it->second == PreDecision::FORCED_SUCCESS) {
        is_forced_success = true;
        pending_pre_decision_.erase(it);
      }
    }
    if (is_forced_success) {
      return BT::Any(true);
    }
    BT::ScriptFunction original;
    {
      std::lock_guard<std::mutex> lk(mutex_original_scripts_);
      auto it = original_success_if_scripts_.find(uid);
      if (it != original_success_if_scripts_.end()) {
        original = it->second;
      }
    }
    return original ? original(environment) : BT::Any(false);
  };

  precondition_scripts[skip_if_index] =
      [this, node_ptr = n](BT::Ast::Environment &environment) -> BT::Any {
    HookExecutionGuard hook_guard(*this);
    uint16_t uid = node_ptr->UID();
    bool is_forced_skipped = false;
    {
      std::lock_guard<std::mutex> lk(mutex_pre_decision_);
      auto it = pending_pre_decision_.find(uid);
      if (it != pending_pre_decision_.end() && it->second == PreDecision::FORCED_SKIPPED) {
        is_forced_skipped = true;
        pending_pre_decision_.erase(it);
      }
    }
    if (is_forced_skipped) {
      return BT::Any(true);
    }
    BT::ScriptFunction original;
    {
      std::lock_guard<std::mutex> lk(mutex_original_scripts_);
      auto it = original_skip_if_scripts_.find(uid);
      if (it != original_skip_if_scripts_.end()) {
        original = it->second;
      }
    }
    return original ? original(environment) : BT::Any(false);
  };

  precondition_scripts[while_true_index] =
      [this, node_ptr = n](BT::Ast::Environment &environment) -> BT::Any {
    HookExecutionGuard hook_guard(*this);
    uint16_t uid = node_ptr->UID();

    // Drain queued live port updates for running node
    {
      std::lock_guard<std::mutex> lock(mutex_port_queue_);
      auto it = queued_port_updates_.find(uid);
      if (it != queued_port_updates_.end()) {
        for (const auto &upd : it->second) {
          apply_port_to_node(node_ptr, upd.port_name, upd.value);
        }
        queued_port_updates_.erase(it);
      }
    }

    BT::ScriptFunction original;
    {
      std::lock_guard<std::mutex> lk(mutex_original_scripts_);
      auto it = original_while_true_scripts_.find(uid);
      if (it != original_while_true_scripts_.end()) {
        original = it->second;
      }
    }
    return original ? original(environment) : BT::Any(true);
  };

  post_scripts[always_index] = [this, node_ptr = n,
                                port_snapshot](BT::Ast::Environment &environment) -> BT::Any {
    HookExecutionGuard hook_guard(*this);
    uint16_t uid = node_ptr->UID();

    std::map<std::string, std::pair<std::string, std::string>> updates;
    updates.insert(port_snapshot->input_values.begin(), port_snapshot->input_values.end());

    for (const auto &baseline : port_snapshot->output_sequences) {
      const auto entry = find_blackboard_entry(*node_ptr, blackboards_, baseline.second.first);
      const auto current = snapshot_blackboard_entry(entry);
      if (current.value && current.sequence_id > baseline.second.second) {
        updates[baseline.first] = {baseline.second.first, *current.value};
      }
    }
    port_snapshot->output_baseline_initialized = false;
    port_snapshot->output_sequences.clear();
    port_snapshot->input_values.clear();

    nlohmann::json update_json = nlohmann::json::array();
    for (const auto &update : updates) {
      update_json.push_back({{"port_name", update.first},
                             {"blackboard_key", update.second.first},
                             {"value", update.second.second}});
    }
    const std::string port_updates_json = update_json.dump();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!updates.empty()) {
        discovery_json_dirty_ = true;
      }
      const auto node_model_id = model_.uid_to_id.find(uid);
      if (node_model_id != model_.uid_to_id.end()) {
        auto node_model = model_.nodes.find(node_model_id->second);
        if (node_model != model_.nodes.end()) {
          for (const auto &update : updates) {
            node_model->second.output_ports_values[update.first] = update.second.second;
            for (auto &detail : node_model->second.port_details) {
              if (detail.name == update.first) {
                detail.runtime_value = update.second.second;
                break;
              }
            }
          }
        }
      }
      pending_port_event_json_[uid] = port_updates_json;
    }

    BT::ScriptFunction original;
    {
      std::lock_guard<std::mutex> lk(mutex_original_scripts_);
      auto it = original_always_post_scripts_.find(uid);
      if (it != original_always_post_scripts_.end()) {
        original = it->second;
      }
    }
    return original ? original(environment) : BT::Any(true);
  };

  if (auto control = dynamic_cast<BT::ControlNode *>(n)) {
    for (auto &child : control->children()) {
      register_hooks_recursive(const_cast<BT::TreeNode *>(child));
    }
  } else if (auto decorator = dynamic_cast<BT::DecoratorNode *>(n)) {
    if (decorator->child()) {
      register_hooks_recursive(const_cast<BT::TreeNode *>(decorator->child()));
    }
  }
}

void TreeInstance::set_breakpoint(uint16_t uid, bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_breakpoint_);
  if (enabled) {
    breakpoints_.insert(uid);
    RCLCPP_DEBUG(logger_, "[BTick] Breakpoint set");
  } else {
    breakpoints_.erase(uid);
    if (resume_bypassed_uid_ == uid) {
      resume_bypassed_uid_ = 0;
    }
    RCLCPP_DEBUG(logger_, "[BTick] Breakpoint cleared");
    if (paused_at_node_uid_ == uid) {
      forced_return_status_ = BT::NodeStatus::IDLE;
      resume_requested_ = true;
      cv_breakpoint_.notify_all();
    }
  }
}

void TreeInstance::clear_all_breakpoints() {
  std::lock_guard<std::mutex> lock(mutex_breakpoint_);
  breakpoints_.clear();
  single_step_mode_ = false;
  step_source_uid_ = 0;
  resume_bypassed_uid_ = 0;
  forced_return_status_ = BT::NodeStatus::IDLE;
  resume_requested_ = true;
  cv_breakpoint_.notify_all();
  RCLCPP_DEBUG(logger_, "[BTick] All breakpoints cleared: tree='%s'", tree_id_.c_str());
}

void TreeInstance::resume() {
  std::lock_guard<std::mutex> lock(mutex_breakpoint_);
  forced_return_status_ = BT::NodeStatus::IDLE;
  single_step_mode_ = false;
  step_source_uid_ = 0;
  resume_bypassed_uid_ = paused_at_node_uid_;
  resume_requested_ = true;
  cv_breakpoint_.notify_all();
  RCLCPP_DEBUG(logger_, "[BTick] Execution resumed: tree='%s'", tree_id_.c_str());
}

void TreeInstance::force_success() {
  std::lock_guard<std::mutex> lock(mutex_breakpoint_);
  forced_return_status_ = BT::NodeStatus::SUCCESS;
  single_step_mode_ = false;
  step_source_uid_ = 0;
  resume_bypassed_uid_ = paused_at_node_uid_;
  resume_requested_ = true;
  cv_breakpoint_.notify_all();
  RCLCPP_INFO(logger_, "[BTick] Execution resumed (forced SUCCESS): tree='%s'", tree_id_.c_str());
}

void TreeInstance::force_failure() {
  std::lock_guard<std::mutex> lock(mutex_breakpoint_);
  forced_return_status_ = BT::NodeStatus::FAILURE;
  single_step_mode_ = false;
  step_source_uid_ = 0;
  resume_bypassed_uid_ = paused_at_node_uid_;
  resume_requested_ = true;
  cv_breakpoint_.notify_all();
  RCLCPP_INFO(logger_, "[BTick] Execution resumed (forced FAILURE): tree='%s'", tree_id_.c_str());
}

void TreeInstance::step() {
  std::lock_guard<std::mutex> lock(mutex_breakpoint_);
  resume_bypassed_uid_ = 0;
  forced_return_status_ = BT::NodeStatus::IDLE;
  single_step_mode_ = true;
  step_source_uid_ = paused_at_node_uid_;
  step_requested_ = true;
  cv_breakpoint_.notify_all();
  RCLCPP_DEBUG(logger_, "[BTick] Single step requested: tree='%s'", tree_id_.c_str());
}

uint16_t TreeInstance::paused_at_node_uid() const {
  std::lock_guard<std::mutex> lock(mutex_breakpoint_);
  return paused_at_node_uid_;
}

void TreeInstance::set_fault_injection(uint16_t uid, const std::string &mode, int delay_ms) {
  std::lock_guard<std::mutex> lock(mutex_faults_);
  FaultInjectionRule rule;
  rule.mode = mode;
  rule.delay_ms = delay_ms;
  fault_rules_[uid] = rule;
  cv_fault_.notify_all();
  RCLCPP_INFO(logger_, "[BTick] Fault injected: tree='%s' mode='%s' (delay: %dms)",
              tree_id_.c_str(), mode.c_str(), delay_ms);
}

void TreeInstance::clear_fault_injection(uint16_t uid) {
  std::lock_guard<std::mutex> lock(mutex_faults_);
  fault_rules_.erase(uid);
  cv_fault_.notify_all();
  RCLCPP_INFO(logger_, "[BTick] Fault cleared: tree='%s'", tree_id_.c_str());
}

void TreeInstance::clear_all_faults() {
  std::lock_guard<std::mutex> lock(mutex_faults_);
  fault_rules_.clear();
  cv_fault_.notify_all();
  RCLCPP_INFO(logger_, "[BTick] All faults cleared: tree='%s'", tree_id_.c_str());
}

void TreeInstance::set_status(TreeStatus new_status) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.store(new_status);
    discovery_json_dirty_ = true;
  }
  { std::lock_guard<std::mutex> lock(mutex_faults_); }
  cv_fault_.notify_all();
  { std::lock_guard<std::mutex> lock(mutex_breakpoint_); }
  cv_breakpoint_.notify_all();
}

TreeInstance::ListenerHandle TreeInstance::add_event_listener(EventListener listener) {
  if (!listener) return 0;
  std::lock_guard<std::mutex> lock(mutex_);
  ListenerHandle handle = next_listener_handle_++;
  event_listeners_[handle] = std::move(listener);
  return handle;
}

bool TreeInstance::remove_event_listener(ListenerHandle handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  return event_listeners_.erase(handle) > 0;
}

void TreeInstance::set_event_callback(EventListener callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (legacy_callback_handle_ != 0) {
    event_listeners_.erase(legacy_callback_handle_);
    legacy_callback_handle_ = 0;
  }
  if (callback) {
    legacy_callback_handle_ = next_listener_handle_++;
    event_listeners_[legacy_callback_handle_] = std::move(callback);
  }
}

void TreeInstance::emit_event(const NodeStatusEvent &ev) {
  std::vector<EventListener> listeners_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    listeners_copy.reserve(event_listeners_.size());
    for (const auto &pair : event_listeners_) {
      if (pair.second) {
        listeners_copy.push_back(pair.second);
      }
    }
  }
  for (const auto &listener : listeners_copy) {
    listener(ev);
  }
}

void TreeInstance::callback(BT::Duration /*timestamp*/, const BT::TreeNode &node,
                            BT::NodeStatus prev_status, BT::NodeStatus status) {
  NodeStatusEvent ev;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::string prev_str = toStr(prev_status, false);
    std::string cur_str = toStr(status, false);

    if (prev_str == cur_str) {
      return;
    }

    double ts = current_time_seconds();
    uint16_t uid = node.UID();
    std::string node_id;

    // Strict UID-to-node_id lookup
    if (uid != 0 && model_.uid_to_id.count(uid)) {
      node_id = model_.uid_to_id[uid];
    } else if (model_.nodes.count(node.name())) {
      node_id = node.name();
    } else {
      node_id = node.name() + "_" + std::to_string(uid);
    }

    auto model_it = model_.nodes.find(node_id);
    if (cur_str == "RUNNING" && prev_str != "RUNNING") {
      start_times_[node_id] = ts;
      if (model_it != model_.nodes.end()) {
        ++model_it->second.tick_count;
      }
    } else if (prev_str == "RUNNING" && cur_str != "RUNNING") {
      const auto start = start_times_.find(node_id);
      if (start != start_times_.end()) {
        if (model_it != model_.nodes.end()) {
          model_it->second.last_duration = (ts - start->second) * 1000.0;
        }
        start_times_.erase(start);
      }
    }

    double last_duration = 0.0;
    uint32_t tick_count = 0;
    if (model_it != model_.nodes.end()) {
      // If cur_str is IDLE after SUCCESS or FAILURE (BT reset state), preserve
      // last executed result
      if (cur_str == "IDLE" &&
          (prev_str == "SUCCESS" || prev_str == "FAILURE" || model_it->second.status == "SUCCESS" ||
           model_it->second.status == "FAILURE")) {
        // Preserve last execution result (SUCCESS or FAILURE)
      } else {
        model_it->second.status = cur_str;
      }
      last_duration = model_it->second.last_duration;
      tick_count = model_it->second.tick_count;
    }

    ev.tree_id = tree_id_;
    ev.tree_name = tree_name_;
    ev.node_id = node_id;
    ev.node_name = node.name();
    ev.uid = uid;
    ev.prev_status = prev_str;
    ev.current_status = cur_str;
    ev.duration = last_duration;
    ev.tick_count = tick_count;
    ev.timestamp = ts;
    ev.blackboard_json = get_blackboard_json(node.config().blackboard.get());
    auto port_event = pending_port_event_json_.find(uid);
    if (port_event != pending_port_event_json_.end()) {
      ev.output_ports_json = std::move(port_event->second);
      pending_port_event_json_.erase(port_event);
    } else {
      ev.output_ports_json = "[]";
    }

    if (status_ == TreeStatus::CREATED) {
      status_ = TreeStatus::RUNNING;
    }
    discovery_json_dirty_ = true;

    if (uid != 0 && uid == resume_bypassed_uid_ &&
        (cur_str == "SUCCESS" || cur_str == "FAILURE" || cur_str == "IDLE")) {
      std::lock_guard<std::mutex> lk_bp(mutex_breakpoint_);
      resume_bypassed_uid_ = 0;
    }
  }

  // Invoke listeners outside the lock to prevent deadlock
  emit_event(ev);
}

void TreeInstance::flush() {}

std::string TreeInstance::get_blackboard_json(const BT::Blackboard *preferred_bb) const {
  if (blackboards_.empty() && !preferred_bb) {
    return "{}";
  }
  nlohmann::json values = nlohmann::json::object();
  std::unordered_set<std::string> seen_keys;

  std::vector<const BT::Blackboard *> bb_list;
  if (preferred_bb) {
    bb_list.push_back(preferred_bb);
  }
  // Reverse order traversal: most specific subtrees first, then root blackboard
  for (auto it = blackboards_.rbegin(); it != blackboards_.rend(); ++it) {
    if (it->get() && it->get() != preferred_bb) {
      bb_list.push_back(it->get());
    }
  }

  for (const auto *bb : bb_list) {
    if (!bb) continue;
    try {
      for (const auto &key_view : bb->getKeys()) {
        std::string key(key_view);
        if (seen_keys.count(key)) continue;
        seen_keys.insert(key);

        auto entry = bb->getEntry(key);
        if (!entry) continue;

        values[key] = "<unknown>";
        try {
          std::unique_lock<std::mutex> lk(entry->entry_mutex);
          if (entry->value.isString()) {
            if (auto res = entry->value.tryCast<std::string>()) {
              values[key] = res.value();
            }
          } else if (entry->value.isNumber()) {
            if (auto res = entry->value.tryCast<double>()) {
              values[key] = res.value();
            }
          } else if (entry->value.isType<bool>()) {
            if (auto res = entry->value.tryCast<bool>()) {
              values[key] = res.value();
            }
          } else {
            nlohmann::json exported_value;
            if (BT::JsonExporter::get().toJson(entry->value, exported_value)) {
              values[key] = std::move(exported_value);
            } else {
              values[key] = entry->info.typeName();
            }
          }
        } catch (const std::exception &error) {
          RCLCPP_WARN(logger_, "[BTick] Unable to serialize blackboard key '%s': %s", key.c_str(),
                      error.what());
          values[key] = "<opaque>";
        }
      }
    } catch (const std::exception &error) {
      RCLCPP_WARN(logger_, "[BTick] Unable to enumerate blackboard: %s", error.what());
    }
  }

  return values.dump();
}

std::string TreeInstance::to_json_tree_created() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!discovery_json_dirty_) {
    return discovery_json_cache_;
  }
  nlohmann::json result = tree_model_to_json(model_);
  result["tree_id"] = tree_id_;
  result["tree_name"] = tree_name_;
  result["display_name"] = display_name_;
  if (!node_name_.empty()) {
    result["node_name"] = node_name_;
  }
  result["pid"] = pid_;
  if (!hostname_.empty()) {
    result["hostname"] = hostname_;
  }
  if (!origin_.empty()) {
    result["origin"] = origin_;
  }
  result["xml_path"] = xml_path_;
  result["status"] = tree_status_to_string(status_);
  result["blackboard"] = nlohmann::json::parse(get_blackboard_json());
  discovery_json_cache_ = result.dump(2) + "\n";
  discovery_json_dirty_ = false;
  return discovery_json_cache_;
}

bool TreeInstance::set_blackboard_value(const std::string &key, const std::string &value_str,
                                        const std::string &type_str, std::string &error_msg,
                                        std::string &actual_value_json) {
  if (key.empty()) {
    error_msg = "Invalid key: Key cannot be empty";
    return false;
  }

  // 1. Locate the blackboard containing this key
  BT::Blackboard::Ptr target_bb = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &bb : blackboards_) {
      if (!bb) continue;
      auto entry = bb->getEntry(key);
      if (entry) {
        target_bb = bb;
        break;
      }
    }
    if (!target_bb) {
      error_msg = "Invalid key: Key '" + key + "' does not exist in monitored Blackboard";
      return false;
    }
  }

  // 2. Validate requested type
  std::string norm_type = type_str;
  for (auto &c : norm_type) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  try {
    if (norm_type == "bool" || norm_type == "boolean") {
      bool val = false;
      if (value_str == "true" || value_str == "1" || value_str == "True") {
        val = true;
      } else if (value_str == "false" || value_str == "0" || value_str == "False") {
        val = false;
      } else {
        error_msg = "Invalid boolean value: '" + value_str + "'. Expected 'true' or 'false'.";
        return false;
      }
      target_bb->set<bool>(key, val);
      actual_value_json = val ? "true" : "false";
    } else if (norm_type == "int" || norm_type == "integer") {
      if (value_str.empty()) {
        error_msg = "Invalid integer value: Value cannot be empty";
        return false;
      }
      int val = 0;
      const auto parsed =
          std::from_chars(value_str.data(), value_str.data() + value_str.size(), val);
      if (parsed.ec != std::errc{} || parsed.ptr != value_str.data() + value_str.size()) {
        error_msg = "Invalid integer value: '" + value_str + "'";
        return false;
      }
      target_bb->set<int>(key, val);
      actual_value_json = std::to_string(val);
    } else if (norm_type == "double" || norm_type == "float" || norm_type == "number") {
      if (value_str.empty()) {
        error_msg = "Invalid floating-point value: Value cannot be empty";
        return false;
      }
      double val = 0.0;
      if (!parse_double_value(value_str, val)) {
        error_msg = "Invalid floating-point value: '" + value_str + "'";
        return false;
      }
      if (!std::isfinite(val)) {
        error_msg = "Non-finite floating-point value: '" + value_str + "'";
        return false;
      }
      target_bb->set<double>(key, val);
      actual_value_json = format_json_number(val);
    } else if (norm_type == "string") {
      target_bb->set<std::string>(key, value_str);
      actual_value_json = nlohmann::json(value_str).dump();
    } else {
      error_msg =
          "Unsupported value type: '" + type_str + "'. Supported types: bool, int, double, string.";
      return false;
    }
  } catch (const std::exception &e) {
    error_msg = std::string("Type mismatch or write error: ") + e.what();
    return false;
  } catch (...) {
    error_msg = "Unknown error while writing to Blackboard";
    return false;
  }

  // 3. Trigger live Blackboard update broadcast
  {
    std::lock_guard<std::mutex> lock(mutex_);
    discovery_json_dirty_ = true;
  }
  NodeStatusEvent ev;
  ev.tree_id = tree_id_;
  ev.tree_name = tree_name_;
  ev.current_status = "BLACKBOARD_UPDATE";
  ev.timestamp = current_time_seconds();
  ev.blackboard_json = get_blackboard_json();
  emit_event(ev);

  return true;
}

void TreeInstance::apply_port_to_node(BT::TreeNode *n, const std::string &port_name,
                                      const std::string &value) {
  if (!n) return;
  auto accessor = static_cast<TreeNodeAccessor *>(n);
  accessor->config().input_ports[port_name] = value;
  accessor->modifyPortsRemapping({{port_name, value}});
}

bool TreeInstance::set_node_port_value(uint16_t uid, const std::string &port_name,
                                       const std::string &value_str, const std::string &mode_str,
                                       std::string &error_msg, std::string &actual_value,
                                       std::string &actual_mode) {
  if (port_name.empty()) {
    error_msg = "Invalid port: Port name cannot be empty";
    return false;
  }

  // Port direction and type come from the immutable construction-time
  // manifest snapshot; command handling must not inspect live node config.
  std::string node_id;
  BTPortDetail resolved_port;
  bool has_port_detail = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto node_id_it = model_.uid_to_id.find(uid);
    if (node_id_it != model_.uid_to_id.end()) {
      node_id = node_id_it->second;
      auto model_it = model_.nodes.find(node_id);
      if (model_it != model_.nodes.end()) {
        for (const auto &detail : model_it->second.port_details) {
          if (detail.name == port_name) {
            resolved_port = detail;
            has_port_detail = true;
            break;
          }
        }
      }
    }
  }
  if (node_id.empty()) {
    error_msg = "Node UID not found: " + std::to_string(uid);
    return false;
  }
  if (!has_port_detail) {
    error_msg = "Port '" + port_name +
                "' does not exist with a reliably known direction on node UID " +
                std::to_string(uid);
    return false;
  }
  if (resolved_port.direction == "Output") {
    error_msg = "Port '" + port_name + "' is an output port and is read-only";
    return false;
  }
  if (resolved_port.direction == "Unknown") {
    error_msg = "Port '" + port_name +
                "' does not exist with a reliably known direction on node UID " +
                std::to_string(uid);
    return false;
  }
  if (resolved_port.direction != "Input" && resolved_port.direction != "Inout") {
    error_msg = "Port '" + port_name + "' does not exist on node UID " + std::to_string(uid);
    return false;
  }
  const std::string port_type = resolved_port.type.empty() ? "string" : resolved_port.type;

  // 4. Normalize mode and value (Blackboard vs Literal)
  std::string norm_mode = mode_str;
  for (auto &c : norm_mode) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  bool is_blackboard = false;
  std::string final_value = value_str;

  if (norm_mode == "blackboard") {
    is_blackboard = true;
    if (final_value.empty() || final_value == "{}") {
      error_msg = "Invalid blackboard reference: Key cannot be empty";
      return false;
    }
    if (final_value.front() != '{' || final_value.back() != '}') {
      final_value = "{" + final_value + "}";
    }
  } else if (!value_str.empty() && value_str.front() == '{' && value_str.back() == '}') {
    is_blackboard = true;
    if (value_str.size() <= 2) {
      error_msg = "Invalid blackboard reference: Key cannot be empty";
      return false;
    }
  } else {
    is_blackboard = false;
  }

  // 5. Type validation for Literal mode
  if (!is_blackboard) {
    std::string norm_type = port_type;
    for (auto &c : norm_type) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if (norm_type == "double" || norm_type == "float" || norm_type == "number") {
      if (final_value.empty()) {
        error_msg = "Invalid double value: Value cannot be empty for port '" + port_name + "'";
        return false;
      }
      double d_val = 0.0;
      if (!parse_double_value(final_value, d_val)) {
        error_msg = "Invalid double value: '" + final_value + "' for port '" + port_name + "'";
        return false;
      }
      if (!std::isfinite(d_val)) {
        error_msg = "Non-finite double value: '" + final_value + "' for port '" + port_name + "'";
        return false;
      }
      final_value = format_json_number(d_val);
      if (final_value.find('.') == std::string::npos &&
          final_value.find('e') == std::string::npos) {
        final_value += ".0";
      }
    } else if (norm_type == "int" || norm_type == "integer" || norm_type == "int32_t" ||
               norm_type == "uint32_t" || norm_type == "int64_t" || norm_type == "uint64_t") {
      if (final_value.empty()) {
        error_msg = "Invalid integer value: Value cannot be empty for port '" + port_name + "'";
        return false;
      }
      if (norm_type == "uint32_t" || norm_type == "uint64_t") {
        uint64_t i_val = 0;
        const auto parsed =
            std::from_chars(final_value.data(), final_value.data() + final_value.size(), i_val);
        const uint64_t max_value = norm_type == "uint32_t" ? std::numeric_limits<uint32_t>::max()
                                                           : std::numeric_limits<uint64_t>::max();
        if (parsed.ec != std::errc{} || parsed.ptr != final_value.data() + final_value.size() ||
            i_val > max_value) {
          error_msg = "Invalid integer value: '" + final_value + "' for port '" + port_name + "'";
          return false;
        }
        final_value = std::to_string(i_val);
      } else {
        int64_t i_val = 0;
        const auto parsed =
            std::from_chars(final_value.data(), final_value.data() + final_value.size(), i_val);
        const bool outside_int_range =
            (norm_type == "int" || norm_type == "integer" || norm_type == "int32_t") &&
            (i_val < std::numeric_limits<int32_t>::min() ||
             i_val > std::numeric_limits<int32_t>::max());
        if (parsed.ec != std::errc{} || parsed.ptr != final_value.data() + final_value.size() ||
            outside_int_range) {
          error_msg = "Invalid integer value: '" + final_value + "' for port '" + port_name + "'";
          return false;
        }
        final_value = std::to_string(i_val);
      }
    } else if (norm_type == "bool" || norm_type == "boolean") {
      if (final_value == "true" || final_value == "1" || final_value == "True") {
        final_value = "true";
      } else if (final_value == "false" || final_value == "0" || final_value == "False") {
        final_value = "false";
      } else {
        error_msg = "Invalid boolean value: '" + final_value + "' for port '" + port_name +
                    "'. Expected 'true' or 'false'.";
        return false;
      }
    }
  }

  // Only the BT tick thread may modify the live node configuration.
  {
    std::lock_guard<std::mutex> lock(mutex_port_queue_);
    queued_port_updates_[uid].push_back({port_name, final_value});
  }

  // 7. Update internal BTTreeModel & port_details
  actual_value = final_value;
  actual_mode = is_blackboard ? "Blackboard" : "Literal";

  std::string node_name;
  std::string current_node_status = "IDLE";
  {
    std::lock_guard<std::mutex> lock(mutex_);
    discovery_json_dirty_ = true;
    if (!node_id.empty() && model_.nodes.count(node_id)) {
      auto &n_mod = model_.nodes.at(node_id);
      node_name = n_mod.name;
      if (!n_mod.status.empty()) {
        current_node_status = n_mod.status;
      }
      if (is_blackboard) {
        n_mod.ports[port_name] = final_value;
        n_mod.attributes.erase(port_name);
      } else {
        n_mod.attributes[port_name] = final_value;
        n_mod.ports.erase(port_name);
      }

      bool pd_updated = false;
      for (auto &pd : n_mod.port_details) {
        if (pd.name == port_name) {
          pd.value = final_value;
          pd.mode = actual_mode;
          pd_updated = true;
          break;
        }
      }
      if (!pd_updated) {
        BTPortDetail pd;
        pd.name = port_name;
        pd.direction = resolved_port.direction;
        pd.type = port_type;
        pd.mode = actual_mode;
        pd.value = final_value;
        n_mod.port_details.push_back(pd);
      }
    }
  }

  // 8. Emit NODE_PORT_UPDATE event through listener path
  std::string bb_key = "";
  if (is_blackboard) {
    if (final_value.size() >= 2 && final_value.front() == '{' && final_value.back() == '}') {
      bb_key = final_value.substr(1, final_value.size() - 2);
    } else {
      bb_key = final_value;
    }
  }

  nlohmann::json port_update_item = {{"port_name", port_name},
                                     {"value", final_value},
                                     {"mode", actual_mode},
                                     {"blackboard_key", bb_key},
                                     {"change_kind", "port_config"}};
  nlohmann::json output_ports_array = nlohmann::json::array();
  output_ports_array.push_back(std::move(port_update_item));

  NodeStatusEvent ev;
  ev.tree_id = tree_id_;
  ev.tree_name = tree_name_;
  ev.node_id = node_id;
  ev.node_name = node_name.empty() ? node_id : node_name;
  ev.uid = uid;
  ev.prev_status = current_node_status;
  ev.current_status = "NODE_PORT_UPDATE";
  ev.timestamp = current_time_seconds();
  ev.blackboard_json = get_blackboard_json();
  ev.output_ports_json = output_ports_array.dump();
  emit_event(ev);

  return true;
}

}  // namespace btick
