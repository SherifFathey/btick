#ifndef BTICK__TREE_INSTANCE_HPP_
#define BTICK__TREE_INSTANCE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <atomic>
#include <behaviortree_cpp/scripting/script_parser.hpp>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "btick/events.hpp"
#include "btick/tree_parser.hpp"

namespace btick {

class Monitor;

struct FaultInjectionRule {
  std::string mode;  // "FORCE_FAILURE", "FORCE_SUCCESS", "FORCE_RUNNING",
                     // "FORCE_SKIPPED", "DELAY"
  int delay_ms{0};
};

class TreeInstance : public BT::StatusChangeLogger {
 public:
  TreeInstance(const BT::Tree &tree, const std::string &tree_name, const std::string &xml_path = "",
               const std::string &tree_id = "", const std::string &display_name = "",
               const std::string &origin = "", const std::string &node_name = "",
               const rclcpp::Logger &logger = rclcpp::get_logger("BTick"));

  ~TreeInstance() override;

  const std::string &tree_id() const { return tree_id_; }
  const std::string &tree_name() const { return tree_name_; }
  const std::string &display_name() const { return display_name_; }
  const std::string &origin() const { return origin_; }
  const std::string &xml_path() const { return xml_path_; }
  const std::string &node_name() const { return node_name_; }
  int pid() const { return pid_; }
  const std::string &hostname() const { return hostname_; }
  TreeStatus status() const { return status_.load(); }
  const BTTreeModel &tree_model() const { return model_; }

  void set_status(TreeStatus new_status);

  // StatusChangeLogger callback override
  void callback(BT::Duration timestamp, const BT::TreeNode &node, BT::NodeStatus prev_status,
                BT::NodeStatus status) override;

  void flush() override;

  std::string to_json_tree_created() const;
  std::string get_blackboard_json(const BT::Blackboard *preferred_bb = nullptr) const;

  using EventListener = std::function<void(const NodeStatusEvent &)>;
  using ListenerHandle = uint64_t;

  ListenerHandle add_event_listener(EventListener listener);
  bool remove_event_listener(ListenerHandle handle);
  void set_event_callback(EventListener callback);
  void emit_event(const NodeStatusEvent &ev);

  // Breakpoint management & Interactive Decision Points
  void set_breakpoint(uint16_t uid, bool enabled);
  void clear_all_breakpoints();
  void resume();
  void force_success();
  void force_failure();
  void step();
  uint16_t paused_at_node_uid() const;

  // Fault Injection & Substitution management
  void set_fault_injection(uint16_t uid, const std::string &mode, int delay_ms = 0);
  void clear_fault_injection(uint16_t uid);
  void clear_all_faults();

  // Live Blackboard write & inspection
  bool set_blackboard_value(const std::string &key, const std::string &value_str,
                            const std::string &type_str, std::string &error_msg,
                            std::string &actual_value_json);

  // Live Node Port Editing
  // Updates are reported immediately but applied to BT::Tree on its next tick.
  bool set_node_port_value(uint16_t uid, const std::string &port_name, const std::string &value_str,
                           const std::string &mode_str, std::string &error_msg,
                           std::string &actual_value, std::string &actual_mode);

 private:
  class HookExecutionGuard {
   public:
    explicit HookExecutionGuard(TreeInstance &owner);
    ~HookExecutionGuard();

    HookExecutionGuard(const HookExecutionGuard &) = delete;
    HookExecutionGuard &operator=(const HookExecutionGuard &) = delete;

   private:
    TreeInstance &owner_;
  };

  void wait_for_hooks();
  BTPortDetail resolve_port(const BT::TreeNode &node, const std::string &port_name,
                            const BTPortDetail *xml_port = nullptr) const;
  void apply_port_to_node(BT::TreeNode *n, const std::string &port_name, const std::string &value);
  // BT.CPP exposes hook setters but no getters; existing pre/post tick hooks
  // cannot be chained or restored, so callers must not install competing hooks.
  void register_hooks_recursive(BT::TreeNode *n);
  void clear_hooks_recursive(BT::TreeNode *n);

  BT::TreeNode *root_node_{nullptr};
  std::string tree_id_;
  std::string tree_name_;
  std::string display_name_;
  std::string origin_;
  std::string xml_path_;
  std::string node_name_;
  int pid_{0};
  std::string hostname_;
  std::atomic<TreeStatus> status_{TreeStatus::CREATED};
  rclcpp::Logger logger_;
  BTTreeModel model_;
  mutable std::string discovery_json_cache_;
  mutable bool discovery_json_dirty_{true};
  BT::Blackboard::Ptr root_bb_;
  std::vector<BT::Blackboard::Ptr> blackboards_;

  enum class PreDecision : uint8_t { NONE = 0, FORCED_FAILURE, FORCED_SUCCESS, FORCED_SKIPPED };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, double> start_times_;
  mutable std::mutex mutex_original_scripts_;
  std::unordered_map<uint16_t, BT::ScriptFunction> original_failure_if_scripts_;
  std::unordered_map<uint16_t, BT::ScriptFunction> original_success_if_scripts_;
  std::unordered_map<uint16_t, BT::ScriptFunction> original_skip_if_scripts_;
  std::unordered_map<uint16_t, BT::ScriptFunction> original_while_true_scripts_;
  std::unordered_map<uint16_t, BT::ScriptFunction> original_always_post_scripts_;
  mutable std::mutex mutex_pre_decision_;
  std::unordered_map<uint16_t, PreDecision> pending_pre_decision_;
  std::unordered_map<ListenerHandle, EventListener> event_listeners_;
  ListenerHandle next_listener_handle_{1};
  ListenerHandle legacy_callback_handle_{0};
  std::unordered_map<uint16_t, std::string> pending_port_event_json_;

  // Breakpoints State
  mutable std::mutex mutex_breakpoint_;
  std::condition_variable cv_breakpoint_;
  std::unordered_set<uint16_t> breakpoints_;
  bool resume_requested_{false};
  bool step_requested_{false};
  BT::NodeStatus forced_return_status_{BT::NodeStatus::IDLE};
  bool single_step_mode_{false};
  uint16_t step_source_uid_{0};
  uint16_t paused_at_node_uid_{0};
  uint16_t resume_bypassed_uid_{0};

  // Fault Injection Rules
  mutable std::mutex mutex_faults_;
  std::condition_variable cv_fault_;
  std::unordered_map<uint16_t, FaultInjectionRule> fault_rules_;

  // Live Node Port Mapping & Thread-Safe Queued Updates
  std::unordered_map<uint16_t, BT::TreeNode *> uid_to_node_;
  struct QueuedPortUpdate {
    std::string port_name;
    std::string value;
  };
  mutable std::mutex mutex_port_queue_;
  std::unordered_map<uint16_t, std::vector<QueuedPortUpdate>> queued_port_updates_;
  mutable std::mutex mutex_hooks_;
  std::condition_variable cv_hooks_;
  size_t active_hooks_{0};
};

}  // namespace btick

#endif  // BTICK__TREE_INSTANCE_HPP_
