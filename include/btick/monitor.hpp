#ifndef BTICK__MONITOR_HPP_
#define BTICK__MONITOR_HPP_

#include <behaviortree_cpp/bt_factory.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "btick/tree_instance.hpp"

namespace btick {

/**
 * @brief Lightweight BehaviorTree Monitor client for ROS 2.
 *
 * Usage in any BT node:
 *   #include "btick/btick.hpp"
 *   ...
 *   btick::Monitor monitor(node, tree, "TurtleTree");
 *
 * Automatically hooks into all nodes in your tree and publishes status changes
 * to /btick/tree_transitions, announces tree structure to /btick/tree_discovery,
 * and processes commands from /btick/command_in replying to /btick/command_out.
 *
 * Commands are routed exclusively by the full unique `tree_id`. When multiple
 * trees or monitors exist across the ROS 2 graph, each monitor selectively handles
 * only commands matching its owned trees and silently ignores non-matching commands.
 *
 * The BT::Tree is non-owning and must outlive the Monitor. Stop/join any thread
 * currently ticking the tree before removing or destroying the Monitor.
 */
class Monitor {
 public:
  Monitor() = default;

  // Constructor accepting raw pointer (safe to call with `this` inside node
  // constructor)
  Monitor(rclcpp::Node *node, const BT::Tree &tree, const std::string &tree_name = "MainTree",
          const std::string &xml_path = "",
          const std::string &discovery_topic = "/btick/tree_discovery",
          const std::string &transitions_topic = "/btick/tree_transitions");

  // Constructor accepting SharedPtr
  Monitor(rclcpp::Node::SharedPtr node, const BT::Tree &tree,
          const std::string &tree_name = "MainTree", const std::string &xml_path = "",
          const std::string &discovery_topic = "/btick/tree_discovery",
          const std::string &transitions_topic = "/btick/tree_transitions");

  ~Monitor();

  // Prevent copy
  Monitor(const Monitor &) = delete;
  Monitor &operator=(const Monitor &) = delete;

  Monitor(Monitor &&) = delete;
  Monitor &operator=(Monitor &&) = delete;

  // Add a tree to this monitor
  void add_tree(rclcpp::Node *node, const BT::Tree &tree, const std::string &tree_name = "MainTree",
                const std::string &xml_path = "",
                const std::string &discovery_topic = "/btick/tree_discovery",
                const std::string &transitions_topic = "/btick/tree_transitions");

  void add_tree(rclcpp::Node::SharedPtr node, const BT::Tree &tree,
                const std::string &tree_name = "MainTree", const std::string &xml_path = "",
                const std::string &discovery_topic = "/btick/tree_discovery",
                const std::string &transitions_topic = "/btick/tree_transitions");

  // Remove a specific tree from this monitor
  bool remove_tree(const std::string &tree_id);

  // Event listener management for logging / external observers
  using EventListener = std::function<void(const NodeStatusEvent &)>;
  using ListenerHandle = uint64_t;

  ListenerHandle add_event_listener(const std::string &tree_id, EventListener listener);
  ListenerHandle add_event_listener(EventListener listener);
  bool remove_event_listener(const std::string &tree_id, ListenerHandle handle);
  bool remove_event_listener(ListenerHandle handle);

  // Returns full tree_created discovery JSON (matching /btick/tree_discovery output)
  std::string get_tree_created_json(const std::string &tree_id = "") const;

  // Access underlying TreeInstance for listeners / logging
  std::shared_ptr<TreeInstance> get_tree_instance(const std::string &tree_id = "") const;

  std::string tree_id() const;
  std::string tree_name() const;

  // Detach all trees and destroy command executor
  void detach();

 private:
  void init(rclcpp::Node *node, const BT::Tree &tree, const std::string &tree_name,
            const std::string &xml_path, const std::string &discovery_topic,
            const std::string &transitions_topic);

  rclcpp::Logger logger_{rclcpp::get_logger("BTick")};
  std::unordered_map<std::string, std::shared_ptr<TreeInstance>> instances_;
  mutable std::mutex instances_mutex_;
  std::shared_ptr<TreeInstance> instance_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr discovery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr transition_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_response_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  // Background dedicated executor for command subscriber (independent of user
  // main thread)
  rclcpp::Node::SharedPtr cmd_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> cmd_executor_;
  std::thread cmd_thread_;

  std::atomic<bool> detached_{false};
};

using BTick = Monitor;
using TreeObserver = Monitor;

}  // namespace btick

#endif  // BTICK__MONITOR_HPP_
