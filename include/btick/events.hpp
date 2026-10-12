#ifndef BTICK__EVENTS_HPP_
#define BTICK__EVENTS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace btick {

enum class TreeStatus { CREATED, RUNNING, DESTROYED };

inline std::string tree_status_to_string(TreeStatus status) {
  switch (status) {
    case TreeStatus::CREATED:
      return "CREATED";
    case TreeStatus::RUNNING:
      return "RUNNING";
    case TreeStatus::DESTROYED:
      return "DESTROYED";
    default:
      return "UNKNOWN";
  }
}

struct NodeStatusEvent {
  std::string tree_id;
  std::string tree_name;
  std::string node_id;
  std::string node_name;
  uint16_t uid{0};
  std::string prev_status;
  std::string current_status;
  double duration{0.0};
  uint32_t tick_count{0};
  double timestamp{0.0};
  std::string blackboard_json{"{}"};
  std::string output_ports_json{"[]"};
};

}  // namespace btick

#endif  // BTICK__EVENTS_HPP_
