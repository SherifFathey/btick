#ifndef BTICK__TREE_PARSER_HPP_
#define BTICK__TREE_PARSER_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace btick {

struct BTPortDetail {
  std::string name;
  std::string direction{"Unknown"};  // "Input", "Output", "Inout", "Unknown"
  std::string type{"string"};        // "double", "int", "string", "bool", etc.
  std::string mode{"Literal"};       // "Literal", "Blackboard"
  std::string value;
  std::string default_value;
  std::string description;
  std::string runtime_value;
};

struct BTNodeModel {
  std::string id;         // Unique identifier (e.g. "FollowPath_12" or "node_1")
  uint16_t uid{0};        // BT.CPP numerical UID if available
  std::string name;       // Node name (e.g. "FollowPath")
  std::string type;       // XML tag (e.g. "Sequence", "ComputePathToPose", "BackUp")
  std::string category;   // "Control", "Decorator", "Action", "Condition",
                          // "SubTree", "Unknown"
  std::string parent_id;  // Parent node ID
  std::vector<std::string> children_ids;     // Children node IDs
  std::map<std::string, std::string> ports;  // In/Out ports ({path}, {goal}, etc.)
  std::map<std::string, std::string>
      attributes;                           // Raw XML attributes (e.g., number_of_retries="6")
  std::map<std::string, std::string> meta;  // BT.CPP metadata attributes
  std::vector<BTPortDetail> port_details;   // Rich runtime manifest port details
  std::map<std::string, std::string> output_ports_values;  // Per-node produced output port values
  std::string status{"IDLE"};                              // Current runtime status
  double last_duration{0.0};
  uint32_t tick_count{0};
};

struct BTTreeModel {
  std::string tree_id;       // Main tree ID (e.g. "MainTree" or "NavigateToPose")
  std::string tree_name;     // Human-friendly tree name
  std::string root_node_id;  // ID of root node
  std::unordered_map<std::string, BTNodeModel> nodes;  // Map from ID to BTNodeModel
  std::map<std::string, std::string> name_to_id;       // Map from node name to ID (fallback lookup)
  std::map<uint16_t, std::string> uid_to_id;           // Map from numerical UID to ID
  std::string raw_xml;
};

class TreeParser {
 public:
  TreeParser() = default;
  ~TreeParser() = default;

  static BTTreeModel parse_xml_string(const std::string &xml_content);
  static BTTreeModel parse_xml_file(const std::string &file_path);
  static std::string to_json(const BTTreeModel &tree);
  static std::string determine_category(const std::string &tag_name);
};

}  // namespace btick

#endif  // BTICK__TREE_PARSER_HPP_
