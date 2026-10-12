#include "btick/tree_parser.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <iostream>
#include <limits>
#include <pugixml.hpp>
#include <set>
#include <sstream>

#include "btick/port_utils.hpp"
#include "btick/tree_serializer.hpp"

namespace btick {

static const std::set<std::string> CONTROL_NODES = {
    "Sequence", "ReactiveSequence", "SequenceWithMemory", "Fallback",   "ReactiveFallback",
    "Parallel", "PipelineSequence", "RecoveryNode",       "RoundRobin", "IfThenElse",
    "Switch2",  "Switch3"};

static const std::set<std::string> DECORATOR_NODES = {
    "RateController", "Inverter",     "RetryUntilSuccessful",    "Repeat",       "Timeout",
    "ForceSuccess",   "ForceFailure", "KeepRunningUntilFailure", "Precondition", "Delay"};

static const std::set<std::string> STANDARD_ACTION_NODES = {"ComputePathToPose",
                                                            "FollowPath",
                                                            "ClearEntireCostmap",
                                                            "Spin",
                                                            "Wait",
                                                            "BackUp",
                                                            "DriveOnHeading",
                                                            "AssistedTeleop",
                                                            "CancelControl",
                                                            "GloballyUpdatedGoal",
                                                            "Script",
                                                            "DockingReset",
                                                            "DockingRescue",
                                                            "UnDocking",
                                                            "DockingStage1",
                                                            "DockingStage2",
                                                            "DockingStage3",
                                                            "DockingCheckCharging"};

static const std::set<std::string> STANDARD_CONDITION_NODES = {"GoalReached",
                                                               "GoalUpdated",
                                                               "InitialPoseReceived",
                                                               "IsBatteryLow",
                                                               "IsPathValid",
                                                               "WouldAControllerRecoveryHelp",
                                                               "WouldAPlannerRecoveryHelp",
                                                               "TransformAvailable",
                                                               "IsStuck",
                                                               "ScriptCondition",
                                                               "DockingCommandCheck",
                                                               "DockingVerifyCharging",
                                                               "AlwaysFailure",
                                                               "AlwaysSuccess"};

std::string TreeParser::determine_category(const std::string &tag_name) {
  if (tag_name == "SubTree" || tag_name == "SubTreePlus" ||
      tag_name.find("SubTree") != std::string::npos)
    return "SubTree";
  if (CONTROL_NODES.count(tag_name)) return "Control";
  if (DECORATOR_NODES.count(tag_name)) return "Decorator";
  if (STANDARD_ACTION_NODES.count(tag_name)) return "Action";
  if (STANDARD_CONDITION_NODES.count(tag_name)) return "Condition";

  if (tag_name.find("Condition") != std::string::npos ||
      tag_name.find("Check") != std::string::npos || tag_name.find("Is") == 0 ||
      tag_name.find("Verify") != std::string::npos || tag_name.find("Would") != std::string::npos) {
    return "Condition";
  }
  return "Action";
}

static void parse_element_recursive(const pugi::xml_node &xml_elem, const std::string &parent_id,
                                    BTTreeModel &tree, size_t &node_counter,
                                    const std::unordered_map<std::string, pugi::xml_node> &bt_map,
                                    std::set<std::string> visited_subtrees) {
  std::string tag_name = xml_elem.name();
  if (tag_name.empty() || tag_name == "root" || tag_name == "BehaviorTree") {
    for (pugi::xml_node child = xml_elem.first_child(); child; child = child.next_sibling()) {
      if (child.type() == pugi::node_element) {
        parse_element_recursive(child, parent_id, tree, node_counter, bt_map, visited_subtrees);
      }
    }
    return;
  }

  node_counter++;
  std::string name_attr = xml_elem.attribute("name").value();
  std::string subtree_id = xml_elem.attribute("ID").value();
  if (subtree_id.empty()) {
    subtree_id = xml_elem.attribute("subtree_id").value();
  }

  if (name_attr.empty()) {
    if (!subtree_id.empty()) {
      name_attr = subtree_id;
    } else {
      name_attr = tag_name;
    }
  }

  std::string node_id = tag_name + "_" + std::to_string(node_counter);
  if (!name_attr.empty() && name_attr != tag_name) {
    node_id = name_attr + "_" + std::to_string(node_counter);
  }

  BTNodeModel node;
  node.id = node_id;
  node.name = name_attr;
  node.type = tag_name;
  node.category = TreeParser::determine_category(tag_name);
  node.parent_id = parent_id;
  node.status = "IDLE";
  node.uid = node_counter <= std::numeric_limits<uint16_t>::max()
                 ? static_cast<uint16_t>(node_counter)
                 : 0;

  if (!subtree_id.empty()) {
    uint32_t parsed_uid = 0;
    const auto parsed =
        std::from_chars(subtree_id.data(), subtree_id.data() + subtree_id.size(), parsed_uid);
    if (parsed.ec == std::errc{} && parsed.ptr == subtree_id.data() + subtree_id.size() &&
        parsed_uid > 0 && parsed_uid <= std::numeric_limits<uint16_t>::max()) {
      node.uid = static_cast<uint16_t>(parsed_uid);
    }
  }

  if (node.uid != 0) {
    tree.uid_to_id[node.uid] = node.id;
  }

  for (pugi::xml_attribute attr = xml_elem.first_attribute(); attr; attr = attr.next_attribute()) {
    std::string attr_name = attr.name();
    std::string attr_val = attr.value();

    if (attr_name == "name") continue;
    if (attr_name == "ID" || (!attr_name.empty() && attr_name.front() == '_')) {
      node.meta[attr_name] = attr_val;
      continue;
    }

    if (!attr_val.empty() && attr_val.front() == '{' && attr_val.back() == '}') {
      node.ports[attr_name] = attr_val;
    } else {
      node.attributes[attr_name] = attr_val;
    }

    BTPortDetail pd;
    pd.name = attr_name;
    pd.value = attr_val;
    pd.mode = (!attr_val.empty() && attr_val.front() == '{' && attr_val.back() == '}')
                  ? "Blackboard"
                  : "Literal";
    pd.direction = "Unknown";
    pd.type = infer_port_type(attr_val);
    node.port_details.push_back(pd);
  }

  if (parent_id.empty() && tree.root_node_id.empty()) {
    tree.root_node_id = node.id;
  } else if (!parent_id.empty() && tree.nodes.count(parent_id)) {
    tree.nodes[parent_id].children_ids.push_back(node.id);
  }

  tree.nodes[node.id] = node;

  bool is_subtree =
      (node.category == "SubTree" || tag_name == "SubTree" || tag_name == "SubTreePlus");
  std::string target_bt_id = !subtree_id.empty() ? subtree_id : name_attr;

  if (is_subtree && !target_bt_id.empty() && bt_map.count(target_bt_id)) {
    if (visited_subtrees.count(target_bt_id) == 0) {
      visited_subtrees.insert(target_bt_id);
      pugi::xml_node target_bt = bt_map.at(target_bt_id);
      for (pugi::xml_node child = target_bt.first_child(); child; child = child.next_sibling()) {
        if (child.type() == pugi::node_element) {
          parse_element_recursive(child, node.id, tree, node_counter, bt_map, visited_subtrees);
        }
      }
    }
  } else {
    for (pugi::xml_node child = xml_elem.first_child(); child; child = child.next_sibling()) {
      if (child.type() == pugi::node_element) {
        parse_element_recursive(child, node.id, tree, node_counter, bt_map, visited_subtrees);
      }
    }
  }
}

BTTreeModel TreeParser::parse_xml_string(const std::string &xml_content) {
  BTTreeModel tree;
  tree.raw_xml = xml_content;

  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_string(xml_content.c_str());
  if (!result) {
    std::cerr << "[BTick] TreeParser: Failed to parse XML string: " << result.description() << "\n";
    return tree;
  }

  pugi::xml_node root = doc.child("root");
  if (!root) {
    std::cerr << "[BTick] TreeParser: XML root node not found\n";
    return tree;
  }

  std::string main_tree = root.attribute("main_tree_to_execute").value();
  std::unordered_map<std::string, pugi::xml_node> bt_map;
  std::vector<std::string> bt_order;

  for (pugi::xml_node bt = root.child("BehaviorTree"); bt; bt = bt.next_sibling("BehaviorTree")) {
    std::string id = bt.attribute("ID").value();
    if (!id.empty()) {
      bt_map[id] = bt;
      bt_order.push_back(id);
    }
  }

  pugi::xml_node target_bt;
  if (!main_tree.empty() && bt_map.count(main_tree)) {
    target_bt = bt_map[main_tree];
    tree.tree_id = main_tree;
  } else if (!bt_order.empty()) {
    target_bt = bt_map[bt_order.front()];
    tree.tree_id = bt_order.front();
  } else {
    target_bt = root;
    tree.tree_id = "MainTree";
  }

  size_t node_counter = 0;
  std::set<std::string> visited_subtrees;
  parse_element_recursive(target_bt, "", tree, node_counter, bt_map, visited_subtrees);

  std::unordered_map<std::string, size_t> name_counts;
  for (const auto &entry : tree.nodes) {
    if (!entry.second.name.empty()) {
      ++name_counts[entry.second.name];
    }
  }
  for (const auto &entry : tree.nodes) {
    if (!entry.second.name.empty() && name_counts[entry.second.name] == 1) {
      tree.name_to_id[entry.second.name] = entry.first;
    }
  }

  return tree;
}

BTTreeModel TreeParser::parse_xml_file(const std::string &file_path) {
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(file_path.c_str());
  if (!result) {
    std::cerr << "[BTick] TreeParser: Failed to parse XML file: " << file_path << " - "
              << result.description() << "\n";
    return BTTreeModel();
  }

  std::stringstream ss;
  doc.save(ss);
  return parse_xml_string(ss.str());
}

std::string TreeParser::to_json(const BTTreeModel &tree) {
  return tree_model_to_json(tree).dump(2) + "\n";
}

}  // namespace btick
