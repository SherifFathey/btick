#ifndef BTICK__TREE_SERIALIZER_HPP_
#define BTICK__TREE_SERIALIZER_HPP_

#include <nlohmann/json.hpp>

#include "btick/events.hpp"
#include "btick/tree_parser.hpp"

namespace btick {

inline nlohmann::json tree_model_to_json(const BTTreeModel &tree) {
  nlohmann::json result = {
      {"type", "tree_created"},
      {"tree_id", tree.tree_id.empty() ? "MainTree" : tree.tree_id},
      {"tree_name", tree.tree_name.empty() ? (tree.tree_id.empty() ? "MainTree" : tree.tree_id)
                                           : tree.tree_name},
      {"root_node_id", tree.root_node_id},
      {"status", "IDLE"},
      {"nodes", nlohmann::json::array()}};

  for (const auto &entry : tree.nodes) {
    const auto &node = entry.second;
    nlohmann::json ports = nlohmann::json::object();
    for (const auto &port : node.ports) {
      ports[port.first] = port.second;
    }
    nlohmann::json attributes = nlohmann::json::object();
    for (const auto &attribute : node.attributes) {
      attributes[attribute.first] = attribute.second;
    }
    nlohmann::json meta = nlohmann::json::object();
    for (const auto &attribute : node.meta) {
      meta[attribute.first] = attribute.second;
    }
    nlohmann::json output_ports = nlohmann::json::object();
    for (const auto &port : node.output_ports_values) {
      output_ports[port.first] = port.second;
    }

    nlohmann::json port_details = nlohmann::json::array();
    for (const auto &detail : node.port_details) {
      port_details.push_back({{"name", detail.name},
                              {"type", detail.type},
                              {"direction", detail.direction},
                              {"mode", detail.mode},
                              {"value", detail.value},
                              {"default_value", detail.default_value},
                              {"runtime_value", detail.runtime_value},
                              {"description", detail.description}});
    }

    result["nodes"].push_back({{"id", node.id},
                               {"uid", node.uid},
                               {"name", node.name},
                               {"type", node.type},
                               {"category", node.category},
                               {"parent_id", node.parent_id},
                               {"status", node.status.empty() ? "IDLE" : node.status},
                               {"children_ids", node.children_ids},
                               {"ports", std::move(ports)},
                               {"attributes", std::move(attributes)},
                               {"meta", std::move(meta)},
                               {"port_details", std::move(port_details)},
                               {"output_ports_values", std::move(output_ports)}});
  }

  return result;
}

inline nlohmann::json node_status_event_to_json(const NodeStatusEvent &ev, uint64_t seq_id = 0) {
  nlohmann::json bb_json;
  try {
    bb_json = nlohmann::json::parse(ev.blackboard_json.empty() ? "{}" : ev.blackboard_json);
  } catch (...) {
    bb_json = nlohmann::json::object();
  }

  nlohmann::json out_json;
  try {
    out_json = nlohmann::json::parse(ev.output_ports_json.empty() ? "[]" : ev.output_ports_json);
  } catch (...) {
    out_json = nlohmann::json::array();
  }

  nlohmann::json transition = {{"type", "transition"},
                               {"tree_id", ev.tree_id},
                               {"tree_name", ev.tree_name},
                               {"node_id", ev.node_id},
                               {"node_name", ev.node_name},
                               {"uid", ev.uid},
                               {"prev_status", ev.prev_status},
                               {"status", ev.current_status},
                               {"duration", ev.duration},
                               {"tick_count", ev.tick_count},
                               {"timestamp", ev.timestamp},
                               {"blackboard", std::move(bb_json)},
                               {"output_ports", std::move(out_json)}};

  if (seq_id > 0) {
    transition["id"] = "evt_" + std::to_string(seq_id);
    transition["source"] = "btick";
    transition["protocol"] = "btick";
  }

  return transition;
}

inline nlohmann::json to_json_transition(const NodeStatusEvent &ev, uint64_t seq_id = 0) {
  return node_status_event_to_json(ev, seq_id);
}

}  // namespace btick

#endif  // BTICK__TREE_SERIALIZER_HPP_
