#ifndef BTICK__PORT_UTILS_HPP_
#define BTICK__PORT_UTILS_HPP_

#include <string>

namespace btick {

inline std::string infer_port_type(const std::string &value) {
  if (value == "true" || value == "false") {
    return "bool";
  }
  if (value.find('.') != std::string::npos &&
      value.find_first_not_of("0123456789.-") == std::string::npos && value.size() > 1) {
    return "double";
  }
  if (!value.empty() && value.find_first_not_of("0123456789-") == std::string::npos) {
    return "int";
  }
  return "string";
}

}  // namespace btick

#endif  // BTICK__PORT_UTILS_HPP_
