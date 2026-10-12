#ifndef BTICK__JSON_UTILS_HPP_
#define BTICK__JSON_UTILS_HPP_

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

namespace btick {

inline std::string escape_json(const std::string &value) {
  std::ostringstream escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        escaped << "\\\"";
        break;
      case '\\':
        escaped << "\\\\";
        break;
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (character < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<unsigned int>(character);
        } else {
          escaped << static_cast<char>(character);
        }
    }
  }
  return escaped.str();
}

inline std::string format_json_number(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream formatted;
  formatted.imbue(std::locale::classic());
  formatted << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return formatted.str();
}

}  // namespace btick

#endif  // BTICK__JSON_UTILS_HPP_
