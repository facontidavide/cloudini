#pragma once

#include <cctype>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace Cloudini {
namespace YAML {

class Node;

using Sequence = std::vector<Node>;
using Mapping = std::map<std::string, Node, std::less<>>;  // std::less<> for heterogeneous lookup
using NodeValue = std::variant<std::monostate, std::string, Sequence, Mapping>;

/**
 * @brief Simple YAML node representation
 *
 * Supports scalars, sequences (lists), and mappings (dictionaries).
 * Provides accessors for reading values with type conversion.
 */
class Node {
 public:
  Node() : value_(std::monostate{}) {}
  Node(std::string_view value) : value_(std::string(value)) {}
  Node(Sequence seq) : value_(std::move(seq)) {}
  Node(Mapping map) : value_(std::move(map)) {}

  // Type checking
  bool isNull() const {
    return std::holds_alternative<std::monostate>(value_);
  }
  bool isString() const {
    return std::holds_alternative<std::string>(value_);
  }
  bool isSequence() const {
    return std::holds_alternative<Sequence>(value_);
  }

  // String conversions
  template <typename T>
  T as() const {
    if (!isString()) {
      throw std::runtime_error("Node is not a string");
    }
    const auto& str = std::get<std::string>(value_);
    return parseScalar<T>(str);
  }

  // Map access (supports std::string_view via heterogeneous lookup)
  const Node& operator[](std::string_view key) const {
    if (!std::holds_alternative<Mapping>(value_)) {
      throw std::runtime_error("Node is not a mapping");
    }
    const auto& map = std::get<Mapping>(value_);
    auto it = map.find(key);
    if (it == map.end()) {
      static Node null_node;
      return null_node;
    }
    return it->second;
  }

  // Sequence access
  const Node& operator[](size_t index) const {
    if (!isSequence()) {
      throw std::runtime_error("Node is not a sequence");
    }
    const auto& seq = std::get<Sequence>(value_);
    if (index >= seq.size()) {
      throw std::out_of_range("Sequence index out of range");
    }
    return seq[index];
  }

  // Get size for sequences
  size_t size() const {
    if (isSequence()) {
      return std::get<Sequence>(value_).size();
    }
    return 0;
  }

 private:
  NodeValue value_;

  template <typename T>
  static T parseScalar(const std::string& str) {
    std::istringstream iss(str);
    T value;
    iss >> value;
    if (iss.fail()) {
      throw std::runtime_error("Failed to convert scalar: " + str);
    }
    return value;
  }
};

// Specialization for std::string
template <>
inline std::string Node::as<std::string>() const {
  if (!isString()) {
    throw std::runtime_error("Node is not a string");
  }
  return std::get<std::string>(value_);
}

// Specialization for std::string_view (zero-copy)
template <>
inline std::string_view Node::as<std::string_view>() const {
  if (!isString()) {
    throw std::runtime_error("Node is not a string");
  }
  return std::get<std::string>(value_);
}

// Specialization for bool
template <>
inline bool Node::as<bool>() const {
  if (!isString()) {
    throw std::runtime_error("Node is not a string");
  }
  const auto& str = std::get<std::string>(value_);
  if (str == "true" || str == "True" || str == "TRUE" || str == "yes" || str == "Yes" || str == "YES") {
    return true;
  }
  if (str == "false" || str == "False" || str == "FALSE" || str == "no" || str == "No" || str == "NO") {
    return false;
  }
  throw std::runtime_error("Invalid boolean value: " + str);
}

/**
 * @brief Simple YAML parser
 *
 * Supports:
 * - Scalars (strings, numbers, booleans)
 * - Sequences (lists with - prefix)
 * - Nested structures
 * - Comments (# to end of line)
 *
 * Limitations:
 * - No support for mappings (key: value pairs)
 * - No support for anchors, aliases, or tags
 * - No support for multi-line strings with | or >
 * - No support for flow style ([], {})
 * - Basic indentation-based parsing
 */
class Parser {
 public:
  static Node parse(std::string_view content) {
    Parser parser(content);
    return parser.parseDocument();
  }

 private:
  std::string content_;  // Keep original content alive for string_view references
  std::vector<std::string_view> lines_;
  size_t current_line_ = 0;

  Parser(std::string_view content) : content_(content) {
    splitLines();
  }

  void splitLines() {
    std::string_view remaining = content_;
    while (!remaining.empty()) {
      size_t pos = remaining.find('\n');
      if (pos == std::string_view::npos) {
        lines_.push_back(remaining);
        break;
      }
      lines_.push_back(remaining.substr(0, pos));
      remaining.remove_prefix(pos + 1);
    }
  }

  Node parseDocument() {
    if (lines_.empty()) {
      return Node();
    }
    return parseNode(0);
  }

  Node parseNode(int base_indent) {
    skipEmptyLines();
    if (current_line_ >= lines_.size()) {
      return Node();
    }

    std::string_view line = lines_[current_line_];
    int indent = getIndentation(line);

    if (indent < base_indent) {
      return Node();
    }

    std::string_view trimmed = trim(line);

    // Check for sequence item
    if (trimmed.starts_with("- ")) {
      return parseSequence(base_indent);
    }

    // Check for mapping
    size_t colon_pos = trimmed.find(':');
    if (colon_pos != std::string_view::npos) {
      return parseMapping(base_indent);
    }

    // Scalar value
    return Node(trimmed);
  }

  Sequence parseSequence(int base_indent) {
    Sequence seq;

    while (current_line_ < lines_.size()) {
      skipEmptyLines();
      if (current_line_ >= lines_.size()) {
        break;
      }

      std::string_view line = lines_[current_line_];
      int indent = getIndentation(line);

      if (indent < base_indent) {
        break;
      }

      std::string_view trimmed = trim(line);
      if (!trimmed.starts_with("- ")) {
        break;
      }

      // Remove "- " prefix
      std::string_view item_value = trimmed.substr(2);

      // Check if it's an inline value or needs nested parsing
      if (item_value.empty()) {
        // Next line(s) contain the value
        current_line_++;
        seq.push_back(parseNode(indent + 2));
      } else {
        // Check if the inline value is actually a mapping (contains ':')
        size_t colon_pos = item_value.find(':');
        if (colon_pos != std::string_view::npos) {
          // This is actually a mapping like "- name: value"
          // We need to parse this and any following indented lines as a mapping
          Mapping item_map;

          // Parse the first key-value pair
          std::string_view key = trim(item_value.substr(0, colon_pos));
          std::string_view value_str = trim(item_value.substr(colon_pos + 1));
          if (!value_str.empty()) {
            item_map[std::string(key)] = Node(value_str);
          }

          current_line_++;

          // Check for additional keys at a more indented level
          while (current_line_ < lines_.size()) {
            skipEmptyLines();
            if (current_line_ >= lines_.size()) {
              break;
            }

            int next_indent = getIndentation(lines_[current_line_]);
            if (next_indent <= indent) {
              break;
            }

            std::string_view next_line = lines_[current_line_];
            std::string_view next_trimmed = trim(next_line);

            // Check if it's another mapping key
            size_t next_colon = next_trimmed.find(':');
            if (next_colon != std::string_view::npos) {
              std::string_view next_key = trim(next_trimmed.substr(0, next_colon));
              std::string_view next_value = trim(next_trimmed.substr(next_colon + 1));
              if (!next_value.empty()) {
                item_map[std::string(next_key)] = Node(next_value);
              } else {
                // Value on next line
                current_line_++;
                item_map[std::string(next_key)] = parseNode(next_indent + 2);
                continue;
              }
            } else {
              break;
            }

            current_line_++;
          }

          seq.push_back(Node(std::move(item_map)));
        } else {
          // Plain scalar value
          current_line_++;
          seq.push_back(Node(item_value));
        }
      }
    }

    return seq;
  }

  Mapping parseMapping(int base_indent) {
    Mapping map;

    while (current_line_ < lines_.size()) {
      skipEmptyLines();
      if (current_line_ >= lines_.size()) {
        break;
      }

      std::string_view line = lines_[current_line_];
      int indent = getIndentation(line);

      if (indent < base_indent) {
        break;
      }

      if (indent > base_indent) {
        break;
      }

      std::string_view trimmed = trim(line);

      // Check if this is still a mapping line
      size_t colon_pos = trimmed.find(':');
      if (colon_pos == std::string_view::npos) {
        break;
      }

      std::string_view key = trim(trimmed.substr(0, colon_pos));
      std::string_view value_str = trim(trimmed.substr(colon_pos + 1));

      current_line_++;

      if (value_str.empty()) {
        // Value is on next line(s)
        map[std::string(key)] = parseNode(indent + 2);
      } else {
        // Inline value
        map[std::string(key)] = Node(value_str);
      }
    }

    return map;
  }

  void skipEmptyLines() {
    while (current_line_ < lines_.size()) {
      std::string_view line = trim(lines_[current_line_]);
      if (line.empty() || line.starts_with('#')) {
        current_line_++;
      } else {
        break;
      }
    }
  }

  static int getIndentation(std::string_view line) {
    int indent = 0;
    for (char c : line) {
      if (c == ' ') {
        indent++;
      } else if (c == '\t') {
        indent += 4;  // Treat tab as 4 spaces
      } else {
        break;
      }
    }
    return indent;
  }

  static std::string_view trim(std::string_view str) {
    // Trim leading whitespace
    size_t start = 0;
    while (start < str.length() && std::isspace(static_cast<unsigned char>(str[start]))) {
      start++;
    }

    if (start == str.length()) {
      return std::string_view();
    }

    str.remove_prefix(start);

    // Trim trailing whitespace
    size_t end = str.length();
    while (end > 0 && std::isspace(static_cast<unsigned char>(str[end - 1]))) {
      end--;
    }
    str.remove_suffix(str.length() - end);

    // Remove trailing comments
    size_t comment_pos = str.find('#');
    if (comment_pos != std::string_view::npos) {
      str = str.substr(0, comment_pos);

      // Trim trailing whitespace after removing comment
      while (!str.empty() && std::isspace(static_cast<unsigned char>(str.back()))) {
        str.remove_suffix(1);
      }
    }

    return str;
  }
};

// Convenience functions
inline Node parse(const std::string_view content) {
  return Parser::parse(content);
}

}  // namespace YAML
}  // namespace Cloudini
