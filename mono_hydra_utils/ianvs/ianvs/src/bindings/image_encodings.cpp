#include "ianvs/bindings/image_encodings.h"

#include <pybind11/stl.h>

#include <regex>

#include <sensor_msgs/image_encodings.hpp>

namespace py = pybind11;
namespace enc = sensor_msgs::image_encodings;

namespace ianvs::bindings::image_encodings {

std::string get_dtype_str(const std::string& encoding) {
  const auto depth = enc::bitDepth(encoding);
  const auto bytes_str = std::to_string(depth / 8);
  std::string prefix = "u";

  std::smatch m;
  if (std::regex_match(encoding, m, enc::cv_type_regex)) {
    if (m.str(2) == "S") {
      prefix = "i";
    } else if (m.str(2) == "F") {
      prefix = "f";
    }
  }

  return prefix + bytes_str;
}

struct EncodingInfo {
  explicit EncodingInfo(const std::string& encoding)
      : is_color(enc::isColor(encoding)),
        has_alpha(enc::hasAlpha(encoding)),
        bit_depth(enc::bitDepth(encoding)),
        channels(enc::numChannels(encoding)),
        dtype_str(get_dtype_str(encoding)) {}

  const bool is_color;
  const bool has_alpha;
  const int bit_depth;
  const int channels;
  const std::string dtype_str;
};

void add(py::module_& m) {
  py::class_<EncodingInfo>(m, "EncodingInfo")
      .def(py::init<const std::string&>())
      .def_readonly("is_color", &EncodingInfo::is_color)
      .def_readonly("has_alpha", &EncodingInfo::has_alpha)
      .def_readonly("bit_depth", &EncodingInfo::bit_depth)
      .def_readonly("channels", &EncodingInfo::channels)
      .def_readonly("dtype_str", &EncodingInfo::dtype_str)
      .def("__repr__", [](const EncodingInfo& info) {
        std::stringstream ss;
        ss << "<EncodingInfo(is_color=" << std::boolalpha << info.is_color
           << ", has_alpha=" << info.has_alpha << ", bit_depth=" << info.bit_depth
           << ", channels=" << info.channels << ")>";
        return ss.str();
      });

  m.def("parse_encoding", [](const std::string& encoding) -> std::optional<EncodingInfo> {
    try {
      return EncodingInfo(encoding);
    } catch (const std::exception& e) {
      return std::nullopt;
    }
  });
}

}  // namespace ianvs::bindings::image_encodings
