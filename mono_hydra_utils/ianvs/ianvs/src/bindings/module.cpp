#include <pybind11/pybind11.h>

#include "ianvs/bindings/image_encodings.h"

PYBIND11_MODULE(_ianvs_bindings, m) { ::ianvs::bindings::image_encodings::add(m); }
