#pragma once

#include <string>
#include "ntop_sdf/field.hpp"

namespace ntop_sdf {

// Loads an nTop-exported .implicit file through the separately installed
// nTop Core SDK. nTop Core uses metres. scale uniformly rescales the geometry
// about its origin; the default preserves the exported dimensions.
// The field owns the nTop Core handle and serializes calls into the SDK.
FieldPtr load_ntop_core_implicit(const std::string& utf8_path, double scale = 1.0);

// The version of the dynamically linked nTop Core runtime.
std::string ntop_core_version();

}  // namespace ntop_sdf
