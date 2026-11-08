#pragma once
#include "ntop_sdf/field.hpp"
#include <mujoco/mujoco.h>
namespace ntop_sdf::detail {
FieldPtr field_for_geom(const mjModel* model, const mjData* data, int geom);
}
