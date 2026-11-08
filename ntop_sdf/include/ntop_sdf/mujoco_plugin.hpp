#pragma once

#include <cstdint>
#include "ntop_sdf/field.hpp"

struct mjModel_;
struct mjData_;

namespace ntop_sdf {

// Register before compiling an MJCF model. The numeric handle is used as the
// plugin's field_id config value. Keep it registered until all models and data
// that refer to it have been destroyed. Handles are process-local.
std::uint64_t register_mujoco_field(FieldPtr field);
bool unregister_mujoco_field(std::uint64_t id);
// Registers plugin name "ntop.sdf.callback" once per process.
void register_mujoco_plugin();

// Opt a particular mjData into the cached field collision path. Call this
// during setup, before concurrent stepping. Detach before freeing mjData.
// The function rejects model configurations that could use a mesh-based
// plane, mesh, height-field, or flex collision against an SDF plugin.
void attach_mujoco_collision(const mjModel_* model, mjData_* data);
void detach_mujoco_collision(mjData_* data);
struct MujocoCollisionStatistics {
  std::uint64_t field_queries{}, generated_contacts{}, pair_queries{}, failures{};
};
MujocoCollisionStatistics mujoco_collision_statistics(const mjData_* data);
std::uint64_t mujoco_collision_failures(const mjData_* data);

}  // namespace ntop_sdf
