#ifndef CTR_COMMON__RUNTIME_PATHS_HPP_
#define CTR_COMMON__RUNTIME_PATHS_HPP_

#include <filesystem>
#include <string>

#include <rclcpp/node.hpp>

namespace ctr_common
{

/// Resolve the workspace data root holding Input_Files/, Output_Files/ and
/// Shared_Files/. Resolution order:
///   1. node parameter "data_root" (declared here if absent)
///   2. environment variable CTR_DATA_ROOT
///   3. legacy fallback with a WARN: climb four parents from the package's
///      share directory (install/<pkg>/share/<pkg> → workspace root), which
///      only works in an in-workspace install layout.
std::filesystem::path resolveDataRoot(rclcpp::Node &node, const std::string &package_name);

/// Resolve the directory holding the trained PINN model pool. Resolution order:
///   1. node parameter "models_dir" (declared here if absent)
///   2. the installed share/ctr_kinematics_pinn/models directory
std::filesystem::path resolveModelsDir(rclcpp::Node &node);

}  // namespace ctr_common

#endif  // CTR_COMMON__RUNTIME_PATHS_HPP_
