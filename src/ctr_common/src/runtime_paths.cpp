#include "ctr_common/runtime_paths.hpp"

#include <cstdlib>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

namespace ctr_common
{

std::filesystem::path resolveDataRoot(rclcpp::Node &node, const std::string &package_name)
{
  std::string data_root;
  if (!node.has_parameter("data_root"))
  {
    data_root = node.declare_parameter<std::string>("data_root", "");
  }
  else
  {
    node.get_parameter("data_root", data_root);
  }
  if (!data_root.empty())
  {
    return data_root;
  }

  if (const char *env = std::getenv("CTR_DATA_ROOT"); env != nullptr && *env != '\0')
  {
    return env;
  }

  const std::filesystem::path share = ament_index_cpp::get_package_share_directory(package_name);
  const std::filesystem::path root =
      share.parent_path().parent_path().parent_path().parent_path();
  RCLCPP_WARN_ONCE(node.get_logger(),
                   "Neither the 'data_root' parameter nor CTR_DATA_ROOT is set - "
                   "falling back to the legacy workspace-layout climb: %s",
                   root.c_str());
  return root;
}

}  // namespace ctr_common
