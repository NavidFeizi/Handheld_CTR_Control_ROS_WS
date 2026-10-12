#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include "ctr_common/package_log.hpp"

// The tee must (1) keep calling the handler that was installed before it
// (console / ~/.ros/log), (2) write every record of the process to the package
// file — including records longer than its stack buffer — and (3) resolve the
// file under <log_dir>/<package>/.

namespace
{
std::atomic<int> g_previous_calls{0};

void countingHandler(const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
                     const char *, va_list *)
{
  ++g_previous_calls;
}

std::string slurp(const std::filesystem::path &p)
{
  std::ifstream in(p);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
}  // namespace

TEST(PackageLog, TeesRecordsIntoPackageDirectoryAndKeepsPreviousHandler)
{
  const auto root = std::filesystem::temp_directory_path() /
                    ("ctr_package_log_test_" + std::to_string(::getpid()));
  std::filesystem::remove_all(root);

  rcutils_logging_set_output_handler(countingHandler);
  auto node = std::make_shared<rclcpp::Node>(
      "pkg_log_probe", rclcpp::NodeOptions().parameter_overrides({{"log_dir", root.string()}}));

  const auto file = ctr_common::installPackageFileLog(*node, "probe_pkg");
  ASSERT_FALSE(file.empty());
  EXPECT_EQ(file.parent_path(), root / "probe_pkg");

  const std::string long_payload(3000, 'x');
  const int before = g_previous_calls.load();
  RCLCPP_WARN(node->get_logger(), "[ik] residual=%.3f mm %s", 1.25, long_payload.c_str());
  ctr_common::writePackageLog(RCUTILS_LOG_SEVERITY_ERROR, "robot_node.spdlog", "[cmd] target dropped");
  EXPECT_EQ(g_previous_calls.load(), before + 1) << "console handler must still run";

  // Second install is a no-op returning the same file.
  EXPECT_EQ(ctr_common::installPackageFileLog(*node, "other_pkg"), file);

  const std::string text = slurp(file);
  EXPECT_NE(text.find("[WARN] [pkg_log_probe]: [ik] residual=1.250 mm " + long_payload), std::string::npos);
  EXPECT_NE(text.find("[ERROR] [robot_node.spdlog]: [cmd] target dropped"), std::string::npos);
  EXPECT_TRUE(std::filesystem::is_symlink(root / "probe_pkg" / "pkg_log_probe_latest.log"));

  std::filesystem::remove_all(root);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
