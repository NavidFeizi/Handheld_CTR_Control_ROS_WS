#ifndef CTR_COMMON__PACKAGE_LOG_HPP_
#define CTR_COMMON__PACKAGE_LOG_HPP_

// Package-separated diagnostic log files.
//
//   <log_root>/[<CTR_RUN_ID>/]<package>/<YYYY-MM-DD_HH-MM-SS>_<node>_<pid>.log
//   <log_root>/[<CTR_RUN_ID>/]<package>/<node>_latest.log   (symlink to the newest file)
//
// log_root resolution: node parameter "log_dir" (declared here if absent)
//   → environment variable CTR_LOG_DIR → <data_root>/log (resolveDataRoot()).
//
// The file is a tee of the process' rcutils log stream: the handler that was
// installed before (rclcpp's console + ~/.ros/log handler) still runs first,
// so standard ROS 2 logging is unchanged. Only records that pass the logger's
// severity threshold reach the file; raise it per node with
//   --ros-args --log-level <node_name>:=debug

#include <filesystem>
#include <string>

#include <rclcpp/node.hpp>

namespace ctr_common
{

/// Install the package log tee for this process (idempotent; the first call
/// wins). Must run after rclcpp::init() and before any executor spins (the
/// rcutils handler pointer is swapped unsynchronised, as rclcpp itself does).
/// Returns the log file path, or an empty path (after a WARN on the node
/// logger) if the file could not be opened.
std::filesystem::path installPackageFileLog(rclcpp::Node &node, const std::string &package_name);

/// Append one already-formatted record to this process' package log file.
/// For loggers that do not go through rcutils (e.g. the robot's spdlog
/// messages). No-op until installPackageFileLog() succeeded.
/// severity uses the RCUTILS_LOG_SEVERITY_* values.
void writePackageLog(int severity, const std::string &logger_name, const std::string &message);

}  // namespace ctr_common

#endif  // CTR_COMMON__PACKAGE_LOG_HPP_
