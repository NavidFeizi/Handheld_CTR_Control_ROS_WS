#ifndef CTR_COMMON__OUTPUT_SESSION_HPP_
#define CTR_COMMON__OUTPUT_SESSION_HPP_

// Timestamped output-session directories:
//     <output_base>/<session>/<YYYY-MM-DD_HH-MM-SS>/

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

namespace ctr_common
{

/// Local wall-clock timestamp formatted for directory names.
inline std::string currentTimestamp()
{
  const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream ss;
  ss << std::put_time(std::localtime(&t), "%Y-%m-%d_%H-%M-%S");
  return ss.str();
}

/// Create (and return) <output_base>/<session>/<timestamp>/. An empty session
/// name maps to "default".
inline std::filesystem::path makeSessionDir(const std::filesystem::path &output_base,
                                            const std::string &session_name)
{
  const std::filesystem::path dir =
      output_base / (session_name.empty() ? "default" : session_name) / currentTimestamp();
  std::filesystem::create_directories(dir);
  return dir;
}

}  // namespace ctr_common

#endif  // CTR_COMMON__OUTPUT_SESSION_HPP_
