#include "ctr_common/package_log.hpp"

#include <unistd.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <system_error>

#include <rcutils/logging.h>
#include <rclcpp/logging.hpp>

#include "ctr_common/output_session.hpp"
#include "ctr_common/runtime_paths.hpp"

namespace ctr_common
{
namespace
{

std::mutex g_file_mutex;           // serialises writes to g_file
std::FILE *g_file = nullptr;       // owned for the process lifetime
std::atomic<rcutils_logging_output_handler_t> g_previous_handler{nullptr};

const char *severityName(int severity)
{
  switch (severity)
  {
  case RCUTILS_LOG_SEVERITY_DEBUG: return "DEBUG";
  case RCUTILS_LOG_SEVERITY_INFO: return "INFO";
  case RCUTILS_LOG_SEVERITY_WARN: return "WARN";
  case RCUTILS_LOG_SEVERITY_ERROR: return "ERROR";
  case RCUTILS_LOG_SEVERITY_FATAL: return "FATAL";
  default: return "UNSET";
  }
}

// "YYYY-MM-DD HH:MM:SS.mmm" in local time.
void formatTime(rcutils_time_point_value_t ns, char *out, std::size_t len)
{
  const std::time_t sec = static_cast<std::time_t>(ns / 1000000000LL);
  const int ms = static_cast<int>((ns / 1000000LL) % 1000LL);
  std::tm tm{};
  localtime_r(&sec, &tm);
  char base[32];
  std::strftime(base, sizeof(base), "%Y-%m-%d %H:%M:%S", &tm);
  std::snprintf(out, len, "%s.%03d", base, ms);
}

void writeLine(rcutils_time_point_value_t ns, int severity, const char *name, const char *msg)
{
  char ts[48];
  formatTime(ns, ts, sizeof(ts));
  std::lock_guard<std::mutex> lock(g_file_mutex);
  if (g_file == nullptr)
    return;
  std::fprintf(g_file, "[%s] [%s] [%s]: %s\n", ts, severityName(severity), name ? name : "", msg);
  // INFO and above are flushed so a crash keeps the lines that explain it;
  // DEBUG (opt-in, potentially per-cycle) stays in the stdio buffer.
  if (severity >= RCUTILS_LOG_SEVERITY_INFO)
    std::fflush(g_file);
}

void teeHandler(const rcutils_log_location_t *location, int severity, const char *name,
                rcutils_time_point_value_t timestamp, const char *format, va_list *args)
{
  // The previous handler consumes *args, so the file copy is formatted first.
  char stack_buf[1024];
  std::string heap_buf;
  const char *msg = stack_buf;
  va_list copy;
  va_copy(copy, *args);
  const int needed = std::vsnprintf(stack_buf, sizeof(stack_buf), format, copy);
  va_end(copy);
  if (needed >= static_cast<int>(sizeof(stack_buf)))
  {
    heap_buf.resize(static_cast<std::size_t>(needed) + 1);
    va_copy(copy, *args);
    std::vsnprintf(heap_buf.data(), heap_buf.size(), format, copy);
    va_end(copy);
    msg = heap_buf.c_str();
  }

  if (const auto previous = g_previous_handler.load(std::memory_order_acquire); previous != nullptr)
    previous(location, severity, name, timestamp, format, args);

  if (needed >= 0)
    writeLine(timestamp, severity, name, msg);
}

std::filesystem::path resolveLogRoot(rclcpp::Node &node, const std::string &package_name)
{
  std::string log_dir;
  if (!node.has_parameter("log_dir"))
    log_dir = node.declare_parameter<std::string>("log_dir", "");
  else
    node.get_parameter("log_dir", log_dir);
  if (!log_dir.empty())
    return log_dir;

  if (const char *env = std::getenv("CTR_LOG_DIR"); env != nullptr && *env != '\0')
    return env;

  return resolveDataRoot(node, package_name) / "log";
}

}  // namespace

std::filesystem::path installPackageFileLog(rclcpp::Node &node, const std::string &package_name)
{
  static std::once_flag once;
  static std::filesystem::path installed_path;

  std::call_once(once, [&]() {
    std::filesystem::path dir = resolveLogRoot(node, package_name);
    if (const char *run = std::getenv("CTR_RUN_ID"); run != nullptr && *run != '\0')
      dir /= run;
    dir /= package_name;

    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string stem = std::string(node.get_name()) + "_";
    const std::filesystem::path file =
        dir / (currentTimestamp() + "_" + node.get_name() + "_" + std::to_string(::getpid()) + ".log");

    std::FILE *f = ec ? nullptr : std::fopen(file.c_str(), "a");
    if (f == nullptr)
    {
      RCLCPP_WARN(node.get_logger(), "Package log disabled: cannot open '%s' (%s)", file.c_str(),
                  ec ? ec.message().c_str() : "fopen failed");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(g_file_mutex);
      g_file = f;
    }

    // Best effort: a stale or unsupported symlink must not disable logging.
    const std::filesystem::path latest = dir / (stem + "latest.log");
    std::filesystem::remove(latest, ec);
    std::filesystem::create_symlink(file.filename(), latest, ec);

    // rcutils' handler pointer is a plain global (rclcpp sets it the same way in
    // rclcpp::init); install before executors spin. Publishing the previous
    // handler first means teeHandler never observes a null predecessor.
    g_previous_handler.store(rcutils_logging_get_output_handler(), std::memory_order_release);
    rcutils_logging_set_output_handler(teeHandler);
    installed_path = file;

    const char *run = std::getenv("CTR_RUN_ID");
    RCLCPP_INFO(node.get_logger(), "Package log: %s (package=%s node=%s pid=%d run_id=%s)", file.c_str(),
                package_name.c_str(), node.get_fully_qualified_name(), static_cast<int>(::getpid()),
                (run != nullptr && *run != '\0') ? run : "-");
  });
  return installed_path;
}

void writePackageLog(int severity, const std::string &logger_name, const std::string &message)
{
  rcutils_time_point_value_t now = 0;
  if (rcutils_system_time_now(&now) != RCUTILS_RET_OK)
    now = 0;
  writeLine(now, severity, logger_name.c_str(), message.c_str());
}

}  // namespace ctr_common
