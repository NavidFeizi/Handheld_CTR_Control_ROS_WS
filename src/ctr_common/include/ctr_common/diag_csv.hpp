#ifndef CTR_COMMON__DIAG_CSV_HPP_
#define CTR_COMMON__DIAG_CSV_HPP_

// Append-only CSV writer for node diagnostics.
//
// Purpose-built for the per-request diagnostic records the planner and manager
// emit (see the per-package *_diag.csv files under
// <data_root>/Output_Files/diagnostics/): open lazily on the FIRST append,
// write the header row exactly once, flush after every record so a crash or
// SIGINT cannot swallow the request that triggered it. Thread-safe; never
// throws from append() -- diagnostics must not be able to take down a service
// callback. On an open failure it reports once to stderr and disables itself.

#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>

namespace ctr_common
{

class DiagCsv
{
public:
  DiagCsv() = default;

  DiagCsv(std::filesystem::path file_path, std::string header)
  {
    configure(std::move(file_path), std::move(header));
  }

  /// Set (or replace) the target file and its header row. May be called before
  /// any append; a previously open file is closed first.
  void configure(std::filesystem::path file_path, std::string header)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_out.is_open())
      m_out.close();
    m_path = std::move(file_path);
    m_header = std::move(header);
    m_failed = false;
  }

  /// Append one CSV record (no trailing newline needed). No-op when
  /// unconfigured or after an open failure.
  void append(const std::string &row)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_failed || m_path.empty())
      return;

    if (!m_out.is_open())
    {
      std::error_code ec;
      if (m_path.has_parent_path())
        std::filesystem::create_directories(m_path.parent_path(), ec);
      m_out.open(m_path, std::ios::out | std::ios::app);
      if (!m_out.is_open())
      {
        std::cerr << "[DiagCsv] cannot open " << m_path << " - diagnostics disabled for this file" << std::endl;
        m_failed = true;
        return;
      }
      // Append mode may be reopening an existing file (e.g. after configure());
      // only a fresh file gets the header.
      if (std::filesystem::file_size(m_path, ec) == 0 && !m_header.empty())
        m_out << m_header << '\n';
    }

    m_out << row << '\n';
    m_out.flush();
  }

  const std::filesystem::path &path() const { return m_path; }

private:
  std::mutex m_mutex;
  std::ofstream m_out;
  std::filesystem::path m_path;
  std::string m_header;
  bool m_failed = false;
};

}  // namespace ctr_common

#endif  // CTR_COMMON__DIAG_CSV_HPP_
