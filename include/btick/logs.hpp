#ifndef BTICK__LOGS_HPP_
#define BTICK__LOGS_HPP_

#include <chrono>
#include <memory>
#include <string>

namespace btick {

class Monitor;

/**
 * @brief Standalone persistent BehaviorTree execution logger.
 *
 * Attaches directly to a live `btick::Monitor` to record exact execution events,
 * timestamps, blackboard values, port changes, and interactive synthetic transitions
 * (breakpoints, pauses, fault delays, blackboard updates) in the exact session schema
 * recognized by BTick / bt_monitor_app.
 *
 * Provides incremental background persistence (configurable flush interval) and
 * explicit manual flush/stop controls.
 */
class Logs {
 public:
  // Attach to monitor for a specific tree_id
  explicit Logs(Monitor &monitor, const std::string &tree_id, const std::string &file_path,
                bool timestamp_filename = false,
                std::chrono::milliseconds flush_interval = std::chrono::milliseconds(2000));

  // Overload with explicit flush_interval
  explicit Logs(Monitor &monitor, const std::string &tree_id, const std::string &file_path,
                std::chrono::milliseconds flush_interval);

  // Convenience overload for single-tree monitor (resolves to monitor.tree_id())
  explicit Logs(Monitor &monitor, const std::string &file_path, bool timestamp_filename = false,
                std::chrono::milliseconds flush_interval = std::chrono::milliseconds(2000));

  // Overload with explicit flush_interval for single-tree monitor
  explicit Logs(Monitor &monitor, const std::string &file_path,
                std::chrono::milliseconds flush_interval);

  ~Logs();

  // Force an immediate atomic write to disk
  void flush();

  // Stop recording and unregister from Monitor
  void stop();

  // Returns the resolved file path on disk
  const std::string &file_path() const noexcept;

  Logs(const Logs &) = delete;
  Logs &operator=(const Logs &) = delete;
  Logs(Logs &&) noexcept;
  Logs &operator=(Logs &&) noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace btick

#endif  // BTICK__LOGS_HPP_
