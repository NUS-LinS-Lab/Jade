#ifndef LOGGING_H_
#define LOGGING_H_

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif  // SPDLOG_ACTIVE_LEVEL
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
// spdlog uses v9 of fmt, in which fmt/ranges.h will conflict with Eigen
#include <sstream>
#include <string>
#include <vector>

namespace dart {
namespace logging {

/// HACK
template <typename T>
inline std::string v2s(const std::vector<T>& v)
{
  std::stringstream ss;
  ss << "[";
  for (const auto& i: v) ss << " " << i;
  ss << " ]";
  return ss.str();
}

std::shared_ptr<spdlog::logger> get();

template <typename... Args>
inline void debug(const char* fmt, const Args &...args)
{
  SPDLOG_LOGGER_DEBUG(get(), fmt, args...);
};

template <typename... Args>
inline void info(const char* fmt, const Args &...args)
{
  SPDLOG_LOGGER_INFO(get(), fmt, args...);
};

template <typename... Args>
inline void warn(const char* fmt, const Args &...args)
{
  SPDLOG_LOGGER_WARN(get(), fmt, args...);
};

template <typename... Args>
inline void error(const char* fmt, const Args &...args)
{
  SPDLOG_LOGGER_ERROR(get(), fmt, args...);
};

template <typename... Args>
inline void critical(const char* fmt, const Args &...args)
{
  SPDLOG_LOGGER_CRITICAL(get(), fmt, args...);
};

}  // namespace common
}  // namespace dart

#endif // LOGGING_H_
