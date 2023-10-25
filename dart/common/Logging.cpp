#include "dart/common/Logging.hpp"

namespace dart {
namespace logging {

std::shared_ptr<spdlog::logger> get()
{
  static std::shared_ptr<spdlog::logger> logger;
  if (!logger)
  {
    logger = spdlog::stdout_color_mt("adamas");
    logger->set_level(spdlog::level::warn);
    logger->set_pattern("[%H:%M:%S.%e][%^%l%$] %v");
    spdlog::cfg::load_env_levels();
  }
  return logger;
}

}  // namespace common
}  // namespace dart
