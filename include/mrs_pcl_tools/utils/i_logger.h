#pragma once

#pragma once

#include <string>

// clang-format off
#ifdef DEBUG
  #define DEBUG_LOG(logger, msg) (logger).info(msg)
#else
  #define DEBUG_LOG(logger, msg) do {} while (0)
#endif
#define INFO_LOG_COND(verbose, logger, msg) if (verbose) { (logger).info(msg); }
#define ERROR_LOG_COND(verbose, logger, msg) if (verbose) { (logger).error(msg); }

enum class LogLevel {Debug, Info, Warn, Error};

class ILogger
{
public:
  virtual ~ILogger() = default;

  virtual void log(const LogLevel level, const std::string& msg) = 0;

  void debug(const std::string& msg) { log(LogLevel::Debug, msg); }
  void info(const std::string& msg) { log(LogLevel::Info, msg); }
  void warn(const std::string& msg) { log(LogLevel::Warn, msg); }
  void error(const std::string& msg) { log(LogLevel::Error, msg); }
};

// clang-format on
