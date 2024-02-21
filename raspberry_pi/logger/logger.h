#ifndef LOGGER_H
#define LOGGER_H

#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/basic_file_sink.h" 

class Logger {
public:
    static constexpr size_t MAX_LOG_SIZE = 1048576 * 5; // 5MB per file
    static constexpr size_t MAX_LOG_FILES = 3;          // 3 files

    static void initialiseLogger(const std::string& logger_name, const std::string& file_name) {
        auto rotating_logger = spdlog::basic_logger_mt(logger_name, file_name);
        //auto rotating_logger = spdlog::rotating_logger_mt(logger_name, file_name, MAX_LOG_SIZE, MAX_LOG_FILES);
        spdlog::set_default_logger(rotating_logger);
        spdlog::set_level(spdlog::level::debug); // Default log level
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S] [%^%l%$] %v"); // Example pattern
    }

    template<typename T>
    static void info(const T& msg) {
        spdlog::info(msg);
    }

    template<typename T>
    static void warn(const T& msg) {
        spdlog::warn(msg);
    }

    template<typename T>
    static void error(const T& msg) {
        spdlog::error(msg);
    }

    template<typename T>
    static void debug(const T& msg) {
        spdlog::debug(msg);
    }
};

#endif // LOGGER_H
