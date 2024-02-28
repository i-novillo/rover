#ifndef LOGGER_H
#define LOGGER_H

#include "spdlog/spdlog.h"
//#include "spdlog/sinks/rotating_file_sink.h"
//#include "spdlog/sinks/stdout_color_sinks.h"

std::string format_log_file_name(const std::string& file_name, const std::string& logs_path);
void ensure_log_dir_exists(const std::string& logs_path);

class Logger {
public:
    static constexpr size_t MAX_LOG_SIZE = 1048576 * 5; // 5MB per file
    static constexpr size_t MAX_LOG_FILES = 3;          // 3 files
    static constexpr const char* LOG_PATTERN = "[%Y-%m-%d %H:%M:%S] [%^%l%$] %v";

    static void initialise_logger(const std::string& logger_name, const std::string& file_name, const std::string& logs_path);

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
