#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "logger.h"

std::string format_log_file_name(const std::string& file_name, const std::string& logs_path) {
    // Get the current time
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    // Format current timestamp string
    std::ostringstream timestamp;
    timestamp << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");

    // Construct the full log file name
    return (logs_path + "/logs/" + file_name + "_" + timestamp.str() + ".log");
}

void ensure_log_dir_exists(const std::string& logs_path) {
    std::filesystem::path log_dir{logs_path};
    if (!std::filesystem::exists(log_dir)) {
        std::filesystem::create_directories(log_dir);
    }
}

void Logger::initialise_logger(const std::string& logger_name, const std::string& file_name, const std::string& logs_path) {
    ensure_log_dir_exists(logs_path + "/logs");
    
    // Define console sink
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern(LOG_PATTERN);
    
    // Define rotating file sink
    std::string full_file_name = format_log_file_name(file_name, logs_path);
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(full_file_name, MAX_LOG_SIZE, MAX_LOG_FILES);
    rotating_sink->set_level(spdlog::level::debug);
    rotating_sink->set_pattern(LOG_PATTERN);
    
    // Combine rotating and console sinks for multi sink logging
    auto logger = std::make_shared<spdlog::logger>(logger_name, spdlog::sinks_init_list{console_sink, rotating_sink});
    logger->set_level(spdlog::level::debug);
    
    // Make the multi sink logger the default one
    spdlog::set_default_logger(logger);

    // If the logger is not registered, add it in case it needs to be retrieved
    if (!spdlog::get(logger_name)) {
        spdlog::register_logger(logger);
    }
}

