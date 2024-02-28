# Create a library target for the logger
add_library(logger logger.cpp)

# Ensure the current directory is included when the logger library is used
target_include_directories(logger PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Include spdlog directory
target_include_directories(logger PUBLIC ../../external_dependencies/spdlog/include)