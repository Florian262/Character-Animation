cmake_minimum_required(VERSION 3.20)
project(KinematicRunPlugin2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Animation plugin library
add_library(kinematic_run_plugin SHARED
    src/KinematicRunPlugin2.cpp
    src/MathUtils.cpp
)

# Include directories
target_include_directories(kinematic_run_plugin PRIVATE
    include
    "C:/N8RO/include/astlib"
    "C:/N8RO/include/astsim"
    "C:/N8RO/include/astdb"
    "C:/N8RO/include/n8ro-db"
)

# Preprocessor definitions
target_compile_definitions(kinematic_run_plugin PRIVATE
    ARKHEON_ASTLIB_EXPORTS
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)

# Link libraries
target_link_directories(kinematic_run_plugin PRIVATE "C:/N8RO/lib")
target_link_libraries(kinematic_run_plugin PRIVATE
    arkheon-astlib.lib
    arkheon-astsim.lib
)

# Output settings
set_target_properties(kinematic_run_plugin PROPERTIES
    OUTPUT_NAME "kinematic_run_plugin"
    PREFIX ""
    WINDOWS_EXPORT_ALL_SYMBOLS TRUE
)

# Mock host for math testing
add_executable(mock_host src/mock_host2.cpp)

target_include_directories(mock_host PRIVATE
    include
    "C:/N8RO/include/astlib"
    "C:/N8RO/include/astsim"
    "C:/N8RO/include/astdb"
    "C:/N8RO/include/n8ro-db"
)

target_compile_definitions(mock_host PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)

target_link_directories(mock_host PRIVATE "C:/N8RO/lib")
target_link_libraries(mock_host PRIVATE
    arkheon-astlib.lib
    arkheon-astsim.lib
)
