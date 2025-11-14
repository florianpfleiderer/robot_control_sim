# Created on Fri Nov 14 2025 by Florian Pfleiderer

cmake_minimum_required(VERSION 3.14)

project(RobotSim
    VERSION 0.1.0
    LANGUAGES CXX
)

# --- Compiler settings -------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable extra warnings (you can relax this if it’s too noisy)
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Library target: core simulation code -----------------------------------

add_library(robot_sim_lib
    src/Robot.cpp
    src/Controller.cpp
    src/Sensor.cpp
)

# Public headers live in include/
target_include_directories(robot_sim_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Make sure consumers see we need C++17
target_compile_features(robot_sim_lib
    PUBLIC cxx_std_17
)

# --- Executable target: main program ----------------------------------------

add_executable(robot_sim
    app/main.cpp
)

target_link_libraries(robot_sim
    PRIVATE robot_sim_lib
)

# Optional: set a nice output directory for the binary
set_target_properties(robot_sim
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
