include(GoogleTest)

add_executable(robot_sim_tests
    test_robot.cpp
    test_pid.cpp
)

target_link_libraries(robot_sim_tests
    PRIVATE robot_sim_lib GTest::gtest_main
)

gtest_discover_tests(robot_sim_tests)
