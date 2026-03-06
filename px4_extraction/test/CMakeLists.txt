# Test executables

# Simple test executable
add_executable(attitude_control_simple_test
    attitude_control_simple_test.cpp
)

target_link_libraries(attitude_control_simple_test
    PRIVATE
        attitude_control
)

# GTest-based unit tests
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    # Fallback: link directly against system gtest libraries

    add_executable(attitude_control_gtest
        AttitudeControlTest.cpp
    )
    target_include_directories(attitude_control_gtest PRIVATE /usr/include)
    target_link_libraries(attitude_control_gtest
        PRIVATE
            attitude_control
            gtest
            gtest_main
            pthread
    )

    add_executable(rate_control_gtest
        RateControlTest.cpp
    )
    target_include_directories(rate_control_gtest PRIVATE /usr/include)
    target_link_libraries(rate_control_gtest
        PRIVATE
            rate_control
            attitude_control
            gtest
            gtest_main
            pthread
    )

    add_executable(control_allocator_gtest
        ControlAllocatorTest.cpp
    )
    target_include_directories(control_allocator_gtest PRIVATE /usr/include)
    target_link_libraries(control_allocator_gtest
        PRIVATE
            control_allocator
            rate_control
            attitude_control
            gtest
            gtest_main
            pthread
    )

    add_executable(actuator_output_gtest
        ActuatorOutputTest.cpp
    )
    target_include_directories(actuator_output_gtest PRIVATE /usr/include)
    target_link_libraries(actuator_output_gtest
        PRIVATE
            actuator_output
            control_allocator
            rate_control
            attitude_control
            gtest
            gtest_main
            pthread
    )

    add_executable(l2_equivalence_gtest
        L2_EquivalenceTest.cpp
    )
    target_include_directories(l2_equivalence_gtest PRIVATE /usr/include)
    target_link_libraries(l2_equivalence_gtest
        PRIVATE
            attitude_control
            rate_control
            control_allocator
            actuator_output
            gtest
            gtest_main
            pthread
    )

    add_executable(l3_replay_gtest
        L3_ReplayTest.cpp
    )
    target_include_directories(l3_replay_gtest PRIVATE /usr/include)
    target_compile_definitions(l3_replay_gtest PRIVATE
        L3_TRACE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/l3_trace.csv"
    )
    target_link_libraries(l3_replay_gtest
        PRIVATE
            attitude_control
            rate_control
            control_allocator
            actuator_output
            gtest
            gtest_main
            pthread
    )
else()
    add_executable(attitude_control_gtest
        AttitudeControlTest.cpp
    )
    target_link_libraries(attitude_control_gtest
        PRIVATE
            attitude_control
            GTest::GTest
            GTest::Main
    )

    add_executable(rate_control_gtest
        RateControlTest.cpp
    )
    target_link_libraries(rate_control_gtest
        PRIVATE
            rate_control
            attitude_control
            GTest::GTest
            GTest::Main
    )

    add_executable(control_allocator_gtest
        ControlAllocatorTest.cpp
    )
    target_link_libraries(control_allocator_gtest
        PRIVATE
            control_allocator
            rate_control
            attitude_control
            GTest::GTest
            GTest::Main
    )

    add_executable(actuator_output_gtest
        ActuatorOutputTest.cpp
    )
    target_link_libraries(actuator_output_gtest
        PRIVATE
            actuator_output
            control_allocator
            rate_control
            attitude_control
            GTest::GTest
            GTest::Main
    )

    add_executable(l2_equivalence_gtest
        L2_EquivalenceTest.cpp
    )
    target_link_libraries(l2_equivalence_gtest
        PRIVATE
            attitude_control
            rate_control
            control_allocator
            actuator_output
            GTest::GTest
            GTest::Main
    )

    add_executable(l3_replay_gtest
        L3_ReplayTest.cpp
    )
    target_compile_definitions(l3_replay_gtest PRIVATE
        L3_TRACE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/l3_trace.csv"
    )
    target_link_libraries(l3_replay_gtest
        PRIVATE
            attitude_control
            rate_control
            control_allocator
            actuator_output
            GTest::GTest
            GTest::Main
    )
endif()

message(STATUS "Test executables configured")
