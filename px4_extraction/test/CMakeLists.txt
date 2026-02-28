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
endif()

message(STATUS "Test executables configured")
