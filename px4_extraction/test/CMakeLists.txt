# Test executables

# Simple test executable
add_executable(attitude_control_simple_test
    attitude_control_simple_test.cpp
)

target_link_libraries(attitude_control_simple_test
    PRIVATE
        attitude_control
)

# Original PX4 tests (requires adaptation)
# Commented out until we adapt them properly
# add_executable(attitude_control_px4_test
#     AttitudeControlTest.cpp
# )
#
# target_link_libraries(attitude_control_px4_test
#     PRIVATE
#         attitude_control
#         matrix
# )

message(STATUS "Test executables configured")
