# Attitude Control Library

# Source files
set(ATTITUDE_CONTROL_SRCS
    AttitudeControl.cpp
    AttitudeControllerWrapper.cpp
)

# Header files (for IDE)
set(ATTITUDE_CONTROL_HDRS
    AttitudeControl.hpp
    AttitudeControlTypes.hpp
    AttitudeControllerWrapper.hpp
)

# Create static library
add_library(attitude_control STATIC
    ${ATTITUDE_CONTROL_SRCS}
    ${ATTITUDE_CONTROL_HDRS}
)

# Matrix library is header-only, no need to link
# Just include directories
target_include_directories(attitude_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib
)

# Installation
install(TARGETS attitude_control
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES ${ATTITUDE_CONTROL_HDRS}
    DESTINATION include/attitude_control
)
