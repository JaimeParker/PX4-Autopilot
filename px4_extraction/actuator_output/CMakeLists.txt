# Actuator Output Library (Phase 4)

# Source files
set(ACTUATOR_OUTPUT_SRCS
    ActuatorOutputWrapper.cpp
)

# Header files (for IDE)
set(ACTUATOR_OUTPUT_HDRS
    ActuatorOutputTypes.hpp
    ThrustCurve.hpp
    ActuatorOutputWrapper.hpp
)

# Create static library
add_library(actuator_output STATIC
    ${ACTUATOR_OUTPUT_SRCS}
    ${ACTUATOR_OUTPUT_HDRS}
)

target_include_directories(actuator_output
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib
)

# Installation
install(TARGETS actuator_output
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES ${ACTUATOR_OUTPUT_HDRS}
    DESTINATION include/actuator_output
)
