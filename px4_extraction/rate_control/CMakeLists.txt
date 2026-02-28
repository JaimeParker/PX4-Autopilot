# Rate Control Library

# Source files
set(RATE_CONTROL_SRCS
    RateControl.cpp
    RateControllerWrapper.cpp
)

# Header files (for IDE)
set(RATE_CONTROL_HDRS
    RateControl.hpp
    RateControlTypes.hpp
    RateControllerWrapper.hpp
)

# Create static library
add_library(rate_control STATIC
    ${RATE_CONTROL_SRCS}
    ${RATE_CONTROL_HDRS}
)

target_include_directories(rate_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib
)

# Installation
install(TARGETS rate_control
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES ${RATE_CONTROL_HDRS}
    DESTINATION include/rate_control
)
