# Control Allocator Library

# Source files
set(CONTROL_ALLOCATOR_SRCS
    ActuatorEffectiveness.cpp
    ControlAllocation.cpp
    ControlAllocationPseudoInverse.cpp
    ControlAllocationSequentialDesaturation.cpp
    ControlAllocatorWrapper.cpp
)

# Header files (for IDE)
set(CONTROL_ALLOCATOR_HDRS
    ActuatorEffectiveness.hpp
    ControlAllocation.hpp
    ControlAllocationPseudoInverse.hpp
    ControlAllocationSequentialDesaturation.hpp
    RotorGeometry.hpp
    ControlAllocatorTypes.hpp
    ControlAllocatorWrapper.hpp
)

# Create static library
add_library(control_allocator STATIC
    ${CONTROL_ALLOCATOR_SRCS}
    ${CONTROL_ALLOCATOR_HDRS}
)

target_include_directories(control_allocator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib
)

# Installation
install(TARGETS control_allocator
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES ${CONTROL_ALLOCATOR_HDRS}
    DESTINATION include/control_allocator
)
