cmake_minimum_required(VERSION 3.5)
project(mc_lqr_control)

# Add source files
set(SRCS
    LQRControl.cpp
)

# Add header files
set(HDRS
    LQRControl.hpp
)

# Add the executable
add_executable(${PROJECT_NAME} ${SRCS} ${HDRS})

# Link against PX4 libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        controllib
        geo
        SlewRate
)

# Add include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
