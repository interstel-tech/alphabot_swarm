cmake_minimum_required(VERSION 3.25)
project(swarm-controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER_PREPROCESSOR_FLAGS} -pthread")

if (ENABLE_ASSEMBLY)
    enable_language(ASM)
endif()

find_package(Threads REQUIRED)

# COSMOS dependencies
set(COSMOS_CORE_GIT_TAG "35db329906e2071")
include(cmake/add-cosmos-dependency.cmake)
include(cmake/add-sttr2020-dependency.cmake)

add_executable(swarm_child src/agent/swarm_child.cpp src/agent/swarm_params.h)
target_link_libraries(swarm_child CosmosAgent Threads::Threads)
add_executable(swarm_controller
    src/agent/swarm_controller.cpp
    src/agent/swarm_params.h
    src/agent/helpers/sph_helper.cpp
    src/agent/helpers/sph_helper.h
)
target_link_libraries(swarm_controller CosmosAgent MacHcl SwarmSimulator Threads::Threads)

# Create a custom target to copy the agent binary to the build directory for easy access
add_custom_target(copy_agent ALL
    COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:agent>
            ${CMAKE_BINARY_DIR}/$<TARGET_FILE_NAME:agent>
    DEPENDS agent
)
