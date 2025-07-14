cmake_minimum_required(VERSION 3.25)
project(swarm-controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER_PREPROCESSOR_FLAGS}")

if (ENABLE_ASSEMBLY)
    enable_language(ASM)
endif()

# COSMOS core dependency
set(COSMOS_CORE_GIT_TAG "35db329906e2071")
include(cmake/add-cosmos-dependency.cmake)

add_executable(swarm_child src/agent/swarm_child.cpp)
target_link_libraries(swarm_child CosmosAgent)

