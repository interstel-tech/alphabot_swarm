
# -----------------------------------------------
# -----------------------------------------------
# COSMOS Core Dependency
if (NOT DEFINED COSMOS_CORE_GIT_TAG)
    message("COSMOS_CORE_GIT_TAG not defined, using default COSMOS core git tag: \"master\"")
    set(COSMOS_CORE_GIT_TAG master)
endif()
include(FetchContent)
FetchContent_Declare(
    cosmos_core
    GIT_REPOSITORY https://github.com/hsfl/cosmos-core.git
    GIT_TAG        ${COSMOS_CORE_GIT_TAG}
    GIT_PROGRESS TRUE
)
FetchContent_GetProperties(cosmos_core)
if(NOT cosmos_core_POPULATED)
    message("Populating cosmos_core dependency...")
    FetchContent_Populate(cosmos_core)
endif()
set(COSMOS_SOURCE_CORE ${cosmos_core_SOURCE_DIR})
message("COSMOS_SOURCE_CORE: ${COSMOS_SOURCE_CORE}")
# -----------------------------------------------
IF (${CROSS_TYPE} MATCHES "arm" OR ${CROSS_TYPE} MATCHES "iobc" OR ${CROSS_TYPE} MATCHES "rpi")
    # Add preprocessor definition to help source files determine code to use for ARMv7 specific implementation
    add_definitions(-DCROSS_TYPE_${CROSS_TYPE})
    # Add include directories for locally compiled arm openssl
    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/arm/include)
ENDIF()
# -----------------------------------------------
set(USE_COSMOS_FROM "SOURCE")
# include(${COSMOS_SOURCE_CORE}/cmake/use_cosmos_from_source.cmake) # included by the sttr2020 dependency
message("COSMOS: ${COSMOS}")
set(CMAKE_INSTALL_PREFIX "${COSMOS}/${PROJECT_NAME}" CACHE PATH "default install path" FORCE )
message("Install: ${CMAKE_INSTALL_PREFIX}")
include_directories(${COSMOS_SOURCE_CORE}/libraries)
# -----------------------------------------------
# -----------------------------------------------
