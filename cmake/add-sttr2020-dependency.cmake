
# -----------------------------------------------
# -----------------------------------------------
# COSMOS sttr2020 Dependency
if (NOT DEFINED COSMOS_STTR2020_GIT_TAG)
    set(COSMOS_STTR2020_GIT_TAG "feature/swarm_controller")
    message("COSMOS_STTR2020_GIT_TAG not defined, using default git tag: \"${COSMOS_STTR2020_GIT_TAG}\"")
endif()
include(FetchContent)
FetchContent_Declare(
    cosmos_sttr2020
    GIT_REPOSITORY git@github.com:interstel-tech/cosmos_sttr2020.git
    GIT_TAG        ${COSMOS_STTR2020_GIT_TAG}
    GIT_PROGRESS TRUE
)
FetchContent_GetProperties(cosmos_sttr2020)
if(NOT cosmos_sttr2020_POPULATED)
    message("Populating cosmos_sttr2020 dependency...")
    FetchContent_Populate(cosmos_sttr2020)
    add_subdirectory(${cosmos_sttr2020_SOURCE_DIR}/source ${cosmos_sttr2020_BINARY_DIR}/source EXCLUDE_FROM_ALL)
    include_directories(${cosmos_sttr2020_SOURCE_DIR}/source/libraries)
endif()
# -----------------------------------------------
# -----------------------------------------------
