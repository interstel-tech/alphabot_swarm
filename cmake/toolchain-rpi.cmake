SET(CMAKE_SYSTEM_NAME Linux)

# SET(CMAKE_C_COMPILER        /opt/cross-pi-gcc/bin/arm-linux-gnueabihf-gcc)
# SET(CMAKE_CXX_COMPILER      /opt/cross-pi-gcc/bin/arm-linux-gnueabihf-g++)
# SET(CMAKE_FIND_ROOT_PATH    /opt/cross-pi-gcc/arm-linux-gnueabihf)
SET(CMAKE_C_COMPILER        /opt/arm-gnu-toolchain-12.2.rel1-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER      /opt/arm-gnu-toolchain-12.2.rel1-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++)
SET(CMAKE_FIND_ROOT_PATH    /opt/arm-gnu-toolchain-12.2.rel1-x86_64-aarch64-none-linux-gnu/aarch64-none-linux-gnu)

SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

