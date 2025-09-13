SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR arm)

SET(HOSTCC gcc)
SET(CMAKE_C_COMPILER        /opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER      /opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-g++)
SET(CMAKE_Fortran_COMPILER  /opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gfortran) # Compiling OpenBLAS requires Fortran compiler
SET(CMAKE_FIND_ROOT_PATH    /opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-linux-gnueabihf/arm-none-linux-gnueabihf)
SET(TARGET ARMV7)   # ARMV7 is the target architecture for Raspberry Pi 4, must be set for OpenBLAS

SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

