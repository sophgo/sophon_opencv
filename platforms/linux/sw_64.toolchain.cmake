set(CMAKE_SYSTEM_PROCESSOR sw_64)
set(GCC_COMPILER_VERSION "" CACHE STRING "GCC Compiler version")
set(GNU_MACHINE "sw_64-sunway-linux-gnu" CACHE STRING "GNU compiler triple")

set(CROSS_COMPILE ${CROSS_COMPILE_PATH}/usr/bin/sw_64-sunway-linux-gnu- )

set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++)

SET(CMAKE_SYSROOT ${CROSS_COMPILE_PATH})
include("${CMAKE_CURRENT_LIST_DIR}/sw.toolchain.cmake")
message(STATUS "####### debug cmake #######")
