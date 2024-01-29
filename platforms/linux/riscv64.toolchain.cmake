set(CMAKE_SYSTEM_PROCESSOR riscv64)
set(GCC_COMPILER_VERSION "" CACHE STRING "GCC Compiler version")
set(GNU_MACHINE "riscv64-linux-gnu" CACHE STRING "GNU compiler triple")

include("${CMAKE_CURRENT_LIST_DIR}/riscv.toolchain.cmake")
message(STATUS "####### debug cmake #######")
