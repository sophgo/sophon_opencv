if(COMMAND toolchain_save_config)
  return() # prevent recursive call
endif()

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

message(STATUS "#########CMAKE_C_COMPILER=${CMAKE_C_COMPILER} is defined")
if(NOT DEFINED CMAKE_SYSTEM_PROCESSOR)
  set(CMAKE_SYSTEM_PROCESSOR x86)
else()
  message("CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/gnu.toolchain.cmake")

if(NOT "x${GCC_COMPILER_VERSION}" STREQUAL "x")
  set(__GCC_VER_SUFFIX "-${GCC_COMPILER_VERSION}")
endif()
if(NOT DEFINED CMAKE_C_COMPILER)
  find_program(CMAKE_C_COMPILER NAMES ${GNU_MACHINE}-gcc${__GCC_VER_SUFFIX})
else()
  message(WARNING "CMAKE_C_COMPILER=${CMAKE_C_COMPILER} is defined")
endif()
message(STATUS "CMAKE_C_COMPILER=${CMAKE_C_COMPILER} is defined")
if(NOT DEFINED CMAKE_CXX_COMPILER)
  find_program(CMAKE_CXX_COMPILER NAMES ${GNU_MACHINE}-g++${__GCC_VER_SUFFIX})
else()
  message(WARNING "CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} is defined")
endif()

if(NOT DEFINED CMAKE_LINKER)
  find_program(CMAKE_LINKER NAMES ${GNU_MACHINE}-ld${__GCC_VER_SUFFIX} ${GNU_MACHINE}-ld)
else()
  message(WARNING "CMAKE_LINKER=${CMAKE_LINKER} is defined")
endif()
if(NOT DEFINED CMAKE_AR)
  find_program(CMAKE_AR NAMES ${GNU_MACHINE}-ar${__GCC_VER_SUFFIX} ${GNU_MACHINE}-ar)
else()
  message(WARNING "CMAKE_AR=${CMAKE_AR} is defined")
endif()

if(NOT DEFINED CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS           "" CACHE INTERNAL "")
  set(CMAKE_C_FLAGS             "" CACHE INTERNAL "")
  set(CMAKE_SHARED_LINKER_FLAGS "" CACHE INTERNAL "")
  set(CMAKE_MODULE_LINKER_FLAGS "" CACHE INTERNAL "")
  set(CMAKE_EXE_LINKER_FLAGS    "" CACHE INTERNAL "")

  set(CMAKE_CXX_FLAGS   "-U_FORTIFY_SOURCE ${CMAKE_CXX_FLAGS}")
  set(CMAKE_C_FLAGS     "-U_FORTIFY_SOURCE ${CMAKE_C_FLAGS}")

  set(CMAKE_CXX_FLAGS   " -lresolv  -lrt -ldl  -lpthread ${CMAKE_CXX_FLAGS}")
  set(CMAKE_C_FLAGS     " -lresolv  -lrt -ldl  -lpthread ${CMAKE_C_FLAGS} ")

  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${RISCV64_LINKER_FLAGS}")
  set(CMAKE_MODULE_LINKER_FLAGS "${RISCV64_LINKER_FLAGS} ${CMAKE_MODULE_LINKER_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS} ${RISCV64_LINKER_FLAGS}")
endif()

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX)
set(CMAKE_INCLUDE_SYSTEM_FLAG_C)
