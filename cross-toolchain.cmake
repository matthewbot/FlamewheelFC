include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
cmake_force_c_compiler(arm-none-eabi-gcc GNU)
cmake_force_cxx_compiler(arm-none-eabi-g++ GNU)
set(CMAKE_CXX_FLAGS "-fno-exceptions" CACHE STRING "Flags used by the compiler" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-nostdlib -T ${CMAKE_SOURCE_DIR}/linker.ld" CACHE STRING "Flags used by the linker" FORCE)
