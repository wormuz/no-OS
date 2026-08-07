message(STATUS "Linux Platform")

# The linux platform is the only *native* no-OS platform: the application is an
# ordinary user-space program built for the host it runs on (e.g. a Raspberry Pi
# 5 compiling for itself), talking to the hardware through spidev, the GPIO
# character device, i2c-dev and termios.
#
# That makes this toolchain file structurally different from every other one in
# drivers/platform/*/toolchain.cmake:
#   - No CMAKE_SYSTEM_NAME/CMAKE_SYSTEM_PROCESSOR: setting either would put CMake
#     into cross-compiling mode, which is wrong here and breaks find_package().
#   - No -mcpu/-mthumb and no arm-none-eabi-* tools: the host compiler and the
#     host libc are the correct ones.
#   - No nosys.specs, no linker script, no startup files: there is an OS.
#
# Cross-compiling from a workstation to a Pi is still possible by passing
# -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc (and friends) on the command line;
# the values below are only defaults and do not override an explicit choice.

if(NOT CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER gcc)
endif()
if(NOT CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER g++)
endif()
if(NOT CMAKE_ASM_COMPILER)
    set(CMAKE_ASM_COMPILER gcc)
endif()

find_program(CMAKE_SIZE NAMES size DOC "Path to the size utility")
find_program(CMAKE_OBJCOPY NAMES objcopy DOC "Path to the objcopy utility")

# Common flags for all build types. -ffunction-sections/-fdata-sections pair
# with the --gc-sections the root CMakeLists adds. _GNU_SOURCE is required for
# the Linux-specific uAPI the platform drivers use (GPIO_V2_* ioctls in
# <linux/gpio.h>, spidev, termios extensions).
set(COMMON_C_FLAGS "-ffunction-sections -fdata-sections -D_GNU_SOURCE")
set(CMAKE_C_FLAGS "${COMMON_C_FLAGS} -std=gnu11 -MD" CACHE STRING "C compiler flags" FORCE)
set(CMAKE_CXX_FLAGS "${COMMON_C_FLAGS} -MD" CACHE STRING "C++ compiler flags" FORCE)
set(CMAKE_ASM_FLAGS "" CACHE STRING "ASM compiler flags" FORCE)

# Debug build flags - Full debug info, no optimization
set(CMAKE_C_FLAGS_DEBUG "-g3 -O0 -DDEBUG" CACHE STRING "C compiler flags for Debug" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0 -DDEBUG" CACHE STRING "C++ compiler flags for Debug" FORCE)
set(CMAKE_ASM_FLAGS_DEBUG "-g3" CACHE STRING "ASM compiler flags for Debug" FORCE)

# Release build flags - Optimize for speed, disable assertions
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "C compiler flags for Release" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "C++ compiler flags for Release" FORCE)
set(CMAKE_ASM_FLAGS_RELEASE "" CACHE STRING "ASM compiler flags for Release" FORCE)

# RelWithDebInfo build flags - Optimize with debug info
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG" CACHE STRING "C compiler flags for RelWithDebInfo" FORCE)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG" CACHE STRING "C++ compiler flags for RelWithDebInfo" FORCE)
set(CMAKE_ASM_FLAGS_RELWITHDEBINFO "-g" CACHE STRING "ASM compiler flags for RelWithDebInfo" FORCE)

# MinSizeRel build flags - Optimize for size
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG" CACHE STRING "C compiler flags for MinSizeRel" FORCE)
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG" CACHE STRING "C++ compiler flags for MinSizeRel" FORCE)
set(CMAKE_ASM_FLAGS_MINSIZEREL "" CACHE STRING "ASM compiler flags for MinSizeRel" FORCE)

set(CMAKE_EXE_LINKER_FLAGS "" CACHE STRING "Linker flags" FORCE)

# There is no debug probe: the binary is executed directly on the target, so the
# flash/erase/debug targets do not apply. An empty PROBE makes add_flash_target()
# (cmake/FlashTools.cmake) skip them.
set(PROBE "" CACHE STRING "Debug probe (unused on the linux platform)" FORCE)
