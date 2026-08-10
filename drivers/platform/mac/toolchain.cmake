message(STATUS "macOS host platform")

# Host build on macOS.
# CMake will discover the host system compiler automatically (AppleClang or
# a Homebrew-installed GCC/Clang).  No cross-compilation toolchain is needed.
set(CMAKE_SYSTEM_NAME Darwin)
