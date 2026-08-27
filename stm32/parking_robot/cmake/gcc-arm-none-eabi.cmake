set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Set ARM_NONE_EABI_ROOT to the toolchain prefix directory (the directory
# containing bin/), or place arm-none-eabi-* on PATH.  Never fall back to a
# host compiler: a successful host link must not be mistaken for flashable
# firmware.
set(ARM_NONE_EABI_ROOT "$ENV{ARM_NONE_EABI_ROOT}" CACHE PATH
    "GNU Arm Embedded toolchain prefix")
set(_ARM_TOOLCHAIN_HINTS)
if(ARM_NONE_EABI_ROOT)
    list(APPEND _ARM_TOOLCHAIN_HINTS "${ARM_NONE_EABI_ROOT}/bin")
endif()

find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc
    HINTS ${_ARM_TOOLCHAIN_HINTS})
find_program(ARM_NONE_EABI_OBJCOPY arm-none-eabi-objcopy
    HINTS ${_ARM_TOOLCHAIN_HINTS})
find_program(ARM_NONE_EABI_SIZE arm-none-eabi-size
    HINTS ${_ARM_TOOLCHAIN_HINTS})
find_program(ARM_NONE_EABI_AR arm-none-eabi-ar
    HINTS ${_ARM_TOOLCHAIN_HINTS})
find_program(ARM_NONE_EABI_RANLIB arm-none-eabi-ranlib
    HINTS ${_ARM_TOOLCHAIN_HINTS})

if(NOT ARM_NONE_EABI_GCC OR NOT ARM_NONE_EABI_OBJCOPY OR
   NOT ARM_NONE_EABI_SIZE OR NOT ARM_NONE_EABI_AR OR
   NOT ARM_NONE_EABI_RANLIB)
    message(FATAL_ERROR
        "GNU Arm Embedded toolchain not found; set ARM_NONE_EABI_ROOT")
endif()

set(CMAKE_C_COMPILER "${ARM_NONE_EABI_GCC}" CACHE FILEPATH "" FORCE)
set(CMAKE_ASM_COMPILER "${ARM_NONE_EABI_GCC}" CACHE FILEPATH "" FORCE)
set(CMAKE_OBJCOPY "${ARM_NONE_EABI_OBJCOPY}" CACHE FILEPATH "" FORCE)
set(CMAKE_SIZE "${ARM_NONE_EABI_SIZE}" CACHE FILEPATH "" FORCE)
set(CMAKE_AR "${ARM_NONE_EABI_AR}" CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB "${ARM_NONE_EABI_RANLIB}" CACHE FILEPATH "" FORCE)

# Debian splits newlib from GCC and its package is not relocatable by default:
# GCC still searches /usr/include/newlib and /usr/lib/arm-none-eabi/newlib.
# When such packages are unpacked below ARM_NONE_EABI_ROOT, expose the relocated
# runtime to the project.  Official GNU Arm bundles already carry a relocatable
# arm-none-eabi sysroot and do not enter this branch.
if(ARM_NONE_EABI_ROOT AND
   EXISTS "${ARM_NONE_EABI_ROOT}/include/newlib/string.h" AND
   EXISTS "${ARM_NONE_EABI_ROOT}/lib/arm-none-eabi/newlib/nano.specs")
    set(ARM_NONE_EABI_NEWLIB_PREFIX
        "${ARM_NONE_EABI_ROOT}/lib/arm-none-eabi/newlib"
        CACHE PATH "Relocated Debian newlib prefix" FORCE)
    set(ARM_NONE_EABI_NEWLIB_INCLUDE
        "${ARM_NONE_EABI_ROOT}/include/newlib"
        CACHE PATH "Relocated Debian newlib headers" FORCE)
else()
    set(ARM_NONE_EABI_NEWLIB_PREFIX "" CACHE PATH
        "Relocated Debian newlib prefix" FORCE)
    set(ARM_NONE_EABI_NEWLIB_INCLUDE "" CACHE PATH
        "Relocated Debian newlib headers" FORCE)
endif()

set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
