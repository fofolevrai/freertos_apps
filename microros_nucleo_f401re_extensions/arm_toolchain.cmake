include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "LwIP")

# Makefile flags
set(ARCH_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F401xE -D_TIMEVAL_DEFINE -O0 -Wall -fdata-sections -ffunction-sections")
set(ARCH_OPT_FLAGS "")

set(CMAKE_C_COMPILER /home/fofolevrai/Documents/mros_ws/firmware/freertos_apps/microros_nucleo_f401re_extensions/../../toolchain/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER /home/fofolevrai/Documents/mros_ws/firmware/freertos_apps/microros_nucleo_f401re_extensions/../../toolchain/bin/arm-none-eabi-g++)

set(CMAKE_C_FLAGS_INIT "-std=c11 ${ARCH_CPU_FLAGS} ${ARCH_OPT_FLAGS} -DCLOCK_MONOTONIC=0" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${ARCH_CPU_FLAGS} ${ARCH_OPT_FLAGS} -DCLOCK_MONOTONIC=0" CACHE STRING "" FORCE)

set(__BIG_ENDIAN__ 0)