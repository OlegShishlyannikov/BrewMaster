cmake_minimum_required(VERSION 3.6)

project(bender_controller)

set(PROJECT_NAME_CAP "${PROJECT_NAME}")
string(TOUPPER "${PROJECT_NAME_CAP}" PROJECT_NAME_CAP)

enable_language(ASM)
enable_language(C)
enable_language(CXX)

set(CMAKE_SYSTEM_NAME "Generic" CACHE INTERNAL "CMake system name")
set(CMAKE_SYSTEM_PROCESSOR "ARM-cortex-m3" CACHE INTERNAL "CMake processor")
set(CMAKE_VERBOSE_MAKEFILE OFF CACHE INTERNAL "CMake verbose makefile")

set(CMAKE_ASM_COMPILER "/usr/bin/arm-none-eabi-as" CACHE INTERNAL "CMake asm compiler")
set(CMAKE_C_COMPILER "/usr/bin/arm-none-eabi-gcc" CACHE INTERNAL "CMake c compiler")
set(CMAKE_CXX_COMPILER "/usr/bin/arm-none-eabi-g++" CACHE INTERNAL "CMake cxx compiler")
set(CMAKE_OBJCOPY_UTIL "/usr/bin/arm-none-eabi-objcopy" CACHE INTERNAL "CMake objcopy util")
set(CMAKE_OBJDUMP_UTIL "/usr/bin/arm-none-eabi-objdump" CACHE INTERNAL "CMake objdump util")
set(CMAKE_SIZE_UTIL "/usr/bin/arm-none-eabi-size" CACHE INTERNAL "CMake size util")
set(STLINK_INFO_UTIL "/usr/local/bin/st-info" CACHE INTERNAL "Stlink info util")
set(STLINK_FLASH_UTIL "/usr/local/bin/st-flash" CACHE INTERNAL "Stlink flash util")
set(CMAKE_GDB_BACKEND "/usr/bin/arm-none-eabi-gdb" CACHE INTERNAL "CMake gdb backend")
set(CMAKE_GDB_SERVER "/usr/local/bin/st-util" CACHE INTERNAL "CMake gdb server")
set(CMAKE_GDB_FRONTEND "/usr/bin/cgdb" CACHE INTERNAL "CMake gdb frontend")

message("CMake ASM compiler is ${CMAKE_ASM_COMPILER}")
message("CMake C compiler is ${CMAKE_C_COMPILER}")
message("CMake CXX compiler is ${CMAKE_CXX_COMPILER}")

set(CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m3 -D${PROJECT_NAME_CAP}_APP" CACHE INTERNAL "CMake asm flags")
set(CMAKE_C_FLAGS "-O0 -g -pipe -mthumb -mcpu=cortex-m3 -std=gnu11 -ffunction-sections -fomit-frame-pointer -fdata-sections -fno-builtin -Wl,--gc-sections -D${PROJECT_NAME_CAP}_APP" CACHE INTERNAL "CMake c flags")
set(CMAKE_CXX_FLAGS "-O0 -g -pipe -mthumb -mcpu=cortex-m3 -std=gnu++20 -flto -ffunction-sections -fomit-frame-pointer -fdata-sections -fno-builtin -fexceptions -Wl,--gc-sections -fconcepts-diagnostics-depth=4 -D${PROJECT_NAME_CAP}_APP" CACHE INTERNAL "CMake cxx flags")
set(CMAKE_EXE_LINKER_FLAGS "-O0 -g -pipe -mthumb -mcpu=cortex-m3 -flto -ffunction-sections -fomit-frame-pointer -fdata-sections -fno-builtin -fexceptions -Wl,--gc-sections -Wl,--no-wchar-size-warning --specs=nano.specs --specs=nosys.specs -lc -lnosys -Wl,-Map,${CMAKE_PROJECT_NAME}.map  -u _printf_float -u _scanf_float -D${PROJECT_NAME_CAP}_APP" CACHE INTERNAL "CMake exe linker flags")

set(C_SYSTEM_HEADERS_PATH "/usr/arm-none-eabi/include/")
set(CXX_SYSTEM_HEADERS_PATH "/usr/arm-none-eabi/include/c++/10.2.0/")

set(LINKER_SCRIPT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ldscripts)
set(LDMEM_SCRIPT ${LINKER_SCRIPT_DIR}/mem.ld)
set(LDLIB_SCRIPT ${LINKER_SCRIPT_DIR}/libs.ld)
set(LDSECTIONS_SCRIPT ${LINKER_SCRIPT_DIR}/sections.ld)

set(CMSIS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmsis" CACHE INTERNAL "CMSIS path")
set(SPL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/spl" CACHE INTERNAL "SPL path")
set(RTOS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/rtos" CACHE INTERNAL "RTOS path")

set(CMSIS_INC_DIR "${CMSIS_PATH}/inc" CACHE INTERNAL "CMSIS include path")
set(SPL_INC_DIR "${SPL_PATH}/inc" CACHE INTERNAL "SPL include path")
set(RTOS_INC_DIR "${RTOS_PATH}/inc" CACHE INTERNAL "RTOS include path")

file(GLOB CMSIS_SOURCES "${CMSIS_PATH}/src/*.c" "${CMSIS_PATH}/src/*.cpp" "${CMSIS_PATH}/src/*.cxx" "${CMSIS_PATH}/src/*.s" CACHE INTERNAL "CMSIS sources")
file(GLOB SPL_SOURCES "${SPL_PATH}/src/*.c" "${SPL_PATH}/src/*.cpp" "${SPL_PATH}/src/*.cxx" "${SPL_PATH}/src/*.s" CACHE INTERNAL "SPL sources")
file(GLOB RTOS_SOURCES "${RTOS_PATH}/src/*.c" "${RTOS_PATH}/src/*.cpp" "${RTOS_PATH}/src/*.cxx" "${RTOS_PATH}/src/*.s" CACHE INTERNAL "RTOS sources")

set(PROJECT_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inc" CACHE INTERNAL "Project include path")
file(GLOB PROJECT_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.s" CACHE INTERNAL "Project sources")

file(GLOB PROJECT_BOARD_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/boards/${CMAKE_PROJECT_NAME}/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/boards/${CMAKE_PROJECT_NAME}/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/boards/${CMAKE_PROJECT_NAME}/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/boards/${CMAKE_PROJECT_NAME}/*.s"
  CACHE INTERNAL "Project board sources")

file(GLOB PROJECT_RTOS_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/rtos/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/rtos/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/rtos/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/rtos/*.s" CACHE INTERNAL "Project rtos sources")

file(GLOB PROJECT_FILEIO_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fio/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fio/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fio/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fio/*.s" CACHE INTERNAL "Project fileio sources")

file(GLOB PROJECT_FONTS_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fonts/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fonts/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fonts/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fonts/*.s" CACHE INTERNAL "Project fonts sources")

file(GLOB PROJECT_LFS_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/lfs/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/lfs/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/lfs/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/lfs/*.s" CACHE INTERNAL "Project LFS sources")

file(GLOB PROJECT_ARGPARSE_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/argparse/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/argparse/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/argparse/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/argparse/*.s" CACHE INTERNAL "Project argparse sources")

file(GLOB PROJECT_JSON_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/json/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/json/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/json/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/json/*.s" CACHE INTERNAL "Project JSON sources")

file(GLOB PROJECT_HASH_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/hash/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/hash/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/hash/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/hash/*.s" CACHE INTERNAL "Project JSON sources")

file(GLOB PROJECT_SHELL_APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/shell/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/shell/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/shell/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/shell/*.s" CACHE INTERNAL "Project shell app sources")

file(GLOB PROJECT_CLEAR_APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/clear/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/clear/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/clear/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/clear/*.s" CACHE INTERNAL "Project clear app sources")

file(GLOB PROJECT_COWSAY_APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/cowsay/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/cowsay/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/cowsay/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/cowsay/*.s" CACHE INTERNAL "Project cowsay app sources")

file(GLOB PROJECT_BREWERY_APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/brewery/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/brewery/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/brewery/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/brewery/*.s" CACHE INTERNAL "Project brewery app sources")

file(GLOB PROJECT_BENDER_APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/bender/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/bender/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/bender/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/bender/*.s" CACHE INTERNAL "Project bender app sources")

file(GLOB PROJECT_DEBUG_APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/debug/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/debug/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/debug/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/debug/*.s" CACHE INTERNAL "Project debug app sources")

file(GLOB PROJECT_REBOOT_APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/reboot/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/reboot/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/reboot/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/reboot/*.s" CACHE INTERNAL "Project reboot app sources")

file(GLOB PROJECT_MODBUS_APP_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/modbus/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/modbus/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/modbus/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/modbus/*.s" CACHE INTERNAL "Project modbus app sources")

set(HSE_VALUE 8000000)
set(MCU_DEVICE STM32F10X_XL)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D${MCU_DEVICE} -DHSE_VALUE=${HSE_VALUE} -DUSE_STDPERIPH_DRIVER" CACHE INTERNAL "CMake c flags")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${MCU_DEVICE} -DHSE_VALUE=${HSE_VALUE} -DUSE_STDPERIPH_DRIVER" CACHE INTERNAL "CMake cxx flags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -D${MCU_DEVICE} -DHSE_VALUE=${HSE_VALUE} -DUSE_STDPERIPH_DRIVER -T${LDMEM_SCRIPT} -T${LDLIB_SCRIPT} -T${LDSECTIONS_SCRIPT}" CACHE INTERNAL "CMake exe linker flags")

include_directories(${PROJECT_INC_DIR} ${STARTUP_INC_DIR} ${CMSIS_INC_DIR} ${SPL_INC_DIR} ${RTOS_INC_DIR})
add_executable(${CMAKE_PROJECT_NAME}.elf
  ${PROJECT_SOURCES}
  ${PROJECT_BOARD_SOURCES}
  ${PROJECT_RTOS_SOURCES}
  ${PROJECT_FILEIO_SOURCES}
  ${PROJECT_FONTS_SOURCES}
  ${PROJECT_LFS_SOURCES}
  ${CMSIS_SOURCES}
  ${SPL_SOURCES}
  ${RTOS_SOURCES}
  ${PROJECT_ARGPARSE_SOURCES}
  ${PROJECT_JSON_SOURCES}
  ${PROJECT_HASH_SOURCES}
  ${PROJECT_SHELL_APP_SOURCES}
  ${PROJECT_CLEAR_APP_SOURCES}
  ${PROJECT_COWSAY_APP_SOURCES}
  ${PROJECT_BREWERY_APP_SOURCES}
  ${PROJECT_BENDER_APP_SOURCES}
  ${PROJECT_DEBUG_APP_SOURCES}
  ${PROJECT_REBOOT_APP_SOURCES}
  ${PROJECT_MODBUS_APP_SOURCES})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/clang_complete.in ${CMAKE_CURRENT_SOURCE_DIR}/.clang_complete)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/clang_complete_c.in ${CMAKE_CURRENT_SOURCE_DIR}/src/lfs/.clang_complete)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/clang_complete_c.in ${CMAKE_CURRENT_SOURCE_DIR}/inc/lfs/.clang_complete)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY_UTIL} ARGS -Oihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY_UTIL} ARGS -Obinary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_SIZE_UTIL} ${CMAKE_PROJECT_NAME}.elf)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD COMMAND ${CMAKE_SIZE_UTIL} ${CMAKE_PROJECT_NAME}.hex)

add_custom_target(flash COMMAND ${STLINK_FLASH_UTIL} write ${CMAKE_PROJECT_NAME}.bin 0x08000000)
add_custom_target(dump COMMAND ${CMAKE_OBJDUMP_UTIL} -t ${CMAKE_PROJECT_NAME}.elf)
add_custom_target(size COMMAND ${CMAKE_SIZE_UTIL} ${CMAKE_PROJECT_NAME}.elf)
