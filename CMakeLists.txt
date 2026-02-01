cmake_minimum_required(VERSION 3.15)

project( pid2 )

add_compile_options( -D_WINDOWS_ )

add_compile_options( -Wall -Wextra -Wpedantic )

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")

set( FREERTOS_KERNEL_PATH "../FreeRTOS/FreeRTOS/Source" )
set( FREERTOS_PLUS_TRACE_PATH "../FreeRTOS/FreeRTOS-Plus/Source/FreeRTOS-Plus-Trace" )

# Add the freertos_config for FreeRTOS-Kernel
add_library( freertos_config INTERFACE )

target_include_directories( freertos_config
    INTERFACE
        ./
        ./Trace_Recorder_Configuration
        ${FREERTOS_PLUS_TRACE_PATH}/include
        ${FREERTOS_PLUS_TRACE_PATH}/kernelports/FreeRTOS/include
)

# Select the heap port
set( FREERTOS_HEAP "3" CACHE STRING "" FORCE)

# Select the native compile PORT
set( FREERTOS_PORT "GCC_POSIX" CACHE STRING "" FORCE)

# Adding the FreeRTOS-Kernel subdirectory
add_subdirectory( ${FREERTOS_KERNEL_PATH} ${CMAKE_CURRENT_BINARY_DIR}/FreeRTOS-Kernel )


file( GLOB FREERTOS_PLUS_TRACE_SOURCES ${FREERTOS_PLUS_TRACE_PATH}/*.c ${FREERTOS_PLUS_TRACE_PATH}/kernelports/FreeRTOS/*.c )

add_executable( pid2
                main.c
                controlTask.c
                serverTask.c
                plant.c 
       )

target_include_directories( pid2
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../Common/include
        #${FREERTOS_PLUS_TRACE_PATH}/Include
        #${FREERTOS_PLUS_TRACE_PATH}/streamports/File/include
        #${FREERTOS_PLUS_TRACE_PATH}/streamports/File/config
)


target_link_libraries( pid2 freertos_kernel freertos_config )
