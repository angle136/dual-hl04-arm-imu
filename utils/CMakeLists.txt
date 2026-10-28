# Utils library
add_library(utils OBJECT
    utils_init.c
    ulog/SEGGER_RTT.c
    ulog/ulog.c
    ulog/printf.c
    kfifo/kfifo.c
)

target_compile_options(utils PRIVATE -O2)

# Up Channel 0 保留给文本日志；arm_imu 调试帧使用独立二进制通道。
set(_rtt_up_buffer_count 1)
if(ARM_IMU_DEBUG_RTT_ENABLE)
    math(EXPR _rtt_up_buffer_count "${ARM_IMU_DEBUG_RTT_CHANNEL} + 1")
endif()
target_compile_definitions(utils PUBLIC
    SEGGER_RTT_MAX_NUM_UP_BUFFERS=${_rtt_up_buffer_count}
)

target_include_directories(utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ulog
    ${CMAKE_CURRENT_SOURCE_DIR}/kfifo
)

target_link_libraries(utils PUBLIC stm32cubemx azrtos::threadx)
