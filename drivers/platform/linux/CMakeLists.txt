# Platform definition. LINUX_PLATFORM is the no-OS macro used in no-OS C code
# for platform selection -- notably iio/iio_app/iio_app.c, which switches the
# iiod transport from UART to a TCP socket when it is defined.
target_compile_definitions(no-os PUBLIC -DLINUX_PLATFORM)

# no-OS linux platform drivers (one source per Kconfig symbol). These sit on the
# user-space kernel interfaces: spidev, the GPIO character device (GPIO_V2_*
# ioctls), i2c-dev and termios.
no_os_sources_ifdef(CONFIG_GPIO_LINUX ${CMAKE_CURRENT_SOURCE_DIR}/linux_gpio.c)
no_os_sources_ifdef(CONFIG_I2C_LINUX ${CMAKE_CURRENT_SOURCE_DIR}/linux_i2c.c)
no_os_sources_ifdef(CONFIG_SPI_LINUX ${CMAKE_CURRENT_SOURCE_DIR}/linux_spi.c)
no_os_sources_ifdef(CONFIG_UART_LINUX ${CMAKE_CURRENT_SOURCE_DIR}/linux_uart.c)
no_os_sources_ifdef(CONFIG_TIMER_LINUX ${CMAKE_CURRENT_SOURCE_DIR}/linux_timer.c)

# Delay helper is always built for the linux platform.
target_sources(no-os PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/linux_delay.c)

# linux_axi_io.c is deliberately not built: it mmap()s an FPGA AXI address range
# via /dev/mem and only applies to SoC/FPGA targets, not to a general-purpose
# Linux host such as a Raspberry Pi.

target_include_directories(no-os PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
