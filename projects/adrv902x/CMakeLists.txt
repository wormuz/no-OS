add_executable(adrv902x)

target_sources(adrv902x PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/common_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/clkgen_routines.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/hal/no_os_platform.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/${PLATFORM}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/${PLATFORM}/parameters.c
)

if(CONFIG_ADRV902X_BASIC_EXAMPLE)
    target_sources(adrv902x PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/basic_example/basic_example.c
    )
    target_include_directories(adrv902x PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/basic_example
    )
endif()

if(CONFIG_ADRV902X_DMA_EXAMPLE)
    target_sources(adrv902x PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/dma_example/dma_example.c
    )
    target_include_directories(adrv902x PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/dma_example
    )
endif()

if(CONFIG_ADRV902X_IIO_EXAMPLE)
    target_sources(adrv902x PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/iio_example/iio_example.c
    )
    target_include_directories(adrv902x PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/iio_example
    )
endif()

target_include_directories(adrv902x PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/firmware/JESD204B_no_ORx
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/firmware/JESD204B_ORx
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/firmware/JESD204C_ORx
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/${PLATFORM}
)

# The madura driver (in no-os library) includes the project-supplied no_os_platform.h
# which defines adrv9025_hal_cfg. It must be on the no-os include path.
target_include_directories(no-os PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/hal
)

# Cap the ADI SPI access batch to fit the Xilinx AXI Quad SPI 16-byte TX FIFO.
# The madura HAL packs register writes into a single CS-held SPI transfer
# (ADI_COMMON_HAL_SPIARRAYSIZE bytes; default 256 -> 255-byte bursts). On the
# no-OS polled AXI Quad SPI path (e.g. VCU118/MicroBlaze) a transfer larger than
# the 16-byte TX FIFO underruns during the un-inhibited BSP refill, corrupting
# the ADRV9025 stream/firmware DMA load so the on-chip ARM never boots. 15 bytes
# = 5 register accesses x 3 bytes, which fits the FIFO with no mid-CS refill.
# MicroBlaze (vcu118) only -- the ZynqMP (zcu102) PS SPI has no 16-byte FIFO
# limit, so it keeps the faster default batch size. The macro is consumed inside
# the madura driver (compiled into the no-os target), so it must be defined here
# rather than in app_config.h, which those driver files do not include.
if(XILINX_ARCH MATCHES "sys_mb")
    target_compile_definitions(no-os PUBLIC ADI_COMMON_HAL_SPIARRAYSIZE=15)
endif()

config_platform_sdk(adrv902x)
add_flash_target(adrv902x)
post_build_config(adrv902x)
target_link_libraries(adrv902x no-os)
