no_os_sources_ifdef(CONFIG_LWIP ${CMAKE_CURRENT_SOURCE_DIR}/tcp_socket.c)

no_os_sources_ifdef(CONFIG_LWIP ${CMAKE_CURRENT_SOURCE_DIR}/lwip_raw_socket/lwip_socket.c)
no_os_include_dir_ifdef(CONFIG_LWIP ${CMAKE_CURRENT_SOURCE_DIR}/lwip_raw_socket)
no_os_include_dir_ifdef(CONFIG_LWIP ${CMAKE_CURRENT_SOURCE_DIR})

if(${CONFIG_LWIP})
        add_subdirectory(lwip_raw_socket/netdevs)
endif()

# BSD-socket backend for the linux platform. iio/iio_app/iio_app.c uses
# linux_net and tcp_socket_init() to serve iiod over TCP when LINUX_PLATFORM is
# defined, so both sources are needed for an IIO application to link there.
# tcp_socket.c is shared with lwIP, hence the guard against adding it twice.
if(CONFIG_LINUX_SOCKET AND NOT CONFIG_LWIP)
        target_sources(no-os PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tcp_socket.c)
        target_include_directories(no-os PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
        # tcp_socket.h pulls in mbedtls/ssl.h unless this is defined, and mbedTLS
        # is not part of the build. It is PUBLIC because the macro also removes a
        # member of struct tcp_socket_desc, so no-os and the application have to
        # agree on it. Projects using the lwIP or ESP8266 paths define this
        # themselves; the linux backend has no project-side hook, so it is set
        # here alongside the source it belongs to.
        target_compile_definitions(no-os PUBLIC DISABLE_SECURE_SOCKET)
endif()

no_os_sources_ifdef(CONFIG_LINUX_SOCKET ${CMAKE_CURRENT_SOURCE_DIR}/linux_socket/linux_socket.c)
no_os_include_dir_ifdef(CONFIG_LINUX_SOCKET ${CMAKE_CURRENT_SOURCE_DIR}/linux_socket)
