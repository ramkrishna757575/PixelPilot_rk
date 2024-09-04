cmake_minimum_required(VERSION 3.1)

project(pixelpilot VERSION 0.12)

set(CMAKE_CXX_STANDARD 17)

add_compile_options("-Wno-address-of-packed-member")

set(SOURCE_FILES 
        src/drm.h
        src/drm.c
        src/osd.h
        src/osd.c
        src/rtp.h
        src/rtp.c
        src/mavlink.h
        src/mavlink.c
        src/main.cpp
        src/main.h
        src/scheduling_helper.hpp
        src/gstrtpreceiver.cpp
        src/gstrtpreceiver.h)

include_directories("/usr/include/libdrm" "/usr/include/cairo")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} rockchip_mpp pthread drm m cairo)

# Embed gstreamer
find_package(PkgConfig REQUIRED)
pkg_search_module(GST REQUIRED
        gstreamer-1.0>=1.4
        gstreamer-app-1.0>=1.4
)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
target_link_libraries(${PROJECT_NAME} PkgConfig::gstreamer PkgConfig::gstreamer-app)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(
    CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -Werror -fsanitize=undefined -fsanitize=address"
  )
  target_link_options(${PROJECT_NAME}
    BEFORE PUBLIC -fsanitize=undefined PUBLIC -fsanitize=address
  )
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}")

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Define paths for HotPlugWifiAdapters setup
set(SCRIPT_SOURCE "${CMAKE_SOURCE_DIR}/wifi-adapters/autoload-wfb-nics.sh")
set(SCRIPT_DEST "/home/radxa/scripts/autoload-wfb-nics.sh")
set(SERVICE_SOURCE "${CMAKE_SOURCE_DIR}/wifi-adapters/init-nics.service")
set(SERVICE_DEST "/etc/systemd/system/init-nics.service")
set(UDEV_RULES_DIR "/etc/udev/rules.d")

# Copy the script to the target directory
configure_file(${SCRIPT_SOURCE} ${SCRIPT_DEST} COPYONLY)

# Set executable permissions for the script
add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND sudo chmod +x ${SCRIPT_DEST}
    COMMENT "Setting executable permissions for autoload-wfb-nics.sh"
)

# Copy the service file
configure_file(${SERVICE_SOURCE} ${SERVICE_DEST} COPYONLY)

# Enable the service
add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND sudo systemctl enable init-nics.service
    COMMENT "Enabling init-nics.service"
)

# Copy udev rules
configure_file(${CMAKE_SOURCE_DIR}/wifi-adapters/98-custom-wifi.rules ${UDEV_RULES_DIR}/98-custom-wifi.rules COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/wifi-adapters/99-custom-wifi.rules ${UDEV_RULES_DIR}/99-custom-wifi.rules COPYONLY)

# Optionally, reboot the system
add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND sudo reboot
    COMMENT "Rebooting the system"
)

# Add a custom target to trigger the above commands
add_custom_target(setup ALL
    DEPENDS ${SCRIPT_DEST} ${SERVICE_DEST}
    COMMENT "Running setup for Hot Plug Wifi Adapters"
)