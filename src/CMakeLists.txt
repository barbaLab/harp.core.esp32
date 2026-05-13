# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS
        src/core_registers.cpp
        src/usb_descriptors.c
        src/harp_core.cpp
        src/harp_synchronizer.cpp
        src/harp_c_app.cpp
    INCLUDE_DIRS
        inc
    REQUIRES
        driver        # uart_driver_install, uart_param_config, etc.
        esp_timer     # esp_timer_get_time()
        esp_system    # esp_restart(), esp_efuse_mac_get_default()
        tinyusb       # TinyUSB stack (espressif/esp_tinyusb managed component)
)