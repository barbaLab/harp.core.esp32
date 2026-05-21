idf_component_register(
    SRCS
        src/core_registers.cpp
        src/harp_core.cpp
        src/harp_synchronizer.cpp
        src/usb_descriptors.c
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_uart        # uart_driver_install, uart_param_config, etc.
        esp_timer     # esp_timer_get_time()
        esp_system    # esp_restart(), esp_efuse_mac_get_default()
        efuse
        esp_tinyusb   # TinyUSB stack (espressif/esp_tinyusb managed component)
)

# # ESP-IDF adds -Wno-old-style-declaration (C-only) to all compile units including
# # C++, and -fuse-cxa-atexit (C++-only) to all units including C. With -Werror these
# # become fatal. Suppress -Werror per language to work around this.
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error>
)
set_source_files_properties(
    "src/usb_descriptors.c"
    PROPERTIES
        COMPILE_FLAGS "-Wno-error"
)
