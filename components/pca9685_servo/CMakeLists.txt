idf_component_register(SRC_DIRS "src"
        INCLUDE_DIRS "include"
        REQUIRES "pca9685" "esp_timer"
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_WARN")