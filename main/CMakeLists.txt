set(srcs
        "main.c"
)

set(requires "pca9685" "esp_timer")

idf_component_register(SRCS "${srcs}"
        INCLUDE_DIRS "."
        REQUIRES "${requires}")
