set(srcs
        "main.cpp"
)

set(requires "pca9685_servo")

idf_component_register(SRCS "${srcs}"
        INCLUDE_DIRS "."
        REQUIRES "${requires}")
