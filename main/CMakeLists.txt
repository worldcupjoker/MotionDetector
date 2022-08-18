idf_component_register(SRCS "MotionDetector.c" "MPU6050.c"
                    INCLUDE_DIRS "")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
