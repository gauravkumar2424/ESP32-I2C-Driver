# Register the main component with ESP-IDF
idf_component_register(
    SRCS 
        "main.c"
        "i2c_low_level.c"
    INCLUDE_DIRS 
        "."
)
