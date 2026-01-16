# Register the library as a component
idf_component_register(SRCS "src/led_animations.c" "src/led_gpio.c"
                      INCLUDE_DIRS "include"
                      REQUIRES driver freertos led_strip)

# Add the example as a component (only if building as a standalone project)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_subdirectory(example)
endif()