# Register the library as a component
idf_component_register(SRCS "src/led_animations.c"
                      INCLUDE_DIRS "include"
                      REQUIRES driver freertos)

# Add the example as a component (only if building as a standalone project)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_subdirectory(example)
endif()