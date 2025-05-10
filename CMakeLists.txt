cmake_minimum_required(VERSION 3.16)

# Include ESP-IDF project setup
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(led_strip_animations)

# Register the library as a component
idf_component_register(SRCS "src/led_animations.c"
                      INCLUDE_DIRS "include"
                      REQUIRES driver freertos)

# Add the example as a component
add_subdirectory(example)