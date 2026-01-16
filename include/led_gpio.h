#ifndef LED_GPIO_H
#define LED_GPIO_H

#include <stdint.h>
#include "esp_err.h"

// Simple GPIO bit-bang driver for WS2812 LEDs (for small strips when RMT unavailable)
typedef struct led_gpio_strip* led_gpio_handle_t;

// Initialize GPIO LED strip
led_gpio_handle_t led_gpio_init(int gpio_num, int led_count);

// Set all LEDs to RGB color
esp_err_t led_gpio_set_color(led_gpio_handle_t handle, uint8_t r, uint8_t g, uint8_t b);

// Set single LED to RGB color
esp_err_t led_gpio_set_pixel(led_gpio_handle_t handle, int index, uint8_t r, uint8_t g, uint8_t b);

// Clear all LEDs
esp_err_t led_gpio_clear(led_gpio_handle_t handle);

#endif
