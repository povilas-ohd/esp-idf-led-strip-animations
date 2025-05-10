/*
 * led_animations.h
 * LED animation library for WS2812 strips using ESP32 RMT peripheral
 *
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef LED_ANIMATIONS_H
#define LED_ANIMATIONS_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"

// Animation command types
typedef enum {
    LED_CMD_SET_COLOR, // Set all LEDs to a solid color
    LED_CMD_BREATHE,   // Breathing effect (fade in/out)
    LED_CMD_VIPER,     // Moving head with fading tail
    LED_CMD_STOP       // Turn off all LEDs
} led_command_type_t;

// Breathing mode types
typedef enum {
    BREATHE_LINEAR, // Linear fade (triangular wave)
    BREATHE_SINE    // Sine wave fade
} breathe_mode_t;

// Structure for animation commands
typedef struct {
    led_command_type_t type;      // Command type
    uint8_t red;                  // RGB color
    uint8_t green;
    uint8_t blue;
    uint8_t min_brightness;       // 0-255, for breathe
    uint8_t max_brightness;       // 0-255, for breathe
    uint8_t global_brightness;    // 0-255, scales all colors (255 = full)
    float speed;                  // Speed multiplier (1.0 = normal, 2.0 = 2x fast)
    int duration_ms;              // Base duration for breathe (adjusted by speed)
    int tail_length;              // For viper
    int repeat_count;             // Number of cycles (0 = infinite)
    breathe_mode_t breathe_mode;  // Linear or sine for breathe
} led_command_t;

// Configuration structure for library initialization
typedef struct {
    int gpio_num;           // GPIO pin for LED strip
    int led_count;          // Number of LEDs in the strip
    uint32_t resolution_hz; // RMT resolution (default: 10MHz)
    size_t mem_block_symbols; // RMT memory block size
    size_t trans_queue_depth; // RMT transaction queue depth
} led_animations_config_t;

// Initialize the LED animation library
// Returns ESP_OK on success, or an error code
esp_err_t led_animations_init(const led_animations_config_t *config);

// Send a command to the LED animation task
void send_led_command(led_command_t cmd);

#endif // LED_ANIMATIONS_H