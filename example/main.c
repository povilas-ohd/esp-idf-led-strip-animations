/*
 * main.c
 * Example application using LED animation library
 *
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_animations.h"

#define EXAMPLE_GPIO_NUM      6
#define EXAMPLE_LED_NUMBERS   15
#define RMT_RESOLUTION_HZ     10000000
#define EXAMPLE_FRAME_DURATION_MS 20

static const char *TAG = "main";

void app_main(void)
{
    // Initialize LED animation library
    led_animations_config_t config = {
        .gpio_num = EXAMPLE_GPIO_NUM,
        .led_count = EXAMPLE_LED_NUMBERS,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4
    };
    if (led_animations_init(&config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED animations");
        return;
    }

    // Demo: Send some commands
    led_command_t cmd;

    // Set red, half brightness
    cmd = (led_command_t){
        .type = LED_CMD_SET_COLOR,
        .red = 255,
        .green = 0,
        .blue = 0,
        .global_brightness = 128
    };
    send_led_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Set blue, full brightness
    cmd = (led_command_t){
        .type = LED_CMD_SET_COLOR,
        .red = 0,
        .green = 0,
        .blue = 255,
        .global_brightness = 255
    };
    send_led_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Breathe red, linear, dim range (0 to 50), normal speed, 3 cycles
    cmd = (led_command_t){
        .type = LED_CMD_BREATHE,
        .red = 255,
        .green = 0,
        .blue = 0,
        .min_brightness = 0,
        .max_brightness = 50,
        .duration_ms = 2000,
        .speed = 1.0f,
        .breathe_mode = BREATHE_LINEAR,
        .global_brightness = 255,
        .repeat_count = 3
    };
    send_led_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(6000));

    // Breathe green, sine, wider range (5 to 100), faster speed, infinite loops
    cmd = (led_command_t){
        .type = LED_CMD_BREATHE,
        .red = 0,
        .green = 255,
        .blue = 0,
        .min_brightness = 5,
        .max_brightness = 100,
        .duration_ms = 2000,
        .speed = 2.0f,
        .breathe_mode = BREATHE_SINE,
        .global_brightness = 255,
        .repeat_count = 0
    };
    send_led_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(6000));

    // Viper blue, normal speed, half brightness, 2 cycles
    cmd = (led_command_t){
        .type = LED_CMD_VIPER,
        .red = 0,
        .green = 0,
        .blue = 255,
        .tail_length = 5,
        .speed = 1.0f,
        .global_brightness = 128,
        .repeat_count = 2
    };
    send_led_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(6000));

    // Viper red, faster speed, full brightness, infinite loops
    cmd = (led_command_t){
        .type = LED_CMD_VIPER,
        .red = 255,
        .green = 0,
        .blue = 0,
        .tail_length = 5,
        .speed = 2.0f,
        .global_brightness = 255,
        .repeat_count = 0
    };
    send_led_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(6000));

    // Stop (turn off)
    cmd = (led_command_t){
        .type = LED_CMD_STOP
    };
    send_led_command(cmd);

    ESP_LOGI(TAG, "Demo complete. Use send_led_command() from other tasks.");
}