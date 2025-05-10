/*
 * led_animations.c
 * Implementation of LED animation library for WS2812 strips
 *
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_animations.h"

static const char *TAG = "led_animations";

// Global variables
static uint8_t *led_strip_pixels = NULL;
static int led_count = 0;
static QueueHandle_t led_command_queue = NULL;

// WS2812 timing definitions
static const rmt_symbol_word_t ws2812_zero = {
    .level0 = 1, .duration0 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000,
    .level1 = 0, .duration1 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000,
};

static const rmt_symbol_word_t ws2812_one = {
    .level0 = 1, .duration0 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000,
    .level1 = 0, .duration1 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000,
};

static const rmt_symbol_word_t ws2812_reset = {
    .level0 = 0, .duration0 = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2,
    .level1 = 0, .duration1 = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2,
};

// Encoder callback: Converts RGB data to RMT symbols
static size_t encoder_callback(const void *data, size_t data_size,
                              size_t symbols_written, size_t symbols_free,
                              rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    if (symbols_free < 8) {
        return 0;
    }
    size_t data_pos = symbols_written / 8;
    uint8_t *data_bytes = (uint8_t*)data;
    if (data_pos < data_size) {
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
            symbols[symbol_pos++] = (data_bytes[data_pos] & bitmask) ? ws2812_one : ws2812_zero;
        }
        return symbol_pos;
    } else {
        symbols[0] = ws2812_reset;
        *done = 1;
        return 1;
    }
}

// Set all LEDs to a specific GRB color with global brightness scaling
static void set_all_leds_color(rmt_channel_handle_t led_chan, rmt_encoder_handle_t encoder,
                               uint8_t red, uint8_t green, uint8_t blue, uint8_t global_brightness)
{
    float scale = global_brightness / 255.0f;
    for (int led = 0; led < led_count; led++) {
        led_strip_pixels[led * 3 + 0] = (uint8_t)(green * scale); // GRB order
        led_strip_pixels[led * 3 + 1] = (uint8_t)(red * scale);
        led_strip_pixels[led * 3 + 2] = (uint8_t)(blue * scale);
    }
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    ESP_ERROR_CHECK(rmt_transmit(led_chan, encoder, led_strip_pixels, led_count * 3, &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

// Breathing animation with configurable mode, brightness, speed, and repeats
static void breathe_leds(rmt_channel_handle_t led_chan, rmt_encoder_handle_t encoder,
                         uint8_t red, uint8_t green, uint8_t blue,
                         uint8_t min_brightness, uint8_t max_brightness,
                         int duration_ms, float speed, breathe_mode_t mode,
                         uint8_t global_brightness, int repeat_count)
{
    const int steps = 100;
    int step = 0;
    int current_repeat = 0;

    // Adjust duration based on speed
    int adjusted_duration_ms = (int)(duration_ms / speed);
    float global_scale = global_brightness / 255.0f;

    while (repeat_count == 0 || current_repeat < repeat_count) {
        // Check for new command to interrupt
        led_command_t new_cmd;
        if (xQueuePeek(led_command_queue, &new_cmd, 0) == pdTRUE) {
            return;
        }

        // Calculate brightness (0 to 1) based on mode
        float t = (float)step / steps;
        float normalized_brightness;
        if (mode == BREATHE_SINE) {
            float sine = sinf(t * M_PI);
            normalized_brightness = (sine + 1.0f) / 2.0f;
        } else {
            normalized_brightness = 1.0f - fabs(2.0f * t - 1.0f);
        }

        // Scale to desired brightness range
        float brightness = (min_brightness + (max_brightness - min_brightness) * normalized_brightness) / 255.0f;

        // Apply brightness to LEDs with global scaling
        for (int led = 0; led < led_count; led++) {
            led_strip_pixels[led * 3 + 0] = (uint8_t)(green * brightness * global_scale);
            led_strip_pixels[led * 3 + 1] = (uint8_t)(red * brightness * global_scale);
            led_strip_pixels[led * 3 + 2] = (uint8_t)(blue * brightness * global_scale);
        }

        // Transmit the frame
        rmt_transmit_config_t tx_config = { .loop_count = 0 };
        ESP_ERROR_CHECK(rmt_transmit(led_chan, encoder, led_strip_pixels, led_count * 3, &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(adjusted_duration_ms / steps));

        // Increment step and handle repeats
        step = (step + 1) % steps;
        if (step == 0 && repeat_count > 0) {
            current_repeat++;
        }
    }
}

// Viper animation with configurable color, speed, and repeats
static void viper_animation(rmt_channel_handle_t led_chan, rmt_encoder_handle_t encoder,
                           uint8_t red, uint8_t green, uint8_t blue, int tail_length,
                           float speed, uint8_t global_brightness, int repeat_count)
{
    int pos = -tail_length;
    int direction = 1;
    int current_repeat = 0;
    bool completed_cycle = false;

    // Adjust frame duration based on speed
    int adjusted_frame_ms = (int)(EXAMPLE_FRAME_DURATION_MS / speed);
    float global_scale = global_brightness / 255.0f;

    while (repeat_count == 0 || current_repeat < repeat_count) {
        // Check for new command to interrupt
        led_command_t new_cmd;
        if (xQueuePeek(led_command_queue, &new_cmd, 0) == pdTRUE) {
            return;
        }

        // Clear the strip
        memset(led_strip_pixels, 0, led_count * 3);

        // Set the head and tail LEDs
        for (int i = 0; i < tail_length; i++) {
            int led = pos - i * direction;
            if (led >= 0 && led < led_count) {
                float brightness = (float)(tail_length - i) / tail_length;
                led_strip_pixels[led * 3 + 0] = (uint8_t)(green * brightness * global_scale);
                led_strip_pixels[led * 3 + 1] = (uint8_t)(red * brightness * global_scale);
                led_strip_pixels[led * 3 + 2] = (uint8_t)(blue * brightness * global_scale);
            }
        }

        // Transmit the frame
        rmt_transmit_config_t tx_config = { .loop_count = 0 };
        ESP_ERROR_CHECK(rmt_transmit(led_chan, encoder, led_strip_pixels, led_count * 3, &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(adjusted_frame_ms));

        // Update position and direction
        pos += direction;
        if (pos >= led_count + tail_length) {
            pos = led_count + tail_length - 1;
            direction = -1;
            completed_cycle = true;
        } else if (pos <= -tail_length) {
            pos = -tail_length + 1;
            direction = 1;
            completed_cycle = true;
        }

        // Increment repeat count after a full cycle
        if (completed_cycle && direction == 1 && pos == -tail_length + 1 && repeat_count > 0) {
            current_repeat++;
            completed_cycle = false;
        }
    }
}

// LED task to process commands from the queue
static void led_task(void *arg)
{
    rmt_channel_handle_t led_chan = NULL;
    rmt_encoder_handle_t encoder = NULL;

    // Initialize RMT
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = (int)arg, // GPIO passed as task argument
        .mem_block_symbols = 64,
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    const rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = encoder_callback
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));

    led_command_t cmd;
    while (1) {
        if (xQueueReceive(led_command_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Received command: %d", cmd.type);
            switch (cmd.type) {
                case LED_CMD_SET_COLOR:
                    set_all_leds_color(led_chan, encoder, cmd.red, cmd.green, cmd.blue, cmd.global_brightness);
                    break;
                case LED_CMD_BREATHE:
                    breathe_leds(led_chan, encoder, cmd.red, cmd.green, cmd.blue,
                                 cmd.min_brightness, cmd.max_brightness,
                                 cmd.duration_ms, cmd.speed, cmd.breathe_mode,
                                 cmd.global_brightness, cmd.repeat_count);
                    break;
                case LED_CMD_VIPER:
                    viper_animation(led_chan, encoder, cmd.red, cmd.green, cmd.blue,
                                    cmd.tail_length, cmd.speed, cmd.global_brightness,
                                    cmd.repeat_count);
                    break;
                case LED_CMD_STOP:
                    set_all_leds_color(led_chan, encoder, 0, 0, 0, 255);
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown command: %d", cmd.type);
            }
        }
    }
}

// Initialize the LED animation library
esp_err_t led_animations_init(const led_animations_config_t *config)
{
    if (config == NULL || config->led_count <= 0 || config->resolution_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate pixel buffer
    led_count = config->led_count;
    led_strip_pixels = malloc(led_count * 3);
    if (led_strip_pixels == NULL) {
        ESP_LOGE(TAG, "Failed to allocate pixel buffer");
        return ESP_ERR_NO_MEM;
    }
    memset(led_strip_pixels, 0, led_count * 3);

    // Create command queue
    led_command_queue = xQueueCreate(1, sizeof(led_command_t));
    if (led_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create LED command queue");
        free(led_strip_pixels);
        return ESP_ERR_NO_MEM;
    }

    // Start LED task
    if (xTaskCreate(led_task, "led_task", 4096, (void *)config->gpio_num, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED task");
        xQueueReset(led_command_queue);
        vQueueDelete(led_command_queue);
        free(led_strip_pixels);
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Send a command to the LED task
void send_led_command(led_command_t cmd)
{
    xQueueReset(led_command_queue);
    if (xQueueSend(led_command_queue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send command to LED queue");
    }
}