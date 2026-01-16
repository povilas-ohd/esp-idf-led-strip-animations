/*
 * led_animations.c
 * Implementation of LED animation library for WS2812 strips - Multi-instance support
 *
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_animations.h"

static const char *TAG = "led_animations";

// LED strip instance structure
struct led_strip_instance {
    uint8_t *pixels;
    int led_count;
    QueueHandle_t command_queue;
    SemaphoreHandle_t rmt_ready_sem;  // Signals when RMT channel allocated
    uint32_t resolution_hz;
    int frame_duration_ms;
    int gpio_num;
    rmt_symbol_word_t ws2812_zero;
    rmt_symbol_word_t ws2812_one;
    rmt_symbol_word_t ws2812_reset;
};

// Encoder callback: Converts RGB data to RMT symbols
static size_t encoder_callback(const void *data, size_t data_size,
                              size_t symbols_written, size_t symbols_free,
                              rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    led_strip_handle_t instance = (led_strip_handle_t)arg;
    if (symbols_free < 8) {
        return 0;
    }
    size_t data_pos = symbols_written / 8;
    uint8_t *data_bytes = (uint8_t*)data;
    if (data_pos < data_size) {
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
            symbols[symbol_pos++] = (data_bytes[data_pos] & bitmask) ? instance->ws2812_one : instance->ws2812_zero;
        }
        return symbol_pos;
    } else {
        symbols[0] = instance->ws2812_reset;
        *done = 1;
        return 1;
    }
}

// Set all LEDs to a specific GRB color with global brightness scaling
static void set_all_leds_color(led_strip_handle_t instance, rmt_channel_handle_t led_chan, rmt_encoder_handle_t encoder,
                               uint8_t red, uint8_t green, uint8_t blue, uint8_t global_brightness)
{
    float scale = global_brightness / 255.0f;
    for (int led = 0; led < instance->led_count; led++) {
        instance->pixels[led * 3 + 0] = (uint8_t)(green * scale); // GRB order
        instance->pixels[led * 3 + 1] = (uint8_t)(red * scale);
        instance->pixels[led * 3 + 2] = (uint8_t)(blue * scale);
    }
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    ESP_ERROR_CHECK(rmt_transmit(led_chan, encoder, instance->pixels, instance->led_count * 3, &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

// Breathing animation
static void breathe_leds(led_strip_handle_t instance, rmt_channel_handle_t led_chan, rmt_encoder_handle_t encoder,
                         uint8_t red, uint8_t green, uint8_t blue,
                         uint8_t min_brightness, uint8_t max_brightness,
                         int duration_ms, float speed, breathe_mode_t mode,
                         uint8_t global_brightness, int repeat_count)
{
    const int steps = 100;
    int step = 0;
    int current_repeat = 0;
    int adjusted_duration_ms = (int)(duration_ms / speed);
    float global_scale = global_brightness / 255.0f;

    while (repeat_count == 0 || current_repeat < repeat_count) {
        led_command_t new_cmd;
        if (xQueuePeek(instance->command_queue, &new_cmd, 0) == pdTRUE) {
            return;
        }

        float t = (float)step / steps;
        float normalized_brightness;
        if (mode == BREATHE_SINE) {
            float sine = sinf(t * M_PI);
            normalized_brightness = (sine + 1.0f) / 2.0f;
        } else {
            normalized_brightness = 1.0f - fabs(2.0f * t - 1.0f);
        }

        float brightness = (min_brightness + (max_brightness - min_brightness) * normalized_brightness) / 255.0f;

        for (int led = 0; led < instance->led_count; led++) {
            instance->pixels[led * 3 + 0] = (uint8_t)(green * brightness * global_scale);
            instance->pixels[led * 3 + 1] = (uint8_t)(red * brightness * global_scale);
            instance->pixels[led * 3 + 2] = (uint8_t)(blue * brightness * global_scale);
        }

        rmt_transmit_config_t tx_config = { .loop_count = 0 };
        ESP_ERROR_CHECK(rmt_transmit(led_chan, encoder, instance->pixels, instance->led_count * 3, &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(adjusted_duration_ms / steps));

        step = (step + 1) % steps;
        if (step == 0 && repeat_count > 0) {
            current_repeat++;
        }
    }
}

// Viper animation
static void viper_animation(led_strip_handle_t instance, rmt_channel_handle_t led_chan, rmt_encoder_handle_t encoder,
                           uint8_t red, uint8_t green, uint8_t blue, int tail_length,
                           float speed, uint8_t global_brightness, int repeat_count)
{
    int pos = -tail_length;
    int direction = 1;
    int current_repeat = 0;
    bool completed_cycle = false;
    int adjusted_frame_ms = (int)(instance->frame_duration_ms / speed);
    float global_scale = global_brightness / 255.0f;

    while (repeat_count == 0 || current_repeat < repeat_count) {
        led_command_t new_cmd;
        if (xQueuePeek(instance->command_queue, &new_cmd, 0) == pdTRUE) {
            return;
        }

        memset(instance->pixels, 0, instance->led_count * 3);

        for (int i = 0; i < tail_length; i++) {
            int led = pos - i * direction;
            if (led >= 0 && led < instance->led_count) {
                float brightness = (float)(tail_length - i) / tail_length;
                instance->pixels[led * 3 + 0] = (uint8_t)(green * brightness * global_scale);
                instance->pixels[led * 3 + 1] = (uint8_t)(red * brightness * global_scale);
                instance->pixels[led * 3 + 2] = (uint8_t)(blue * brightness * global_scale);
            }
        }

        rmt_transmit_config_t tx_config = { .loop_count = 0 };
        ESP_ERROR_CHECK(rmt_transmit(led_chan, encoder, instance->pixels, instance->led_count * 3, &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(adjusted_frame_ms));

        pos += direction;
        if (pos >= instance->led_count + tail_length) {
            pos = instance->led_count + tail_length - 1;
            direction = -1;
            completed_cycle = true;
        } else if (pos <= -tail_length) {
            pos = -tail_length + 1;
            direction = 1;
            completed_cycle = true;
        }

        if (completed_cycle && direction == 1 && pos == -tail_length + 1 && repeat_count > 0) {
            current_repeat++;
            completed_cycle = false;
        }
    }
}

// LED task to process commands from the queue
static void led_task(void *arg)
{
    led_strip_handle_t instance = (led_strip_handle_t)arg;
    rmt_channel_handle_t led_chan = NULL;
    rmt_encoder_handle_t encoder = NULL;

    // Initialize RMT with auto channel selection
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = instance->gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = instance->resolution_hz,
        .trans_queue_depth = 4,
        .flags.with_dma = false,
    };
    
    // Try to allocate RMT channel - will auto-select next available
    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d: Failed to allocate RMT channel: %s", instance->gpio_num, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    const rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = encoder_callback,
        .arg = instance
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));
    
    // Signal that RMT channel is allocated
    xSemaphoreGive(instance->rmt_ready_sem);

    led_command_t cmd;
    while (1) {
        if (xQueueReceive(instance->command_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "GPIO%d: Received command: %d", instance->gpio_num, cmd.type);
            switch (cmd.type) {
                case LED_CMD_SET_COLOR:
                    set_all_leds_color(instance, led_chan, encoder, cmd.red, cmd.green, cmd.blue, cmd.global_brightness);
                    break;
                case LED_CMD_BREATHE:
                    breathe_leds(instance, led_chan, encoder, cmd.red, cmd.green, cmd.blue,
                                 cmd.min_brightness, cmd.max_brightness,
                                 cmd.duration_ms, cmd.speed, cmd.breathe_mode,
                                 cmd.global_brightness, cmd.repeat_count);
                    break;
                case LED_CMD_VIPER:
                    viper_animation(instance, led_chan, encoder, cmd.red, cmd.green, cmd.blue,
                                    cmd.tail_length, cmd.speed, cmd.global_brightness,
                                    cmd.repeat_count);
                    break;
                case LED_CMD_STOP:
                    set_all_leds_color(instance, led_chan, encoder, 0, 0, 0, 255);
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown command: %d", cmd.type);
            }
        }
    }
}

// Initialize a LED strip instance
led_strip_handle_t led_animations_init(const led_animations_config_t *config)
{
    if (config == NULL || config->led_count <= 0 || config->resolution_hz == 0) {
        return NULL;
    }

    led_strip_handle_t instance = malloc(sizeof(struct led_strip_instance));
    if (instance == NULL) {
        ESP_LOGE(TAG, "Failed to allocate instance");
        return NULL;
    }

    instance->led_count = config->led_count;
    instance->resolution_hz = config->resolution_hz;
    instance->frame_duration_ms = (config->frame_duration_ms > 0) ? 
                                  config->frame_duration_ms : LED_STRIP_DEFAULT_FRAME_DURATION_MS;
    instance->gpio_num = config->gpio_num;
    
    // Update WS2812 timing based on resolution
    instance->ws2812_zero.level0 = 1;
    instance->ws2812_zero.duration0 = 0.3 * instance->resolution_hz / 1000000;
    instance->ws2812_zero.level1 = 0;
    instance->ws2812_zero.duration1 = 0.9 * instance->resolution_hz / 1000000;
    
    instance->ws2812_one.level0 = 1;
    instance->ws2812_one.duration0 = 0.9 * instance->resolution_hz / 1000000;
    instance->ws2812_one.level1 = 0;
    instance->ws2812_one.duration1 = 0.3 * instance->resolution_hz / 1000000;
    
    instance->ws2812_reset.level0 = 0;
    instance->ws2812_reset.duration0 = instance->resolution_hz / 1000000 * 50 / 2;
    instance->ws2812_reset.level1 = 0;
    instance->ws2812_reset.duration1 = instance->resolution_hz / 1000000 * 50 / 2;

    // Allocate pixel buffer
    instance->pixels = malloc(instance->led_count * 3);
    if (instance->pixels == NULL) {
        ESP_LOGE(TAG, "Failed to allocate pixel buffer");
        free(instance);
        return NULL;
    }
    memset(instance->pixels, 0, instance->led_count * 3);

    // Create command queue
    instance->command_queue = xQueueCreate(1, sizeof(led_command_t));
    if (instance->command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create LED command queue");
        free(instance->pixels);
        free(instance);
        return NULL;
    }
    
    // Create semaphore for RMT ready signal
    instance->rmt_ready_sem = xSemaphoreCreateBinary();
    if (instance->rmt_ready_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create RMT ready semaphore");
        vQueueDelete(instance->command_queue);
        free(instance->pixels);
        free(instance);
        return NULL;
    }

    // Start LED task
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "led_task_%d", config->gpio_num);
    if (xTaskCreate(led_task, task_name, 4096, instance, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED task");
        vQueueDelete(instance->command_queue);
        free(instance->pixels);
        free(instance);
        return NULL;
    }

    ESP_LOGI(TAG, "LED strip initialized on GPIO%d with %d LEDs", config->gpio_num, config->led_count);
    
    // Wait for RMT channel to be allocated by task (max 1 second)
    if (xSemaphoreTake(instance->rmt_ready_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout waiting for RMT channel allocation on GPIO%d", config->gpio_num);
    }
    
    return instance;
}

// Send a command to a specific LED strip instance
void send_led_command(led_strip_handle_t handle, led_command_t cmd)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return;
    }
    
    xQueueReset(handle->command_queue);
    if (xQueueSend(handle->command_queue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send command to LED queue");
    }
}