#include "led_gpio.h"
#include "led_strip.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "led_gpio";

struct led_gpio_strip {
    led_strip_handle_t strip;
    int led_count;
};

led_gpio_handle_t led_gpio_init(int gpio_num, int led_count) {
    led_gpio_handle_t handle = malloc(sizeof(struct led_gpio_strip));
    if (!handle) return NULL;
    
    handle->led_count = led_count;
    
    // Use SPI backend for GPIO46
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    
    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio_num,
        .max_leds = led_count,
        .led_model = LED_MODEL_WS2812,
    };
    
    esp_err_t ret = led_strip_new_spi_device(&strip_config, &spi_config, &handle->strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create SPI LED strip: %s", esp_err_to_name(ret));
        free(handle);
        return NULL;
    }
    
    led_strip_clear(handle->strip);
    ESP_LOGI(TAG, "SPI LED strip initialized on GPIO%d with %d LEDs", gpio_num, led_count);
    return handle;
}

esp_err_t led_gpio_set_color(led_gpio_handle_t handle, uint8_t r, uint8_t g, uint8_t b) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    for (int i = 0; i < handle->led_count; i++) {
        led_strip_set_pixel(handle->strip, i, r, g, b);
    }
    return led_strip_refresh(handle->strip);
}

esp_err_t led_gpio_clear(led_gpio_handle_t handle) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    return led_strip_clear(handle->strip);
}
