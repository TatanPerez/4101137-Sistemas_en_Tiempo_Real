#include "led_rgb.h"
#include "esp_err.h"
// #include "string.h"

// ================= PWM: LED RGB Ánodo común ===================

static void ledc_setup_channel(ledc_channel_t channel, int gpio_num) {
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = channel,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio_num,
        .duty           = LEDC_DUTY_MAX,  // Apagado (ánodo común)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void led_rgb_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_setup_channel(RED_CHANNEL, RED_GPIO);
    ledc_setup_channel(GREEN_CHANNEL, GREEN_GPIO);
    ledc_setup_channel(BLUE_CHANNEL, BLUE_GPIO);
}

void led_rgb_set_color(uint16_t red, uint16_t green, uint16_t blue) {
    // Ánodo común: inversa la señal PWM (máximo duty = apagado)
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, RED_CHANNEL, LEDC_DUTY_MAX - red));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, RED_CHANNEL));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, GREEN_CHANNEL, LEDC_DUTY_MAX - green));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, GREEN_CHANNEL));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, BLUE_CHANNEL, LEDC_DUTY_MAX - blue));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, BLUE_CHANNEL));
}

void led_rgb_set_color_percent(uint8_t red_percent, uint8_t green_percent, uint8_t blue_percent) {
    if (red_percent > 100) red_percent = 100;
    if (green_percent > 100) green_percent = 100;
    if (blue_percent > 100) blue_percent = 100;

    uint16_t red = (LEDC_DUTY_MAX * red_percent) / 100;
    uint16_t green = (LEDC_DUTY_MAX * green_percent) / 100;
    uint16_t blue = (LEDC_DUTY_MAX * blue_percent) / 100;

    led_rgb_set_color(red, green, blue);
}