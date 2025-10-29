#include "led_rgb.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "system_handles.h"   // Estructura que contiene colas y mutex
#include <string.h>

#define UART_NUM            UART_NUM_0
#define UART_BUF_SIZE       128
#define ADC_AVG_SAMPLES     5

static const char *TAG = "LED_RGB";
static const char *TAG_POT = "LED_POT";

// 🔸 ADC compartido (definido en oneshot_read_main.c)
extern adc_oneshot_unit_handle_t g_adc1_handle;

/* =========================================================================================
                                FUNCIONES LED PWM
=========================================================================================*/

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

// Aplica color en duty PWM (inverso porque es ánodo común)
void led_rgb_set_color(uint16_t red, uint16_t green, uint16_t blue) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, RED_CHANNEL, LEDC_DUTY_MAX - red));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, RED_CHANNEL));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, GREEN_CHANNEL, LEDC_DUTY_MAX - green));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, GREEN_CHANNEL));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, BLUE_CHANNEL, LEDC_DUTY_MAX - blue));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, BLUE_CHANNEL));
}

// Ajusta PWM según porcentaje
void led_rgb_set_color_percent(uint8_t red_percent, uint8_t green_percent, uint8_t blue_percent) {
    if (red_percent > 100) red_percent = 100;
    if (green_percent > 100) green_percent = 100;
    if (blue_percent > 100) blue_percent = 100;

    uint16_t red = (LEDC_DUTY_MAX * red_percent) / 100;
    uint16_t green = (LEDC_DUTY_MAX * green_percent) / 100;
    uint16_t blue = (LEDC_DUTY_MAX * blue_percent) / 100;

    led_rgb_set_color(red, green, blue);
}

/* =========================================================================================
                        TAREA UART → recibe color base por comandos
   Formato:  Rxx Gyy Bzz  (xx,yy,zz entre 0 y 100)
   Formato rangos: Range rMin rMax gMin gMax bMin bMax (enteros)

=========================================================================================*/

void led_rgb_uart_task(void *pvParameters) {
    system_handles_t *sys = (system_handles_t *)pvParameters;
    configASSERT(sys != NULL);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "Control de LED RGB por UART (Color: Rxx Gxx Bxx, Rangos: Range rMin rMax gMin gMax bMin bMax)");

    uint8_t data[1];
    char input_line[UART_BUF_SIZE];
    int idx = 0;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            char c = (char)data[0];

            // Mostrar tecla recibida
            if (c >= 32 && c <= 126) {
                ESP_LOGI(TAG, "Tecla recibida: %c", c);
            } else if (c == '\r' || c == '\n') {
                ESP_LOGI(TAG, "Enter presionado");
            } else {
                ESP_LOGI(TAG, "Carácter especial recibido: 0x%02X", c);
            }

            // Procesar línea completa
            if (c == '\r' || c == '\n') {
                input_line[idx] = '\0';
                if (idx > 0) {
                    // Intentar leer color base
                    rgb_color_t cmd;
                    int matched = sscanf(input_line, "R%hhu G%hhu B%hhu",
                                         &cmd.red_percent, &cmd.green_percent, &cmd.blue_percent);
                    if (matched == 3) {
                        // Color base: guardar, pero NO cambiar el modo
                        xQueueOverwrite(sys->rgb_cmd_queue, &cmd);
                        ESP_LOGI(TAG, "Color base actualizado → R=%d G=%d B=%d",
                                cmd.red_percent, cmd.green_percent, cmd.blue_percent);
                    }
                    else if (strncasecmp(input_line, "Range", 5) == 0) {
                        rgb_temp_ranges_t ranges;
                        int rMin, rMax, gMin, gMax, bMin, bMax;
                        matched = sscanf(input_line + 5, "%d %d %d %d %d %d",
                                        &rMin, &rMax, &gMin, &gMax, &bMin, &bMax);
                        if (matched == 6) {
                            ranges.red.min = rMin;   ranges.red.max = rMax;
                            ranges.green.min = gMin; ranges.green.max = gMax;
                            ranges.blue.min = bMin;  ranges.blue.max = bMax;

                            xQueueOverwrite(sys->temp_range_queue, &ranges);

                            // Activar modo AUTO cuando se define un rango
                            led_mode_t auto_mode = LED_MODE_AUTO;
                            xQueueOverwrite(sys->led_mode_queue, &auto_mode);

                            ESP_LOGI(TAG, "Modo AUTO → Rangos guardados: R(%d-%d) G(%d-%d) B(%d-%d)",
                                    rMin, rMax, gMin, gMax, bMin, bMax);
                        } else {
                            ESP_LOGW(TAG, "Formato Range inválido. Usa: Range rMin rMax gMin gMax bMin bMax");
                        }
                    }
                    else {
                        ESP_LOGW(TAG, "Formato inválido. Usa: Rxx Gxx Bxx o Range rMin rMax gMin gMax bMin bMax");
                    }
                }
                idx = 0;
                memset(input_line, 0, sizeof(input_line));
            }
            // Acumular caracteres imprimibles
            else if (c >= 32 && c <= 126 && idx < UART_BUF_SIZE - 1) {
                input_line[idx++] = c;
            }
        }
    }
}

/* =========================================================================================
                        TAREA POTENCIÓMETRO → ajusta brillo
=========================================================================================*/

void led_rgb_pot_task(void *pvParameters)
{
    system_handles_t *sys = (system_handles_t *)pvParameters;
    configASSERT(sys != NULL);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1_handle, ADC_CHANNEL, &chan_cfg));

    int adc_samples[ADC_AVG_SAMPLES] = {0};
    int sample_index = 0;
    uint8_t last_brightness = 0xFF;

    ESP_LOGI(TAG_POT, "Tarea del potenciómetro iniciada (solo control de brillo)");

    while (1)
    {
        // Leer ADC del potenciómetro
        int raw;
        if (adc_oneshot_read(g_adc1_handle, ADC_CHANNEL, &raw) != ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Promediar muestras
        adc_samples[sample_index++] = raw;
        if (sample_index >= ADC_AVG_SAMPLES)
            sample_index = 0;

        int sum = 0;
        for (int i = 0; i < ADC_AVG_SAMPLES; i++)
            sum += adc_samples[i];
        int avg_raw = sum / ADC_AVG_SAMPLES;
        if (avg_raw > 4095)
            avg_raw = 4095;

        // Convertir a porcentaje 0–100
        uint8_t brightness = (avg_raw * 100) / 4095;

        // Solo actualizar si hay un cambio significativo
        if (brightness != last_brightness)
        {
            xQueueOverwrite(sys->brightness_queue, &brightness);
            last_brightness = brightness;

            ESP_LOGI(TAG_POT, "Brillo actualizado: %d%%", brightness);
        }

        // No se controla el LED aquí — lo hace led_rgb_temp_task()
        vTaskDelay(pdMS_TO_TICKS(POT_TASK_DELAY_MS));
    }
}
