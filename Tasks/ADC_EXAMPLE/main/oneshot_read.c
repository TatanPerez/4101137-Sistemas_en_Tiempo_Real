#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

#include "oneshot_read.h"
#include "led_rgb.h"
#include "system_handles.h"   // <- incluye las colas y el mutex compartido

#ifndef NAN
#define NAN (0.0/0.0)
#endif

static const char *TAG = "EXAMPLE_NTC";
static const char *TAG_TEMP = "LED_RGB_TEMP";
// ✅ ADC compartido (definido en oneshot_read_main.c)
extern adc_oneshot_unit_handle_t g_adc1_handle;

/*---------------------------------------------------------------
        ADC Calibration helpers
---------------------------------------------------------------*/
bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

/*---------------------------------------------------------------
        Main ADC Task (NTC + botón)
---------------------------------------------------------------*/

void oneshot_read_task(void *pvParameters)
{
    // Recibir estructura con colas y mutex
    system_handles_t *sys = (system_handles_t *)pvParameters;
    configASSERT(sys != NULL);

    static int adc_raw[2][10];
    static int voltage[2][10];

    //------------- ADC1 Config ---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    //------------- ADC1 Calibration Init ---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(
        ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);

    //------------- Parámetros NTC ---------------//
    const double V_in = 3.3;      // Voltaje de referencia
    const double R_fixed = 10000.0;  // Resistencia fija de 10kΩ
    const double R0 = 10000.0;       // NTC 10kΩ @ 25°C
    const double B = 3950.0;         // Constante Beta
    const double T0 = 298.15;        // 25°C en Kelvin

    //------------- Configurar botón ---------------//
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn_config));

    bool print_enabled = true; // toggleado por botón
    int last_btn_state = 1;
    TickType_t last_print_time = 0;

    while (1) {
        // ------ Leer botón y alternar impresión ------
        int current_btn_state = gpio_get_level(BUTTON_GPIO);
        if (last_btn_state == 1 && current_btn_state == 0) { // flanco de bajada
            print_enabled = !print_enabled;
            ESP_LOGI(TAG, "Print %s", print_enabled ? "ENABLED" : "DISABLED");
        }
        last_btn_state = current_btn_state;

        // ------ Leer ADC ------
        ESP_ERROR_CHECK(adc_oneshot_read(g_adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));

        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle,
                                                    adc_raw[0][0], &voltage[0][0]));

            double V_out = voltage[0][0] / 1000.0; // mV → V
            double R_ntc = NAN;
            if (V_out > 0 && V_out < V_in) {
                R_ntc = R_fixed * (V_out / (V_in - V_out));
            }

            double tempC = NAN;
            if (!isnan(R_ntc) && R_ntc > 0) {
                double tempK = (B * T0) / (B + (T0 * log(R_ntc / R0)));
                tempC = tempK - 273.15;
            }

            // ------ Enviar temperatura a la cola ------
            if (!isnan(tempC)) {
                float tempF = (float)tempC;
                // Mantiene solo el último valor (no bloquea si llena)
                xQueueOverwrite(sys->temp_queue, &tempF);
            }

            // ------ Imprimir periódicamente si habilitado ------
            TickType_t now = xTaskGetTickCount();
            if (print_enabled && (now - last_print_time >= pdMS_TO_TICKS(2000))) {
                ESP_LOGI(TAG, "ADC Raw=%d  Voltage=%d mV  Temp=%.2f °C",
                         adc_raw[0][0], voltage[0][0], tempC);
                last_print_time = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Nunca llega aquí (tarea infinita)
    ESP_ERROR_CHECK(adc_oneshot_del_unit(g_adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}

void led_rgb_temp_task(void *pvParameters)
{
    system_handles_t *sys = (system_handles_t *)pvParameters;
    configASSERT(sys != NULL);

    rgb_color_t base_color = {0, 0, 0};
    bool base_received = false;

    rgb_temp_ranges_t ranges = {0};
    bool ranges_received = false;

    led_mode_t current_mode = LED_MODE_MANUAL;
    uint8_t brightness = 100;
    float temp = 0.0f;

    ESP_LOGI(TAG_TEMP, "Tarea de control de LED RGB según temperatura iniciada");

    for (;;)
    {
        // Actualizar color base si llega uno nuevo
        rgb_color_t new_base;
        if (xQueueReceive(sys->rgb_cmd_queue, &new_base, 0) == pdTRUE)
        {
            base_color = new_base;
            base_received = true;
        }

        // Actualizar rangos si llegan nuevos por UART
        rgb_temp_ranges_t new_ranges;
        if (xQueueReceive(sys->temp_range_queue, &new_ranges, 0) == pdTRUE)
        {
            ranges = new_ranges;
            ranges_received = true;
            ESP_LOGI(TAG_TEMP, "Nuevos rangos guardados: R(%.1f-%.1f) G(%.1f-%.1f) B(%.1f-%.1f)",
                     ranges.red.min, ranges.red.max,
                     ranges.green.min, ranges.green.max,
                     ranges.blue.min, ranges.blue.max);
        }

        // Leer modo actual (manual o automático)
        led_mode_t new_mode;
        if (xQueueReceive(sys->led_mode_queue, &new_mode, 0) == pdTRUE)
            current_mode = new_mode;

        // Actualizar brillo
        uint8_t new_brightness;
        if (xQueueReceive(sys->brightness_queue, &new_brightness, 0) == pdTRUE)
            brightness = new_brightness;

        // Esperar nueva temperatura
        if (xQueueReceive(sys->temp_queue, &temp, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            rgb_color_t chosen = {0, 0, 0};

            if (current_mode == LED_MODE_AUTO && ranges_received)
            {
                bool in_range = false;

                if (temp >= ranges.red.min && temp <= ranges.red.max)
                {
                    chosen = (rgb_color_t){100, 0, 0};
                    in_range = true;
                }
                else if (temp >= ranges.green.min && temp <= ranges.green.max)
                {
                    chosen = (rgb_color_t){0, 100, 0};
                    in_range = true;
                }
                else if (temp >= ranges.blue.min && temp <= ranges.blue.max)
                {
                    chosen = (rgb_color_t){0, 0, 100};
                    in_range = true;
                }

                // Fuera de todos los rangos → usar color base definido por UART
                if (!in_range)
                {
                    if (base_received)
                    {
                        chosen = base_color;
                        ESP_LOGI(TAG_TEMP, "Temp %.2f°C fuera de rangos → usando color base R=%d G=%d B=%d",
                                temp, base_color.red_percent, base_color.green_percent, base_color.blue_percent);
                    }
                    else
                    {
                        // No hay color base, mantener LED apagado
                        chosen = (rgb_color_t){0, 0, 0};
                        ESP_LOGI(TAG_TEMP, "Temp %.2f°C fuera de rangos y sin color base definido → LED apagado", temp);
                    }
                }
            }
            else if (current_mode == LED_MODE_MANUAL && base_received)
            {
                chosen = base_color;
            }

            // Aplicar brillo actual
            uint8_t r = (chosen.red_percent * brightness) / 100;
            uint8_t g = (chosen.green_percent * brightness) / 100;
            uint8_t b = (chosen.blue_percent * brightness) / 100;

            if (xSemaphoreTake(sys->led_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
            {
                led_rgb_set_color_percent(r, g, b);
                xSemaphoreGive(sys->led_mutex);
            }

            ESP_LOGI(TAG_TEMP, "Temp=%.2f°C | Modo=%s | LED→R=%d G=%d B=%d (Brillo=%d%%)",
                     temp,
                     current_mode == LED_MODE_AUTO ? "AUTO" : "MANUAL",
                     r, g, b, brightness);
        }
    }
}
