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

// #include "string.h"

#define UART_NUM            UART_NUM_0
#define UART_BUF_SIZE       128
#define ADC_AVG_SAMPLES 5   // Número de muestras para promediado ADC

static const char *TAG = "LED_RGB_UART";
static const char *TAG_POT = "LED_RGB_POT";

// Variables globales para el color base definido por UART
static uint8_t base_r_percent = 100;
static uint8_t base_g_percent = 100;
static uint8_t base_b_percent = 100;


// ================= PWM: LED RGB Ánodo común ===================

// Configura un canal LEDC PWM para el LED RGB
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
// Inicializa el timer y canales LEDC para el RGB
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
// Ajusta el color y brillo del LED RGB.
void led_rgb_set_color(uint16_t red, uint16_t green, uint16_t blue) {
    // Ánodo común: inversa la señal PWM (máximo duty = apagado)
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, RED_CHANNEL, LEDC_DUTY_MAX - red));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, RED_CHANNEL));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, GREEN_CHANNEL, LEDC_DUTY_MAX - green));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, GREEN_CHANNEL));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, BLUE_CHANNEL, LEDC_DUTY_MAX - blue));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, BLUE_CHANNEL));
}
// Ajusta color pasando porcentaje 0-100 para cada canal
void led_rgb_set_color_percent(uint8_t red_percent, uint8_t green_percent, uint8_t blue_percent) {
    if (red_percent > 100) red_percent = 100;
    if (green_percent > 100) green_percent = 100;
    if (blue_percent > 100) blue_percent = 100;

    uint16_t red = (LEDC_DUTY_MAX * red_percent) / 100;
    uint16_t green = (LEDC_DUTY_MAX * green_percent) / 100;
    uint16_t blue = (LEDC_DUTY_MAX * blue_percent) / 100;

    led_rgb_set_color(red, green, blue);
}
/*-------------------------------------------------------------------------------------------------------
    Tarea que lee comandos por UART para ajustar el color del LED RGB
    Formato comando: Rxx Gyy Bzz (ej: R100 G50 B25)
--------------------------------------------------------------------------------------------------------- */    

void led_rgb_uart_task(void *pvParameters) {
    uart_config_t uart_config = {
        .baud_rate = 115200,                    // Velocidad típica de consola
        .data_bits = UART_DATA_8_BITS,          // Bits de datos
        .parity    = UART_PARITY_DISABLE,       // Sin paridad
        .stop_bits = UART_STOP_BITS_1,          // 1 bit de parada
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE   // Sin control de flujo
    };

    uart_param_config(UART_NUM, &uart_config);  // Configura parámetros UART
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // Usa pines por defecto
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);               // Instala driver UART

    ESP_LOGI(TAG, "Control de LED RGB por UART");
    ESP_LOGI(TAG, "Formato: Rxx Gyy Bzz (ej: R100 G50 B25)");
    ESP_LOGI(TAG, "Ingrese comando:");

    uint8_t data[1];                    // Buffer para leer un byte a la vez
    char input_line[UART_BUF_SIZE];     // Buffer para la línea de entrada
    int idx = 0;                        // Índice en el buffer de entrada

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            char c = (char)data[0];

            if (c == '\r' || c == '\n') {   // Fin de línea
                input_line[idx] = '\0';

                if (idx > 0) {              
                    int r = -1, g = -1, b = -1;
                    int matched = sscanf(input_line, "R%d G%d B%d", &r, &g, &b);

                    if (matched == 3 && r >= 0 && r <= 100 && g >= 0 && g <= 100 && b >= 0 && b <= 100) {   // Comando válido
                        base_r_percent = r;
                        base_g_percent = g;
                        base_b_percent = b;
                        led_rgb_set_color_percent(r, g, b);
                        ESP_LOGI(TAG, "-> Comando ejecutado: %s", input_line);
                        ESP_LOGI(TAG, "-> LED ajustado a R:%d%% G:%d%% B:%d%%", r, g, b);
                    } else {
                        ESP_LOGI(TAG, "Entrada inválida. Usa: Rxx Gyy Bzz");
                    }
                }
                idx = 0;
                input_line[0] = '\0';
                ESP_LOGI(TAG, "Ingrese comando:");
            }
            else if (c == '\b' || c == 127) {       // Backspace    
                if (idx > 0) {
                    idx--;
                    input_line[idx] = '\0';
                    ESP_LOGI(TAG, "Comando actual: %s", input_line);
                }
            }
            else if (c >= 32 && c <= 126 && idx < UART_BUF_SIZE - 1) {  // Caracter imprimible
                input_line[idx++] = c;
                input_line[idx] = '\0';
                ESP_LOGI(TAG, "Comando actual: %s", input_line);
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------
    Tarea que lee el valor de un potenciómetro vía ADC y ajusta el brillo del LED RGB en consecuencia
--------------------------------------------------------------------------------------------------------- 
*/
void led_rgb_pot_task(void *pvParameters) {
    // Configuración ADC para ESP32‑C6
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,          // Rango 0-3.3V
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 12 bits aprox.
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));

    uint8_t last_percent = 0xFF;
    int adc_samples[ADC_AVG_SAMPLES] = {0};
    int sample_index = 0;

    while (1) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_POT, "Error leyendo ADC: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(POT_TASK_DELAY_MS));
            continue;
        }
        adc_samples[sample_index++] = raw;
        if (sample_index >= ADC_AVG_SAMPLES) sample_index = 0;

        int sum = 0;
        for (int i = 0; i < ADC_AVG_SAMPLES; i++) {
            sum += adc_samples[i];
        }
        int avg_raw = sum / ADC_AVG_SAMPLES;

        // Limitar valor raw para evitar overflow y pasar a porcentaje
        if (avg_raw > 4095) avg_raw = 4095;

        uint8_t percent = (avg_raw * 100) / 4095;

        if (percent != last_percent) {
            // Aplica brillo multiplicando por base de color
            uint8_t r = (base_r_percent * percent) / 100;
            uint8_t g = (base_g_percent * percent) / 100;
            uint8_t b = (base_b_percent * percent) / 100;

            led_rgb_set_color_percent(r, g, b);

            ESP_LOGI(TAG_POT, "Potenciómetro: raw=%d, brillo=%d%% → R=%d G=%d B=%d (base R=%d G=%d B=%d)", 
                     avg_raw, percent, r, g, b, base_r_percent, base_g_percent, base_b_percent);

            last_percent = percent;
        }
        vTaskDelay(pdMS_TO_TICKS(POT_TASK_DELAY_MS));
    }
}
