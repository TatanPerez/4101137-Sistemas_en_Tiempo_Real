#include "led_rgb.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "string.h"

#define UART_NUM            UART_NUM_0
#define UART_BUF_SIZE       128

static const char *TAG = "LED_RGB_UART";

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
// Tarea que lee comandos por UART para ajustar el color del LED RGB en formato Rxx Gyy Bzz
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