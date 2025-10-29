#ifndef SYSTEM_HANDLES_H
#define SYSTEM_HANDLES_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdint.h>

/* Mensajes y tipos */
typedef struct {
    uint8_t red_percent;
    uint8_t green_percent;
    uint8_t blue_percent;
} rgb_color_t;

typedef struct {
    float min;
    float max;
} temp_range_t;

typedef struct {
    temp_range_t red;
    temp_range_t green;
    temp_range_t blue;
} rgb_temp_ranges_t;

typedef enum {
    LED_MODE_MANUAL = 0,
    LED_MODE_AUTO
} led_mode_t;

/* Estructura de handles del sistema pasada a cada tarea */
typedef struct {
    QueueHandle_t rgb_cmd_queue;       // cola para rgb_color_t
    QueueHandle_t brightness_queue;    // cola para uint8_t (0..100)
    QueueHandle_t temp_queue;          // cola para float (temperatura °C)
    QueueHandle_t temp_range_queue;    // cola para rgb_temp_ranges_t (rango de temp)
    QueueHandle_t led_mode_queue;      // cola para led_mode_t
    QueueHandle_t print_enable_queue;  // cola para bool (habilitar impresión)
    SemaphoreHandle_t led_mutex;       // mutex para proteger acceso al LED
} system_handles_t;

#endif // SYSTEM_HANDLES_H
