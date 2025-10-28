// SPDX-License-Identifier: MIT
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static const char *TAG = "BUSQUEDA_HOME_BIDIRECCIONAL";

// ===================== Pines =====================
#define PIN_STEP      18
#define PIN_DIR       17
#define PIN_EN        16
#define PIN_HALL      26
#define PIN_LED_D2    2

// ===================== LEDC (Control del motor) =====================
#define LEDC_MODO         LEDC_HIGH_SPEED_MODE
#define LEDC_TEMPORIZADOR LEDC_TIMER_0
#define LEDC_CANAL        LEDC_CHANNEL_0
#define LEDC_RESOLUCION   LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON      (1 << 6)
#define LEDC_DUTY_OFF     0

// ===================== Parámetros de Movimiento =====================
#define FREQ_BUSQUEDA_HZ    1000.0f
#define FREQ_VERIFICACION_HZ  400.0f
#define TIEMPO_RAMPA_MS     120U
#define TIEMPO_SOBREPASO_MS 4000U

// ===================== Parámetros del sensor Hall =====================
#define MUESTRA_HALL_MS     1
#define NUMERO_MUESTRAS_N   2

// ===================== Direcciones del motor =====================
#define DERECHA   1
#define IZQUIERDA 0

/* =====================================================================
 * FUNCIONES AUXILIARES (Sin cambios)
 * ===================================================================== */
static inline int  leer_sensor_hall(void) { return gpio_get_level(PIN_HALL) == 0; }
static inline void habilitar_driver(bool encendido) { gpio_set_level(PIN_EN, encendido ? 0 : 1); }
static inline void fijar_direccion(int dir) { gpio_set_level(PIN_DIR, dir); esp_rom_delay_us(20); }
static void inicializar_pwm(float frecuencia_inicial_hz) {
    ledc_timer_config_t t = { .speed_mode = LEDC_MODO, .duty_resolution = LEDC_RESOLUCION, .timer_num = LEDC_TEMPORIZADOR, .freq_hz = (uint32_t)frecuencia_inicial_hz, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num = PIN_STEP, .speed_mode = LEDC_MODO, .channel = LEDC_CANAL, .timer_sel = LEDC_TEMPORIZADOR, .duty = LEDC_DUTY_OFF, .hpoint = 0 };
    ledc_channel_config(&c);
}
static inline void arrancar_motor(void) { ledc_set_duty(LEDC_MODO, LEDC_CANAL, LEDC_DUTY_ON); ledc_update_duty(LEDC_MODO, LEDC_CANAL); }
static inline void parar_motor(void) { ledc_stop(LEDC_MODO, LEDC_CANAL, LEDC_DUTY_OFF); }
static inline void fijar_frecuencia_pasos(float hz) { if (hz < 1.0f) hz = 1.0f; ledc_set_freq(LEDC_MODO, LEDC_TEMPORIZADOR, (uint32_t)hz); }
static void esperar_hall_bajo_estable(void) {
    int contador = 0;
    while (true) { if (leer_sensor_hall()) { if (++contador >= NUMERO_MUESTRAS_N) return; } else { contador = 0; } vTaskDelay(pdMS_TO_TICKS(MUESTRA_HALL_MS)); }
}
static void esperar_hall_alto_estable(void) {
    int contador = 0;
    while (true) { if (!leer_sensor_hall()) { if (++contador >= NUMERO_MUESTRAS_N) return; } else { contador = 0; } vTaskDelay(pdMS_TO_TICKS(MUESTRA_HALL_MS)); }
}
static void rampa_de_velocidad(float frec_inicial, float frec_final, uint32_t duracion_ms) {
    if (duracion_ms == 0 || fabsf(frec_final - frec_inicial) < 1.0f) { fijar_frecuencia_pasos(frec_final); return; }
    uint32_t pasos = duracion_ms / 5U; if (!pasos) pasos = 1;
    for (uint32_t i = 0; i <= pasos; i++) { float t = (float)i / (float)pasos; float s = 0.5f * (1.0f - cosf((float)M_PI * t)); fijar_frecuencia_pasos(frec_inicial + (frec_final - frec_inicial) * s); vTaskDelay(pdMS_TO_TICKS(5)); }
}

/* =====================================================================
 * FUNCIONES PRINCIPALES DE LÓGICA
 * ===================================================================== */
typedef enum { RESULTADO_SOBREPASO_OK, RESULTADO_SOBREPASO_ENDSTOP } resultado_sobrepaso_t;

static resultado_sobrepaso_t realizar_verificacion_sobrepaso(void) {
    fijar_frecuencia_pasos(FREQ_VERIFICACION_HZ);
    arrancar_motor();
    if (leer_sensor_hall()) {
        esperar_hall_alto_estable();
        ESP_LOGD(TAG, "Verificación: Se salió del imán inicial para empezar a contar.");
    }
    const int64_t tiempo_inicio_us = esp_timer_get_time();
    int contador_bajo = 0;
    while (true) {
        uint32_t tiempo_transcurrido_ms = (uint32_t)((esp_timer_get_time() - tiempo_inicio_us) / 1000);
        if (leer_sensor_hall()) {
            if (++contador_bajo >= NUMERO_MUESTRAS_N) {
                ESP_LOGW(TAG, "¡FINAL DE CARRERA DETECTADO! (Segundo imán encontrado).");
                return RESULTADO_SOBREPASO_ENDSTOP;
            }
        } else {
            contador_bajo = 0;
        }
        if (tiempo_transcurrido_ms >= TIEMPO_SOBREPASO_MS) {
            ESP_LOGI(TAG, "Verificación OK. No se encontró un segundo imán.");
            return RESULTADO_SOBREPASO_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(MUESTRA_HALL_MS));
    }
}

static void gestionar_final_de_carrera(int *p_dir) {
    ESP_LOGW(TAG, "INICIANDO SECUENCIA DE ESCAPE DE ENDSTOP...");
    gpio_set_level(PIN_LED_D2, 1);
    *p_dir = (*p_dir == DERECHA) ? IZQUIERDA : DERECHA;
    fijar_direccion(*p_dir);
    fijar_frecuencia_pasos(FREQ_BUSQUEDA_HZ);
    arrancar_motor();
    ESP_LOGI(TAG, "Paso 1/3: Alejándose del imán de disparo...");
    esperar_hall_alto_estable();
    ESP_LOGI(TAG, "Paso 2/3: Buscando el imán de límite para ignorarlo...");
    esperar_hall_bajo_estable();
    ESP_LOGW(TAG, "Imán de límite detectado. IGNORANDO y pasando de largo...");
    ESP_LOGI(TAG, "Paso 3/3: Pasando de largo el imán de límite...");
    esperar_hall_alto_estable();
    ESP_LOGW(TAG, "SECUENCIA DE ESCAPE COMPLETA. Reanudando búsqueda normal de HOME.");
    gpio_set_level(PIN_LED_D2, 0);
}

// ===================== NUEVA FUNCIÓN DE CHEQUEO BIDIRECCIONAL =====================
/**
 * @brief Gestiona el caso especial de arrancar sobre un imán.
 * Chequea en ambas direcciones para determinar si es HOME o un ENDSTOP.
 * @param p_dir Puntero a la dirección actual.
 * @return true si se encontró y posicionó el HOME, false si era un ENDSTOP.
 */
static bool gestionar_arranque_sobre_iman(int *p_dir) {
    ESP_LOGW(TAG, "Arranque sobre imán. Iniciando chequeo bidireccional...");

    // --- CHEQUEO 1: HACIA LA DERECHA ---
    ESP_LOGI(TAG, "Chequeo Bidireccional (1/2): Probando hacia la DERECHA...");
    fijar_direccion(DERECHA);
    if (realizar_verificacion_sobrepaso() == RESULTADO_SOBREPASO_ENDSTOP) {
        ESP_LOGW(TAG, "ENDSTOP detectado a la derecha.");
        *p_dir = DERECHA; // La dirección actual es la que causó el endstop
        gestionar_final_de_carrera(p_dir); // Escapamos en la dirección contraria
        return false; // Continuar con la búsqueda normal
    }

    // --- Si llegamos aquí, el chequeo a la derecha fue OK. Volvemos y chequeamos a la izquierda ---
    ESP_LOGI(TAG, "Chequeo a la derecha OK. Volviendo al imán para probar hacia la IZQUIERDA...");
    fijar_direccion(IZQUIERDA);
    arrancar_motor();
    esperar_hall_bajo_estable(); // Volver al imán de partida

    // --- CHEQUEO 2: HACIA LA IZQUIERDA ---
    ESP_LOGI(TAG, "Chequeo Bidireccional (2/2): Probando hacia la IZQUIERDA...");
    if (realizar_verificacion_sobrepaso() == RESULTADO_SOBREPASO_ENDSTOP) {
        ESP_LOGW(TAG, "ENDSTOP detectado a la izquierda.");
        *p_dir = IZQUIERDA; // La dirección actual es la que causó el endstop
        gestionar_final_de_carrera(p_dir); // Escapamos en la dirección contraria
        return false; // Continuar con la búsqueda normal
    }

    // --- Si llegamos aquí, AMBOS chequeos fueron OK. Es el HOME. ---
    ESP_LOGI(TAG, "Chequeo bidireccional completo. Confirmado: arranque en HOME. Posicionando...");
    fijar_direccion(DERECHA); // Volvemos a la derecha para centrarnos en el imán
    arrancar_motor();
    esperar_hall_bajo_estable();

    // Posicionamiento final
    parar_motor();
    habilitar_driver(false);
    gpio_set_level(PIN_LED_D2, 1);
    ESP_LOGI(TAG, "¡HOME confirmado y posicionado en arranque!");
    return true; // Homing completado
}


static bool verificar_si_es_home_o_final_carrera(int *p_dir) {
    rampa_de_velocidad(FREQ_BUSQUEDA_HZ, FREQ_VERIFICACION_HZ, TIEMPO_RAMPA_MS);
    
    if (realizar_verificacion_sobrepaso() == RESULTADO_SOBREPASO_ENDSTOP) {
        gestionar_final_de_carrera(p_dir);
        return false;
    }

    ESP_LOGI(TAG, "Acción: HOME encontrado. Volviendo para posicionamiento final.");
    *p_dir = (*p_dir == DERECHA) ? IZQUIERDA : DERECHA;
    fijar_direccion(*p_dir);
    fijar_frecuencia_pasos(FREQ_VERIFICACION_HZ);
    arrancar_motor();
    esperar_hall_bajo_estable();

    parar_motor();
    habilitar_driver(false);
    gpio_set_level(PIN_LED_D2, 1);
    ESP_LOGI(TAG, "¡HOME ENCONTRADO Y POSICIONADO! Secuencia finalizada.");
    return true;
}

/* =====================================================================
 * TAREA PRINCIPAL Y app_main
 * ===================================================================== */
void tarea_de_busqueda_home(void) {
    int direccion_actual = DERECHA;
    fijar_direccion(direccion_actual);

    // --- LÓGICA DE ARRANQUE MODIFICADA ---
    if (leer_sensor_hall()) {
        // Llamamos a la nueva función que hace todo el trabajo.
        // Si devuelve true, el homing terminó y salimos de la tarea.
        if (gestionar_arranque_sobre_iman(&direccion_actual)) {
            return;
        }
        // Si devuelve false, era un endstop y la función de escape
        // ya dejó el motor andando en la dirección correcta para continuar.
    } else {
        ESP_LOGI(TAG, "Arranque normal. Iniciando búsqueda de HOME...");
    }

    ESP_LOGI(TAG, "Motor en marcha a %d Hz...", (int)FREQ_BUSQUEDA_HZ);
    fijar_frecuencia_pasos(FREQ_BUSQUEDA_HZ);
    arrancar_motor();

    while (true) {
        esperar_hall_bajo_estable();
        ESP_LOGI(TAG, "Imán detectado. Iniciando análisis...");

        if (verificar_si_es_home_o_final_carrera(&direccion_actual)) {
            break;
        }
        
        ESP_LOGI(TAG, "Continuando búsqueda en la nueva dirección...");
    }

    ESP_LOGI(TAG, "Tarea de Homing finalizada.");
}

static void inicializar_gpios(void) {
    gpio_config_t out_conf = { .pin_bit_mask = (1ULL << PIN_DIR) | (1ULL << PIN_EN) | (1ULL << PIN_LED_D2), .mode = GPIO_MODE_OUTPUT, };
    gpio_config(&out_conf);
    gpio_config_t in_conf = { .pin_bit_mask = (1ULL << PIN_HALL), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, };
    gpio_config(&in_conf);
    gpio_set_level(PIN_DIR, DERECHA);
    gpio_set_level(PIN_EN, 1);
    gpio_set_level(PIN_LED_D2, 0);
}

void app_main(void) {
    ESP_LOGI(TAG, "Inicializando sistema de Homing (con chequeo bidireccional)...");
    inicializar_gpios();
    inicializar_pwm(FREQ_BUSQUEDA_HZ);

    habilitar_driver(true);
    vTaskDelay(pdMS_TO_TICKS(20));

    tarea_de_busqueda_home();

    ESP_LOGI(TAG, "app_main: Proceso completado.");
}
