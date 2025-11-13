/*
 * ESP32 CNC FIRMWARE: ORIGINAL HOMING + MANUAL JOGGING
 * BASE: Tu código "HOMING_ALL_AXES" (Versión estable)
 * AGREGADO: Comandos SPI 601, 602, 603 para movimiento manual.
 * AGREGADO: Respuesta 501 al terminar Homing.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/spi_slave.h"

static const char *TAG = "CNC_CONTROLLER";

/* ============================================================
 * DEFINES DE SPI SLAVE
 * ============================================================ */
#define SPI_HOST_ID   SPI2_HOST
#define PIN_SPI_MOSI  13
#define PIN_SPI_MISO  12
#define PIN_SPI_SCK   14
#define PIN_SPI_CS    15

// Flag de seguridad
static volatile bool is_sequence_running = false;

// Frecuencia para Jogging Manual
#define JOG_FREQ_HZ   1500.0f 

/* ============================================================
 * EJE 1 (TB6560 STYLE) - CONFIG ORIGINAL
 * ============================================================ */
#define PIN_TB6560_STEP   1
#define PIN_TB6560_DIR    22
#define PIN_TB6560_EN     23
#define PIN_HALL_1        26

#define MOTOR_STEPS_PER_REV_1   200U
#define MICROSTEP_1             8U
#define HALF_TURN_STEPS_1   ((MOTOR_STEPS_PER_REV_1 * MICROSTEP_1) / 2U)

#define FREQ_MIN_HZ_1        400.0f
#define FREQ_PASS1_HZ_1      900.0f
#define FREQ_PASS2_HZ_1      450.0f
#define FREQ_ALIGN_HZ_1      250.0f
#define FREQ_RELEASE_HZ_1    500.0f
#define FREQ_HALF_TURN_1     800.0f

#define RAMP_MS_1            1200U
#define OVERTRAVEL_MS_1      15U
#define SEPARATE_MS_1        60U
#define HALL_SAMPLE_MS_1     1U
#define HALL_DEBOUNCE_N_1    3

#define LEDC_MODE_1          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_1_ID      LEDC_TIMER_0
#define LEDC_CH_1            LEDC_CHANNEL_0
#define LEDC_RES_1           LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_1       (1 << (LEDC_RES_1 - 1)) // 512
#define LEDC_DUTY_OFF_1      0


/* ============================================================
 * EJE 2 - CONFIG ORIGINAL
 * ============================================================ */
#define PIN_STEP_2       19
#define PIN_DIR_2        21
#define PIN_EN_2         3
#define PIN_HALL_2       27
#define PIN_LED_2        2

#define DERECHA_2    1
#define IZQUIERDA_2  0

#define LEDC_MODE_2          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_2_ID      LEDC_TIMER_1
#define LEDC_CH_2            LEDC_CHANNEL_1
#define LEDC_RES_2           LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_2       (1 << (LEDC_RES_2 - 1))
#define LEDC_DUTY_OFF_2      0

#define FREQ_BUSQUEDA_HZ_2      1000.0f
#define FREQ_VERIFICACION_HZ_2  400.0f
#define TIEMPO_RAMPA_MS_2       120U
#define TIEMPO_SOBREPASO_MS_2   4000U
#define HALL_SAMPLE_MS_2        1U
#define HALL_DEBOUNCE_N_2       2


/* ============================================================
 * EJE 3 - CONFIG ORIGINAL
 * ============================================================ */
#define PIN_STEP_3       17
#define PIN_DIR_3        5
#define PIN_EN_3         18
#define PIN_HALL_3       25
#define PIN_LED_3        4

#define DERECHA_3    1
#define IZQUIERDA_3  0

#define LEDC_MODE_3          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_3_ID      LEDC_TIMER_2
#define LEDC_CH_3            LEDC_CHANNEL_2
#define LEDC_RES_3           LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_3       (1 << (LEDC_RES_3 - 1))
#define LEDC_DUTY_OFF_3      0

#define FREQ_BUSQUEDA_HZ_3        1000.0f
#define FREQ_VERIFICACION_HZ_3     400.0f
#define TIEMPO_RAMPA_MS_3           120U
#define TIEMPO_SOBREPASO_MS_3      4000U
#define HALL_SAMPLE_MS_3           1U
#define HALL_DEBOUNCE_N_3          2

/* ============================================================
 * PROTOTIPOS ORIGINALES
 * ============================================================ */
static void homing_eje1(void);
static void homing_eje2(void);
static void homing_eje3(void);

/* ============================================================
 * UTILIDADES EJE1 (CÓDIGO ORIGINAL SIN CAMBIOS)
 * ============================================================ */
static inline void eje1_enable_driver(bool on) { gpio_set_level(PIN_TB6560_EN, on ? 0 : 1); }
static inline void eje1_set_dir(bool cw) { gpio_set_level(PIN_TB6560_DIR, cw ? 1 : 0); }
static inline int eje1_hall_raw(void) { return (gpio_get_level(PIN_HALL_1) == 0); }

static int eje1_wait_hall_active(void) {
    int cnt = 0;
    for(;;){
        if (eje1_hall_raw()) cnt++; else cnt = 0;
        if (cnt >= HALL_DEBOUNCE_N_1) return 1;
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_1));
    }
}
static int eje1_wait_hall_inactive(void) {
    int cnt = 0;
    for(;;){
        if (!eje1_hall_raw()) cnt++; else cnt = 0;
        if (cnt >= HALL_DEBOUNCE_N_1) return 1;
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_1));
    }
}
static void eje1_pwm_init(float start_hz) {
    ledc_timer_config_t t = { .speed_mode=LEDC_MODE_1, .duty_resolution=LEDC_RES_1, .timer_num=LEDC_TIMER_1_ID, .freq_hz=(uint32_t)start_hz, .clk_cfg=LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num=PIN_TB6560_STEP, .speed_mode=LEDC_MODE_1, .channel=LEDC_CH_1, .timer_sel=LEDC_TIMER_1_ID, .duty=LEDC_DUTY_ON_1, .hpoint=0 };
    ledc_channel_config(&c);
}
static inline void eje1_pwm_start(void) { ledc_set_duty(LEDC_MODE_1, LEDC_CH_1, LEDC_DUTY_ON_1); ledc_update_duty(LEDC_MODE_1, LEDC_CH_1); }
static inline void eje1_pwm_stop(void) { ledc_stop(LEDC_MODE_1, LEDC_CH_1, LEDC_DUTY_OFF_1); }
static inline void eje1_set_freq(float hz) {
    if (hz < FREQ_ALIGN_HZ_1) hz = FREQ_ALIGN_HZ_1;
    ledc_set_freq(LEDC_MODE_1, LEDC_TIMER_1_ID, (uint32_t)hz);
}
static void eje1_ramp_freq(float f0, float f1, uint32_t ramp_ms) {
    TickType_t tick = pdMS_TO_TICKS(5);
    uint32_t steps = ramp_ms / 5U;
    if (steps == 0){ eje1_set_freq(f1); return; }
    eje1_pwm_start();
    for(uint32_t i=0; i<=steps; i++){
        float u = (float)i / (float)steps;
        float s = 0.5f * (1.0f - cosf((float)M_PI * u));
        eje1_set_freq(f0 + (f1 - f0) * s);
        vTaskDelay(tick);
    }
}

// --- Lógica Homing Eje 1 (Original) ---
static void eje1_pass1_seek_cw(void) {
    if (eje1_hall_raw()) {
        eje1_set_dir(false); // CCW
        eje1_set_freq(FREQ_RELEASE_HZ_1); eje1_pwm_start();
        eje1_wait_hall_inactive();
        vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1)); eje1_pwm_stop();
    }
    eje1_set_dir(true); // CW
    eje1_ramp_freq(FREQ_MIN_HZ_1, FREQ_PASS1_HZ_1, RAMP_MS_1);
    ESP_LOGI(TAG, "[EJE1] P1: Buscando Hall en CW");
    eje1_pwm_start(); eje1_wait_hall_active(); eje1_pwm_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    eje1_set_dir(true); // CW
    eje1_set_freq(FREQ_ALIGN_HZ_1); eje1_pwm_start();
    vTaskDelay(pdMS_TO_TICKS(OVERTRAVEL_MS_1)); eje1_pwm_stop();
}
static void eje1_pass2_return_align_ccw(void) {
    if (eje1_hall_raw()) {
        eje1_set_dir(true); // CW
        eje1_set_freq(FREQ_RELEASE_HZ_1); eje1_pwm_start();
        uint32_t t0 = (uint32_t)esp_log_timestamp();
        while (eje1_hall_raw() && (esp_log_timestamp() - t0) < 3000) vTaskDelay(pdMS_TO_TICKS(5));
        eje1_pwm_stop(); vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1));
    }
    eje1_set_dir(false); // CCW
    eje1_ramp_freq(FREQ_MIN_HZ_1, FREQ_PASS2_HZ_1, 800U);
    ESP_LOGI(TAG, "[EJE1] P2: Regresando lento CCW");
    eje1_pwm_start();
    uint32_t t1 = (uint32_t)esp_log_timestamp();
    while (!eje1_hall_raw() && (esp_log_timestamp() - t1) < 4000) vTaskDelay(pdMS_TO_TICKS(2));
    eje1_pwm_stop();
    eje1_set_dir(false); // CCW
    eje1_set_freq(FREQ_ALIGN_HZ_1); eje1_pwm_start();
    if (!eje1_hall_raw()) eje1_wait_hall_active();
    vTaskDelay(pdMS_TO_TICKS(20)); eje1_pwm_stop();
}

// GPIO Init Eje 1
static void eje1_gpio_init(void) {
    gpio_config_t out_conf = { .pin_bit_mask = (1ULL << PIN_TB6560_STEP)|(1ULL << PIN_TB6560_DIR)|(1ULL << PIN_TB6560_EN), .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0, .pull_down_en = 0, .intr_type = 0 };
    gpio_config(&out_conf);
    gpio_config_t in_conf = { .pin_bit_mask = (1ULL << PIN_HALL_1), .mode = GPIO_MODE_INPUT, .pull_up_en = 1, .pull_down_en = 0, .intr_type = 0 };
    gpio_config(&in_conf);
    gpio_set_level(PIN_TB6560_EN, 1); gpio_set_level(PIN_TB6560_DIR, 1);
}

// Homing Eje 1 Principal
static void homing_eje1(void) {
    ESP_LOGI(TAG, "[EJE1] Inicio homing 2-pass CW/CCW");
    eje1_gpio_init();
    eje1_pwm_init(FREQ_MIN_HZ_1);
    eje1_enable_driver(true);
    if (eje1_hall_raw()) {
        double seconds = (double)HALF_TURN_STEPS_1 / (double)FREQ_HALF_TURN_1;
        eje1_set_dir(0); eje1_set_freq(FREQ_HALF_TURN_1);
        eje1_pwm_start(); vTaskDelay(pdMS_TO_TICKS((uint32_t)(seconds * 1000.0 + 0.5)));
        eje1_pwm_stop(); vTaskDelay(pdMS_TO_TICKS(60));
    }
    eje1_pass1_seek_cw();
    eje1_pass2_return_align_ccw();
    eje1_enable_driver(false); eje1_pwm_stop();
    ESP_LOGI(TAG, "[EJE1] Homing COMPLETO.");
}


/* ============================================================
 * UTILIDADES EJE 2 (CÓDIGO ORIGINAL SIN CAMBIOS)
 * ============================================================ */
static inline int eje2_hall_read(void) { return (gpio_get_level(PIN_HALL_2) == 0); }
static void eje2_wait_hall_low_estable(void) {
    int cnt = 0;
    for(;;){ if (eje2_hall_read()) { if (++cnt >= HALL_DEBOUNCE_N_2) return; } else { cnt = 0; } vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2)); }
}
static void eje2_wait_hall_high_estable(void) {
    int cnt = 0;
    for(;;){ if (!eje2_hall_read()) { if (++cnt >= HALL_DEBOUNCE_N_2) return; } else { cnt = 0; } vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2)); }
}
static inline void eje2_enable_driver(bool on) { gpio_set_level(PIN_EN_2, on ? 0 : 1); }
static inline void eje2_set_dir(int dir) { gpio_set_level(PIN_DIR_2, dir); esp_rom_delay_us(20); }
static void eje2_pwm_init(float start_hz) {
    ledc_timer_config_t t = { .speed_mode = LEDC_MODE_2, .duty_resolution = LEDC_RES_2, .timer_num = LEDC_TIMER_2_ID, .freq_hz = (uint32_t)start_hz, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num = PIN_STEP_2, .speed_mode = LEDC_MODE_2, .channel = LEDC_CH_2, .timer_sel = LEDC_TIMER_2_ID, .duty = LEDC_DUTY_OFF_2, .hpoint = 0 };
    ledc_channel_config(&c);
}
static inline void eje2_pwm_start(void) { ledc_set_duty(LEDC_MODE_2, LEDC_CH_2, LEDC_DUTY_ON_2); ledc_update_duty(LEDC_MODE_2, LEDC_CH_2); }
static inline void eje2_pwm_stop(void) { ledc_stop(LEDC_MODE_2, LEDC_CH_2, LEDC_DUTY_OFF_2); }
static inline void eje2_set_freq(float hz) { if (hz < 1.0f) hz = 1.0f; ledc_set_freq(LEDC_MODE_2, LEDC_TIMER_2_ID, (uint32_t)hz); }
static void eje2_ramp_freq(float fi, float ff, uint32_t dur_ms) {
    if (dur_ms == 0 || fabsf(ff - fi) < 1.0f) { eje2_set_freq(ff); return; }
    uint32_t steps = dur_ms / 5U; if (!steps) steps = 1;
    for(uint32_t i=0; i<=steps; i++){
        float u = (float)i / (float)steps; float s = 0.5f * (1.0f - cosf((float)M_PI * u));
        eje2_set_freq(fi + (ff - fi) * s); vTaskDelay(pdMS_TO_TICKS(5));
    }
}


/* ============================================================
 * UTILIDADES EJE 3 (CÓDIGO ORIGINAL SIN CAMBIOS)
 * ============================================================ */
static inline int eje3_hall_read(void) { return (gpio_get_level(PIN_HALL_3) == 0); }
static void eje3_wait_hall_low_estable(void) {
    int cnt = 0;
    for(;;){ if (eje3_hall_read()) { if (++cnt >= HALL_DEBOUNCE_N_3) return; } else { cnt = 0; } vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_3)); }
}
static void eje3_wait_hall_high_estable(void) {
    int cnt = 0;
    for(;;){ if (!eje3_hall_read()) { if (++cnt >= HALL_DEBOUNCE_N_3) return; } else { cnt = 0; } vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_3)); }
}
static inline void eje3_enable_driver(bool on) { gpio_set_level(PIN_EN_3, on ? 0 : 1); }
static inline void eje3_set_dir(int dir) { gpio_set_level(PIN_DIR_3, dir); esp_rom_delay_us(20); }
static void eje3_pwm_init(float start_hz) {
    ledc_timer_config_t t = { .speed_mode = LEDC_MODE_3, .duty_resolution = LEDC_RES_3, .timer_num = LEDC_TIMER_3_ID, .freq_hz = (uint32_t)start_hz, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num = PIN_STEP_3, .speed_mode = LEDC_MODE_3, .channel = LEDC_CH_3, .timer_sel = LEDC_TIMER_3_ID, .duty = LEDC_DUTY_OFF_3, .hpoint = 0 };
    ledc_channel_config(&c);
}
static inline void eje3_pwm_start(void) { ledc_set_duty(LEDC_MODE_3, LEDC_CH_3, LEDC_DUTY_ON_3); ledc_update_duty(LEDC_MODE_3, LEDC_CH_3); }
static inline void eje3_pwm_stop(void) { ledc_stop(LEDC_MODE_3, LEDC_CH_3, LEDC_DUTY_OFF_3); }
static inline void eje3_set_freq(float hz) { if (hz < 1.0f) hz = 1.0f; ledc_set_freq(LEDC_MODE_3, LEDC_TIMER_3_ID, (uint32_t)hz); }
static void eje3_ramp_freq(float fi, float ff, uint32_t dur_ms) {
    if (dur_ms == 0 || fabsf(ff - fi) < 1.0f) { eje3_set_freq(ff); return; }
    uint32_t steps = dur_ms / 5U; if (!steps) steps = 1;
    for(uint32_t i=0; i<=steps; i++){
        float u = (float)i / (float)steps; float s = 0.5f * (1.0f - cosf((float)M_PI * u));
        eje3_set_freq(fi + (ff - fi) * s); vTaskDelay(pdMS_TO_TICKS(5));
    }
}


/* ============================================================
 * LOGICA INTELIGENTE EJES 2 Y 3 (ORIGINAL)
 * ============================================================ */
typedef enum { SOBREPASO_OK = 0, SOBREPASO_ENDSTOP = 1 } sobrepaso_result_t;

static sobrepaso_result_t eje2_verificar_sobrepaso(void) {
    eje2_set_freq(FREQ_VERIFICACION_HZ_2); eje2_pwm_start();
    if (eje2_hall_read()) eje2_wait_hall_high_estable();
    const int64_t t0 = esp_timer_get_time();
    int cnt_low = 0;
    while (1) {
        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - t0)/1000);
        if (eje2_hall_read()) {
            if (++cnt_low >= HALL_DEBOUNCE_N_2) { eje2_pwm_stop(); return SOBREPASO_ENDSTOP; }
        } else cnt_low = 0;
        if (elapsed_ms >= TIEMPO_SOBREPASO_MS_2) { eje2_pwm_stop(); return SOBREPASO_OK; }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2));
    }
}

static sobrepaso_result_t eje3_verificar_sobrepaso(void) {
    eje3_set_freq(FREQ_VERIFICACION_HZ_3); eje3_pwm_start();
    if (eje3_hall_read()) eje3_wait_hall_high_estable();
    const int64_t t0 = esp_timer_get_time();
    int cnt_low = 0;
    while (1) {
        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - t0)/1000);
        if (eje3_hall_read()) {
            if (++cnt_low >= HALL_DEBOUNCE_N_3) { eje3_pwm_stop(); return SOBREPASO_ENDSTOP; }
        } else cnt_low = 0;
        if (elapsed_ms >= TIEMPO_SOBREPASO_MS_3) { eje3_pwm_stop(); return SOBREPASO_OK; }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_3));
    }
}

static void eje2_escape_endstop(int *p_dir_actual) {
    gpio_set_level(PIN_LED_2, 1);
    *p_dir_actual = (*p_dir_actual == DERECHA_2) ? IZQUIERDA_2 : DERECHA_2;
    eje2_set_dir(*p_dir_actual); eje2_set_freq(FREQ_BUSQUEDA_HZ_2);
    eje2_pwm_start(); eje2_wait_hall_high_estable(); eje2_wait_hall_low_estable();
    eje2_wait_hall_high_estable(); eje2_pwm_stop();
    gpio_set_level(PIN_LED_2, 0);
}

static void eje3_escape_endstop(int *p_dir_actual) {
    gpio_set_level(PIN_LED_3, 1);
    *p_dir_actual = (*p_dir_actual == DERECHA_3) ? IZQUIERDA_3 : DERECHA_3;
    eje3_set_dir(*p_dir_actual); eje3_set_freq(FREQ_BUSQUEDA_HZ_3);
    eje3_pwm_start(); eje3_wait_hall_high_estable(); eje3_wait_hall_low_estable();
    eje3_wait_hall_high_estable(); eje3_pwm_stop();
    gpio_set_level(PIN_LED_3, 0);
}

static bool eje2_arranque_sobre_iman(int *p_dir_actual) {
    *p_dir_actual = DERECHA_2; eje2_set_dir(DERECHA_2);
    if (eje2_verificar_sobrepaso() == SOBREPASO_ENDSTOP) { eje2_escape_endstop(p_dir_actual); return false; }
    eje2_set_dir(IZQUIERDA_2); eje2_pwm_start(); eje2_wait_hall_low_estable(); eje2_pwm_stop();
    *p_dir_actual = IZQUIERDA_2; eje2_set_dir(IZQUIERDA_2);
    if (eje2_verificar_sobrepaso() == SOBREPASO_ENDSTOP) { eje2_escape_endstop(p_dir_actual); return false; }
    eje2_set_dir(DERECHA_2); eje2_pwm_start(); eje2_wait_hall_low_estable(); eje2_pwm_stop();
    eje2_enable_driver(false); gpio_set_level(PIN_LED_2, 1); return true;
}

static bool eje3_arranque_sobre_iman(int *p_dir_actual) {
    *p_dir_actual = DERECHA_3; eje3_set_dir(DERECHA_3);
    if (eje3_verificar_sobrepaso() == SOBREPASO_ENDSTOP) { eje3_escape_endstop(p_dir_actual); return false; }
    eje3_set_dir(IZQUIERDA_3); eje3_pwm_start(); eje3_wait_hall_low_estable(); eje3_pwm_stop();
    *p_dir_actual = IZQUIERDA_3; eje3_set_dir(IZQUIERDA_3);
    if (eje3_verificar_sobrepaso() == SOBREPASO_ENDSTOP) { eje3_escape_endstop(p_dir_actual); return false; }
    eje3_set_dir(DERECHA_3); eje3_pwm_start(); eje3_wait_hall_low_estable(); eje3_pwm_stop();
    eje3_enable_driver(false); gpio_set_level(PIN_LED_3, 1); return true;
}

static bool eje2_confirmar_home_o_endstop(int *p_dir_actual) {
    eje2_ramp_freq(FREQ_BUSQUEDA_HZ_2, FREQ_VERIFICACION_HZ_2, TIEMPO_RAMPA_MS_2);
    if (eje2_verificar_sobrepaso() == SOBREPASO_ENDSTOP) { eje2_escape_endstop(p_dir_actual); return false; }
    *p_dir_actual = (*p_dir_actual == DERECHA_2) ? IZQUIERDA_2 : DERECHA_2;
    eje2_set_dir(*p_dir_actual); eje2_set_freq(FREQ_VERIFICACION_HZ_2);
    eje2_pwm_start(); eje2_wait_hall_low_estable(); eje2_pwm_stop();
    eje2_enable_driver(false); gpio_set_level(PIN_LED_2, 1); return true;
}

static bool eje3_confirmar_home_o_endstop(int *p_dir_actual) {
    eje3_ramp_freq(FREQ_BUSQUEDA_HZ_3, FREQ_VERIFICACION_HZ_3, TIEMPO_RAMPA_MS_3);
    if (eje3_verificar_sobrepaso() == SOBREPASO_ENDSTOP) { eje3_escape_endstop(p_dir_actual); return false; }
    *p_dir_actual = (*p_dir_actual == DERECHA_3) ? IZQUIERDA_3 : DERECHA_3;
    eje3_set_dir(*p_dir_actual); eje3_set_freq(FREQ_VERIFICACION_HZ_3);
    eje3_pwm_start(); eje3_wait_hall_low_estable(); eje3_pwm_stop();
    eje3_enable_driver(false); gpio_set_level(PIN_LED_3, 1); return true;
}

// GPIO INIT
static void eje2_gpio_init(void) {
    gpio_config_t out = { .pin_bit_mask=(1ULL<<PIN_DIR_2)|(1ULL<<PIN_EN_2)|(1ULL<<PIN_STEP_2)|(1ULL<<PIN_LED_2), .mode=GPIO_MODE_OUTPUT }; gpio_config(&out);
    gpio_config_t in = { .pin_bit_mask=(1ULL<<PIN_HALL_2), .mode=GPIO_MODE_INPUT, .pull_up_en=1 }; gpio_config(&in);
    gpio_set_level(PIN_DIR_2, DERECHA_2); gpio_set_level(PIN_EN_2, 1);
}
static void eje3_gpio_init(void) {
    gpio_config_t out = { .pin_bit_mask=(1ULL<<PIN_DIR_3)|(1ULL<<PIN_EN_3)|(1ULL<<PIN_STEP_3)|(1ULL<<PIN_LED_3), .mode=GPIO_MODE_OUTPUT }; gpio_config(&out);
    gpio_config_t in = { .pin_bit_mask=(1ULL<<PIN_HALL_3), .mode=GPIO_MODE_INPUT, .pull_up_en=1 }; gpio_config(&in);
    gpio_set_level(PIN_DIR_3, DERECHA_3); gpio_set_level(PIN_EN_3, 1);
}

// HOMING LOOPS
static void homing_eje2(void) {
    eje2_gpio_init(); eje2_pwm_init(FREQ_BUSQUEDA_HZ_2); eje2_enable_driver(true);
    int dir = DERECHA_2; eje2_set_dir(dir); vTaskDelay(pdMS_TO_TICKS(20));
    if (eje2_hall_read()) { if (eje2_arranque_sobre_iman(&dir)) return; }
    eje2_set_freq(FREQ_BUSQUEDA_HZ_2); eje2_pwm_start();
    while (1) {
        eje2_wait_hall_low_estable(); eje2_pwm_stop();
        if (eje2_confirmar_home_o_endstop(&dir)) break;
        eje2_set_freq(FREQ_BUSQUEDA_HZ_2); eje2_pwm_start();
    }
    eje2_enable_driver(false); eje2_pwm_stop();
}

static void homing_eje3(void) {
    eje3_gpio_init(); eje3_pwm_init(FREQ_BUSQUEDA_HZ_3); eje3_enable_driver(true);
    int dir = DERECHA_3; eje3_set_dir(dir); vTaskDelay(pdMS_TO_TICKS(20));
    if (eje3_hall_read()) { if (eje3_arranque_sobre_iman(&dir)) return; }
    eje3_set_freq(FREQ_BUSQUEDA_HZ_3); eje3_pwm_start();
    while (1) {
        eje3_wait_hall_low_estable(); eje3_pwm_stop();
        if (eje3_confirmar_home_o_endstop(&dir)) break;
        eje3_set_freq(FREQ_BUSQUEDA_HZ_3); eje3_pwm_start();
    }
    eje3_enable_driver(false); eje3_pwm_stop();
}

static void homing_sequence_task(void *arg) {
    is_sequence_running = true;
    ESP_LOGI(TAG, "--- SECUENCIA HOMING INICIADA ---");
    homing_eje1(); vTaskDelay(pdMS_TO_TICKS(500));
    homing_eje2(); vTaskDelay(pdMS_TO_TICKS(500));
    homing_eje3();
    ESP_LOGI(TAG, "--- SECUENCIA FINALIZADA ---");
    is_sequence_running = false;
    vTaskDelete(NULL);
}


/* ============================================================
 * [NUEVO] LÓGICA DE JOGGING MANUAL (Insertado sin romper homing)
 * ============================================================ */
static void handle_jog(int cmd, int arg) {
    // Configurar punteros a las funciones de cada eje
    // Mapeo manual porque las funciones "ejeX_..." son estáticas y hardcodeadas
    
    // Eje 1
    if (cmd == 601) {
        // Apagar otros
        eje2_enable_driver(false); eje2_pwm_stop();
        eje3_enable_driver(false); eje3_pwm_stop();
        
        // Inicializar si no se hizo (o reasegurar)
        // Nota: Usamos init para configurar PWM si estaba apagado
        ledc_timer_config_t t = { .speed_mode=LEDC_MODE_1, .duty_resolution=LEDC_RES_1, .timer_num=LEDC_TIMER_1_ID, .freq_hz=(uint32_t)JOG_FREQ_HZ, .clk_cfg=LEDC_AUTO_CLK };
        ledc_timer_config(&t);
        ledc_channel_config_t c = { .gpio_num=PIN_TB6560_STEP, .speed_mode=LEDC_MODE_1, .channel=LEDC_CH_1, .timer_sel=LEDC_TIMER_1_ID, .duty=0 };
        ledc_channel_config(&c);
        
        if (arg == 0) {
            eje1_pwm_stop();
        } else {
            eje1_enable_driver(true);
            eje1_set_dir(arg == 1); // 1=CW, 2=CCW -> arg==1 es true(CW), arg==2 false(CCW)
            eje1_set_freq(JOG_FREQ_HZ);
            eje1_pwm_start();
        }
    }
    // Eje 2
    else if (cmd == 602) {
        eje1_enable_driver(false); eje1_pwm_stop();
        eje3_enable_driver(false); eje3_pwm_stop();
        
        eje2_pwm_init(JOG_FREQ_HZ);
        
        if (arg == 0) {
            eje2_pwm_stop();
        } else {
            eje2_enable_driver(true);
            eje2_set_dir((arg == 1) ? DERECHA_2 : IZQUIERDA_2);
            eje2_set_freq(JOG_FREQ_HZ);
            eje2_pwm_start();
        }
    }
    // Eje 3
    else if (cmd == 603) {
        eje1_enable_driver(false); eje1_pwm_stop();
        eje2_enable_driver(false); eje2_pwm_stop();
        
        eje3_pwm_init(JOG_FREQ_HZ);
        
        if (arg == 0) {
            eje3_pwm_stop();
        } else {
            eje3_enable_driver(true);
            eje3_set_dir((arg == 1) ? DERECHA_3 : IZQUIERDA_3);
            eje3_set_freq(JOG_FREQ_HZ);
            eje3_pwm_start();
        }
    }
}


/* ============================================================
 * TAREA SPI SLAVE (MODIFICADA)
 * ============================================================ */
static void spi_slave_task(void *arg) {
    spi_bus_config_t buscfg = { .mosi_io_num=PIN_SPI_MOSI, .miso_io_num=PIN_SPI_MISO, .sclk_io_num=PIN_SPI_SCK, .quadwp_io_num=-1, .quadhd_io_num=-1, .max_transfer_sz=32 };
    spi_slave_interface_config_t slvcfg = { .spics_io_num=PIN_SPI_CS, .flags=0, .queue_size=3, .mode=1 };
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST_ID, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    WORD_ALIGNED_ATTR uint8_t rx[16];
    WORD_ALIGNED_ATTR uint8_t tx[16];
    memset(tx, 0, 16); strcpy((char*)tx, "RDY");

    while (1) {
        spi_slave_transaction_t t = { .length=12*8, .tx_buffer=tx, .rx_buffer=rx };
        spi_slave_transmit(SPI_HOST_ID, &t, portMAX_DELAY);

        int32_t cmd=0, arg1=0;
        memcpy(&cmd, rx, 4); memcpy(&arg1, rx+4, 4);
        
        // ACTUALIZAR ESTADO PARA LA PRÓXIMA LECTURA
        if (!is_sequence_running) sprintf((char*)tx, "501"); // IDLE / DONE
        else sprintf((char*)tx, "BSY");

        // Bloquear JOG si hay Homing
        if (is_sequence_running && cmd >= 600) continue;

        if (cmd == 500 && !is_sequence_running) {
             xTaskCreate(homing_sequence_task, "HomeTask", 4096, NULL, 5, NULL);
        } 
        else if (cmd >= 601 && cmd <= 603) {
            handle_jog(cmd, arg1);
        }
    }
}

void app_main(void) {
    // Init básico para garantizar pines en estado seguro antes de SPI
    eje1_gpio_init(); eje2_gpio_init(); eje3_gpio_init();
    xTaskCreatePinnedToCore(spi_slave_task, "spi_task", 4096, NULL, 5, NULL, 1);
}
