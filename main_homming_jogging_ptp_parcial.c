#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h" // Necesario para el PTP Paralelo
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/spi_slave.h"

static const char *TAG = "ROBOT_HYBRID_MASTER";

/* ============================================================
 * CONFIGURACIÓN GLOBAL Y SINCRONIZACIÓN
 * ============================================================ */
// Event Group para mover los 3 ejes a la vez (Solo usado en PTP)
#define BIT_EJE1_DONE  (1 << 0)
#define BIT_EJE2_DONE  (1 << 1)
#define BIT_EJE3_DONE  (1 << 2)
#define ALL_AXES_DONE  (BIT_EJE1_DONE | BIT_EJE2_DONE | BIT_EJE3_DONE)
static EventGroupHandle_t robot_event_group;

// SPI
#define SPI_HOST_ID   SPI2_HOST
#define PIN_SPI_MOSI  13
#define PIN_SPI_MISO  12
#define PIN_SPI_SCK   14
#define PIN_SPI_CS    15

// Estado y Variables PTP
static volatile bool is_busy = false; 
static volatile int32_t target_steps_1 = 0;
static volatile int32_t target_steps_2 = 0;
static volatile int32_t target_steps_3 = 0;

#define JOG_FREQ_HZ       1500.0f 
#define PTP_SPEED_HZ      900.0f // Velocidad rápida para PTP

/* ============================================================
 * DEFINICIONES DE HARDWARE
 * ============================================================ */

// --- EJE 1 (Base) ---
#define PIN_TB6560_STEP   1
#define PIN_TB6560_DIR    22
#define PIN_TB6560_EN     23
#define PIN_HALL_1        26
#define LEDC_MODE_1       LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_1_ID   LEDC_TIMER_0
#define LEDC_CH_1         LEDC_CHANNEL_0
#define LEDC_RES_1        LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_1    (1 << (LEDC_RES_1 - 1)) 
#define LEDC_DUTY_OFF_1   0

// Params Homing Eje 1
#define MOTOR_STEPS_PER_REV_1 200U
#define MICROSTEP_1           8U
#define HALF_TURN_STEPS_1     ((MOTOR_STEPS_PER_REV_1 * MICROSTEP_1) / 2U)
#define FREQ_MIN_HZ_1         400.0f
#define FREQ_PASS1_HZ_1       900.0f
#define FREQ_PASS2_HZ_1       450.0f
#define FREQ_ALIGN_HZ_1       250.0f
#define FREQ_RELEASE_HZ_1     500.0f
#define FREQ_HALF_TURN_1      800.0f
#define RAMP_MS_1             1200U
#define OVERTRAVEL_MS_1       15U
#define SEPARATE_MS_1         60U
#define HALL_SAMPLE_MS_1      1U
#define HALL_DEBOUNCE_N_1     3

// --- EJE 2 (Brazo) ---
#define PIN_STEP_2        19
#define PIN_DIR_2         21
#define PIN_EN_2          3
#define PIN_HALL_2        27
#define PIN_LED_2         2
#define DERECHA_2         1
#define IZQUIERDA_2       0
#define LEDC_MODE_2       LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_2_ID   LEDC_TIMER_1
#define LEDC_CH_2         LEDC_CHANNEL_1
#define LEDC_RES_2        LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_2    (1 << (LEDC_RES_2 - 1))
#define LEDC_DUTY_OFF_2   0

// Homing Params Eje 2
#define FREQ_BUSQUEDA_HZ_2      1000.0f
#define FREQ_VERIFICACION_HZ_2   400.0f
#define TIEMPO_RAMPA_MS_2        120U
#define TIEMPO_SOBREPASO_MS_2    4000U
#define HALL_SAMPLE_MS_2         1U
#define HALL_DEBOUNCE_N_2        2

// --- EJE 3 (Antebrazo) - CORREGIDO ---
//  -> Pin 5 es VSPI CS (Conflicto). Pin 32 es seguro.
#define PIN_STEP_3        33  
#define PIN_DIR_3         5  // <--- CAMBIADO DE 5 A 32 PARA EVITAR OSCILACIÓN
#define PIN_EN_3          18  
#define PIN_HALL_3        25
#define PIN_LED_3         4
#define DERECHA_3         1
#define IZQUIERDA_3       0
#define LEDC_MODE_3       LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_3_ID   LEDC_TIMER_2
#define LEDC_CH_3         LEDC_CHANNEL_2
#define LEDC_RES_3        LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_3    (1 << (LEDC_RES_3 - 1))
#define LEDC_DUTY_OFF_3   0

// Homing Params Eje 3
#define FREQ_BUSQUEDA_HZ_3      1000.0f
#define FREQ_VERIFICACION_HZ_3   400.0f
#define TIEMPO_RAMPA_MS_3        120U
#define TIEMPO_SOBREPASO_MS_3    4000U
#define HALL_SAMPLE_MS_3         1U
#define HALL_DEBOUNCE_N_3        2


/* ============================================================
 * PROTOTIPOS
 * ============================================================ */
static void homing_eje1(void);
static void homing_eje2(void);
static void homing_eje3(void);
static void handle_jog(int cmd, int arg1);

static void eje1_gpio_init(void); static void eje1_pwm_init(float hz);
static void eje2_gpio_init(void); static void eje2_pwm_init(float hz);
static void eje3_gpio_init(void); static void eje3_pwm_init(float hz);

static inline void eje1_pwm_start(void); static inline void eje1_pwm_stop(void);
static inline void eje2_pwm_start(void); static inline void eje2_pwm_stop(void);
static inline void eje3_pwm_start(void); static inline void eje3_pwm_stop(void);

static inline void eje1_set_freq(float hz);
static inline void eje2_set_freq(float hz);
static inline void eje3_set_freq(float hz);

/* ============================================================
 * IMPLEMENTACIÓN BAJO NIVEL EJE 1 (TB6560)
 * ============================================================ */
static inline void eje1_enable_driver(bool on) { gpio_set_level(PIN_TB6560_EN, on ? 0 : 1); }
static inline void eje1_set_dir(bool cw)       { gpio_set_level(PIN_TB6560_DIR, cw ? 1 : 0); }
static inline int eje1_hall_raw(void)          { return (gpio_get_level(PIN_HALL_1) == 0); }

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
static inline void eje1_pwm_stop(void)  { ledc_stop(LEDC_MODE_1, LEDC_CH_1, LEDC_DUTY_OFF_1); }
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

static void eje1_pass1_seek_cw(void) {
    if (eje1_hall_raw()) {
        eje1_set_dir(false); eje1_set_freq(FREQ_RELEASE_HZ_1); eje1_pwm_start();
        eje1_wait_hall_inactive(); vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1)); eje1_pwm_stop();
    }
    eje1_set_dir(true); 
    eje1_ramp_freq(FREQ_MIN_HZ_1, FREQ_PASS1_HZ_1, RAMP_MS_1);
    eje1_pwm_start(); eje1_wait_hall_active(); eje1_pwm_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    eje1_set_dir(true); eje1_set_freq(FREQ_ALIGN_HZ_1); eje1_pwm_start();
    vTaskDelay(pdMS_TO_TICKS(OVERTRAVEL_MS_1)); eje1_pwm_stop();
}

static void eje1_pass2_return_align_ccw(void) {
    if (eje1_hall_raw()) {
        eje1_set_dir(true); eje1_set_freq(FREQ_RELEASE_HZ_1); eje1_pwm_start();
        uint32_t t0 = (uint32_t)esp_log_timestamp();
        while (eje1_hall_raw() && (esp_log_timestamp()-t0)<3000) vTaskDelay(pdMS_TO_TICKS(5));
        eje1_pwm_stop(); vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1));
    }
    eje1_set_dir(false); 
    eje1_ramp_freq(FREQ_MIN_HZ_1, FREQ_PASS2_HZ_1, 800U);
    eje1_pwm_start();
    uint32_t t1 = (uint32_t)esp_log_timestamp();
    while (!eje1_hall_raw() && (esp_log_timestamp()-t1)<4000) vTaskDelay(pdMS_TO_TICKS(2));
    eje1_pwm_stop();
    eje1_set_dir(false); eje1_set_freq(FREQ_ALIGN_HZ_1); eje1_pwm_start();
    if (!eje1_hall_raw()) eje1_wait_hall_active();
    vTaskDelay(pdMS_TO_TICKS(20)); eje1_pwm_stop();
}

static void eje1_gpio_init(void) {
    gpio_config_t out = { .pin_bit_mask=(1ULL<<PIN_TB6560_STEP)|(1ULL<<PIN_TB6560_DIR)|(1ULL<<PIN_TB6560_EN), .mode=GPIO_MODE_OUTPUT };
    gpio_config(&out);
    gpio_config_t in = { .pin_bit_mask=(1ULL<<PIN_HALL_1), .mode=GPIO_MODE_INPUT, .pull_up_en=1 };
    gpio_config(&in);
    gpio_set_level(PIN_TB6560_EN, 1); gpio_set_level(PIN_TB6560_DIR, 1);
}

static void homing_eje1(void) {
    ESP_LOGI(TAG, "[EJE1] Homing secuencial...");
    eje1_gpio_init(); eje1_pwm_init(FREQ_MIN_HZ_1); eje1_enable_driver(true);
    if (eje1_hall_raw()) {
        double s = (double)HALF_TURN_STEPS_1 / (double)FREQ_HALF_TURN_1;
        uint32_t ms = (uint32_t)(s*1000.0+0.5);
        eje1_set_dir(0); eje1_set_freq(FREQ_HALF_TURN_1); eje1_pwm_start();
        vTaskDelay(pdMS_TO_TICKS(ms)); eje1_pwm_stop(); vTaskDelay(pdMS_TO_TICKS(60));
    }
    eje1_pass1_seek_cw();
    eje1_pass2_return_align_ccw();
    eje1_enable_driver(false); eje1_pwm_stop();
    ESP_LOGI(TAG, "[EJE1] Homing OK.");
}


/* ============================================================
 * IMPLEMENTACIÓN BAJO NIVEL EJE 2 & 3
 * ============================================================ */
// Helpers Eje 2
static inline int eje2_hall_read(void) { return (gpio_get_level(PIN_HALL_2) == 0); }
static void eje2_wait_hall_low(void) { int c=0; for(;;){ if(eje2_hall_read()){ if(++c>=2)return; }else c=0; vTaskDelay(1); } }
static void eje2_wait_hall_high(void){ int c=0; for(;;){ if(!eje2_hall_read()){ if(++c>=2)return; }else c=0; vTaskDelay(1); } }
static inline void eje2_enable_driver(bool on) { gpio_set_level(PIN_EN_2, on ? 0 : 1); }
static inline void eje2_set_dir(int d) { gpio_set_level(PIN_DIR_2, d); esp_rom_delay_us(20); }
static void eje2_pwm_init(float hz) {
    ledc_timer_config_t t = {.speed_mode=LEDC_MODE_2, .duty_resolution=LEDC_RES_2, .timer_num=LEDC_TIMER_2_ID, .freq_hz=(uint32_t)hz, .clk_cfg=LEDC_AUTO_CLK}; ledc_timer_config(&t);
    ledc_channel_config_t c = {.gpio_num=PIN_STEP_2, .speed_mode=LEDC_MODE_2, .channel=LEDC_CH_2, .timer_sel=LEDC_TIMER_2_ID, .duty=LEDC_DUTY_OFF_2}; ledc_channel_config(&c);
}
static inline void eje2_pwm_start() { ledc_set_duty(LEDC_MODE_2, LEDC_CH_2, LEDC_DUTY_ON_2); ledc_update_duty(LEDC_MODE_2, LEDC_CH_2); }
static inline void eje2_pwm_stop()  { ledc_stop(LEDC_MODE_2, LEDC_CH_2, LEDC_DUTY_OFF_2); }
static inline void eje2_set_freq(float hz) { if(hz<1)hz=1; ledc_set_freq(LEDC_MODE_2, LEDC_TIMER_2_ID, (uint32_t)hz); }
static void eje2_ramp_freq(float fi, float ff, uint32_t ms) {
    uint32_t steps = ms/5; if(!steps) steps=1;
    for(uint32_t i=0; i<=steps; i++){ float u=(float)i/steps; float s=0.5f*(1.0f-cosf(M_PI*u)); eje2_set_freq(fi+(ff-fi)*s); vTaskDelay(pdMS_TO_TICKS(5)); }
}

// Helpers Eje 3
static inline int eje3_hall_read(void) { return (gpio_get_level(PIN_HALL_3) == 0); }
static void eje3_wait_hall_low(void) { int c=0; for(;;){ if(eje3_hall_read()){ if(++c>=2)return; }else c=0; vTaskDelay(1); } }
static void eje3_wait_hall_high(void){ int c=0; for(;;){ if(!eje3_hall_read()){ if(++c>=2)return; }else c=0; vTaskDelay(1); } }
static inline void eje3_enable_driver(bool on) { gpio_set_level(PIN_EN_3, on ? 0 : 1); }
static inline void eje3_set_dir(int d) { gpio_set_level(PIN_DIR_3, d); esp_rom_delay_us(20); }
static void eje3_pwm_init(float hz) {
    ledc_timer_config_t t = {.speed_mode=LEDC_MODE_3, .duty_resolution=LEDC_RES_3, .timer_num=LEDC_TIMER_3_ID, .freq_hz=(uint32_t)hz, .clk_cfg=LEDC_AUTO_CLK}; ledc_timer_config(&t);
    ledc_channel_config_t c = {.gpio_num=PIN_STEP_3, .speed_mode=LEDC_MODE_3, .channel=LEDC_CH_3, .timer_sel=LEDC_TIMER_3_ID, .duty=LEDC_DUTY_OFF_3}; ledc_channel_config(&c);
}
static inline void eje3_pwm_start() { ledc_set_duty(LEDC_MODE_3, LEDC_CH_3, LEDC_DUTY_ON_3); ledc_update_duty(LEDC_MODE_3, LEDC_CH_3); }
static inline void eje3_pwm_stop()  { ledc_stop(LEDC_MODE_3, LEDC_CH_3, LEDC_DUTY_OFF_3); }
static inline void eje3_set_freq(float hz) { if(hz<1)hz=1; ledc_set_freq(LEDC_MODE_3, LEDC_TIMER_3_ID, (uint32_t)hz); }
static void eje3_ramp_freq(float fi, float ff, uint32_t ms) {
    uint32_t steps = ms/5; if(!steps) steps=1;
    for(uint32_t i=0; i<=steps; i++){ float u=(float)i/steps; float s=0.5f*(1.0f-cosf(M_PI*u)); eje3_set_freq(fi+(ff-fi)*s); vTaskDelay(pdMS_TO_TICKS(5)); }
}

typedef enum { SOBREPASO_OK=0, SOBREPASO_ENDSTOP=1 } sobrepaso_res_t;

// Homing Logic Eje 2
static sobrepaso_res_t eje2_check_over(void) {
    eje2_set_freq(FREQ_VERIFICACION_HZ_2); eje2_pwm_start();
    if(eje2_hall_read()) eje2_wait_hall_high();
    int64_t t0 = esp_timer_get_time(); int cl=0;
    while(1) {
        if(eje2_hall_read()){ if(++cl>=2){ eje2_pwm_stop(); return SOBREPASO_ENDSTOP; } } else cl=0;
        if(((esp_timer_get_time()-t0)/1000) >= TIEMPO_SOBREPASO_MS_2) { eje2_pwm_stop(); return SOBREPASO_OK; }
        vTaskDelay(1);
    }
}
static void eje2_esc_end(int *d) {
    gpio_set_level(PIN_LED_2, 1); *d = (*d==DERECHA_2)?IZQUIERDA_2:DERECHA_2;
    eje2_set_dir(*d); eje2_set_freq(FREQ_BUSQUEDA_HZ_2); eje2_pwm_start();
    eje2_wait_hall_high(); eje2_wait_hall_low(); eje2_wait_hall_high();
    eje2_pwm_stop(); gpio_set_level(PIN_LED_2, 0);
}
static bool eje2_start_on_hall(int *d) {
    *d=DERECHA_2; eje2_set_dir(*d); if(eje2_check_over()==SOBREPASO_ENDSTOP){eje2_esc_end(d); return false;}
    eje2_set_dir(IZQUIERDA_2); eje2_pwm_start(); eje2_wait_hall_low(); eje2_pwm_stop();
    *d=IZQUIERDA_2; eje2_set_dir(*d); if(eje2_check_over()==SOBREPASO_ENDSTOP){eje2_esc_end(d); return false;}
    eje2_set_dir(DERECHA_2); eje2_pwm_start(); eje2_wait_hall_low(); eje2_pwm_stop();
    eje2_enable_driver(false); gpio_set_level(PIN_LED_2, 1); return true;
}
static bool eje2_confirm(int *d) {
    eje2_ramp_freq(FREQ_BUSQUEDA_HZ_2, FREQ_VERIFICACION_HZ_2, TIEMPO_RAMPA_MS_2);
    if(eje2_check_over()==SOBREPASO_ENDSTOP){ eje2_esc_end(d); return false; }
    *d = (*d==DERECHA_2)?IZQUIERDA_2:DERECHA_2; eje2_set_dir(*d);
    eje2_set_freq(FREQ_VERIFICACION_HZ_2); eje2_pwm_start(); eje2_wait_hall_low(); eje2_pwm_stop();
    eje2_enable_driver(false); gpio_set_level(PIN_LED_2, 1); return true;
}
static void eje2_gpio_init(void){
    gpio_config_t o={.pin_bit_mask=(1ULL<<PIN_DIR_2)|(1ULL<<PIN_EN_2)|(1ULL<<PIN_STEP_2)|(1ULL<<PIN_LED_2), .mode=GPIO_MODE_OUTPUT}; gpio_config(&o);
    gpio_config_t i={.pin_bit_mask=(1ULL<<PIN_HALL_2), .mode=GPIO_MODE_INPUT, .pull_up_en=1}; gpio_config(&i);
    gpio_set_level(PIN_DIR_2, DERECHA_2); gpio_set_level(PIN_EN_2, 1);
}
static void homing_eje2(void) {
    ESP_LOGI(TAG, "[EJE2] Homing secuencial...");
    eje2_gpio_init(); eje2_pwm_init(FREQ_BUSQUEDA_HZ_2); eje2_enable_driver(true); vTaskDelay(20/portTICK_PERIOD_MS);
    int d = DERECHA_2; eje2_set_dir(d);
    if(eje2_hall_read()) { if(eje2_start_on_hall(&d)) return; }
    eje2_set_freq(FREQ_BUSQUEDA_HZ_2); eje2_pwm_start();
    while(1) {
        eje2_wait_hall_low(); eje2_pwm_stop();
        if(eje2_confirm(&d)) break;
        eje2_set_freq(FREQ_BUSQUEDA_HZ_2); eje2_pwm_start();
    }
    eje2_enable_driver(false); eje2_pwm_stop();
    ESP_LOGI(TAG, "[EJE2] Homing OK.");
}

// Homing Logic Eje 3
static sobrepaso_res_t eje3_check_over(void) {
    eje3_set_freq(FREQ_VERIFICACION_HZ_3); eje3_pwm_start();
    if(eje3_hall_read()) eje3_wait_hall_high();
    int64_t t0 = esp_timer_get_time(); int cl=0;
    while(1) {
        if(eje3_hall_read()){ if(++cl>=2){ eje3_pwm_stop(); return SOBREPASO_ENDSTOP; } } else cl=0;
        if(((esp_timer_get_time()-t0)/1000) >= TIEMPO_SOBREPASO_MS_3) { eje3_pwm_stop(); return SOBREPASO_OK; }
        vTaskDelay(1);
    }
}
static void eje3_esc_end(int *d) {
    gpio_set_level(PIN_LED_3, 1); *d = (*d==DERECHA_3)?IZQUIERDA_3:DERECHA_3;
    eje3_set_dir(*d); eje3_set_freq(FREQ_BUSQUEDA_HZ_3); eje3_pwm_start();
    eje3_wait_hall_high(); eje3_wait_hall_low(); eje3_wait_hall_high();
    eje3_pwm_stop(); gpio_set_level(PIN_LED_3, 0);
}
static bool eje3_start_on_hall(int *d) {
    *d=DERECHA_3; eje3_set_dir(*d); if(eje3_check_over()==SOBREPASO_ENDSTOP){eje3_esc_end(d); return false;}
    eje3_set_dir(IZQUIERDA_3); eje3_pwm_start(); eje3_wait_hall_low(); eje3_pwm_stop();
    *d=IZQUIERDA_3; eje3_set_dir(*d); if(eje3_check_over()==SOBREPASO_ENDSTOP){eje3_esc_end(d); return false;}
    eje3_set_dir(DERECHA_3); eje3_pwm_start(); eje3_wait_hall_low(); eje3_pwm_stop();
    eje3_enable_driver(false); gpio_set_level(PIN_LED_3, 1); return true;
}
static bool eje3_confirm(int *d) {
    eje3_ramp_freq(FREQ_BUSQUEDA_HZ_3, FREQ_VERIFICACION_HZ_3, TIEMPO_RAMPA_MS_3);
    if(eje3_check_over()==SOBREPASO_ENDSTOP){ eje3_esc_end(d); return false; }
    *d = (*d==DERECHA_3)?IZQUIERDA_3:DERECHA_3; eje3_set_dir(*d);
    eje3_set_freq(FREQ_VERIFICACION_HZ_3); eje3_pwm_start(); eje3_wait_hall_low(); eje3_pwm_stop();
    eje3_enable_driver(false); gpio_set_level(PIN_LED_3, 1); return true;
}
static void eje3_gpio_init(void){
    gpio_config_t o={.pin_bit_mask=(1ULL<<PIN_DIR_3)|(1ULL<<PIN_EN_3)|(1ULL<<PIN_STEP_3)|(1ULL<<PIN_LED_3), .mode=GPIO_MODE_OUTPUT}; gpio_config(&o);
    gpio_config_t i={.pin_bit_mask=(1ULL<<PIN_HALL_3), .mode=GPIO_MODE_INPUT, .pull_up_en=1}; gpio_config(&i);
    gpio_set_level(PIN_DIR_3, DERECHA_3); gpio_set_level(PIN_EN_3, 1);
}
static void homing_eje3(void) {
    ESP_LOGI(TAG, "[EJE3] Homing secuencial...");
    eje3_gpio_init(); eje3_pwm_init(FREQ_BUSQUEDA_HZ_3); eje3_enable_driver(true); vTaskDelay(20/portTICK_PERIOD_MS);
    int d = DERECHA_3; eje3_set_dir(d);
    if(eje3_hall_read()) { if(eje3_start_on_hall(&d)) goto done; }
    eje3_set_freq(FREQ_BUSQUEDA_HZ_3); eje3_pwm_start();
    while(1) {
        eje3_wait_hall_low(); eje3_pwm_stop();
        if(eje3_confirm(&d)) break;
        eje3_set_freq(FREQ_BUSQUEDA_HZ_3); eje3_pwm_start();
    }
    eje3_pwm_stop();
done:
    eje3_enable_driver(false); eje3_pwm_stop();
    ESP_LOGI(TAG, "[EJE3] Homing OK.");
}


/* ============================================================
 * TAREA HOMING (SECUENCIAL - CMD 500)
 * ============================================================ */
static void homing_sequence_task(void *arg) {
    ESP_LOGI(TAG, "INICIO SECUENCIA HOMING (Eje 1 -> 2 -> 3)");
    is_busy = true;

    homing_eje1(); vTaskDelay(pdMS_TO_TICKS(500));
    homing_eje2(); vTaskDelay(pdMS_TO_TICKS(500));
    homing_eje3(); 

    ESP_LOGI(TAG, "HOMING COMPLETO.");
    is_busy = false;
    vTaskDelete(NULL);
}


/* ============================================================
 * FUNCIONES PARA MOVIMIENTO PTP PARALELO (CMD 400/401)
 * ============================================================ */

// --- PTP Helpers ---
static void move_steps_eje1(int32_t steps) {
    if (steps == 0) return;
    eje1_enable_driver(true); eje1_set_dir(steps > 0); eje1_set_freq(PTP_SPEED_HZ);
    
    uint32_t dur = (uint32_t)((float)abs(steps) / PTP_SPEED_HZ * 1000.0f);
    if(dur==0) dur=1;

    eje1_pwm_start(); vTaskDelay(pdMS_TO_TICKS(dur)); eje1_pwm_stop();
}

static void move_steps_eje2(int32_t steps) {
    if (steps == 0) return;
    eje2_enable_driver(true); eje2_set_dir(steps > 0 ? DERECHA_2 : IZQUIERDA_2); eje2_set_freq(PTP_SPEED_HZ);
    
    uint32_t dur = (uint32_t)((float)abs(steps) / PTP_SPEED_HZ * 1000.0f);
    if(dur==0) dur=1;

    eje2_pwm_start(); vTaskDelay(pdMS_TO_TICKS(dur)); eje2_pwm_stop();
}

static void move_steps_eje3(int32_t steps) {
    if (steps == 0) return;
    eje3_enable_driver(true); eje3_set_dir(steps > 0 ? DERECHA_3 : IZQUIERDA_3); eje3_set_freq(PTP_SPEED_HZ);
    
    uint32_t dur = (uint32_t)((float)abs(steps) / PTP_SPEED_HZ * 1000.0f);
    if(dur==0) dur=1;

    eje3_pwm_start(); vTaskDelay(pdMS_TO_TICKS(dur)); eje3_pwm_stop();
}

// --- PTP Wrappers ---
static void wrapper_ptp_eje1(void *arg) { move_steps_eje1(target_steps_1); xEventGroupSetBits(robot_event_group, BIT_EJE1_DONE); vTaskDelete(NULL); }
static void wrapper_ptp_eje2(void *arg) { move_steps_eje2(target_steps_2); xEventGroupSetBits(robot_event_group, BIT_EJE2_DONE); vTaskDelete(NULL); }
static void wrapper_ptp_eje3(void *arg) { move_steps_eje3(target_steps_3); xEventGroupSetBits(robot_event_group, BIT_EJE3_DONE); vTaskDelete(NULL); }

// --- Tarea Coordinadora PTP (PARALELO) ---
static void ptp_sequence_task(void *arg) {
    ESP_LOGI(TAG, "INICIO PTP PARALELO -> E1:%d, E2:%d, E3:%d", (int)target_steps_1, (int)target_steps_2, (int)target_steps_3);
    is_busy = true;

    if (robot_event_group == NULL) robot_event_group = xEventGroupCreate();
    xEventGroupClearBits(robot_event_group, ALL_AXES_DONE);

    // Lanzar en paralelo usando ambos nucleos
    xTaskCreatePinnedToCore(wrapper_ptp_eje1, "PTP_E1", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(wrapper_ptp_eje2, "PTP_E2", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(wrapper_ptp_eje3, "PTP_E3", 4096, NULL, 10, NULL, 0);

    // Esperar a que todos terminen
    xEventGroupWaitBits(robot_event_group, ALL_AXES_DONE, pdTRUE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "PTP FINALIZADO.");
    is_busy = false;
    vTaskDelete(NULL);
}


/* ============================================================
 * JOGGING MANUAL (CMD 601-603)
 * ============================================================ */
static void handle_jog(int cmd, int arg1) {
    if (cmd == 601) {
        eje2_enable_driver(false); eje2_pwm_stop(); eje3_enable_driver(false); eje3_pwm_stop();
        eje1_pwm_init(JOG_FREQ_HZ);
        if (arg1 == 0) eje1_pwm_stop();
        else { eje1_enable_driver(true); eje1_set_dir(arg1==1); eje1_pwm_start(); }
    } else if (cmd == 602) {
        eje1_enable_driver(false); eje1_pwm_stop(); eje3_enable_driver(false); eje3_pwm_stop();
        eje2_pwm_init(JOG_FREQ_HZ);
        if (arg1 == 0) eje2_pwm_stop();
        else { eje2_enable_driver(true); eje2_set_dir(arg1==1?DERECHA_2:IZQUIERDA_2); eje2_pwm_start(); }
    } else if (cmd == 603) {
        eje1_enable_driver(false); eje1_pwm_stop(); eje2_enable_driver(false); eje2_pwm_stop();
        eje3_pwm_init(JOG_FREQ_HZ);
        if (arg1 == 0) eje3_pwm_stop();
        else { eje3_enable_driver(true); eje3_set_dir(arg1==1?DERECHA_3:IZQUIERDA_3); eje3_pwm_start(); }
    }
}


/* ============================================================
 * TAREA SPI (CEREBRO)
 * ============================================================ */
static void spi_slave_task(void *arg) {
    spi_bus_config_t buscfg = { .mosi_io_num=PIN_SPI_MOSI, .miso_io_num=PIN_SPI_MISO, .sclk_io_num=PIN_SPI_SCK, .quadwp_io_num=-1, .quadhd_io_num=-1, .max_transfer_sz=32 };
    spi_slave_interface_config_t slvcfg = { .spics_io_num=PIN_SPI_CS, .flags=0, .queue_size=3, .mode=1 };
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST_ID, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    WORD_ALIGNED_ATTR uint8_t rx[16];
    WORD_ALIGNED_ATTR uint8_t tx[16];
    memset(tx, 0, 16); sprintf((char*)tx, "501");

    while (1) {
        spi_slave_transaction_t t = { .length=12*8, .tx_buffer=tx, .rx_buffer=rx };
        spi_slave_transmit(SPI_HOST_ID, &t, portMAX_DELAY);

        int32_t cmd=0, arg1=0, arg2=0;
        memcpy(&cmd, rx, 4); memcpy(&arg1, rx+4, 4); memcpy(&arg2, rx+8, 4);
        
        sprintf((char*)tx, is_busy ? "500" : "501");

        if (is_busy && cmd >= 400) continue; 

        if (cmd == 500) { 
             // CMD 500 -> HOMING SECUENCIAL
             xTaskCreate(homing_sequence_task, "HomeSeq", 4096, NULL, 5, NULL);
        } 
        else if (cmd == 400) { 
             target_steps_1 = arg1; target_steps_2 = arg2;
        }
        else if (cmd == 401) { 
             // CMD 401 -> MOVE PTP PARALELO
             target_steps_3 = arg1; 
             xTaskCreate(ptp_sequence_task, "PTPPar", 4096, NULL, 5, NULL);
        }
        else if (cmd >= 601 && cmd <= 603) { 
             handle_jog(cmd, arg1);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Robot Hybrid: Seq Home + Par Move...");
    eje1_gpio_init(); eje2_gpio_init(); eje3_gpio_init();
    xTaskCreatePinnedToCore(spi_slave_task, "spi", 4096, NULL, 5, NULL, 1);
}
