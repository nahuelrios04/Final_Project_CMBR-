#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/spi_slave.h"

static const char *TAG = "ROBOT_V9_1_FIXED";

/* ============================================================
 * CONFIGURACIÓN GLOBAL
 * ============================================================ */
#define BIT_EJE1_DONE  (1 << 0)
#define BIT_EJE2_DONE  (1 << 1)
#define BIT_EJE3_DONE  (1 << 2)
#define ALL_AXES_DONE  (BIT_EJE1_DONE | BIT_EJE2_DONE | BIT_EJE3_DONE)
static EventGroupHandle_t robot_event_group;

// SPI
#define SPI_HOST_ID    SPI2_HOST
#define PIN_SPI_MOSI   13
#define PIN_SPI_MISO   12
#define PIN_SPI_SCK    14
#define PIN_SPI_CS     15

// Estado
static volatile bool is_busy = false; 
static volatile int32_t target_steps_1 = 0; 
static volatile int32_t target_steps_2 = 0; 
static volatile int32_t target_steps_3 = 0; 

// VELOCIDAD DINÁMICA
static volatile float GLOBAL_SPEED_HZ = 1000.0f; 
static volatile uint32_t GLOBAL_PTP_DELAY_US = 500; 

// Constantes PTP
#define DELAY_START_US          3000
#define DELAY_PTP_MIN_US        400

// --- CONSTANTES HOMING ---
// Eje 1
#define FREQ_MIN_HZ_H1        400.0f
#define FREQ_PASS1_HZ_H1      900.0f
#define FREQ_PASS2_HZ_H1      450.0f
#define FREQ_ALIGN_HZ_1       250.0f
#define FREQ_RELEASE_HZ_1     500.0f
#define FREQ_HALF_TURN_1      800.0f
#define RAMP_MS_1             1200U
#define OVERTRAVEL_MS_1       15U
#define SEPARATE_MS_1         60U
#define MOTOR_STEPS_PER_REV_1 200U
#define MICROSTEP_1           8U
#define HALF_TURN_STEPS_1     ((MOTOR_STEPS_PER_REV_1 * MICROSTEP_1) / 2U)

// Ejes 2 y 3 (Usando sufijo _23 para ambos)
#define FREQ_BUSQUEDA_HZ_23     1000.0f
#define FREQ_VERIFICACION_HZ_23 400.0f
#define TIEMPO_RAMPA_MS_23      120U
#define TIEMPO_SOBREPASO_MS_23  4000U
#define HALL_SAMPLE_MS          1U
#define HALL_DEBOUNCE_N         3

// LEDC
#define LEDC_RES_1  LEDC_TIMER_10_BIT
#define LEDC_RES_2  LEDC_TIMER_10_BIT
#define LEDC_RES_3  LEDC_TIMER_10_BIT

/* ============================================================
 * HARDWARE
 * ============================================================ */
// EJE 1
#define PIN_TB6560_STEP    32
#define PIN_TB6560_DIR     33
#define PIN_TB6560_EN      25
#define PIN_HALL_1         26
#define LEDC_MODE_1        LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_1_ID    LEDC_TIMER_0
#define LEDC_CH_1          LEDC_CHANNEL_0
#define DIR_CW_1           1
#define DIR_CCW_1          0

// EJE 2
#define PIN_STEP_2         19
#define PIN_DIR_2          18
#define PIN_EN_2           5
#define PIN_HALL_2         27
#define PIN_LED_2          2
#define DIR_CW_2           1
#define DIR_CCW_2          0
#define LEDC_MODE_2        LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_2_ID    LEDC_TIMER_1
#define LEDC_CH_2          LEDC_CHANNEL_1

// EJE 3
#define PIN_STEP_3         21
#define PIN_DIR_3          22
#define PIN_EN_3           23
#define PIN_HALL_3         4
#define PIN_LED_3          4
#define DIR_CW_3           0 
#define DIR_CCW_3          1 
#define LEDC_MODE_3        LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_3_ID    LEDC_TIMER_2
#define LEDC_CH_3          LEDC_CHANNEL_2


/* ============================================================
 * HELPERS BASE
 * ============================================================ */
static inline void update_ptp_delay() {
    if (GLOBAL_SPEED_HZ < 100.0f) GLOBAL_SPEED_HZ = 100.0f;
    if (GLOBAL_SPEED_HZ > 5000.0f) GLOBAL_SPEED_HZ = 5000.0f;
    GLOBAL_PTP_DELAY_US = (uint32_t)(500000.0f / GLOBAL_SPEED_HZ);
    if (GLOBAL_PTP_DELAY_US < 80) GLOBAL_PTP_DELAY_US = 80;
}

static inline void eje1_enable_driver(bool on) { gpio_set_level(PIN_TB6560_EN, on ? 0 : 1); }
static inline void eje2_enable_driver(bool on) { gpio_set_level(PIN_EN_2, on ? 0 : 1); }
static inline void eje3_enable_driver(bool on) { gpio_set_level(PIN_EN_3, on ? 0 : 1); }

static inline void eje1_set_dir(bool cw) { gpio_set_level(PIN_TB6560_DIR, cw ? DIR_CW_1 : DIR_CCW_1); esp_rom_delay_us(100); }
static inline void eje2_set_dir(bool cw) { gpio_set_level(PIN_DIR_2, cw ? DIR_CW_2 : DIR_CCW_2); esp_rom_delay_us(100); }
static inline void eje3_set_dir(bool cw) { gpio_set_level(PIN_DIR_3, cw ? DIR_CW_3 : DIR_CCW_3); esp_rom_delay_us(100); }

static inline bool check_sensor(int pin) { return (gpio_get_level(pin) == 0); }

static inline void step_once(int pin, uint32_t delay) {
    gpio_set_level(pin, 1); esp_rom_delay_us(delay);
    gpio_set_level(pin, 0); esp_rom_delay_us(delay);
}

// LEDC Helpers
static inline void ledc_set_duty_on(ledc_mode_t mode, ledc_channel_t ch, ledc_timer_bit_t res) {
    uint32_t duty_on = (1 << (res - 1));
    ledc_set_duty(mode, ch, duty_on); 
    ledc_update_duty(mode, ch);
}
static inline void eje1_pwm_start(void) { ledc_set_duty_on(LEDC_MODE_1, LEDC_CH_1, LEDC_RES_1); }
static inline void eje1_pwm_stop(void)  { ledc_stop(LEDC_MODE_1, LEDC_CH_1, 0); esp_rom_delay_us(100); }
static inline void eje2_pwm_start() { ledc_set_duty_on(LEDC_MODE_2, LEDC_CH_2, LEDC_RES_2); }
static inline void eje2_pwm_stop()  { ledc_stop(LEDC_MODE_2, LEDC_CH_2, 0); esp_rom_delay_us(100); }
static inline void eje3_pwm_start() { ledc_set_duty_on(LEDC_MODE_3, LEDC_CH_3, LEDC_RES_3); }
static inline void eje3_pwm_stop()  { ledc_stop(LEDC_MODE_3, LEDC_CH_3, 0); esp_rom_delay_us(100); }

static inline void eje1_set_freq(float hz) { 
    if (hz < FREQ_ALIGN_HZ_1) hz = FREQ_ALIGN_HZ_1; 
    ledc_set_freq(LEDC_MODE_1, LEDC_TIMER_0, (uint32_t)hz); 
}
static inline void eje2_set_freq(float hz) { ledc_set_freq(LEDC_MODE_2, LEDC_TIMER_1_ID, (uint32_t)hz); }
static inline void eje3_set_freq(float hz) { ledc_set_freq(LEDC_MODE_3, LEDC_TIMER_2_ID, (uint32_t)hz); }

// Wait Hall
static int eje_wait_hall_active(int pin) {
    int cnt = 0;
    for(;;){
        if (check_sensor(pin)) cnt++; else cnt = 0;
        if (cnt >= HALL_DEBOUNCE_N) return 1;
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS)); 
    }
}
static int eje_wait_hall_inactive(int pin) {
    int cnt = 0;
    for(;;){
        if (!check_sensor(pin)) cnt++; else cnt = 0;
        if (cnt >= HALL_DEBOUNCE_N) return 1;
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS)); 
    }
}

static void eje2_wait_hall_low(void) { int c=0; for(;;){ if(check_sensor(PIN_HALL_2)) {if(++c>=HALL_DEBOUNCE_N) return;} else c=0; vTaskDelay(1); } }
static void eje2_wait_hall_high(void){ int c=0; for(;;){ if(!check_sensor(PIN_HALL_2)){if(++c>=HALL_DEBOUNCE_N) return;} else c=0; vTaskDelay(1); } }
static void eje3_wait_hall_low(void) { int c=0; for(;;){ if(check_sensor(PIN_HALL_3)) {if(++c>=HALL_DEBOUNCE_N) return;} else c=0; vTaskDelay(1); } }
static void eje3_wait_hall_high(void){ int c=0; for(;;){ if(!check_sensor(PIN_HALL_3)){if(++c>=HALL_DEBOUNCE_N) return;} else c=0; vTaskDelay(1); } }

// Rampas
static void eje1_ramp_freq(float f0, float f1, uint32_t ramp_ms) {
    TickType_t tick = pdMS_TO_TICKS(5); uint32_t steps = ramp_ms / 5U; if(steps==0) {eje1_set_freq(f1); return;}
    eje1_pwm_start();
    for(uint32_t i=0; i<=steps; i++){ float u=(float)i/steps; float s=0.5f*(1.0f-cosf(M_PI*u)); eje1_set_freq(f0+(f1-f0)*s); vTaskDelay(tick); }
}
static void eje2_ramp_freq(float fi, float ff, uint32_t ms) {
    uint32_t steps = ms/5; if(steps==0)steps=1;
    for(uint32_t i=0; i<=steps; i++){ float u=(float)i/steps; float s=0.5f*(1.0f-cosf(M_PI*u)); eje2_set_freq(fi+(ff-fi)*s); vTaskDelay(pdMS_TO_TICKS(5)); }
}
static void eje3_ramp_freq(float fi, float ff, uint32_t ms) {
    uint32_t steps = ms/5; if(steps==0)steps=1;
    for(uint32_t i=0; i<=steps; i++){ float u=(float)i/steps; float s=0.5f*(1.0f-cosf(M_PI*u)); eje3_set_freq(fi+(ff-fi)*s); vTaskDelay(pdMS_TO_TICKS(5)); }
}

// Reconfiguraciones
static void reconfigure_step_to_ledc(int pin_step, ledc_channel_t channel, ledc_timer_t timer, ledc_mode_t mode, ledc_timer_bit_t res, float freq_hz) {
    gpio_set_direction(pin_step, GPIO_MODE_OUTPUT);
    ledc_stop(mode, channel, 0); 
    ledc_timer_config_t timer_cfg = { .speed_mode = mode, .duty_resolution = res, .timer_num = timer, .freq_hz = (uint32_t)freq_hz, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&timer_cfg);
    ledc_channel_config_t c = { .gpio_num=(uint32_t)pin_step, .speed_mode=mode, .channel=channel, .timer_sel=timer, .duty=0, .hpoint=0 };
    ledc_channel_config(&c);
}
static void reconfigure_step_to_gpio(int pin_step) {
    switch(pin_step) {
        case PIN_TB6560_STEP: ledc_stop(LEDC_MODE_1, LEDC_CH_1, 0); break;
        case PIN_STEP_2: ledc_stop(LEDC_MODE_2, LEDC_CH_2, 0); break;
        case PIN_STEP_3: ledc_stop(LEDC_MODE_3, LEDC_CH_3, 0); break;
    }
    gpio_set_direction(pin_step, GPIO_MODE_OUTPUT); gpio_set_level(pin_step, 0); 
}


/* ============================================================
 * HOMING EJE 1 (Lógica Original 2-Pass)
 * ============================================================ */
static void eje1_pass1_seek_cw(void) {
    if (check_sensor(PIN_HALL_1)) {
        eje1_set_dir(false); // CCW
        eje1_set_freq(FREQ_RELEASE_HZ_1); eje1_pwm_start();
        eje_wait_hall_inactive(PIN_HALL_1); 
        vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1)); eje1_pwm_stop();
    }
    eje1_set_dir(true);
    eje1_ramp_freq(FREQ_MIN_HZ_H1, FREQ_PASS1_HZ_H1, RAMP_MS_1);
    eje_wait_hall_active(PIN_HALL_1); eje1_pwm_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    
    eje1_set_dir(true); eje1_set_freq(FREQ_ALIGN_HZ_1); eje1_pwm_start();
    vTaskDelay(pdMS_TO_TICKS(OVERTRAVEL_MS_1)); eje1_pwm_stop();
}

static void eje1_pass2_return_align_ccw(void) {
    if (check_sensor(PIN_HALL_1)) {
        eje1_set_dir(true); eje1_set_freq(FREQ_RELEASE_HZ_1); eje1_pwm_start();
        uint32_t t0 = (uint32_t)esp_log_timestamp();
        while(check_sensor(PIN_HALL_1) && (esp_log_timestamp()-t0)<3000) vTaskDelay(pdMS_TO_TICKS(5));
        eje1_pwm_stop(); vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1));
    }
    eje1_set_dir(false);
    eje1_ramp_freq(FREQ_MIN_HZ_H1, FREQ_PASS2_HZ_H1, 800U);
    uint32_t t1 = (uint32_t)esp_log_timestamp();
    while(!check_sensor(PIN_HALL_1) && (esp_log_timestamp()-t1)<4000) vTaskDelay(pdMS_TO_TICKS(2));
    eje1_pwm_stop();
    
    eje1_set_dir(false); eje1_set_freq(FREQ_ALIGN_HZ_1); eje1_pwm_start();
    if(!check_sensor(PIN_HALL_1)) eje_wait_hall_active(PIN_HALL_1);
    vTaskDelay(pdMS_TO_TICKS(20)); eje1_pwm_stop();
}

static void homing_eje1_logic(void) {
    reconfigure_step_to_ledc(PIN_TB6560_STEP, LEDC_CH_1, LEDC_TIMER_0, LEDC_MODE_1, LEDC_RES_1, FREQ_MIN_HZ_H1);
    if (check_sensor(PIN_HALL_1)) {
        double s = (double)HALF_TURN_STEPS_1 / FREQ_HALF_TURN_1;
        uint32_t ms = (uint32_t)(s*1000.0+0.5);
        eje1_set_dir(false); eje1_set_freq(FREQ_HALF_TURN_1); eje1_pwm_start();
        vTaskDelay(pdMS_TO_TICKS(ms)); eje1_pwm_stop(); vTaskDelay(pdMS_TO_TICKS(60));
    }
    eje1_pass1_seek_cw();
    eje1_pass2_return_align_ccw();
    ESP_LOGI(TAG, "Homing E1 OK");
}

/* ============================================================
 * HOMING EJE 2/3 (Lógica Original Compleja con Sobrepaso)
 * ============================================================ */
typedef enum { SOBREPASO_OK=0, SOBREPASO_ENDSTOP=1 } sobrepaso_res_t;

// --- EJE 2 ---
static sobrepaso_res_t eje2_verificar_sobrepaso(void) {
    eje2_set_freq(FREQ_VERIFICACION_HZ_23); eje2_pwm_start(); // Corrección constante
    if(check_sensor(PIN_HALL_2)) eje2_wait_hall_high();
    int64_t t0 = esp_timer_get_time(); int cl=0;
    while(1) {
        if(check_sensor(PIN_HALL_2)){ 
            if(++cl>=HALL_DEBOUNCE_N){ eje2_pwm_stop(); return SOBREPASO_ENDSTOP; } 
        } else cl=0;
        if(((esp_timer_get_time()-t0)/1000) >= TIEMPO_SOBREPASO_MS_23) { eje2_pwm_stop(); return SOBREPASO_OK; } // Corrección
        vTaskDelay(1);
    }
}
static void eje2_escape_endstop(int *d) {
    gpio_set_level(PIN_LED_2, 1); *d = (*d==DIR_CW_2)?DIR_CCW_2:DIR_CW_2;
    eje2_set_dir(*d); eje2_set_freq(FREQ_BUSQUEDA_HZ_23); eje2_pwm_start(); // Corrección
    eje2_wait_hall_high(); eje2_wait_hall_low(); eje2_wait_hall_high();
    eje2_pwm_stop(); gpio_set_level(PIN_LED_2, 0);
}
static bool eje2_start_on_hall(int *d) {
    *d=DIR_CW_2; eje2_set_dir(*d); 
    if(eje2_verificar_sobrepaso()==SOBREPASO_ENDSTOP){ eje2_escape_endstop(d); return false; }
    eje2_set_dir(DIR_CCW_2); eje2_pwm_start(); eje2_wait_hall_low(); eje2_pwm_stop();
    *d=DIR_CCW_2; eje2_set_dir(*d);
    if(eje2_verificar_sobrepaso()==SOBREPASO_ENDSTOP){ eje2_escape_endstop(d); return false; }
    eje2_set_dir(DIR_CW_2); eje2_pwm_start(); eje2_wait_hall_low(); eje2_pwm_stop();
    return true;
}
static bool eje2_confirm(int *d) {
    eje2_ramp_freq(FREQ_BUSQUEDA_HZ_23, FREQ_VERIFICACION_HZ_23, TIEMPO_RAMPA_MS_23); // Corrección
    if(eje2_verificar_sobrepaso()==SOBREPASO_ENDSTOP){ eje2_escape_endstop(d); return false; }
    *d = (*d==DIR_CW_2)?DIR_CCW_2:DIR_CW_2; eje2_set_dir(*d);
    eje2_set_freq(FREQ_VERIFICACION_HZ_23); eje2_pwm_start(); eje2_wait_hall_low(); eje2_pwm_stop(); // Corrección
    return true;
}
static void homing_eje2_logic(void) {
    reconfigure_step_to_ledc(PIN_STEP_2, LEDC_CH_2, LEDC_TIMER_1_ID, LEDC_MODE_2, LEDC_RES_2, FREQ_BUSQUEDA_HZ_23);
    int d = DIR_CW_2; eje2_set_dir(d);
    if(check_sensor(PIN_HALL_2)) { if(eje2_start_on_hall(&d)) return; }
    eje2_set_freq(FREQ_BUSQUEDA_HZ_23); eje2_pwm_start(); // Corrección
    while(1) {
        eje2_wait_hall_low(); eje2_pwm_stop();
        if(eje2_confirm(&d)) break;
        eje2_set_freq(FREQ_BUSQUEDA_HZ_23); eje2_pwm_start(); // Corrección
    }
    eje2_pwm_stop();
    ESP_LOGI(TAG, "Homing E2 OK");
}

// --- EJE 3 ---
static sobrepaso_res_t eje3_verificar_sobrepaso(void) {
    eje3_set_freq(FREQ_VERIFICACION_HZ_23); eje3_pwm_start(); // Corrección
    if(check_sensor(PIN_HALL_3)) eje3_wait_hall_high();
    int64_t t0 = esp_timer_get_time(); int cl=0;
    while(1) {
        if(check_sensor(PIN_HALL_3)){ 
            if(++cl>=HALL_DEBOUNCE_N){ eje3_pwm_stop(); return SOBREPASO_ENDSTOP; } 
        } else cl=0;
        if(((esp_timer_get_time()-t0)/1000) >= TIEMPO_SOBREPASO_MS_23) { eje3_pwm_stop(); return SOBREPASO_OK; } // Corrección
        vTaskDelay(1);
    }
}
static void eje3_escape_endstop(int *d) {
    gpio_set_level(PIN_LED_3, 1); *d = (*d==DIR_CW_3)?DIR_CCW_3:DIR_CW_3;
    eje3_set_dir(*d); eje3_set_freq(FREQ_BUSQUEDA_HZ_23); eje3_pwm_start(); // Corrección
    eje3_wait_hall_high(); eje3_wait_hall_low(); eje3_wait_hall_high();
    eje3_pwm_stop(); gpio_set_level(PIN_LED_3, 0);
}
static bool eje3_start_on_hall(int *d) {
    *d=DIR_CW_3; eje3_set_dir(*d); 
    if(eje3_verificar_sobrepaso()==SOBREPASO_ENDSTOP){ eje3_escape_endstop(d); return false; }
    eje3_set_dir(DIR_CCW_3); eje3_pwm_start(); eje3_wait_hall_low(); eje3_pwm_stop();
    *d=DIR_CCW_3; eje3_set_dir(*d);
    if(eje3_verificar_sobrepaso()==SOBREPASO_ENDSTOP){ eje3_escape_endstop(d); return false; }
    eje3_set_dir(DIR_CW_3); eje3_pwm_start(); eje3_wait_hall_low(); eje3_pwm_stop();
    return true;
}
static bool eje3_confirm(int *d) {
    eje3_ramp_freq(FREQ_BUSQUEDA_HZ_23, FREQ_VERIFICACION_HZ_23, TIEMPO_RAMPA_MS_23); // Corrección
    if(eje3_verificar_sobrepaso()==SOBREPASO_ENDSTOP){ eje3_escape_endstop(d); return false; }
    *d = (*d==DIR_CW_3)?DIR_CCW_3:DIR_CW_3; eje3_set_dir(*d);
    eje3_set_freq(FREQ_VERIFICACION_HZ_23); eje3_pwm_start(); eje3_wait_hall_low(); eje3_pwm_stop(); // Corrección
    return true;
}
static void homing_eje3_logic(void) {
    ESP_LOGI(TAG, "Home E3");
    reconfigure_step_to_ledc(PIN_STEP_3, LEDC_CH_3, LEDC_TIMER_2_ID, LEDC_MODE_3, LEDC_RES_3, FREQ_BUSQUEDA_HZ_23); // Corrección
    int d = DIR_CW_3; eje3_set_dir(d);
    if(check_sensor(PIN_HALL_3)) { if(eje3_start_on_hall(&d)) goto done; }
    eje3_set_freq(FREQ_BUSQUEDA_HZ_23); eje3_pwm_start(); // Corrección
    while(1) {
        eje3_wait_hall_low(); eje3_pwm_stop();
        if(eje3_confirm(&d)) break;
        eje3_set_freq(FREQ_BUSQUEDA_HZ_23); eje3_pwm_start(); // Corrección
    }
    eje3_pwm_stop();
done:
    ESP_LOGI(TAG, "Homing E3 OK");
}

static void homing_sequence_task(void *arg) {
    if (is_busy) { vTaskDelete(NULL); return; }
    is_busy = true;
    homing_eje1_logic(); vTaskDelay(pdMS_TO_TICKS(200));
    homing_eje2_logic(); vTaskDelay(pdMS_TO_TICKS(200));
    homing_eje3_logic();
    reconfigure_step_to_gpio(PIN_TB6560_STEP); reconfigure_step_to_gpio(PIN_STEP_2); reconfigure_step_to_gpio(PIN_STEP_3);
    is_busy = false;
    vTaskDelete(NULL);
}

/* ============================================================
 * PTP
 * ============================================================ */
static void move_with_ramp(int pin_step, int pin_dir, int steps, bool dir_high_cw, const char* eje_name) {
    if (steps == 0) return;
    int d_logic = (steps > 0) ? 1 : 0;
    if (!dir_high_cw) d_logic = !d_logic; 
    gpio_set_level(pin_dir, d_logic);
    esp_rom_delay_us(100);
    int total = abs(steps);
    int racel = total / 5; if(racel < 10) racel = 10; if(racel > total/2) racel = total/2;
    
    // VELOCIDAD DINAMICA
    uint32_t d_start = DELAY_START_US;
    uint32_t d_target = GLOBAL_PTP_DELAY_US; 
    float d_dec = (float)(d_start - d_target) / (float)racel;
    uint32_t d_curr = d_start;

    for(int i=1; i<=total; i++) {
        step_once(pin_step, d_curr);
        if(i <= racel) { if (d_curr > d_target) d_curr -= (uint32_t)d_dec; } 
        else if(i > (total - racel)) { d_curr += (uint32_t)d_dec; }
    }
}

static void w_e1(void *arg) { move_with_ramp(PIN_TB6560_STEP, PIN_TB6560_DIR, target_steps_1, true, "E1"); xEventGroupSetBits(robot_event_group, BIT_EJE1_DONE); vTaskDelete(NULL); }
static void w_e2(void *arg) { move_with_ramp(PIN_STEP_2, PIN_DIR_2, target_steps_2, true, "E2"); xEventGroupSetBits(robot_event_group, BIT_EJE2_DONE); vTaskDelete(NULL); }
static void w_e3(void *arg) { move_with_ramp(PIN_STEP_3, PIN_DIR_3, target_steps_3, false, "E3"); xEventGroupSetBits(robot_event_group, BIT_EJE3_DONE); vTaskDelete(NULL); }

static void ptp_sequence_task(void *arg) {
    if (is_busy) { vTaskDelete(NULL); return; }
    is_busy = true;
    if (!robot_event_group) { robot_event_group = xEventGroupCreate(); }
    xEventGroupClearBits(robot_event_group, ALL_AXES_DONE);
    reconfigure_step_to_gpio(PIN_TB6560_STEP); reconfigure_step_to_gpio(PIN_STEP_2); reconfigure_step_to_gpio(PIN_STEP_3);
    xTaskCreatePinnedToCore(w_e1, "P1", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(w_e2, "P2", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(w_e3, "P3", 4096, NULL, 10, NULL, 0);
    xEventGroupWaitBits(robot_event_group, ALL_AXES_DONE, pdTRUE, pdTRUE, portMAX_DELAY);
    is_busy = false;
    vTaskDelete(NULL);
}

/* ============================================================
 * JOGGING
 * ============================================================ */
static void handle_jog(int cmd, int arg1) {
    float freq = GLOBAL_SPEED_HZ;
    reconfigure_step_to_ledc(PIN_TB6560_STEP, LEDC_CH_1, LEDC_TIMER_0, LEDC_MODE_1, LEDC_RES_1, freq);
    reconfigure_step_to_ledc(PIN_STEP_2, LEDC_CH_2, LEDC_TIMER_1_ID, LEDC_MODE_2, LEDC_RES_2, freq);
    reconfigure_step_to_ledc(PIN_STEP_3, LEDC_CH_3, LEDC_TIMER_2_ID, LEDC_MODE_3, LEDC_RES_3, freq);

    if (cmd == 601) { eje2_pwm_stop(); eje3_pwm_stop(); if (arg1 == 0) eje1_pwm_stop(); else { eje1_set_dir(arg1==1); eje1_pwm_start(); } }
    else if (cmd == 602) { eje1_pwm_stop(); eje3_pwm_stop(); if (arg1 == 0) eje2_pwm_stop(); else { eje2_set_dir(arg1==1); eje2_pwm_start(); } }
    else if (cmd == 603) { eje1_pwm_stop(); eje2_pwm_stop(); if (arg1 == 0) eje3_pwm_stop(); else { eje3_set_dir(arg1==1); eje3_pwm_start(); } }
}

/* ============================================================
 * SPI
 * ============================================================ */
static void spi_slave_task(void *arg) {
    spi_bus_config_t buscfg = { .mosi_io_num=PIN_SPI_MOSI, .miso_io_num=PIN_SPI_MISO, .sclk_io_num=PIN_SPI_SCK, .quadwp_io_num=-1, .quadhd_io_num=-1, .max_transfer_sz=32 };
    spi_slave_interface_config_t slvcfg = { .spics_io_num=PIN_SPI_CS, .flags=0, .queue_size=3, .mode=1 };
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST_ID, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    WORD_ALIGNED_ATTR uint8_t rx[16]; WORD_ALIGNED_ATTR uint8_t tx[16]; memset(tx, 0, 16); sprintf((char*)tx, "501");
    
    update_ptp_delay();

    while (1) {
        spi_slave_transaction_t t = { .length=12*8, .tx_buffer=tx, .rx_buffer=rx };
        spi_slave_transmit(SPI_HOST_ID, &t, portMAX_DELAY);
        int32_t cmd=0, arg1=0, arg2=0; memcpy(&cmd, rx, 4); memcpy(&arg1, rx+4, 4); memcpy(&arg2, rx+8, 4);
        sprintf((char*)tx, is_busy ? "500" : "501");
        
        if (is_busy && (cmd == 500 || (cmd >= 400 && cmd <= 401))) continue; 

        if (cmd == 700) { GLOBAL_SPEED_HZ = (float)arg1; update_ptp_delay(); }
        else if (cmd == 500) { xTaskCreate(homing_sequence_task, "HomeSeq", 4096, NULL, 5, NULL); } 
        else if (cmd == 400) { target_steps_1 = arg1; target_steps_2 = arg2; }
        else if (cmd == 401) { target_steps_3 = arg1; xTaskCreate(ptp_sequence_task, "PTPPar", 4096, NULL, 5, NULL); }
        else if (cmd >= 601 && cmd <= 603) { handle_jog(cmd, arg1); }
    }
}

static void init_gpio_all_ptp_safe(void) {
    gpio_config_t out_conf = { .pin_bit_mask=(1ULL<<PIN_TB6560_DIR)|(1ULL<<PIN_TB6560_EN)|(1ULL<<PIN_DIR_2)|(1ULL<<PIN_EN_2)|(1ULL<<PIN_LED_2)|(1ULL<<PIN_DIR_3)|(1ULL<<PIN_EN_3)|(1ULL<<PIN_LED_3), .mode=GPIO_MODE_OUTPUT };
    gpio_config(&out_conf);
    gpio_config_t in_conf = { .pin_bit_mask=(1ULL<<PIN_HALL_1)|(1ULL<<PIN_HALL_2)|(1ULL<<PIN_HALL_3), .mode=GPIO_MODE_INPUT, .pull_up_en=1 };
    gpio_config(&in_conf);
    reconfigure_step_to_gpio(PIN_TB6560_STEP); reconfigure_step_to_gpio(PIN_STEP_2); reconfigure_step_to_gpio(PIN_STEP_3);
    eje1_enable_driver(true); eje2_enable_driver(true); eje3_enable_driver(true);
    reconfigure_step_to_ledc(PIN_TB6560_STEP, LEDC_CH_1, LEDC_TIMER_0, LEDC_MODE_1, LEDC_RES_1, 1000.0f);
    eje1_pwm_stop();
    reconfigure_step_to_gpio(PIN_TB6560_STEP);
}

void app_main(void) {
    init_gpio_all_ptp_safe();
    xTaskCreatePinnedToCore(spi_slave_task, "spi", 4096, NULL, 5, NULL, 1);
}
