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

static const char *TAG = "ROBOT_HYBRID_FINAL_V4";

/* ============================================================
 * CONFIGURACIÓN GLOBAL Y SINCRONIZACIÓN
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

// Estado y Variables PTP
static volatile bool is_busy = false; 
static volatile int32_t target_steps_1 = 0;
static volatile int32_t target_steps_2 = 0;
static volatile int32_t target_steps_3 = 0;

// Constantes de Velocidad
#define DELAY_PTP_MIN_US   400
#define DELAY_START_US     3000
#define JOG_FREQ_HZ        1500.0f
#define DELAY_HOMING_FAST  1000 
#define DELAY_HOMING_SLOW  2500 
#define FREQ_ALIGN_HZ_1       250.0f
#define LEDC_RES_1            LEDC_TIMER_10_BIT
#define LEDC_RES_2            LEDC_TIMER_10_BIT
#define LEDC_RES_3            LEDC_TIMER_10_BIT

/* ============================================================
 * DEFINICIONES DE HARDWARE (PINES DEFINITIVOS)
 * ============================================================ */
// --- EJE 1 ---
#define PIN_TB6560_STEP    32
#define PIN_TB6560_DIR     33
#define PIN_TB6560_EN      25
#define PIN_HALL_1         26
#define LEDC_MODE_1        LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_1_ID    LEDC_TIMER_0
#define LEDC_CH_1          LEDC_CHANNEL_0

// --- EJE 2 ---
#define PIN_STEP_2         19
#define PIN_DIR_2          18
#define PIN_EN_2           5
#define PIN_HALL_2         27
#define PIN_LED_2          2
#define DERECHA_2          1
#define IZQUIERDA_2        0
#define LEDC_MODE_2        LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_2_ID    LEDC_TIMER_1
#define LEDC_CH_2          LEDC_CHANNEL_1

// --- EJE 3 ---
#define PIN_STEP_3         21
#define PIN_DIR_3          22
#define PIN_EN_3           23
#define PIN_HALL_3         4
#define PIN_LED_3          4
#define DERECHA_3          1
#define IZQUIERDA_3        0
#define LEDC_MODE_3        LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_3_ID    LEDC_TIMER_2
#define LEDC_CH_3          LEDC_CHANNEL_2


/* ============================================================
 * HELPERS DE BAJO NIVEL (GPIO/LEDC)
 * ============================================================ */

// >>> LÓGICA ENABLE CORRECTA: Activo por BAJO (LOW=0) <<<
static inline void eje1_enable_driver(bool on) { gpio_set_level(PIN_TB6560_EN, on ? 0 : 1); }
static inline void eje2_enable_driver(bool on) { gpio_set_level(PIN_EN_2, on ? 0 : 1); }
static inline void eje3_enable_driver(bool on) { gpio_set_level(PIN_EN_3, on ? 0 : 1); }

// Direcciones
static inline void eje1_set_dir(bool cw)        { gpio_set_level(PIN_TB6560_DIR, cw ? 1 : 0); }
static inline void eje2_set_dir(int d)          { gpio_set_level(PIN_DIR_2, d); esp_rom_delay_us(20); }
static inline void eje3_set_dir(int d)          { gpio_set_level(PIN_DIR_3, d); esp_rom_delay_us(20); }

// Lectura de Hall
static inline bool check_sensor(int pin) { return (gpio_get_level(pin) == 0); }

// Bit-Banging Step
static inline void step_once(int pin, uint32_t delay) {
    gpio_set_level(pin, 1); esp_rom_delay_us(delay);
    gpio_set_level(pin, 0); esp_rom_delay_us(delay);
}

// LEDC/PWM Control
static inline void ledc_set_duty_on(ledc_mode_t mode, ledc_channel_t ch, ledc_timer_bit_t res) {
    uint32_t duty_on = (1 << (res - 1));
    ledc_set_duty(mode, ch, duty_on); 
    ledc_update_duty(mode, ch);
}
static inline void eje1_pwm_start(void) { ledc_set_duty_on(LEDC_MODE_1, LEDC_CH_1, LEDC_RES_1); }
static inline void eje1_pwm_stop(void)  { ledc_stop(LEDC_MODE_1, LEDC_CH_1, 0); }
static inline void eje2_pwm_start() { ledc_set_duty_on(LEDC_MODE_2, LEDC_CH_2, LEDC_RES_2); }
static inline void eje2_pwm_stop()  { ledc_stop(LEDC_MODE_2, LEDC_CH_2, 0); }
static inline void eje3_pwm_start() { ledc_set_duty_on(LEDC_MODE_3, LEDC_CH_3, LEDC_RES_3); }
static inline void eje3_pwm_stop()  { ledc_stop(LEDC_MODE_3, LEDC_CH_3, 0); }

static inline void eje1_set_freq(float hz) { ledc_set_freq(LEDC_MODE_1, LEDC_TIMER_1_ID, (uint32_t)hz); }
static inline void eje2_set_freq(float hz) { ledc_set_freq(LEDC_MODE_2, LEDC_TIMER_2_ID, (uint32_t)hz); }
static inline void eje3_set_freq(float hz) { ledc_set_freq(LEDC_MODE_3, LEDC_TIMER_3_ID, (uint32_t)hz); }

// Helpers de Homing (Bit-Banging style)
static bool run_until_sensor(int step_pin, int sensor_pin, uint32_t delay, int max_steps) {
    for(int i=0; i<max_steps; i++) {
        if(check_sensor(sensor_pin)) return true; 
        step_once(step_pin, delay);
    }
    return false; 
}
static void run_blind(int step_pin, uint32_t delay, int steps) {
    for(int i=0; i<steps; i++) step_once(step_pin, delay);
}

// LEDC Init (Solo configura el pin STEP para LEDC, no el Timer)
static void reconfigure_step_to_ledc(int pin_step, ledc_channel_t channel, ledc_timer_t timer, ledc_mode_t mode, ledc_timer_bit_t res, float freq_hz) {
    ledc_stop(mode, channel, 0); 
    ledc_set_freq(mode, timer, (uint32_t)freq_hz);

    ledc_channel_config_t c = {
        .gpio_num=(uint32_t)pin_step, .speed_mode=mode, .channel=channel, .timer_sel=timer,
        .duty=0, .hpoint=0
    };
    ledc_channel_config(&c);
}

// GPIO Reconfigure (Para PTP)
static void reconfigure_step_to_gpio(int pin_step) {
    gpio_set_direction(pin_step, GPIO_MODE_OUTPUT);
    gpio_set_level(pin_step, 0); 
}


/* ============================================================
 * INICIALIZACIÓN GPIO GENERAL
 * ============================================================ */
static void init_gpio_all_ptp_safe(void) {
    // DIR / EN / LED (Salidas)
    gpio_config_t out_dir_en_led = {
        .pin_bit_mask=(1ULL<<PIN_TB6560_DIR)|(1ULL<<PIN_TB6560_EN)|
                      (1ULL<<PIN_DIR_2)|(1ULL<<PIN_EN_2)|(1ULL<<PIN_LED_2)|
                      (1ULL<<PIN_DIR_3)|(1ULL<<PIN_EN_3),
        .mode=GPIO_MODE_OUTPUT
    };
    gpio_config(&out_dir_en_led);

    // HALL (Entradas)
    gpio_config_t in_hall = {
        .pin_bit_mask=(1ULL<<PIN_HALL_1)|(1ULL<<PIN_HALL_2)|(1ULL<<PIN_HALL_3), 
        .mode=GPIO_MODE_INPUT, .pull_up_en=1
    };
    gpio_config(&in_hall);

    // STEP (Inicialmente GPIO OUTPUT para PTP)
    gpio_set_direction(PIN_TB6560_STEP, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_STEP_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_STEP_3, GPIO_MODE_OUTPUT);

    // Habilitar drivers (Mantener Torque)
    eje1_enable_driver(true); // EN=0
    eje2_enable_driver(true); // EN=0
    eje3_enable_driver(true); // EN=0

    // Inicialización LEDC Timers (Frecuencia base 1kHz)
    ledc_timer_config_t t1 = { .speed_mode=LEDC_MODE_1, .duty_resolution=LEDC_RES_1, .timer_num=LEDC_TIMER_1_ID, .freq_hz=1000, .clk_cfg=LEDC_AUTO_CLK };
    ledc_timer_config_t t2 = { .speed_mode=LEDC_MODE_2, .duty_resolution=LEDC_RES_2, .timer_num=LEDC_TIMER_2_ID, .freq_hz=1000, .clk_cfg=LEDC_AUTO_CLK };
    ledc_timer_config_t t3 = { .speed_mode=LEDC_MODE_3, .duty_resolution=LEDC_RES_3, .timer_num=LEDC_TIMER_3_ID, .freq_hz=1000, .clk_cfg=LEDC_AUTO_CLK };
    ledc_timer_config(&t1);
    ledc_timer_config(&t2);
    ledc_timer_config(&t3);
}

/* ============================================================
 * LÓGICA HOMING (USA Bit-Banging)
 * ============================================================ */

static void homing_eje1_logic(void) {
    ESP_LOGI(TAG, "Homing Eje 1...");
    reconfigure_step_to_gpio(PIN_TB6560_STEP);
    
    if(check_sensor(PIN_HALL_1)) {
        eje1_set_dir(false); run_blind(PIN_TB6560_STEP, DELAY_HOMING_FAST, 1000);
        eje1_set_dir(true); run_until_sensor(PIN_TB6560_STEP, PIN_HALL_1, DELAY_HOMING_SLOW, 1500);
    } else {
        eje1_set_dir(true); 
        if(run_until_sensor(PIN_TB6560_STEP, PIN_HALL_1, DELAY_HOMING_FAST, 30000)) {
            run_blind(PIN_TB6560_STEP, DELAY_HOMING_FAST, 1000);
            eje1_set_dir(false); run_until_sensor(PIN_TB6560_STEP, PIN_HALL_1, DELAY_HOMING_SLOW, 5000);
            eje1_set_dir(true); run_until_sensor(PIN_TB6560_STEP, PIN_HALL_1, DELAY_HOMING_SLOW, 1000);
        }
    }
}

static void homing_eje2_logic(void) {
    ESP_LOGI(TAG, "Homing Eje 2...");
    reconfigure_step_to_gpio(PIN_STEP_2);

    if(check_sensor(PIN_HALL_2)) {
        eje2_set_dir(IZQUIERDA_2); run_blind(PIN_STEP_2, DELAY_HOMING_FAST, 1000);
    }
    eje2_set_dir(DERECHA_2);
    if(run_until_sensor(PIN_STEP_2, PIN_HALL_2, DELAY_HOMING_FAST, 30000)) {
        eje2_set_dir(IZQUIERDA_2);
        gpio_set_level(PIN_LED_2, 1);
        run_blind(PIN_STEP_2, DELAY_HOMING_FAST, 7100);
        gpio_set_level(PIN_LED_2, 0);
        run_until_sensor(PIN_STEP_2, PIN_HALL_2, DELAY_HOMING_SLOW, 10000);
    }
}

static void homing_eje3_logic(void) {
    ESP_LOGI(TAG, "Homing Eje 3...");
    reconfigure_step_to_gpio(PIN_STEP_3);

    if(check_sensor(PIN_HALL_3)) {
        eje3_set_dir(IZQUIERDA_3); run_blind(PIN_STEP_3, DELAY_HOMING_FAST, 1000);
    }
    eje3_set_dir(DERECHA_3);
    if(run_until_sensor(PIN_STEP_3, PIN_HALL_3, DELAY_HOMING_FAST, 30000)) {
        eje3_set_dir(IZQUIERDA_3);
        gpio_set_level(PIN_LED_3, 1);
        run_blind(PIN_STEP_3, DELAY_HOMING_FAST, 7100);
        gpio_set_level(PIN_LED_3, 0);
        run_until_sensor(PIN_STEP_3, PIN_HALL_3, DELAY_HOMING_SLOW, 10000);
    }
}

static void homing_sequence_task(void *arg) {
    is_busy = true;
    // Los EN están activos por init_gpio_all_ptp_safe

    homing_eje1_logic(); vTaskDelay(pdMS_TO_TICKS(200));
    homing_eje2_logic(); vTaskDelay(pdMS_TO_TICKS(200));
    homing_eje3_logic();
    is_busy = false;
    ESP_LOGI(TAG, "HOMING COMPLETO");
    vTaskDelete(NULL);
}


/* ============================================================
 * PTP PARALELO (CMD 400/401) - USA BIT-BANGING CON RAMPA
 * ============================================================ */

static void move_with_ramp(int pin_step, int pin_dir, int steps, bool dir_high_cw) {
    if (steps == 0) return;
    
    // El pin STEP ya debe ser GPIO OUTPUT

    int d_logic = (steps > 0) ? 1 : 0;
    if (!dir_high_cw) d_logic = !d_logic; 
    gpio_set_level(pin_dir, d_logic);

    int total = abs(steps);
    int racel = total / 5; 
    if(racel < 10) racel = 10; 
    if(racel > total/2) racel = total/2;

    uint32_t d_curr = DELAY_START_US;
    float d_dec = (float)(DELAY_START_US - DELAY_PTP_MIN_US) / (float)racel;

    for(int i=1; i<=total; i++) {
        step_once(pin_step, d_curr);
        if(i <= racel) { 
            if (d_curr > DELAY_PTP_MIN_US) d_curr -= (uint32_t)d_dec; 
        } 
        else if(i > (total - racel)) { 
            d_curr += (uint32_t)d_dec; 
        }
    }
    // EN queda en 0 (activo). STEP queda como GPIO OUTPUT.
}

static void w_e1(void *arg) { move_with_ramp(PIN_TB6560_STEP, PIN_TB6560_DIR, target_steps_1, true); xEventGroupSetBits(robot_event_group, BIT_EJE1_DONE); vTaskDelete(NULL); }
static void w_e2(void *arg) { move_with_ramp(PIN_STEP_2, PIN_DIR_2, target_steps_2, true); xEventGroupSetBits(robot_event_group, BIT_EJE2_DONE); vTaskDelete(NULL); }
static void w_e3(void *arg) { move_with_ramp(PIN_STEP_3, PIN_DIR_3, target_steps_3, true); xEventGroupSetBits(robot_event_group, BIT_EJE3_DONE); vTaskDelete(NULL); }

static void ptp_sequence_task(void *arg) {
    is_busy = true;
    if (!robot_event_group) robot_event_group = xEventGroupCreate();
    xEventGroupClearBits(robot_event_group, ALL_AXES_DONE);
    
    // Asegurar que los pines STEP son GPIO OUTPUT antes de iniciar PTP
    reconfigure_step_to_gpio(PIN_TB6560_STEP);
    reconfigure_step_to_gpio(PIN_STEP_2);
    reconfigure_step_to_gpio(PIN_STEP_3);

    xTaskCreatePinnedToCore(w_e1, "P1", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(w_e2, "P2", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(w_e3, "P3", 4096, NULL, 10, NULL, 0);
    
    xEventGroupWaitBits(robot_event_group, ALL_AXES_DONE, pdTRUE, pdTRUE, portMAX_DELAY);
    is_busy = false;
    vTaskDelete(NULL);
}


/* ============================================================
 * JOGGING MANUAL (CMD 601-603) - USA PWM/LEDC
 * ============================================================ */
static void handle_jog(int cmd, int arg1) {
    // 1. Reconfigurar los pines STEP de GPIO a LEDC
    reconfigure_step_to_ledc(PIN_TB6560_STEP, LEDC_CH_1, LEDC_TIMER_1_ID, LEDC_MODE_1, LEDC_RES_1, JOG_FREQ_HZ);
    reconfigure_step_to_ledc(PIN_STEP_2, LEDC_CH_2, LEDC_TIMER_2_ID, LEDC_MODE_2, LEDC_RES_2, JOG_FREQ_HZ);
    reconfigure_step_to_ledc(PIN_STEP_3, LEDC_CH_3, LEDC_TIMER_3_ID, LEDC_MODE_3, LEDC_RES_3, JOG_FREQ_HZ);

    // Los EN están activos por defecto

    if (cmd == 601) {
        eje2_pwm_stop(); eje3_pwm_stop(); // Detener otros PWM
        if (arg1 == 0) eje1_pwm_stop();
        else { eje1_set_dir(arg1==1); eje1_pwm_start(); } 
    } else if (cmd == 602) {
        eje1_pwm_stop(); eje3_pwm_stop();
        if (arg1 == 0) eje2_pwm_stop();
        else { eje2_set_dir(arg1==1?DERECHA_2:IZQUIERDA_2); eje2_pwm_start(); }
    } else if (cmd == 603) {
        eje1_pwm_stop(); eje2_pwm_stop();
        if (arg1 == 0) eje3_pwm_stop();
        else { eje3_set_dir(arg1==1?DERECHA_3:IZQUIERDA_3); eje3_pwm_start(); }
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

        if (is_busy && cmd >= 400 && cmd <= 401) continue; 

        if (cmd == 500) { 
             xTaskCreate(homing_sequence_task, "HomeSeq", 4096, NULL, 5, NULL);
        } 
        else if (cmd == 400) { 
             target_steps_1 = arg1; target_steps_2 = arg2;
        }
        else if (cmd == 401) { 
             target_steps_3 = arg1; 
             xTaskCreate(ptp_sequence_task, "PTPPar", 4096, NULL, 5, NULL);
        }
        else if (cmd >= 601 && cmd <= 603) { 
             handle_jog(cmd, arg1);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Robot Hybrid Final: PTP(BB) + Home/Jog(PWM). ENABLE=LOW");
    init_gpio_all_ptp_safe();
    xTaskCreatePinnedToCore(spi_slave_task, "spi", 4096, NULL, 5, NULL, 1);
}
