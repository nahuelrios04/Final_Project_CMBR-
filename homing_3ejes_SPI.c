#include <stdio.h>
#include <math.h>
#include <string.h> // <--- AÑADIDO PARA SPI
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"      // esp_timer_get_time()
#include "esp_rom_sys.h"    // esp_rom_delay_us()
#include "driver/spi_slave.h" // <--- AÑADIDO PARA SPI

static const char *TAG = "HOMING_ALL_AXES";


/* ============================================================
 * DEFINES DE SPI SLAVE (NUEVO)
 * ============================================================ */
// Pines SPI solicitados por el usuario
#define SPI_HOST_ID   SPI2_HOST   // Usamos VSPI (SPI2_HOST)
#define PIN_SPI_MOSI  13
#define PIN_SPI_MISO  12
#define PIN_SPI_SCK   14
#define PIN_SPI_CS    15

// Flag de seguridad para la SECUENCIA COMPLETA
static volatile bool is_sequence_running = false;


/* ============================================================
 * EJE 1 (TB6560 STYLE)
 * ============================================================ */

// --- Pines eje1 ---
#define PIN_TB6560_STEP   1
#define PIN_TB6560_DIR    22
#define PIN_TB6560_EN     23
#define PIN_HALL_1        26   // activo-bajo (pull-up interna)

// --- Motor eje1 ---
#define MOTOR_STEPS_PER_REV_1   200U
#define MICROSTEP_1               8U
#define HALF_TURN_STEPS_1   ((MOTOR_STEPS_PER_REV_1 * MICROSTEP_1) / 2U)

// --- Frecuencias eje1 (Hz) ---
#define FREQ_MIN_HZ_1        400.0f
#define FREQ_PASS1_HZ_1      900.0f
#define FREQ_PASS2_HZ_1      450.0f
#define FREQ_ALIGN_HZ_1      250.0f
#define FREQ_RELEASE_HZ_1    500.0f
#define FREQ_HALF_TURN_1     800.0f

// --- Tiempos eje1 (ms) ---
#define RAMP_MS_1            1200U
#define OVERTRAVEL_MS_1        15U
#define SEPARATE_MS_1          60U
#define HALL_SAMPLE_MS_1        1U
#define HALL_DEBOUNCE_N_1       3

// --- LEDC eje1 ---
#define LEDC_MODE_1          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_1_ID      LEDC_TIMER_0       // Timer 0 para eje1
#define LEDC_CH_1            LEDC_CHANNEL_0     // Canal 0 para eje1
#define LEDC_RES_1           LEDC_TIMER_10_BIT
// --- MEJORA: Duty cycle al 50% real ---
#define LEDC_DUTY_ON_1       (1 << (LEDC_RES_1 - 1)) // 512
#define LEDC_DUTY_OFF_1      0


/* ============================================================
 * EJE 2
 * ============================================================ */

// --- Pines eje2 (Según tu código) ---
#define PIN_STEP_2      19
#define PIN_DIR_2       21
#define PIN_EN_2        3
#define PIN_HALL_2      27
#define PIN_LED_2        2

// --- Direcciones eje2 ---
#define DERECHA_2    1
#define IZQUIERDA_2  0

// --- LEDC eje2 ---
#define LEDC_MODE_2          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_2_ID      LEDC_TIMER_1       // Timer 1 para eje2
#define LEDC_CH_2            LEDC_CHANNEL_1     // Canal 1 para eje2
#define LEDC_RES_2           LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_2       (1 << (LEDC_RES_2 - 1)) // 50%
#define LEDC_DUTY_OFF_2      0

// --- Movimiento eje2 ---
#define FREQ_BUSQUEDA_HZ_2        1000.0f
#define FREQ_VERIFICACION_HZ_2     400.0f
#define TIEMPO_RAMPA_MS_2           120U
#define TIEMPO_SOBREPASO_MS_2      4000U

// --- Hall eje2 ---
#define HALL_SAMPLE_MS_2           1U
#define HALL_DEBOUNCE_N_2          2


/* ============================================================
 * EJE 3
 * ============================================================ */

// --- Pines eje3 (Según tu código) ---
#define PIN_STEP_3      17
#define PIN_DIR_3       5
#define PIN_EN_3        18
#define PIN_HALL_3      25    // <--- Pin 25 según tu código
#define PIN_LED_3       4     // <--- Pin 4 según tu código

// --- Direcciones eje3 ---
#define DERECHA_3    1
#define IZQUIERDA_3  0

// --- LEDC eje3 ---
#define LEDC_MODE_3          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_3_ID      LEDC_TIMER_2       // Timer 2 para eje3
#define LEDC_CH_3            LEDC_CHANNEL_2     // Canal 2 para eje3
#define LEDC_RES_3           LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_3       (1 << (LEDC_RES_3 - 1)) // 50%
#define LEDC_DUTY_OFF_3      0

// --- Movimiento eje3 ---
#define FREQ_BUSQUEDA_HZ_3        1000.0f
#define FREQ_VERIFICACION_HZ_3     400.0f
#define TIEMPO_RAMPA_MS_3           120U
#define TIEMPO_SOBREPASO_MS_3      4000U

// --- Hall eje3 ---
#define HALL_SAMPLE_MS_3           1U
#define HALL_DEBOUNCE_N_3          2


/* ============================================================
 * Protos eje1 / eje2 / eje3
 * ============================================================ */
static void homing_eje1(void);
static void homing_eje2(void);
static void homing_eje3(void);

// --- Protos de helpers Eje 1 ---
static void eje1_gpio_init(void);
static void eje1_pwm_init(float start_hz);
static void eje1_pass1_seek_cw(void);
static void eje1_pass2_return_align_ccw(void);
static void eje2_gpio_init(void);
static void eje3_gpio_init(void);
static void eje2_pwm_init(float start_hz);
static void eje3_pwm_init(float start_hz);


/* ============================================================
 * UTILIDADES EJE1
 * ============================================================ */

// --- Low-level eje1 ---
static inline void eje1_enable_driver(bool on)
{
    // EN activo-bajo: 0 = habilitado, 1 = deshabilitado
    gpio_set_level(PIN_TB6560_EN, on ? 0 : 1);
}
static inline void eje1_set_dir(bool cw)
{
    gpio_set_level(PIN_TB6560_DIR, cw ? 1 : 0);
}
static inline int eje1_hall_raw(void)
{
    // sensor activo-bajo
    return (gpio_get_level(PIN_HALL_1) == 0);
}

// debounce hall LOW estable (detecta imán)
static int eje1_wait_hall_active(void)
{
    int cnt = 0;
    for(;;){
        if (eje1_hall_raw()) cnt++; else cnt = 0;
        if (cnt >= HALL_DEBOUNCE_N_1) return 1;
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_1));
    }
}

// debounce hall HIGH estable (liberado)
static int eje1_wait_hall_inactive(void)
{
    int cnt = 0;
    for(;;){
        if (!eje1_hall_raw()) cnt++; else cnt = 0;
        if (cnt >= HALL_DEBOUNCE_N_1) return 1;
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_1));
    }
}

// PWM setup eje1
static void eje1_pwm_init(float start_hz)
{
    ledc_timer_config_t t = {
        .speed_mode       = LEDC_MODE_1,
        .duty_resolution  = LEDC_RES_1,
        .timer_num        = LEDC_TIMER_1_ID,
        .freq_hz          = (uint32_t)start_hz,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num   = PIN_TB6560_STEP,
        .speed_mode = LEDC_MODE_1,
        .channel    = LEDC_CH_1,
        .timer_sel  = LEDC_TIMER_1_ID,
        .duty       = LEDC_DUTY_ON_1,   // dejamos duty ya cargado
        .hpoint     = 0
    };
    ledc_channel_config(&c);
}

static inline void eje1_pwm_start(void)
{
    ledc_set_duty(LEDC_MODE_1, LEDC_CH_1, LEDC_DUTY_ON_1);
    ledc_update_duty(LEDC_MODE_1, LEDC_CH_1);
}
static inline void eje1_pwm_stop(void)
{
    ledc_stop(LEDC_MODE_1, LEDC_CH_1, LEDC_DUTY_OFF_1);
}

static inline void eje1_set_freq(float hz)
{
    if (hz < FREQ_ALIGN_HZ_1) hz = FREQ_ALIGN_HZ_1;
    esp_err_t err = ledc_set_freq(LEDC_MODE_1, LEDC_TIMER_1_ID, (uint32_t)hz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[EJE1] ledc_set_freq fallo (%.1f Hz)", (double)hz);
    }
}

// rampa S-curve eje1
static void eje1_ramp_freq(float f0, float f1, uint32_t ramp_ms)
{
    TickType_t tick = pdMS_TO_TICKS(5);

    uint32_t steps = ramp_ms / 5U;
    if (steps == 0){
        eje1_set_freq(f1);
        return;
    }

    eje1_pwm_start();
    for(uint32_t i=0; i<=steps; i++){
        float u = (float)i / (float)steps;
        float s = 0.5f * (1.0f - cosf((float)M_PI * u));
        float f = f0 + (f1 - f0) * s;
        eje1_set_freq(f);
        vTaskDelay(tick);
    }
}

// Paso 1: buscar imán en CW, sobrepasar un toque
static void eje1_pass1_seek_cw(void)
{
    if (eje1_hall_raw()) {
        eje1_set_dir(false); // CCW
        eje1_set_freq(FREQ_RELEASE_HZ_1);
        eje1_pwm_start();
        eje1_wait_hall_inactive();
        vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1));
        eje1_pwm_stop();
    }

    eje1_set_dir(true); // CW
    eje1_ramp_freq(FREQ_MIN_HZ_1, FREQ_PASS1_HZ_1, RAMP_MS_1);
    ESP_LOGI(TAG, "[EJE1] P1: Buscando Hall en CW (%.0f Hz)", (double)FREQ_PASS1_HZ_1);

    eje1_pwm_start();
    eje1_wait_hall_active(); 
    eje1_pwm_stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    eje1_set_dir(true); // CW
    eje1_set_freq(FREQ_ALIGN_HZ_1);
    eje1_pwm_start();
    vTaskDelay(pdMS_TO_TICKS(OVERTRAVEL_MS_1));
    eje1_pwm_stop();

    ESP_LOGI(TAG, "[EJE1] P1: Hall detectado y sobrepasado.");
}

// Paso 2: volver lento CCW y parar justo arriba del imán
static void eje1_pass2_return_align_ccw(void)
{
    if (eje1_hall_raw()) {
        eje1_set_dir(true); // CW para liberar
        eje1_set_freq(FREQ_RELEASE_HZ_1);
        eje1_pwm_start();
        uint32_t t0 = (uint32_t)esp_log_timestamp();
        while (eje1_hall_raw() && (esp_log_timestamp() - t0) < 3000) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        eje1_pwm_stop();
        vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1));
        ESP_LOGI(TAG, "[EJE1] P2: Hall liberado, listo para volver CCW.");
    }

    eje1_set_dir(false); // CCW
    eje1_ramp_freq(FREQ_MIN_HZ_1, FREQ_PASS2_HZ_1, 800U);
    ESP_LOGI(TAG, "[EJE1] P2: Regresando lento CCW (%.0f Hz)", (double)FREQ_PASS2_HZ_1);

    eje1_pwm_start();
    uint32_t t1 = (uint32_t)esp_log_timestamp();
    while (!eje1_hall_raw() && (esp_log_timestamp() - t1) < 4000) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    eje1_pwm_stop();

    if (!eje1_hall_raw()) {
        ESP_LOGW(TAG, "[EJE1] P2: No se detecto Hall a tiempo. Puede que ya esté encima.");
    }

    eje1_set_dir(false); // CCW
    eje1_set_freq(FREQ_ALIGN_HZ_1);
    eje1_pwm_start();
    if (!eje1_hall_raw()) {
        eje1_wait_hall_active();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    eje1_pwm_stop();

    ESP_LOGI(TAG, "[EJE1] P2: Alineado sobre HOME (hall).");
}

// GPIO init eje1
static void eje1_gpio_init(void)
{
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << PIN_TB6560_STEP) |
                        (1ULL << PIN_TB6560_DIR ) |
                        (1ULL << PIN_TB6560_EN  ),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_conf);

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_HALL_1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_conf);

    gpio_set_level(PIN_TB6560_EN, 1);
    gpio_set_level(PIN_TB6560_DIR, 1);
}

/* HOMING EJE1 (Quitamos los flags individuales) */
static void homing_eje1(void)
{
    ESP_LOGI(TAG, "[EJE1] Inicio homing 2-pass CW/CCW");
    eje1_gpio_init();
    eje1_pwm_init(FREQ_MIN_HZ_1);
    eje1_enable_driver(true);

    if (eje1_hall_raw()) {
        ESP_LOGI(TAG, "[EJE1] Hall activo al inicio -> media vuelta CW para despegar");
        double seconds = (double)HALF_TURN_STEPS_1 / (double)FREQ_HALF_TURN_1;
        uint32_t ms = (uint32_t)(seconds * 1000.0 + 0.5);
        eje1_set_dir(0); // CW 
        eje1_set_freq(FREQ_HALF_TURN_1);
        eje1_pwm_start();
        vTaskDelay(pdMS_TO_TICKS(ms));
        eje1_pwm_stop();
        vTaskDelay(pdMS_TO_TICKS(60));
    }

    eje1_pass1_seek_cw();
    eje1_pass2_return_align_ccw();

    eje1_enable_driver(false);
    eje1_pwm_stop();
    ESP_LOGI(TAG, "[EJE1] Homing COMPLETO y posicionado en HOME.");
}


/* ============================================================
 * UTILIDADES GENERICAS PARA EJE2 / EJE3
 * ============================================================ */

/* ---------- EJE2 LOW LEVEL ---------- */
static inline int eje2_hall_read(void) { return (gpio_get_level(PIN_HALL_2) == 0); }
static void eje2_wait_hall_low_estable(void) {
    int cnt = 0;
    for(;;){
        if (eje2_hall_read()) { if (++cnt >= HALL_DEBOUNCE_N_2) return; } else { cnt = 0; }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2));
    }
}
static void eje2_wait_hall_high_estable(void) {
    int cnt = 0;
    for(;;){
        if (!eje2_hall_read()) { if (++cnt >= HALL_DEBOUNCE_N_2) return; } else { cnt = 0; }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2));
    }
}
static inline void eje2_enable_driver(bool on) { gpio_set_level(PIN_EN_2, on ? 0 : 1); }
static inline void eje2_set_dir(int dir) {
    gpio_set_level(PIN_DIR_2, dir);
    esp_rom_delay_us(20);
}
static void eje2_pwm_init(float start_hz) {
    ledc_timer_config_t t = { .speed_mode = LEDC_MODE_2, .duty_resolution = LEDC_RES_2, .timer_num = LEDC_TIMER_2_ID, .freq_hz = (uint32_t)start_hz, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num = PIN_STEP_2, .speed_mode = LEDC_MODE_2, .channel = LEDC_CH_2, .timer_sel = LEDC_TIMER_2_ID, .duty = LEDC_DUTY_OFF_2, .hpoint = 0 };
    ledc_channel_config(&c);
}
static inline void eje2_pwm_start(void) {
    ledc_set_duty(LEDC_MODE_2, LEDC_CH_2, LEDC_DUTY_ON_2);
    ledc_update_duty(LEDC_MODE_2, LEDC_CH_2);
}
static inline void eje2_pwm_stop(void) { ledc_stop(LEDC_MODE_2, LEDC_CH_2, LEDC_DUTY_OFF_2); }
static inline void eje2_set_freq(float hz) {
    if (hz < 1.0f) hz = 1.0f;
    ledc_set_freq(LEDC_MODE_2, LEDC_TIMER_2_ID, (uint32_t)hz);
}
static void eje2_ramp_freq(float fi, float ff, uint32_t dur_ms) {
    if (dur_ms == 0 || fabsf(ff - fi) < 1.0f) { eje2_set_freq(ff); return; }
    uint32_t steps = dur_ms / 5U; if (!steps) steps = 1;
    for(uint32_t i=0; i<=steps; i++){
        float u = (float)i / (float)steps;
        float s = 0.5f * (1.0f - cosf((float)M_PI * u));
        eje2_set_freq(fi + (ff - fi) * s);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ---------- EJE3 LOW LEVEL ---------- */
static inline int eje3_hall_read(void) { return (gpio_get_level(PIN_HALL_3) == 0); }
static void eje3_wait_hall_low_estable(void) {
    int cnt = 0;
    for(;;){
        if (eje3_hall_read()) { if (++cnt >= HALL_DEBOUNCE_N_3) return; } else { cnt = 0; }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_3));
    }
}
static void eje3_wait_hall_high_estable(void) {
    int cnt = 0;
    for(;;){
        if (!eje3_hall_read()) { if (++cnt >= HALL_DEBOUNCE_N_3) return; } else { cnt = 0; }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_3));
    }
}
static inline void eje3_enable_driver(bool on) { gpio_set_level(PIN_EN_3, on ? 0 : 1); }
static inline void eje3_set_dir(int dir) {
    gpio_set_level(PIN_DIR_3, dir);
    esp_rom_delay_us(20);
}
static void eje3_pwm_init(float start_hz) {
    ledc_timer_config_t t = { .speed_mode = LEDC_MODE_3, .duty_resolution = LEDC_RES_3, .timer_num = LEDC_TIMER_3_ID, .freq_hz = (uint32_t)start_hz, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num = PIN_STEP_3, .speed_mode = LEDC_MODE_3, .channel = LEDC_CH_3, .timer_sel = LEDC_TIMER_3_ID, .duty = LEDC_DUTY_OFF_3, .hpoint = 0 };
    ledc_channel_config(&c);
}
static inline void eje3_pwm_start(void) {
    ledc_set_duty(LEDC_MODE_3, LEDC_CH_3, LEDC_DUTY_ON_3);
    ledc_update_duty(LEDC_MODE_3, LEDC_CH_3);
}
static inline void eje3_pwm_stop(void) { ledc_stop(LEDC_MODE_3, LEDC_CH_3, LEDC_DUTY_OFF_3); }
static inline void eje3_set_freq(float hz) {
    if (hz < 1.0f) hz = 1.0f;
    ledc_set_freq(LEDC_MODE_3, LEDC_TIMER_3_ID, (uint32_t)hz);
}
static void eje3_ramp_freq(float fi, float ff, uint32_t dur_ms) {
    if (dur_ms == 0 || fabsf(ff - fi) < 1.0f) { eje3_set_freq(ff); return; }
    uint32_t steps = dur_ms / 5U; if (!steps) steps = 1;
    for(uint32_t i=0; i<=steps; i++){
        float u = (float)i / (float)steps;
        float s = 0.5f * (1.0f - cosf((float)M_PI * u));
        eje3_set_freq(fi + (ff - fi) * s);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


/* ============================================================
 * LÓGICA DE HOMING EJE2 (HOME vs ENDSTOP)
 * ============================================================ */

typedef enum { SOBREPASO_OK = 0, SOBREPASO_ENDSTOP = 1 } sobrepaso_result_t;

/* ---------- EJE2: chequeo sobrepaso ---------- */
static sobrepaso_result_t eje2_verificar_sobrepaso(void) {
    eje2_set_freq(FREQ_VERIFICACION_HZ_2);
    eje2_pwm_start();
    if (eje2_hall_read()) {
        eje2_wait_hall_high_estable();
        ESP_LOGD(TAG, "[EJE2] Salimos del imán inicial antes de contar tiempo.");
    }
    const int64_t t0 = esp_timer_get_time();
    int cnt_low = 0;
    while (1) {
        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - t0)/1000);
        if (eje2_hall_read()) {
            if (++cnt_low >= HALL_DEBOUNCE_N_2) {
                ESP_LOGW(TAG, "[EJE2] ENDSTOP detectado (segundo imán).");
                eje2_pwm_stop(); return SOBREPASO_ENDSTOP;
            }
        } else { cnt_low = 0; }
        if (elapsed_ms >= TIEMPO_SOBREPASO_MS_2) {
            ESP_LOGI(TAG, "[EJE2] Sin segundo imán -> OK (HOME probable).");
            eje2_pwm_stop(); return SOBREPASO_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2));
    }
}

/* ---------- EJE3: chequeo sobrepaso ---------- */
static sobrepaso_result_t eje3_verificar_sobrepaso(void) {
    eje3_set_freq(FREQ_VERIFICACION_HZ_3);
    eje3_pwm_start();
    if (eje3_hall_read()) {
        eje3_wait_hall_high_estable();
        ESP_LOGD(TAG, "[EJE3] Salimos del imán inicial antes de contar tiempo.");
    }
    const int64_t t0 = esp_timer_get_time();
    int cnt_low = 0;
    while (1) {
        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - t0)/1000);
        if (eje3_hall_read()) {
            if (++cnt_low >= HALL_DEBOUNCE_N_3) {
                ESP_LOGW(TAG, "[EJE3] ENDSTOP detectado (segundo imán).");
                eje3_pwm_stop(); return SOBREPASO_ENDSTOP;
            }
        } else { cnt_low = 0; }
        if (elapsed_ms >= TIEMPO_SOBREPASO_MS_3) {
            ESP_LOGI(TAG, "[EJE3] Sin segundo imán -> OK (HOME probable).");
            eje3_pwm_stop(); return SOBREPASO_OK;
}
