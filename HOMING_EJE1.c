#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "TB6560_HOMING_2PASS";

// ===== Pines =====
#define PIN_TB6560_STEP   18
#define PIN_TB6560_DIR    17
#define PIN_TB6560_EN     16
#define PIN_HALL          26   // activo-bajo (usar pull-up)

// ===== Motor / Cinematica =====
#define MOTOR_STEPS_PER_REV   200U
#define MICROSTEP               8U
#define HALF_TURN_STEPS   ((MOTOR_STEPS_PER_REV * MICROSTEP) / 2U)

// ===== Frecuencias (Hz) =====
#define FREQ_MIN_HZ        400.0f   // arranque suave
#define FREQ_PASS1_HZ      900.0f   // pasada 1 (CW, mas rapida)
#define FREQ_PASS2_HZ      450.0f   // pasada 2 (CCW, mas lenta para afinar)
#define FREQ_ALIGN_HZ      250.0f   // creep/alineacion muy lento
#define FREQ_RELEASE_HZ    500.0f   // para soltar sensor si esta LOW
#define FREQ_HALF_TURN     800.0f   // media vuelta de despegue

// ===== Tiempos (ms) =====
#define RAMP_MS           1200U     // rampa principal mas corta
#define RAMP_DOWN_MS      1000U
#define OVERTRAVEL_MS       15U     // micro avance tras detectar Hall (pasada 1)
#define SEPARATE_MS         60U     // separacion minima si el Hall sigue activo antes de P2
#define HALL_SAMPLE_MS       1U
#define HALL_DEBOUNCE_N      3

// ===== LEDC =====
#define LEDC_TIMER_RES     LEDC_TIMER_10_BIT
#define LEDC_MODE          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_CH            LEDC_CHANNEL_0
#define LEDC_DUTY_ON       (1 << 6)   // ~50% (64/127)
#define LEDC_DUTY_OFF      0

// ---------- Utilidades GPIO/PWM ----------
static inline void tb_enable(bool on){ gpio_set_level(PIN_TB6560_EN, on ? 0 : 1); } // activo-bajo
static inline void tb_set_dir(bool cw){ gpio_set_level(PIN_TB6560_DIR, cw ? 1 : 0); }
static inline int  hall_raw(void)     { return gpio_get_level(PIN_HALL) == 0; }     // 1 si LOW

static int hall_is_active(void) {  // debounce a LOW
    int cnt = 0;
    for (;;) {
        if (hall_raw()) cnt++; else cnt = 0;
        if (cnt >= HALL_DEBOUNCE_N) return 1;
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS));
    }
}
static int hall_is_inactive(void) { // debounce a HIGH
    int cnt = 0;
    for (;;) {
        if (!hall_raw()) cnt++; else cnt = 0;
        if (cnt >= HALL_DEBOUNCE_N) return 1;
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS));
    }
}

static void pwm_start(void){
    ledc_set_duty(LEDC_MODE, LEDC_CH, LEDC_DUTY_ON);
    ledc_update_duty(LEDC_MODE, LEDC_CH);
}
static void pwm_stop(void){
    ledc_stop(LEDC_MODE, LEDC_CH, LEDC_DUTY_OFF);
}

static void ledc_init(float start_hz){
    ledc_timer_config_t t = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_TIMER_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = (uint32_t)start_hz,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num   = PIN_TB6560_STEP,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CH,
        .timer_sel  = LEDC_TIMER,
        .duty       = LEDC_DUTY_ON,
        .hpoint     = 0
    };
    ledc_channel_config(&c);
}

static inline void set_step_freq(float hz){
    if (hz < FREQ_ALIGN_HZ) hz = FREQ_ALIGN_HZ; // mantener rango util
    uint32_t set = ledc_set_freq(LEDC_MODE, LEDC_TIMER, (uint32_t)hz);
    if (set == 0) ESP_LOGW(TAG, "No se pudo setear freq (%.1f Hz)", (double)hz);
}

// ---------- Rampa S-curve ----------
static void ramp_to_freq(float f0, float f1, uint32_t ramp_ms){
    const double PI = 3.14159265358979323846;
    const TickType_t tick = pdMS_TO_TICKS(5);
    uint32_t steps = (ramp_ms / 5U);
    if (steps == 0) { set_step_freq(f1); return; }

    pwm_start();
    for (uint32_t i = 0; i <= steps; ++i) {
        double t = (double)i / (double)steps;
        double s = 0.5 * (1.0 - cos(PI * t));
        double f = f0 + (f1 - f0) * s;
        set_step_freq((float)f);
        vTaskDelay(tick);
    }
}

// ---------- Avanzar fijo en ms con la freq actual ----------
static void advance_ms(uint32_t ms){
    pwm_start();
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ---------- Mover por CANTIDAD DE PASOS (aprox por tiempo) ----------
static void move_steps_by_time(uint32_t steps, float freq_hz, bool cw){
    tb_set_dir(cw);
    set_step_freq(freq_hz);
    pwm_start();
    double seconds = (double)steps / (double)freq_hz;
    uint32_t ms = (uint32_t)(seconds * 1000.0 + 0.5);
    vTaskDelay(pdMS_TO_TICKS(ms));
    pwm_stop();
}

// ---------- Pasada 1 (CW): detectar Hall por flanco, micro overtravel y STOP ----------
static void pass1_seek_cw(void){
    tb_set_dir(true); // CW
    // Si arranco encima del hall, separo apenas en CCW y retomo CW
    if (hall_raw()) {
        tb_set_dir(false);
        set_step_freq(FREQ_RELEASE_HZ);
        pwm_start();
        (void)hall_is_inactive();
        vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS));
        pwm_stop();
        tb_set_dir(true);
    }

    ramp_to_freq(FREQ_MIN_HZ, FREQ_PASS1_HZ, RAMP_MS);
    ESP_LOGI(TAG, "P1: Buscando Hall (%.0f Hz, CW)", (double)FREQ_PASS1_HZ);

    // Girar hasta detectar LOW (debounce)
    pwm_start();
    (void)hall_is_active();   // bloquea hasta LOW confirmado

    // Freno inmediato
    pwm_stop();
    vTaskDelay(pdMS_TO_TICKS(10)); // mini pausa para inercia

    // Micro avance (overtravel) para asegurar cruce completo del borde en CW
    tb_set_dir(true);
    set_step_freq(FREQ_ALIGN_HZ);
    pwm_start();
    vTaskDelay(pdMS_TO_TICKS(OVERTRAVEL_MS)); // 10–15 ms
    pwm_stop();

    ESP_LOGI(TAG, "P1: Hall detectado y cruzado (CW).");
}

// ---------- Pasada 2 (CCW): volver lento y quedar exactamente en el Hall ----------
static void pass2_return_align_ccw(void){
    // 1) Asegurar que no iniciamos P2 todavia sobre el sensor
    if (hall_raw()) {
        tb_set_dir(true); // CW para salir del Hall
        set_step_freq(FREQ_RELEASE_HZ);
        pwm_start();

        uint32_t t0 = (uint32_t)esp_log_timestamp();
        while (hall_raw() && (esp_log_timestamp() - t0) < 3000) { // max 3 s
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        pwm_stop();
        vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS)); // separacion minima
        ESP_LOGI(TAG, "P2: Hall liberado, listo para volver CCW");
    }

    // 2) Volver CCW lentamente hasta detectar el Hall nuevamente
    tb_set_dir(false); // CCW
    ramp_to_freq(FREQ_MIN_HZ, FREQ_PASS2_HZ, 800U); // rampa mas corta en P2
    ESP_LOGI(TAG, "P2: Regresando hacia Hall (%.0f Hz, CCW, mas lento)", (double)FREQ_PASS2_HZ);

    pwm_start();
    uint32_t t1 = (uint32_t)esp_log_timestamp();
    while (!hall_raw() && (esp_log_timestamp() - t1) < 4000) { // max 4 s
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    pwm_stop();

    if (!hall_raw()) {
        ESP_LOGW(TAG, "P2: No se detecto Hall dentro del tiempo. Quizas ya estaba encima.");
        return;
    }

    // 3) Alineacion fina: microajuste CCW a velocidad muy baja
    tb_set_dir(false); // CCW
    set_step_freq(FREQ_ALIGN_HZ);
    pwm_start();
    // Si por timing ya quedo LOW, no hace falta mover; si no, espera a que se active
    if (!hall_raw()) (void)hall_is_active();
    vTaskDelay(pdMS_TO_TICKS(20)); // pequeno avance controlado para sentar en el borde
    pwm_stop();

    ESP_LOGI(TAG, "P2: Alineado exactamente sobre Hall (CCW).");
}

// ---------- GPIO ----------
static void io_init(void){
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<PIN_TB6560_DIR) | (1ULL<<PIN_TB6560_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    gpio_config_t hall = {
        .pin_bit_mask = (1ULL<<PIN_HALL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     // Hall activo-bajo
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&hall);
}

// ---------- APP PRINCIPAL ----------
void app_main(void){
    ESP_LOGI(TAG, "Inicio homing (2 pasadas: CW y CCW)");
    io_init();
    ledc_init(FREQ_MIN_HZ);
    tb_enable(true);

    // Arranque inteligente: si el Hall esta en LOW, media vuelta para despegar
    if (hall_raw()) {
        ESP_LOGI(TAG, "Hall LOW al iniciar -> media vuelta CW");
        double seconds = (double)HALF_TURN_STEPS / (double)FREQ_HALF_TURN;
        uint32_t ms = (uint32_t)(seconds * 1000.0 + 0.5);
        tb_set_dir(!true);
        set_step_freq(FREQ_HALF_TURN);
        pwm_start();
        vTaskDelay(pdMS_TO_TICKS(ms));
        pwm_stop();
        vTaskDelay(pdMS_TO_TICKS(60));
    }

    // 1) Pasada CW (detecta, micro overtravel y se detiene)
    pass1_seek_cw();

    // 2) Pasada CCW (mas lenta) y queda exactamente sobre el Hall
    pass2_return_align_ccw();

    tb_enable(false);
    pwm_stop();
    ESP_LOGI(TAG, "Homing COMPLETO: 2 pasadas y posicionado en Hall.");
}
