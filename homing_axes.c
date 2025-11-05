#include "homing_axes.h" // Incluimos nuestra cabecera

// Incluimos todas las dependencias del sistema
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"     // esp_timer_get_time()
#include "esp_rom_sys.h"   // esp_rom_delay_us()

// TAG para los logs, ahora específico de este "módulo"
static const char *TAG = "HOMING_AXES";

/* ============================================================
 * UTILIDADES EJE1 (Estáticas)
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
        .duty       = LEDC_DUTY_ON_1,    // dejamos duty ya cargado
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
    uint32_t ok = ledc_set_freq(LEDC_MODE_1, LEDC_TIMER_1_ID, (uint32_t)hz);
    if (ok == 0) {
        ESP_LOGW(TAG, "[EJE1] ledc_set_freq fallo (%.1f Hz)", (double)hz);
    }
}

// rampa S-curve eje1
static void eje1_ramp_freq(float f0, float f1, uint32_t ramp_ms)
{
    const double PI = 3.14159265358979323846;
    TickType_t tick = pdMS_TO_TICKS(5);

    uint32_t steps = ramp_ms / 5U;
    if (steps == 0){
        eje1_set_freq(f1);
        return;
    }

    eje1_pwm_start();
    for(uint32_t i=0; i<=steps; i++){
        double u = (double)i / (double)steps;
        double s = 0.5*(1.0 - cos(PI*u));
        double f = f0 + (f1 - f0)*s;
        eje1_set_freq((float)f);
        vTaskDelay(tick);
    }
}

// Paso 1: buscar imán en CW, sobrepasar un toque
static void eje1_pass1_seek_cw(void)
{
    // Asegurar que no arrancamos ya encima del hall.
    if (eje1_hall_raw()) {
        // mover un toque en CCW (liberar)
        eje1_set_dir(false); // CCW
        eje1_set_freq(FREQ_RELEASE_HZ_1);
        eje1_pwm_start();
        eje1_wait_hall_inactive();
        vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1));
        eje1_pwm_stop();
    }

    // Buscar rápido en CW
    eje1_set_dir(true); // CW
    eje1_ramp_freq(FREQ_MIN_HZ_1, FREQ_PASS1_HZ_1, RAMP_MS_1);
    ESP_LOGI(TAG, "[EJE1] P1: Buscando Hall en CW (%.0f Hz)", (double)FREQ_PASS1_HZ_1);

    eje1_pwm_start();
    eje1_wait_hall_active(); // bloquea hasta LOW estable
    eje1_pwm_stop();

    // pequeño delay para inercia
    vTaskDelay(pdMS_TO_TICKS(10));

    // Overtravel pequeño para cruzar bien el borde
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
    // Si sigo arriba del hall al iniciar P2, salgo un toque en CW
    if (eje1_hall_raw()) {
        eje1_set_dir(true); // CW para liberar
        eje1_set_freq(FREQ_RELEASE_HZ_1);
        eje1_pwm_start();

        uint32_t t0 = (uint32_t)esp_log_timestamp();
        while (eje1_hall_raw() &&
               (esp_log_timestamp() - t0) < 3000) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        eje1_pwm_stop();
        vTaskDelay(pdMS_TO_TICKS(SEPARATE_MS_1));

        ESP_LOGI(TAG, "[EJE1] P2: Hall liberado, listo para volver CCW.");
    }

    // Ahora volver CCW más lento hasta re-enganchar el hall
    eje1_set_dir(false); // CCW
    eje1_ramp_freq(FREQ_MIN_HZ_1, FREQ_PASS2_HZ_1, 800U);
    ESP_LOGI(TAG, "[EJE1] P2: Regresando lento CCW (%.0f Hz)", (double)FREQ_PASS2_HZ_1);

    eje1_pwm_start();
    uint32_t t1 = (uint32_t)esp_log_timestamp();
    while (!eje1_hall_raw() &&
           (esp_log_timestamp() - t1) < 4000) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    eje1_pwm_stop();

    if (!eje1_hall_raw()) {
        ESP_LOGW(TAG, "[EJE1] P2: No se detecto Hall a tiempo. Puede que ya esté encima.");
        // seguimos igual
    }

    // Alineación fina CCW MUY lento
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
    // STEP, DIR, EN -> salida
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

    // Hall -> entrada pull-up
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_HALL_1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     // sensor activo-bajo
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_conf);

    // Estado inicial
    gpio_set_level(PIN_TB6560_EN, 1); // deshabilitado (activo-bajo)
    gpio_set_level(PIN_TB6560_DIR, 1);
}


/* ============================================================
 * UTILIDADES GENERICAS EJE2 / EJE3 (Estáticas)
 * ============================================================ */

/* ---------- EJE2 LOW LEVEL ---------- */

static inline int eje2_hall_read(void)
{
    return (gpio_get_level(PIN_HALL_2) == 0); // activo-bajo
}

static void eje2_wait_hall_low_estable(void)
{
    int cnt = 0;
    for(;;){
        if (eje2_hall_read()) {
            if (++cnt >= HALL_DEBOUNCE_N_2) return;
        } else {
            cnt = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2));
    }
}
static void eje2_wait_hall_high_estable(void)
{
    int cnt = 0;
    for(;;){
        if (!eje2_hall_read()) {
            if (++cnt >= HALL_DEBOUNCE_N_2) return;
        } else {
            cnt = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2));
    }
}

static inline void eje2_enable_driver(bool on)
{
    // activo-bajo
    gpio_set_level(PIN_EN_2, on ? 0 : 1);
}
static inline void eje2_set_dir(int dir)
{
    gpio_set_level(PIN_DIR_2, dir);
    esp_rom_delay_us(20);
}
static void eje2_pwm_init(float start_hz)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_MODE_2,
        .duty_resolution = LEDC_RES_2,
        .timer_num       = LEDC_TIMER_2_ID,
        .freq_hz         = (uint32_t)start_hz,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num   = PIN_STEP_2,
        .speed_mode = LEDC_MODE_2,
        .channel    = LEDC_CH_2,
        .timer_sel  = LEDC_TIMER_2_ID,
        .duty       = LEDC_DUTY_OFF_2,
        .hpoint     = 0
    };
    ledc_channel_config(&c);
}
static inline void eje2_pwm_start(void)
{
    ledc_set_duty(LEDC_MODE_2, LEDC_CH_2, LEDC_DUTY_ON_2);
    ledc_update_duty(LEDC_MODE_2, LEDC_CH_2);
}
static inline void eje2_pwm_stop(void)
{
    ledc_stop(LEDC_MODE_2, LEDC_CH_2, LEDC_DUTY_OFF_2);
}
static inline void eje2_set_freq(float hz)
{
    if (hz < 1.0f) hz = 1.0f;
    ledc_set_freq(LEDC_MODE_2, LEDC_TIMER_2_ID, (uint32_t)hz);
}
static void eje2_ramp_freq(float fi, float ff, uint32_t dur_ms)
{
    if (dur_ms == 0 || fabsf(ff - fi) < 1.0f) {
        eje2_set_freq(ff);
        return;
    }
    uint32_t steps = dur_ms / 5U;
    if (!steps) steps = 1;
    for(uint32_t i=0; i<=steps; i++){
        float u = (float)i / (float)steps;
        float s = 0.5f * (1.0f - cosf((float)M_PI * u));
        eje2_set_freq(fi + (ff - fi) * s);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ---------- EJE3 LOW LEVEL ---------- */

static inline int eje3_hall_read(void)
{
    return (gpio_get_level(PIN_HALL_3) == 0); // activo-bajo
}
static void eje3_wait_hall_low_estable(void)
{
    int cnt = 0;
    for(;;){
        if (eje3_hall_read()) {
            if (++cnt >= HALL_DEBOUNCE_N_3) return;
        } else {
            cnt = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_3));
    }
}
static void eje3_wait_hall_high_estable(void)
{
    int cnt = 0;
    for(;;){
        if (!eje3_hall_read()) {
            if (++cnt >= HALL_DEBOUNCE_N_3) return;
        } else {
            cnt = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_3));
    }
}

static inline void eje3_enable_driver(bool on)
{
    gpio_set_level(PIN_EN_3, on ? 0 : 1); // activo-bajo
}
static inline void eje3_set_dir(int dir)
{
    gpio_set_level(PIN_DIR_3, dir);
    esp_rom_delay_us(20);
}
static void eje3_pwm_init(float start_hz)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_MODE_3,
        .duty_resolution = LEDC_RES_3,
        .timer_num       = LEDC_TIMER_3_ID,
        .freq_hz         = (uint32_t)start_hz,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num   = PIN_STEP_3,
        .speed_mode = LEDC_MODE_3,
        .channel    = LEDC_CH_3,
        .timer_sel  = LEDC_TIMER_3_ID,
        .duty       = LEDC_DUTY_OFF_3,
        .hpoint     = 0
    };
    ledc_channel_config(&c);
}
static inline void eje3_pwm_start(void)
{
    ledc_set_duty(LEDC_MODE_3, LEDC_CH_3, LEDC_DUTY_ON_3);
    ledc_update_duty(LEDC_MODE_3, LEDC_CH_3);
}
static inline void eje3_pwm_stop(void)
{
    ledc_stop(LEDC_MODE_3, LEDC_CH_3, LEDC_DUTY_OFF_3);
}
static inline void eje3_set_freq(float hz)
{
    if (hz < 1.0f) hz = 1.0f;
    ledc_set_freq(LEDC_MODE_3, LEDC_TIMER_3_ID, (uint32_t)hz);
}
static void eje3_ramp_freq(float fi, float ff, uint32_t dur_ms)
{
    if (dur_ms == 0 || fabsf(ff - fi) < 1.0f) {
        eje3_set_freq(ff);
        return;
    }
    uint32_t steps = dur_ms / 5U;
    if (!steps) steps = 1;
    for(uint32_t i=0; i<=steps; i++){
        float u = (float)i / (float)steps;
        float s = 0.5f * (1.0f - cosf((float)M_PI * u));
        eje3_set_freq(fi + (ff - fi) * s);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


/* ============================================================
 * LÓGICA DE HOMING EJE2 / EJE3 (Estáticas)
 * ============================================================ */

/* ---------- EJE2: chequeo sobrepaso ---------- */
static sobrepaso_result_t eje2_verificar_sobrepaso(void)
{
    eje2_set_freq(FREQ_VERIFICACION_HZ_2);
    eje2_pwm_start();

    // si estaba sobre el imán al inicio, salir primero
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
                eje2_pwm_stop();
                return SOBREPASO_ENDSTOP;
            }
        } else {
            cnt_low = 0;
        }

        if (elapsed_ms >= TIEMPO_SOBREPASO_MS_2) {
            ESP_LOGI(TAG, "[EJE2] Sin segundo imán -> OK (HOME probable).");
            eje2_pwm_stop();
            return SOBREPASO_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_2));
    }
}

/* ---------- EJE3: chequeo sobrepaso ---------- */
static sobrepaso_result_t eje3_verificar_sobrepaso(void)
{
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
                eje3_pwm_stop();
                return SOBREPASO_ENDSTOP;
            }
        } else {
            cnt_low = 0;
        }

        if (elapsed_ms >= TIEMPO_SOBREPASO_MS_3) {
            ESP_LOGI(TAG, "[EJE3] Sin segundo imán -> OK (HOME probable).");
            eje3_pwm_stop();
            return SOBREPASO_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(HALL_SAMPLE_MS_3));
    }
}


/* ---------- EJE2: rutina escape endstop ---------- */
static void eje2_escape_endstop(int *p_dir_actual)
{
    ESP_LOGW(TAG, "[EJE2] ESCAPE ENDSTOP...");
    gpio_set_level(PIN_LED_2, 1);

    // invertimos dirección
    *p_dir_actual = (*p_dir_actual == DERECHA_2) ? IZQUIERDA_2 : DERECHA_2;
    eje2_set_dir(*p_dir_actual);

    eje2_set_freq(FREQ_BUSQUEDA_HZ_2);
    eje2_pwm_start();

    // 1) Alejarse del primer imán
    eje2_wait_hall_high_estable();

    // 2) Detectar siguiente imán (límite duro)
    eje2_wait_hall_low_estable();
    ESP_LOGW(TAG, "[EJE2] Límite detectado, ignorando y pasando de largo...");

    // 3) Pasar de largo ese imán límite
    eje2_wait_hall_high_estable();

    eje2_pwm_stop();
    gpio_set_level(PIN_LED_2, 0);
    ESP_LOGW(TAG, "[EJE2] ESCAPE READY. Continuar búsqueda normal.");
}

/* ---------- EJE3: rutina escape endstop ---------- */
static void eje3_escape_endstop(int *p_dir_actual)
{
    ESP_LOGW(TAG, "[EJE3] ESCAPE ENDSTOP...");
    gpio_set_level(PIN_LED_3, 1);

    *p_dir_actual = (*p_dir_actual == DERECHA_3) ? IZQUIERDA_3 : DERECHA_3;
    eje3_set_dir(*p_dir_actual);

    eje3_set_freq(FREQ_BUSQUEDA_HZ_3);
    eje3_pwm_start();

    eje3_wait_hall_high_estable(); // alejarse
    eje3_wait_hall_low_estable();  // detectar límite
    ESP_LOGW(TAG, "[EJE3] Límite detectado, ignorar y pasar de largo...");
    eje3_wait_hall_high_estable(); // salir del límite

    eje3_pwm_stop();
    gpio_set_level(PIN_LED_3, 0);
    ESP_LOGW(TAG, "[EJE3] ESCAPE READY. Continuar búsqueda normal.");
}


/* ---------- EJE2: arranque si ya estás sobre el imán ---------- */
static bool eje2_arranque_sobre_iman(int *p_dir_actual)
{
    ESP_LOGW(TAG, "[EJE2] Arrancamos SOBRE imán. Chequeo bidireccional...");

    // 1) Probar DERECHA
    ESP_LOGI(TAG, "[EJE2] Test DERECHA");
    *p_dir_actual = DERECHA_2;
    eje2_set_dir(DERECHA_2);

    if (eje2_verificar_sobrepaso() == SOBREPASO_ENDSTOP) {
        ESP_LOGW(TAG, "[EJE2] ENDSTOP a la derecha");
        eje2_escape_endstop(p_dir_actual);
        return false; // no homing todavía
    }

    // 2) Volver al imán y probar IZQUIERDA
    ESP_LOGI(TAG, "[EJE2] Derecha OK. Volviendo al imán y probando IZQUIERDA...");
    eje2_set_dir(IZQUIERDA_2);
    eje2_pwm_start();
    eje2_wait_hall_low_estable(); // volver al mismo imán
    eje2_pwm_stop();

    *p_dir_actual = IZQUIERDA_2;
    eje2_set_dir(IZQUIERDA_2);

    if (eje2_verificar_sobrepaso() == SOBREPASO_ENDSTOP) {
        ESP_LOGW(TAG, "[EJE2] ENDSTOP a la izquierda");
        eje2_escape_endstop(p_dir_actual);
        return false;
    }

    // Si llegamos acá, ninguna dirección fue endstop => estamos en HOME real.
    ESP_LOGI(TAG, "[EJE2] HOME confirmado (en arranque). Posicionando en el borde...");
    eje2_set_dir(DERECHA_2);
    eje2_pwm_start();
    eje2_wait_hall_low_estable();
    eje2_pwm_stop();

    eje2_enable_driver(false);
    gpio_set_level(PIN_LED_2, 1);
    ESP_LOGI(TAG, "[EJE2] HOME posicionado.");
    return true;
}

/* ---------- EJE3: arranque si ya estás sobre el imán ---------- */
static bool eje3_arranque_sobre_iman(int *p_dir_actual)
{
    ESP_LOGW(TAG, "[EJE3] Arrancamos SOBRE imán. Chequeo bidireccional...");

    // 1) Probar DERECHA
    ESP_LOGI(TAG, "[EJE3] Test DERECHA");
    *p_dir_actual = DERECHA_3;
    eje3_set_dir(DERECHA_3);

    if (eje3_verificar_sobrepaso() == SOBREPASO_ENDSTOP) {
        ESP_LOGW(TAG, "[EJE3] ENDSTOP a la derecha");
        eje3_escape_endstop(p_dir_actual);
        return false;
    }

    // 2) Volver al imán y probar IZQUIERDA
    ESP_LOGI(TAG, "[EJE3] Derecha OK. Volviendo al imán y probando IZQUIERDA...");
    eje3_set_dir(IZQUIERDA_3);
    eje3_pwm_start();
    eje3_wait_hall_low_estable(); // volver al mismo imán
    eje3_pwm_stop();

    *p_dir_actual = IZQUIERDA_3;
    eje3_set_dir(IZQUIERDA_3);

    if (eje3_verificar_sobrepaso() == SOBREPASO_ENDSTOP) {
        ESP_LOGW(TAG, "[EJE3] ENDSTOP a la izquierda");
        eje3_escape_endstop(p_dir_actual);
        return false;
    }

    ESP_LOGI(TAG, "[EJE3] HOME confirmado (en arranque). Posicionando en el borde...");
    eje3_set_dir(DERECHA_3);
    eje3_pwm_start();
    eje3_wait_hall_low_estable();
    eje3_pwm_stop();

    eje3_enable_driver(false);
    gpio_set_level(PIN_LED_3, 1);
    ESP_LOGI(TAG, "[EJE3] HOME posicionado.");
    return true;
}


/* ---------- EJE2: decide si es HOME o ENDSTOP cuando detecta un imán en carrera ---------- */
static bool eje2_confirmar_home_o_endstop(int *p_dir_actual)
{
    // rampa de búsqueda -> verificación más lenta
    eje2_ramp_freq(FREQ_BUSQUEDA_HZ_2, FREQ_VERIFICACION_HZ_2, TIEMPO_RAMPA_MS_2);

    if (eje2_verificar_sobrepaso() == SOBREPASO_ENDSTOP) {
        eje2_escape_endstop(p_dir_actual);
        return false; // seguir buscando
    }

    // Es HOME, ahora retroceso corto para quedar justo en el imán
    ESP_LOGI(TAG, "[EJE2] HOME detectado. Ajustando posición fina...");
    *p_dir_actual = (*p_dir_actual == DERECHA_2) ? IZQUIERDA_2 : DERECHA_2;
    eje2_set_dir(*p_dir_actual);

    eje2_set_freq(FREQ_VERIFICACION_HZ_2);
    eje2_pwm_start();
    eje2_wait_hall_low_estable();
    eje2_pwm_stop();

    eje2_enable_driver(false);
    gpio_set_level(PIN_LED_2, 1);
    ESP_LOGI(TAG, "[EJE2] HOME POSICIONADO FINAL.");
    return true;
}

/* ---------- EJE3: igual que eje2 pero con sufijo 3 ---------- */
static bool eje3_confirmar_home_o_endstop(int *p_dir_actual)
{
    eje3_ramp_freq(FREQ_BUSQUEDA_HZ_3, FREQ_VERIFICACION_HZ_3, TIEMPO_RAMPA_MS_3);

    if (eje3_verificar_sobrepaso() == SOBREPASO_ENDSTOP) {
        eje3_escape_endstop(p_dir_actual);
        return false;
    }

    ESP_LOGI(TAG, "[EJE3] HOME detectado. Ajustando posición fina...");
    *p_dir_actual = (*p_dir_actual == DERECHA_3) ? IZQUIERDA_3 : DERECHA_3;
    eje3_set_dir(*p_dir_actual);

    eje3_set_freq(FREQ_VERIFICACION_HZ_3);
    eje3_pwm_start();
    eje3_wait_hall_low_estable();
    eje3_pwm_stop();

    eje3_enable_driver(false);
    gpio_set_level(PIN_LED_3, 1);
    ESP_LOGI(TAG, "[EJE3] HOME POSICIONADO FINAL.");
    return true;
}


/* ============================================================
 * GPIO INIT eje2 / eje3 (Estáticas)
 * ============================================================ */

static void eje2_gpio_init(void)
{
    // DIR, EN, STEP, LED -> salida
    gpio_config_t out_conf = {
        .pin_bit_mask =
            (1ULL << PIN_DIR_2 ) |
            (1ULL << PIN_EN_2  ) |
            (1ULL << PIN_STEP_2) |
            (1ULL << PIN_LED_2 ),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out_conf);

    // HALL -> entrada pull-up
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_HALL_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in_conf);

    // estados iniciales
    gpio_set_level(PIN_DIR_2, DERECHA_2);
    gpio_set_level(PIN_EN_2, 1);     // deshabilitado
    gpio_set_level(PIN_LED_2, 0);
}

static void eje3_gpio_init(void)
{
    gpio_config_t out_conf = {
        .pin_bit_mask =
            (1ULL << PIN_DIR_3 ) |
            (1ULL << PIN_EN_3  ) |
            (1ULL << PIN_STEP_3) |
            (1ULL << PIN_LED_3 ),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out_conf);

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_HALL_3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in_conf);

    gpio_set_level(PIN_DIR_3, DERECHA_3);
    gpio_set_level(PIN_EN_3, 1); // deshabilitado
    gpio_set_level(PIN_LED_3, 0);
}


/* ============================================================
 * IMPLEMENTACIÓN FUNCIONES PÚBLICAS
 * ============================================================ */

/* HOMING EJE1 */
void homing_eje1(void)
{
    ESP_LOGI(TAG, "[EJE1] Inicio homing 2-pass CW/CCW");
    eje1_gpio_init();
    eje1_pwm_init(FREQ_MIN_HZ_1);
    eje1_enable_driver(true);

    // Si arranca encima del hall -> media vuelta rápida CW para despegar
    if (eje1_hall_raw()) {
        ESP_LOGI(TAG, "[EJE1] Hall activo al inicio -> media vuelta CW para despegar");
        double seconds = (double)HALF_TURN_STEPS_1 / (double)FREQ_HALF_TURN_1;
        uint32_t ms = (uint32_t)(seconds * 1000.0 + 0.5);

        eje1_set_dir(true); // CW
        eje1_set_freq(FREQ_HALF_TURN_1);
        eje1_pwm_start();
        vTaskDelay(pdMS_TO_TICKS(ms));
        eje1_pwm_stop();
        vTaskDelay(pdMS_TO_TICKS(60));
    }

    // Pasada 1 CW + sobrepaso
    eje1_pass1_seek_cw();

    // Pasada 2 CCW y ajuste fino
    eje1_pass2_return_align_ccw();

    eje1_enable_driver(false);
    eje1_pwm_stop();
    ESP_LOGI(TAG, "[EJE1] Homing COMPLETO y posicionado en HOME.");
}


/* HOMING EJE2 */
void homing_eje2(void)
{
    ESP_LOGI(TAG, "[EJE2] Iniciando homing bidireccional...");
    eje2_gpio_init();
    eje2_pwm_init(FREQ_BUSQUEDA_HZ_2);

    eje2_enable_driver(true);
    vTaskDelay(pdMS_TO_TICKS(20));

    int dir_actual = DERECHA_2;
    eje2_set_dir(dir_actual);

    // Caso especial: ya arrancamos sobre el imán
    if (eje2_hall_read()) {
        if (eje2_arranque_sobre_iman(&dir_actual)) {
            ESP_LOGI(TAG, "[EJE2] Homing finalizado directamente en arranque.");
            return;
        }
    } else {
        ESP_LOGI(TAG, "[EJE2] Arranque normal. Buscando HOME...");
    }

    // Movimiento continuo buscando el primer imán
    ESP_LOGI(TAG, "[EJE2] Giro buscando imán a %.0f Hz", (double)FREQ_BUSQUEDA_HZ_2);
    eje2_set_freq(FREQ_BUSQUEDA_HZ_2);
    eje2_pwm_start();

    while (1) {
        // Esperar a tocar un imán
        eje2_wait_hall_low_estable();
        eje2_pwm_stop();
        ESP_LOGI(TAG, "[EJE2] Imán detectado. Analizando si es HOME o ENDSTOP...");

        if (eje2_confirmar_home_o_endstop(&dir_actual)) {
            break;
        }

        // no era home: seguimos moviendo en la nueva dirección
        ESP_LOGI(TAG, "[EJE2] No era HOME. Continuando búsqueda...");
        eje2_set_freq(FREQ_BUSQUEDA_HZ_2);
        eje2_pwm_start();
    }

    eje2_enable_driver(false);
    eje2_pwm_stop();
    ESP_LOGI(TAG, "[EJE2] Homing COMPLETO.");
}

/* HOMING EJE3 */
void homing_eje3(void)
{
    ESP_LOGI(TAG, "[EJE3] Iniciando homing bidireccional...");
    eje3_gpio_init();
    eje3_pwm_init(FREQ_BUSQUEDA_HZ_3);

    eje3_enable_driver(true);
    vTaskDelay(pdMS_TO_TICKS(20));

    int dir_actual = DERECHA_3;
    eje3_set_dir(dir_actual);

    if (eje3_hall_read()) {
        if (eje3_arranque_sobre_iman(&dir_actual)) {
            ESP_LOGI(TAG, "[EJE3] Homing finalizado directamente en arranque.");
            goto done;
        }
    } else {
        ESP_LOGI(TAG, "[EJE3] Arranque normal. Buscando HOME...");
    }

    ESP_LOGI(TAG, "[EJE3] Giro buscando imán a %.0f Hz", (double)FREQ_BUSQUEDA_HZ_3);
    eje3_set_freq(FREQ_BUSQUEDA_HZ_3);
    eje3_pwm_start();

    while (1) {
        eje3_wait_hall_low_estable();
        eje3_pwm_stop();
        ESP_LOGI(TAG, "[EJE3] Imán detectado. Analizando si es HOME o ENDSTOP...");

        if (eje3_confirmar_home_o_endstop(&dir_actual)) {
            break;
        }

        ESP_LOGI(TAG, "[EJE3] No era HOME. Continuando búsqueda...");
        eje3_set_freq(FREQ_BUSQUEDA_HZ_3);
        eje3_pwm_start();
    }

    eje3_pwm_stop();

done:
    eje3_enable_driver(false);
    eje3_pwm_stop();
    ESP_LOGI(TAG, "[EJE3] Homing COMPLETO.");
}
