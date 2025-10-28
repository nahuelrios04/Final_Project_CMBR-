#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"      // <-- para esp_timer_get_time()
#include "esp_rom_sys.h"    // <-- para esp_rom_delay_us()

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

//////////////////////////////////////////////////////////////////////////////////////// aca se mexcla con el homing 2 

void homing_eje1(void);
void homing_eje2(void);

// (ELIMINADO: segunda definición de TAG)  // static const char *TAG = "BUSQUEDA_HOME_BIDIRECCIONAL";

// ===================== Pines =====================
#define PIN_STEP_2      21
#define PIN_DIR_2       22
#define PIN_EN_2        23
#define PIN_HALL_2      25
#define PIN_LED_D2      2

// ===================== LEDC (Control del motor) =====================
#define LEDC_MODO_H2         LEDC_HIGH_SPEED_MODE
#define LEDC_TEMPORIZADOR_H2 LEDC_TIMER_0
#define LEDC_CANAL_H2        LEDC_CHANNEL_0
#define LEDC_RESOLUCION_H2   LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_H2      (1 << 6)
#define LEDC_DUTY_OFF_H2     0

// ===================== Parámetros de Movimiento =====================
#define FREQ_BUSQUEDA_HZ_H2    1000.0f
#define FREQ_VERIFICACION_HZ_2  400.0f     // <-- este reemplaza a FREQ_BUSQUEDA_HZ_2
#define TIEMPO_RAMPA_MS_2       120U
#define TIEMPO_SOBREPASO_MS_2  4000U       // <-- corrige TTIEMPO_SOBREPASO_MS_2

// ===================== Parámetros del sensor Hall =====================
#define MUESTRA_HALL_MS_2     1
#define NUMERO_MUESTRAS_N_2   2

// ===================== Direcciones del motor =====================
#define DERECHA_2   1
#define IZQUIERDA_2 0

static inline int  leer_sensor_hall(void) {
    return gpio_get_level(PIN_HALL_2) == 0;
}
static inline void habilitar_driver(bool encendido) {
    gpio_set_level(PIN_EN_2, encendido ? 0 : 1);
}
static inline void fijar_direccion(int dir) {
    gpio_set_level(PIN_DIR_2, dir);
    esp_rom_delay_us(20);
}
static void inicializar_pwm(float frecuencia_inicial_hz) {
    ledc_timer_config_t t = { .speed_mode = LEDC_MODO_H2, .duty_resolution = LEDC_RESOLUCION_H2, .timer_num = LEDC_TEMPORIZADOR_H2, .freq_hz = (uint32_t)frecuencia_inicial_hz, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num = PIN_STEP_2, .speed_mode = LEDC_MODO_H2, .channel = LEDC_CANAL_H2, .timer_sel = LEDC_TEMPORIZADOR_H2, .duty = LEDC_DUTY_OFF_H2, .hpoint = 0 };
    ledc_channel_config(&c);
}
static inline void arrancar_motor(void) {
    ledc_set_duty(LEDC_MODO_H2, LEDC_CANAL_H2, LEDC_DUTY_ON_H2);
    ledc_update_duty(LEDC_MODO_H2, LEDC_CANAL_H2);
}
static inline void parar_motor(void) {
    ledc_stop(LEDC_MODO_H2, LEDC_CANAL_H2, LEDC_DUTY_OFF_H2);
}
static inline void fijar_frecuencia_pasos(float hz) {
    if (hz < 1.0f) hz = 1.0f;
    ledc_set_freq(LEDC_MODO_H2, LEDC_TEMPORIZADOR_H2, (uint32_t)hz);
}
static void esperar_hall_bajo_estable(void) {
    int contador = 0;
    while (true) {
        if (leer_sensor_hall()) {
            if (++contador >= NUMERO_MUESTRAS_N_2) return;
        } else {
            contador = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(MUESTRA_HALL_MS_2));
    }
}
static void esperar_hall_alto_estable(void) {
    int contador = 0;
    while (true) {
        if (!leer_sensor_hall()) {
            if (++contador >= NUMERO_MUESTRAS_N_2) return;
        } else {
            contador = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(MUESTRA_HALL_MS_2));
    }
}
static void rampa_de_velocidad(float frec_inicial, float frec_final, uint32_t duracion_ms) {
    if (duracion_ms == 0 || fabsf(frec_final - frec_inicial) < 1.0f) { fijar_frecuencia_pasos(frec_final); return; }
    uint32_t pasos = duracion_ms / 5U; if (!pasos) pasos = 1;
    for (uint32_t i = 0; i <= pasos; i++) {
        float t = (float)i / (float)pasos;
        float s = 0.5f * (1.0f - cosf((float)M_PI * t));
        fijar_frecuencia_pasos(frec_inicial + (frec_final - frec_inicial) * s);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* =====================================================================
 * FUNCIONES PRINCIPALES DE LÓGICA
 * ===================================================================== */
typedef enum { RESULTADO_SOBREPASO_OK, RESULTADO_SOBREPASO_ENDSTOP } resultado_sobrepaso_t;

static resultado_sobrepaso_t realizar_verificacion_sobrepaso(void) {
    fijar_frecuencia_pasos(FREQ_VERIFICACION_HZ_2);   // <-- antes FREQ_BUSQUEDA_HZ_2
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
            if (++contador_bajo >= NUMERO_MUESTRAS_N_2) {
                ESP_LOGW(TAG, "¡FINAL DE CARRERA DETECTADO! (Segundo imán encontrado).");
                return RESULTADO_SOBREPASO_ENDSTOP;
            }
        } else {
            contador_bajo = 0;
        }
        if (tiempo_transcurrido_ms >= TIEMPO_SOBREPASO_MS_2) {   // <-- corrige TTIEMPO_...
            ESP_LOGI(TAG, "Verificación OK. No se encontró un segundo imán.");
            return RESULTADO_SOBREPASO_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(MUESTRA_HALL_MS_2));
    }
}

static void gestionar_final_de_carrera(int *p_dir) {
    ESP_LOGW(TAG, "INICIANDO SECUENCIA DE ESCAPE DE ENDSTOP...");
    gpio_set_level(PIN_LED_D2, 1);
    *p_dir = (*p_dir == DERECHA_2) ? IZQUIERDA_2 : DERECHA_2;   // <-- IZQUIERDA_2
    fijar_direccion(*p_dir);
    fijar_frecuencia_pasos(FREQ_BUSQUEDA_HZ_H2);
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
static bool gestionar_arranque_sobre_iman(int *p_dir) {
    ESP_LOGW(TAG, "Arranque sobre imán. Iniciando chequeo bidireccional...");

    // --- CHEQUEO 1: HACIA LA DERECHA ---
    ESP_LOGI(TAG, "Chequeo Bidireccional (1/2): Probando hacia la DERECHA...");
    fijar_direccion(DERECHA_2);
    if (realizar_verificacion_sobrepaso() == RESULTADO_SOBREPASO_ENDSTOP) {
        ESP_LOGW(TAG, "ENDSTOP detectado a la derecha.");
        *p_dir = DERECHA_2;
        gestionar_final_de_carrera(p_dir);
        return false;
    }

    // --- Volver al imán e ir a IZQUIERDA_2 ---
    ESP_LOGI(TAG, "Chequeo a la DERECHA_2 OK. Volviendo al imán para probar hacia la IZQUIERDA...");
    fijar_direccion(IZQUIERDA_2);     // <-- IZQUIERDA_2
    arrancar_motor();
    esperar_hall_bajo_estable(); // Volver al imán de partida

    // --- CHEQUEO 2: HACIA LA IZQUIERDA ---
    ESP_LOGI(TAG, "Chequeo Bidireccional (2/2): Probando hacia la IZQUIERDA...");
    if (realizar_verificacion_sobrepaso() == RESULTADO_SOBREPASO_ENDSTOP) {
        ESP_LOGW(TAG, "ENDSTOP detectado a la izquierda.");
        *p_dir = IZQUIERDA_2;         // <-- IZQUIERDA_2
        gestionar_final_de_carrera(p_dir);
        return false;
    }

    // --- HOME confirmado ---
    ESP_LOGI(TAG, "Chequeo bidireccional completo. Confirmado: arranque en HOME. Posicionando...");
    fijar_direccion(DERECHA_2);
    arrancar_motor();
    esperar_hall_bajo_estable();

    parar_motor();
    habilitar_driver(false);
    gpio_set_level(PIN_LED_D2, 1);
    ESP_LOGI(TAG, "¡HOME confirmado y posicionado en arranque!");
    return true;
}

static bool verificar_si_es_home_o_final_carrera(int *p_dir) {
    rampa_de_velocidad(FREQ_BUSQUEDA_HZ_H2, FREQ_VERIFICACION_HZ_2, TIEMPO_RAMPA_MS_2);  // <-- FREQ_VERIFICACION_HZ_2
    
    if (realizar_verificacion_sobrepaso() == RESULTADO_SOBREPASO_ENDSTOP) {
        gestionar_final_de_carrera(p_dir);
        return false;
    }

    ESP_LOGI(TAG, "Acción: HOME encontrado. Volviendo para posicionamiento final.");
    *p_dir = (*p_dir == DERECHA_2) ? IZQUIERDA_2 : DERECHA_2;  // <-- IZQUIERDA_2
    fijar_direccion(*p_dir);
    fijar_frecuencia_pasos(FREQ_VERIFICACION_HZ_2);           // <-- FREQ_VERIFICACION_HZ_2
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
    int direccion_actual = DERECHA_2;
    fijar_direccion(direccion_actual);

    if (leer_sensor_hall()) {
        if (gestionar_arranque_sobre_iman(&direccion_actual)) {
            return;
        }
    } else {
        ESP_LOGI(TAG, "Arranque normal. Iniciando búsqueda de HOME...");
    }

    ESP_LOGI(TAG, "Motor en marcha a %d Hz...", (int)FREQ_BUSQUEDA_HZ_H2);
    fijar_frecuencia_pasos(FREQ_BUSQUEDA_HZ_H2);
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
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << PIN_DIR_2) | (1ULL << PIN_EN_2) | (1ULL << PIN_LED_D2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out_conf);

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_HALL_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in_conf);

    gpio_set_level(PIN_DIR_2, DERECHA_2);  // <-- corrige PIN_DIR -> PIN_DIR_2
    gpio_set_level(PIN_EN_2, 1);           // <-- corrige PIN_EN  -> PIN_EN_2
    gpio_set_level(PIN_LED_D2, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

void homing_eje1(void){
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

void homing_eje2(void){
    ESP_LOGI(TAG, "Inicializando sistema de Homing (con chequeo bidireccional)...");
    inicializar_gpios();
    inicializar_pwm(FREQ_BUSQUEDA_HZ_H2);

    habilitar_driver(true);
    vTaskDelay(pdMS_TO_TICKS(20));

    tarea_de_busqueda_home();

    ESP_LOGI(TAG, "app_main: Proceso completado.");
}

// ---------- APP PRINCIPAL ----------
void app_main(void){
    homing_eje1();
    vTaskDelay(pdMS_TO_TICKS(50));
    homing_eje2();
}
