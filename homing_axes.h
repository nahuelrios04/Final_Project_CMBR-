#ifndef HOMING_AXES_H_
#define HOMING_AXES_H_

#ifdef __cplusplus
extern "C" {
#endif

// Incluimos los headers del sistema que se usarán en los defines
#include "driver/ledc.h"

/* ============================================================
 * EJE 1 (TB6560 STYLE)
 * ============================================================ */

// --- Pines eje1 ---
#define PIN_TB6560_STEP   18
#define PIN_TB6560_DIR    17
#define PIN_TB6560_EN     16
#define PIN_HALL_1        26    // activo-bajo (pull-up interna)

// --- Motor eje1 ---
#define MOTOR_STEPS_PER_REV_1   200U
#define MICROSTEP_1             8U
#define HALF_TURN_STEPS_1   ((MOTOR_STEPS_PER_REV_1 * MICROSTEP_1) / 2U)

// --- Frecuencias eje1 (Hz) ---
#define FREQ_MIN_HZ_1       400.0f
#define FREQ_PASS1_HZ_1     900.0f
#define FREQ_PASS2_HZ_1     450.0f
#define FREQ_ALIGN_HZ_1     250.0f
#define FREQ_RELEASE_HZ_1   500.0f
#define FREQ_HALF_TURN_1    800.0f

// --- Tiempos eje1 (ms) ---
#define RAMP_MS_1           1200U
#define OVERTRAVEL_MS_1       15U
#define SEPARATE_MS_1         60U
#define HALL_SAMPLE_MS_1      1U
#define HALL_DEBOUNCE_N_1     3

// --- LEDC eje1 ---
#define LEDC_MODE_1         LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_1_ID     LEDC_TIMER_0      // Timer 0 para eje1
#define LEDC_CH_1           LEDC_CHANNEL_0    // Canal 0 para eje1
#define LEDC_RES_1          LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_1      (1 << 6)          // ~50%
#define LEDC_DUTY_OFF_1     0


/* ============================================================
 * EJE 2
 * ============================================================ */

// --- Pines eje2 ---
#define PIN_STEP_2    21
#define PIN_DIR_2     22
#define PIN_EN_2      23
#define PIN_HALL_2    25
#define PIN_LED_2     2     // LED indicador eje2

// --- Direcciones eje2 ---
#define DERECHA_2     1
#define IZQUIERDA_2   0

// --- LEDC eje2 ---
#define LEDC_MODE_2         LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_2_ID     LEDC_TIMER_1      // Timer 1 para eje2
#define LEDC_CH_2           LEDC_CHANNEL_1    // Canal 1 para eje2
#define LEDC_RES_2          LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_2      (1 << 6)
#define LEDC_DUTY_OFF_2     0

// --- Movimiento eje2 ---
#define FREQ_BUSQUEDA_HZ_2        1000.0f
#define FREQ_VERIFICACION_HZ_2    400.0f
#define TIEMPO_RAMPA_MS_2         120U
#define TIEMPO_SOBREPASO_MS_2     4000U

// --- Hall eje2 ---
#define HALL_SAMPLE_MS_2          1U
#define HALL_DEBOUNCE_N_2         2


/* ============================================================
 * EJE 3
 * ============================================================ */

// --- Pines eje3 ---
#define PIN_STEP_3    4
#define PIN_DIR_3     5
#define PIN_EN_3      15
#define PIN_HALL_3    24
#define PIN_LED_3     27    // LED indicador eje3

// --- Direcciones eje3 ---
#define DERECHA_3     1
#define IZQUIERDA_3   0

// --- LEDC eje3 ---
#define LEDC_MODE_3         LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_3_ID     LEDC_TIMER_2      // Timer 2 para eje3
#define LEDC_CH_3           LEDC_CHANNEL_2    // Canal 2 para eje3
#define LEDC_RES_3          LEDC_TIMER_10_BIT
#define LEDC_DUTY_ON_3      (1 << 6)
#define LEDC_DUTY_OFF_3     0

// --- Movimiento eje3 ---
#define FREQ_BUSQUEDA_HZ_3        1000.0f
#define FREQ_VERIFICACION_HZ_3    400.0f
#define TIEMPO_RAMPA_MS_3         120U
#define TIEMPO_SOBREPASO_MS_3     4000U

// --- Hall eje3 ---
#define HALL_SAMPLE_MS_3          1U
#define HALL_DEBOUNCE_N_3         2

/* ============================================================
 * DEFINICIONES Y PROTOTIPOS PÚBLICOS
 * ============================================================ */

typedef enum {
    SOBREPASO_OK = 0,
    SOBREPASO_ENDSTOP = 1
} sobrepaso_result_t;


/**
 * @brief Ejecuta la secuencia de homing completa para el Eje 1.
 */
void homing_eje1(void);

/**
 * @brief Ejecuta la secuencia de homing completa para el Eje 2.
 */
void homing_eje2(void);

/**
 * @brief Ejecuta la secuencia de homing completa para el Eje 3.
 */
void homing_eje3(void);


#ifdef __cplusplus
}
#endif

#endif // HOMING_AXES_H_
