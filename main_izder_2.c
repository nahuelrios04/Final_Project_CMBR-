// ESP32 SPI Slave + stepper por LEDC (jog continuo y nudge por pasos) + AUTOTEST
// Pines: STEP=18 (LEDC), DIR=17, EN=16 (¡ver ACTIVO/INACTIVO abajo!)
// SPI: MOSI=13, MISO=19, SCLK=14, CS=15, MODE=1, 12 bytes (cmd,arg1,arg2)

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

// ---------- Config ----------
#define TAG              "SPI_STEP"
#define BUS_HOST         SPI2_HOST
#define SPI_MODE         1
#define TRANSFER_SIZE    12

// SPI pins
#define GPIO_MOSI        13
#define GPIO_MISO        12
#define GPIO_SCLK        14
#define GPIO_CS          15

// Stepper pins
#define PIN_STEP         18
#define PIN_DIR          17
#define PIN_EN           16

// ==== IMPORTANTE: polaridad de ENABLE ====
// Si tu driver se habilita con nivel BAJO (típico TB6560): dejar en 1.
// Si tu driver se habilita con nivel ALTO: poner 0.
#define EN_ACTIVO_BAJO   1

// Movimiento
#define STEPS_PER_MOVE   (51200/4)     // 12800
#define MOVE_FREQ_HZ     3000.0f       // Hz del tren de pasos (ajustable)
#define DUTY_10B_50PCT   512           // 10bit ~50%

// LEDC
#define LEDC_MODE_X      LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_X     LEDC_TIMER_0
#define LEDC_CH_X        LEDC_CHANNEL_0
#define LEDC_RES_X       LEDC_TIMER_10_BIT

// Direcciones
#define DIR_DERECHA      0
#define DIR_IZQUIERDA    1

// Buffers SPI
DMA_ATTR static uint8_t txbuf[TRANSFER_SIZE] __attribute__((aligned(4)));
DMA_ATTR static uint8_t rxbuf[TRANSFER_SIZE] __attribute__((aligned(4)));

// ===== GPIO/LEDC utils =====
static inline void driver_enable(int on) {
    // on=1 => habilitar; on=0 => deshabilitar
    if (EN_ACTIVO_BAJO) {
        gpio_set_level(PIN_EN, on ? 0 : 1);
    } else {
        gpio_set_level(PIN_EN, on ? 1 : 0);
    }
}
static inline void set_dir(int d)        { gpio_set_level(PIN_DIR, d ? 1 : 0); }
static inline void dir_delay_us(void)    { esp_rom_delay_us(20); }

static void ledc_init_fixed(float start_hz) {
    ledc_timer_config_t t = {
        .speed_mode       = LEDC_MODE_X,
        .duty_resolution  = LEDC_RES_X,
        .timer_num        = LEDC_TIMER_X,
        .freq_hz          = (uint32_t)start_hz,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK( ledc_timer_config(&t) );
    ledc_channel_config_t c = {
        .gpio_num   = PIN_STEP,
        .speed_mode = LEDC_MODE_X,
        .channel    = LEDC_CH_X,
        .timer_sel  = LEDC_TIMER_X,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK( ledc_channel_config(&c) );
}

static inline void step_set_freq(float hz) {
    if (hz < 1.0f) hz = 1.0f;
    ledc_set_freq(LEDC_MODE_X, LEDC_TIMER_X, (uint32_t)hz);
}
static inline void step_run_on(void)  { ledc_set_duty(LEDC_MODE_X, LEDC_CH_X, DUTY_10B_50PCT); ledc_update_duty(LEDC_MODE_X, LEDC_CH_X); }
static inline void step_run_off(void) { ledc_set_duty(LEDC_MODE_X, LEDC_CH_X, 0);              ledc_update_duty(LEDC_MODE_X, LEDC_CH_X); }

// N pasos aproximando por tiempo (N / freq). Útil para NUDGE.
static void move_steps_blocking(int dir, uint32_t steps, float hz) {
    driver_enable(1);
    set_dir(dir); dir_delay_us();
    step_set_freq(hz);
    step_run_on();
    double seconds = (double)steps / (double)hz;
    int64_t usec   = (int64_t)(seconds * 1e6 + 0.5);
    esp_rom_delay_us((uint32_t)(usec>0 ? usec : 0));
    step_run_off();
}

// ===== JOG continuo =====
static volatile bool g_jog_running = false;
static volatile int  g_jog_dir     = DIR_DERECHA;

static inline void jog_start(int dir) {
    g_jog_dir = (dir == DIR_IZQUIERDA) ? DIR_IZQUIERDA : DIR_DERECHA;
    driver_enable(1);
    set_dir(g_jog_dir); dir_delay_us();
    step_set_freq(MOVE_FREQ_HZ);
    step_run_on();
    g_jog_running = true;
    ESP_LOGI(TAG, "JOG START dir=%s @ %.1f Hz", g_jog_dir==DIR_DERECHA?"DER":"IZQ", (double)MOVE_FREQ_HZ);
}
static inline void jog_stop(void) {
    step_run_off();
    // driver_enable(0); // si querés liberar torque al soltar, destapá esto
    g_jog_running = false;
    ESP_LOGI(TAG, "JOG STOP");
}

// ===== SPI callbacks/task =====
static void IRAM_ATTR post_setup_cb(spi_slave_transaction_t *t) {
    const int32_t v1 = 23, v2 = 12, v3 = 25;
    memcpy(&txbuf[0], &v1, 4);
    memcpy(&txbuf[4], &v2, 4);
    memcpy(&txbuf[8], &v3, 4);
}
static inline void unpack3_le(const uint8_t *b, int32_t *a, int32_t *b1, int32_t *b2) {
    memcpy(a,  &b[0], 4);
    memcpy(b1, &b[4], 4);
    memcpy(b2, &b[8], 4);
}

static void spi_task(void *arg) {
    while (1) {
        spi_slave_transaction_t t = {0};
        t.length = TRANSFER_SIZE * 8;
        t.tx_buffer = txbuf;
        t.rx_buffer = rxbuf;
        if (spi_slave_transmit(BUS_HOST, &t, portMAX_DELAY) != ESP_OK) continue;

        int32_t cmd=0, a1=0, a2=0;
        unpack3_le(rxbuf, &cmd, &a1, &a2);
        ESP_LOGI(TAG, "RX cmd=%" PRId32 ", a1=%" PRId32 ", a2=%" PRId32, cmd, a1, a2);

        if (cmd == 210) {                 // START_JOG
            if (a1 == 3) jog_start(DIR_IZQUIERDA);
            else if (a1 == 4) jog_start(DIR_DERECHA);
        } else if (cmd == 211) {          // STOP_JOG
            jog_stop();
        } else if (cmd == 200) {          // NUDGE por pasos
            if (a1 == 3) move_steps_blocking(DIR_IZQUIERDA, STEPS_PER_MOVE, MOVE_FREQ_HZ);
            else if (a1 == 4) move_steps_blocking(DIR_DERECHA,  STEPS_PER_MOVE, MOVE_FREQ_HZ);
        }
    }
}

// ===== Init =====
static void io_init(void) {
    gpio_config_t out = {
        .pin_bit_mask = (1ULL<<PIN_DIR) | (1ULL<<PIN_EN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK( gpio_config(&out) );

    // Estados iniciales seguros: driver deshabilitado, dir por defecto
    driver_enable(0);
    gpio_set_level(PIN_DIR, 1);

    // MISO fuerte p/ >3 MHz
    gpio_set_drive_capability(GPIO_NUM_19, GPIO_DRIVE_CAP_3);
}

// ==== AUTOTEST AL INICIO ====
// Mueve 1 s a derecha y 1 s a izquierda a MOVE_FREQ_HZ.
// Si esto NO mueve, el problema es pinout/EN/frecuencia, no SPI.
static void autotest_run(void) {
    ESP_LOGW(TAG, "AUTOTEST: start");
    driver_enable(1);
    set_dir(DIR_DERECHA); dir_delay_us();
    step_set_freq(MOVE_FREQ_HZ);
    step_run_on();
    vTaskDelay(pdMS_TO_TICKS(1000));
    step_run_off();

    vTaskDelay(pdMS_TO_TICKS(200));

    set_dir(DIR_IZQUIERDA); dir_delay_us();
    step_run_on();
    vTaskDelay(pdMS_TO_TICKS(1000));
    step_run_off();

    driver_enable(0);
    ESP_LOGW(TAG, "AUTOTEST: end");
}

void app_main(void) {
    ESP_LOGI(TAG, "Init… EN_ACTIVO_BAJO=%d", EN_ACTIVO_BAJO);
    io_init();
    ledc_init_fixed(MOVE_FREQ_HZ);

    // --- AUTOTEST ---
    autotest_run();

    // --- SPI slave ---
    spi_bus_config_t buscfg = {
        .mosi_io_num     = GPIO_MOSI,
        .miso_io_num     = GPIO_MISO,
        .sclk_io_num     = GPIO_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = TRANSFER_SIZE
    };
    spi_slave_interface_config_t slvcfg = {
        .mode          = SPI_MODE,
        .spics_io_num  = GPIO_CS,
        .queue_size    = 2,
        .flags         = 0,
        .post_setup_cb = post_setup_cb,
        .post_trans_cb = NULL
    };
    ESP_ERROR_CHECK( spi_slave_initialize(BUS_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO) );
    ESP_LOGI(TAG, "SPI Slave listo (MODE %d). Esperando frames de %d bytes…", SPI_MODE, TRANSFER_SIZE);

    xTaskCreatePinnedToCore(spi_task, "spi_task", 4096, NULL, 10, NULL, tskNO_AFFINITY);
}
