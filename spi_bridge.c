// spi_bridge.c — Puente SPI: lee /tmp/spi_tx y envía 3 int32 (12 bytes) por SPI a la ESP32.
// Formato en FIFO:  I <a> <b> <c>   p.ej: "I 210 4 0" (START_JOG derecha), "I 211 0 0" (STOP_JOG)
// Compilar:  gcc spi_bridge.c -o spi_bridge
// Ejecutar:  ./spi_bridge   (dejarlo corriendo)

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE     "/dev/spidev0.0"
#define SPI_MODE_SET   1            // CPOL=0, CPHA=1
#define SPI_SPEED_HZ   8000000      // 8 MHz (ajustable)
#define TRANSFER_SIZE  12           // 3 * int32
#define FIFO_PATH      "/tmp/spi_tx"

static int spi_fd = -1;

static int spi_init(void) {
    uint8_t mode = SPI_MODE_SET, rdmode = 0, bits = 8;
    uint32_t speed = SPI_SPEED_HZ;

    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) { perror("open SPI"); return -1; }

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode)  == -1) { perror("WR_MODE");  return -1; }
    if (ioctl(spi_fd, SPI_IOC_RD_MODE, &rdmode)== -1) { perror("RD_MODE");  return -1; }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) { perror("WR_BITS"); return -1; }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) { perror("WR_SPEED"); return -1; }

    printf("SPI listo: %s, mode=%u/%u, bits=%u, speed=%u\n",
           SPI_DEVICE, mode, rdmode, bits, speed);
    return 0;
}

static int spi_xfer(const uint8_t* tx, uint8_t* rx, size_t len, uint32_t speed_hz) {
    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)(uintptr_t)tx;
    tr.rx_buf = (unsigned long)(uintptr_t)rx;
    tr.len = len;
    tr.speed_hz = speed_hz;
    tr.bits_per_word = 8;
    tr.cs_change = 0;
    tr.delay_usecs = 2;
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) perror("SPI_IOC_MESSAGE");
    return ret;
}

static int ensure_fifo(void) {
    struct stat st;
    if (stat(FIFO_PATH, &st) == 0) {
        if (!S_ISFIFO(st.st_mode)) {
            fprintf(stderr, "Existe %s pero no es FIFO\n", FIFO_PATH);
            return -1;
        }
        return 0;
    }
    if (mkfifo(FIFO_PATH, 0666) < 0) { perror("mkfifo"); return -1; }
    return 0;
}

static void send_three_ints(int32_t a, int32_t b, int32_t c) {
    uint8_t tx[TRANSFER_SIZE] = {0};
    uint8_t rx[TRANSFER_SIZE] = {0};
    memcpy(&tx[0], &a, 4);
    memcpy(&tx[4], &b, 4);
    memcpy(&tx[8], &c, 4);

    if (spi_xfer(tx, rx, TRANSFER_SIZE, SPI_SPEED_HZ) < 0) {
        fprintf(stderr, "[SPI] fallo TX %d,%d,%d\n", a, b, c);
    } else {
        int32_t r1=0, r2=0, r3=0;
        memcpy(&r1, &rx[0], 4);
        memcpy(&r2, &rx[4], 4);
        memcpy(&r3, &rx[8], 4);
        printf("[SPI] TX: %d,%d,%d  |  RX: %d,%d,%d\n", a,b,c, r1,r2,r3);
        fflush(stdout);
    }
}

int main(void) {
    if (spi_init() != 0) return 1;
    if (ensure_fifo() != 0) return 2;

    // Dummy de alineación
    uint8_t dz_tx[TRANSFER_SIZE] = {0}, dz_rx[TRANSFER_SIZE] = {0};
    usleep(100000);
    spi_xfer(dz_tx, dz_rx, TRANSFER_SIZE, SPI_SPEED_HZ);

    while (1) {
        int fifo_fd = open(FIFO_PATH, O_RDONLY);
        if (fifo_fd < 0) { perror("open FIFO"); usleep(200000); continue; }
        FILE* f = fdopen(fifo_fd, "r");
        if (!f) { perror("fdopen"); close(fifo_fd); usleep(200000); continue; }

        char line[256];
        while (fgets(line, sizeof(line), f)) {
            char tag; int32_t a=0, b=0, c=0;
            if (sscanf(line, " %c %d %d %d", &tag, &a, &b, &c) == 4 && (tag=='I'||tag=='i')) {
                send_three_ints(a,b,c);
            } else if (sscanf(line, " %c , %d , %d , %d", &tag, &a, &b, &c) == 4 && (tag=='I'||tag=='i')) {
                send_three_ints(a,b,c);
            } else {
                fprintf(stderr, "[FIFO] ignorada: %s", line);
            }
        }
        fclose(f);
    }
    close(spi_fd);
    return 0;
}
