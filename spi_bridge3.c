#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>
#include <errno.h>
#include <stdint.h>

// Configuración de Rutas y SPI
#define SPI_DEVICE "/dev/spidev0.0"
#define FIFO_TX_PATH "/tmp/spi_tx" // Python escribe, C lee
#define FIFO_RX_PATH "/tmp/spi_rx" // C escribe, Python lee

// Configuración SPI
static uint8_t mode = 1; // SPI_MODE_1
static uint8_t bits = 8;
static uint32_t speed = 1000000; // 1 MHz
static uint16_t delay = 0;

int spi_fd = -1;

void init_spi() {
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("[Error] No se pudo abrir SPI");
        exit(1);
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) perror("[Error] Set SPI mode");
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) perror("[Error] Set bits");
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) perror("[Error] Set speed");
    
    printf("[Bridge] SPI inicializado: %s (%d Hz)\n", SPI_DEVICE, speed);
}

void transfer_spi(int cmd, int arg1, int arg2) {
    uint8_t tx_buf[12];
    uint8_t rx_buf[12];
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));

    memcpy(tx_buf, &cmd, 4);
    memcpy(tx_buf + 4, &arg1, 4);
    memcpy(tx_buf + 8, &arg2, 4);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = 12,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("[Error] Fallo SPI");
        return;
    }

    // RESPUESTA
    rx_buf[11] = 0; 
    char response_str[32];
    snprintf(response_str, sizeof(response_str), "%s\n", (char*)rx_buf);
    
    // --- LOG VISUAL DE RESPUESTA ---
    // Eliminamos el salto de linea extra para que se vea lindo en consola
    char debug_resp[32]; strcpy(debug_resp, response_str);
    debug_resp[strcspn(debug_resp, "\n")] = 0; 
    printf("   └── [ESP32 -> Bridge] Status: %s\n", debug_resp);
    // -------------------------------

    int fd_rx = open(FIFO_RX_PATH, O_WRONLY | O_NONBLOCK);
    if (fd_rx != -1) {
        write(fd_rx, response_str, strlen(response_str));
        close(fd_rx);
    }
}

int main() {
    // Vacuna para que no se cierre si Python se va
    signal(SIGPIPE, SIG_IGN);

    umask(0);
    mkfifo(FIFO_TX_PATH, 0666);
    mkfifo(FIFO_RX_PATH, 0666);

    init_spi();

    printf("[Bridge] Esperando comandos de Python...\n");

    char buffer[128];
    int fd_tx;

    while (1) {
        fd_tx = open(FIFO_TX_PATH, O_RDONLY);
        if (fd_tx < 0) {
            perror("[Error] Abriendo FIFO TX");
            sleep(1);
            continue;
        }

        printf("[Bridge] Conectado a Python.\n");

        while (1) {
            ssize_t bytes_read = read(fd_tx, buffer, sizeof(buffer) - 1);
            
            if (bytes_read > 0) {
                buffer[bytes_read] = 0; 
                int cmd, a1, a2;
                if (sscanf(buffer, "I %d %d %d", &cmd, &a1, &a2) == 3) {
                    
                    // --- LOG VISUAL DE ENTRADA ---
                    if (cmd != 0) { // No mostramos el ping (0) para no ensuciar
                        printf("[Python -> Bridge] CMD: %d | A1: %d | A2: %d\n", cmd, a1, a2);
                    }
                    // -----------------------------

                    transfer_spi(cmd, a1, a2);
                }
            } 
            else {
                printf("[Bridge] Python desconectado. Esperando...\n");
                close(fd_tx);
                break; 
            }
        }
    }

    close(spi_fd);
    return 0;
}
