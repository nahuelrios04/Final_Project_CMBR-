// spi_bridge.c (Versión 2: Bidireccional)
// Compilar: gcc spi_bridge.c -o spi_bridge.out

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <linux/spi/spidev.h>
#include <pthread.h> // Necesario para hilos

#define SPI_DEVICE    "/dev/spidev0.0"
#define SPI_MODE_SET  1
#define SPI_SPEED_HZ  8000000
#define TRANSFER_SIZE 12 // 3 * int32_t (aunque ahora usamos 16)
#define BUFFER_SIZE   16 // Coincidir con el ESP32

#define FIFO_TX_PATH  "/tmp/spi_tx" // Python escribe aquí (TX -> ESP32)
#define FIFO_RX_PATH  "/tmp/spi_rx" // Python lee aquí (RX <- ESP32)

static int spi_fd = -1;
static int fifo_rx_fd = -1;
static pthread_mutex_t spi_lock;

// Asegura que ambos FIFOs existan
static int ensure_fifos(void) {
    if (mkfifo(FIFO_TX_PATH, 0666) < 0 && errno != EEXIST) {
        perror("mkfifo tx"); return -1;
    }
    if (mkfifo(FIFO_RX_PATH, 0666) < 0 && errno != EEXIST) {
        perror("mkfifo rx"); return -1;
    }
    return 0;
}

static int spi_init(void) {
    uint8_t mode = SPI_MODE_SET, bits = 8;
    uint32_t speed = SPI_SPEED_HZ;

    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) { perror("open SPI"); return -1; }
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1) return -1;
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) return -1;
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) return -1;
    
    printf("SPI listo: %s, mode=%u, speed=%u\n", SPI_DEVICE, mode, speed);
    return 0;
}

// Función de transferencia SPI (protegida por mutex)
static int spi_xfer(uint8_t* tx, uint8_t* rx, size_t len) {
    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = 8;
    
    pthread_mutex_lock(&spi_lock);
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    pthread_mutex_unlock(&spi_lock);
    
    if (ret < 0) perror("SPI_IOC_MESSAGE");
    return ret;
}

// Hilo para escribir respuestas al FIFO_RX
void* rx_writer_thread(void* arg) {
    uint8_t rx_buffer[BUFFER_SIZE];
    memcpy(rx_buffer, (uint8_t*)arg, BUFFER_SIZE);
    
    // Abre el FIFO de respuesta (puede bloquear si Python no está escuchando)
    int fd = open(FIFO_RX_PATH, O_WRONLY);
    if (fd < 0) {
        perror("[Writer] Abrir FIFO_RX");
        return NULL;
    }
    
    // Escribe la respuesta (como string) y una nueva línea
    rx_buffer[BUFFER_SIZE - 1] = '\0'; // Asegurar NUL termination
    dprintf(fd, "%s\n", (char*)rx_buffer);
    close(fd);
    
    return NULL;
}

int main(void) {
    if (spi_init() != 0) return 1;
    if (ensure_fifos() != 0) return 2;
    pthread_mutex_init(&spi_lock, NULL);

    printf("Bridge Bidireccional listo. Esperando comandos en %s...\n", FIFO_TX_PATH);

    while (1) {
        int fifo_fd = open(FIFO_TX_PATH, O_RDONLY);
        if (fifo_fd < 0) { perror("open FIFO_TX"); usleep(200000); continue; }
        
        FILE* f = fdopen(fifo_fd, "r");
        if (!f) { perror("fdopen"); close(fifo_fd); continue; }

        char line[256];
        while (fgets(line, sizeof(line), f)) {
            char tag;
            int32_t a=0, b=0, c=0;
            
            // 1. Leer comando de Python
            if (sscanf(line, " %c %d %d %d", &tag, &a, &b, &c) != 4) {
                 fprintf(stderr, "[Bridge] Ignorando línea malformada: %s", line);
                 continue;
            }

            // 2. Preparar búferes SPI
            uint8_t tx_buf[BUFFER_SIZE] = {0};
            uint8_t rx_buf[BUFFER_SIZE] = {0};
            memcpy(&tx_buf[0], &a, 4);
            memcpy(&tx_buf[4], &b, 4);
            memcpy(&tx_buf[8], &c, 4);

            // 3. Ejecutar transacción SPI
            if (spi_xfer(tx_buf, rx_buf, BUFFER_SIZE) < 0) {
                 fprintf(stderr, "[Bridge] Fallo en spi_xfer\n");
                 continue;
            }

            // 4. Interpretar y mostrar en consola
            // (¡Ahora leemos la respuesta como string!)
            rx_buf[BUFFER_SIZE - 1] = '\0'; // Asegurar fin de string
            printf("[SPI] TX: %d,%d,%d | RX: \"%s\"\n", a, b, c, (char*)rx_buf);
            fflush(stdout);

            // 5. Enviar la respuesta a la interfaz Python
            // (Lanzamos un hilo corto para no bloquearnos si Python no lee)
            pthread_t tid;
            pthread_create(&tid, NULL, rx_writer_thread, (void*)rx_buf);
            pthread_detach(tid);
        }
        fclose(f); // Cierra el FILE y el fifo_fd
    }

    close(spi_fd);
    pthread_mutex_destroy(&spi_lock);
    return 0;
}
