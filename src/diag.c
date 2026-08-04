#include "diag.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "servo_config.h"

static const char *TAG = "diag";

#define DIAG_UART    SERVO_UART_PORT
#define DIAG_RX_CAP  32

/* Every baud rate the servo's UartBaud menu can be set to, plus the factory
 * default (38400) which is what it still uses if a menu change did not stick. */
static const int kBauds[] = { 9600, 19200, 25000, 38400, 57600, 115200 };
#define N_BAUDS (sizeof(kBauds) / sizeof(kBauds[0]))

static void hexdump(const char *prefix, const uint8_t *bytes, size_t n)
{
    char line[DIAG_RX_CAP * 3 + 1];
    size_t off = 0;
    for (size_t i = 0; i < n && off + 4 < sizeof(line); i++) {
        off += (size_t)snprintf(line + off, sizeof(line) - off, "%02x ", bytes[i]);
    }
    line[off] = '\0';
    ESP_LOGI(TAG, "%s%s", prefix, line);
}

/* Sends a read-encoder request and returns however many bytes come back. */
static int probe(int baud, uint8_t addr, bool with_checksum,
                 uint8_t *rx, size_t rx_cap)
{
    uart_set_baudrate(DIAG_UART, baud);
    uart_flush_input(DIAG_UART);

    uint8_t frame[3];
    size_t n = 0;
    frame[n++] = addr;
    frame[n++] = 0x30;                                   /* read encoder */
    if (with_checksum) {
        frame[n++] = (uint8_t)((addr + 0x30) & 0xFF);
    }

    uart_write_bytes(DIAG_UART, frame, n);
    uart_wait_tx_done(DIAG_UART, pdMS_TO_TICKS(100));

    return uart_read_bytes(DIAG_UART, rx, rx_cap, pdMS_TO_TICKS(250));
}

/* Phase 1: say nothing, just watch RX. Any byte at all proves the servo's Tx
 * line reaches GPIO SERVO_RX_GPIO. */
static void phase_listen(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "== phase 1: passive listen on GPIO%d for 3 s ==", SERVO_RX_GPIO);
    ESP_LOGI(TAG, "press the servo's screen buttons now; some firmware chatters on boot");

    uart_set_baudrate(DIAG_UART, SERVO_BAUD_RATE);
    uart_flush_input(DIAG_UART);

    uint8_t rx[DIAG_RX_CAP];
    int total = 0;
    for (int i = 0; i < 12; i++) {
        int n = uart_read_bytes(DIAG_UART, rx, sizeof(rx), pdMS_TO_TICKS(250));
        if (n > 0) {
            total += n;
            hexdump("  rx: ", rx, (size_t)n);
        }
    }
    if (total == 0) {
        ESP_LOGW(TAG, "  nothing received (expected: the servo only talks when asked)");
    } else {
        ESP_LOGI(TAG, "  %d bytes seen unprompted", total);
    }
}

/* Phase 2: sweep baud rates at the configured address, with and without the
 * trailing checksum byte. V1.0 firmware expects no checksum. */
static void phase_baud_sweep(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "== phase 2: baud sweep at addr 0x%02X ==", SERVO_ADDRESS);

    uint8_t rx[DIAG_RX_CAP];
    int hits = 0;

    for (size_t i = 0; i < N_BAUDS; i++) {
        for (int variant = 0; variant < 2; variant++) {
            bool with_chk = (variant == 0);
            int n = probe(kBauds[i], SERVO_ADDRESS, with_chk, rx, sizeof(rx));
            const char *framing = with_chk ? "e0 30 10   " : "e0 30 (no chk)";
            if (n > 0) {
                hits++;
                char prefix[64];
                snprintf(prefix, sizeof(prefix), "  %6d %s -> %d bytes: ",
                         kBauds[i], framing, n);
                hexdump(prefix, rx, (size_t)n);
            } else {
                ESP_LOGI(TAG, "  %6d %s -> silence", kBauds[i], framing);
            }
        }
    }

    if (hits == 0) {
        ESP_LOGW(TAG, "  no reply at any baud rate: the servo is not hearing us,");
        ESP_LOGW(TAG, "  or its Tx is not reaching GPIO%d", SERVO_RX_GPIO);
    }
}

/* Phase 3: sweep the ten valid slave addresses at the configured baud rate. */
static void phase_address_sweep(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "== phase 3: address sweep at %d baud ==", SERVO_BAUD_RATE);

    uint8_t rx[DIAG_RX_CAP];
    int hits = 0;

    for (uint8_t addr = 0xE0; addr <= 0xE9; addr++) {
        int n = probe(SERVO_BAUD_RATE, addr, true, rx, sizeof(rx));
        if (n > 0) {
            hits++;
            char prefix[48];
            snprintf(prefix, sizeof(prefix), "  addr 0x%02X -> %d bytes: ", addr, n);
            hexdump(prefix, rx, (size_t)n);
        }
    }
    if (hits == 0) {
        ESP_LOGW(TAG, "  no address answered");
    }
}

/* Phase 4: proves the ESP32 half of the link. Needs a jumper wire from
 * SERVO_TX_GPIO to SERVO_RX_GPIO and the servo unplugged. */
static void phase_loopback(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "== phase 4: loopback on GPIO%d -> GPIO%d ==",
             SERVO_TX_GPIO, SERVO_RX_GPIO);

    static const uint8_t pattern[] = { 0xE0, 0x30, 0x10, 0x55, 0xAA };
    uart_set_baudrate(DIAG_UART, SERVO_BAUD_RATE);
    uart_flush_input(DIAG_UART);
    uart_write_bytes(DIAG_UART, pattern, sizeof(pattern));
    uart_wait_tx_done(DIAG_UART, pdMS_TO_TICKS(100));

    uint8_t rx[DIAG_RX_CAP];
    int n = uart_read_bytes(DIAG_UART, rx, sizeof(rx), pdMS_TO_TICKS(300));
    if (n == (int)sizeof(pattern) && memcmp(rx, pattern, sizeof(pattern)) == 0) {
        ESP_LOGI(TAG, "  PASS: UART%d and both pins work, so the fault is "
                      "off-board", DIAG_UART);
    } else if (n > 0) {
        hexdump("  PARTIAL, got: ", rx, (size_t)n);
        ESP_LOGW(TAG, "  bytes came back but corrupted: check the jumper and baud rate");
    } else {
        ESP_LOGW(TAG, "  FAIL: nothing looped back.");
        ESP_LOGW(TAG, "  With the jumper fitted this means GPIO%d/GPIO%d cannot be",
                 SERVO_TX_GPIO, SERVO_RX_GPIO);
        ESP_LOGW(TAG, "  used as a UART on this board (GPIO16/17 are wired to the");
        ESP_LOGW(TAG, "  PSRAM chip on ESP32-WROVER modules). Try other pins.");
        ESP_LOGW(TAG, "  Without the jumper this result is expected.");
    }
}

void diag_run(void)
{
    ESP_LOGI(TAG, "===== SERVO42C link diagnostics =====");
    ESP_LOGI(TAG, "UART%d  TX=GPIO%d -> servo Rx   RX=GPIO%d <- servo Tx",
             DIAG_UART, SERVO_TX_GPIO, SERVO_RX_GPIO);
    ESP_LOGI(TAG, "configured for %d baud, addr 0x%02X",
             SERVO_BAUD_RATE, SERVO_ADDRESS);

    const uart_config_t cfg = {
        .baud_rate  = SERVO_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(DIAG_UART, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(DIAG_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(DIAG_UART, SERVO_TX_GPIO, SERVO_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    phase_listen();
    phase_baud_sweep();
    phase_address_sweep();
    phase_loopback();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===== diagnostics done =====");
    ESP_LOGI(TAG, "A reply in phase 2 or 3 tells you the working baud/address:");
    ESP_LOGI(TAG, "put them in include/servo_config.h and rebuild the demo.");

    uart_driver_delete(DIAG_UART);
}
