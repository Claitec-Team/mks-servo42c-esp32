#include "console_serial.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "servo_cmd.h"

static const char *TAG = "console";

/* UART0 is the USB console. The servo is on SERVO_UART_PORT, so there is no
 * contention; we install a driver here only to get a blocking read. */
#define CONSOLE_UART      UART_NUM_0
#define CONSOLE_RX_BUF    256
#define CONSOLE_LINE_MAX  128
#define CONSOLE_STACK     4096
#define CONSOLE_PRIO      5

#define PROMPT "servo> "

static void console_write(void *ctx, const char *text)
{
    (void)ctx;
    uart_write_bytes(CONSOLE_UART, text, strlen(text));
}

static const cmd_sink_t g_sink = { .write = console_write, .ctx = NULL };

static void echo(const char *text)
{
    uart_write_bytes(CONSOLE_UART, text, strlen(text));
}

static void console_task(void *arg)
{
    (void)arg;
    char line[CONSOLE_LINE_MAX];
    size_t len = 0;

    echo("\r\n");
    echo("MKS SERVO42C control console. Type 'help'.\r\n");
    echo(PROMPT);

    for (;;) {
        uint8_t ch = 0;
        int n = uart_read_bytes(CONSOLE_UART, &ch, 1, pdMS_TO_TICKS(200));
        if (n != 1) {
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            echo("\r\n");
            if (len > 0) {
                line[len] = '\0';
                servo_cmd_execute_line(line, &g_sink);
                len = 0;
            }
            echo(PROMPT);
        } else if (ch == 0x08 || ch == 0x7F) {          /* backspace / delete */
            if (len > 0) {
                len--;
                echo("\b \b");
            }
        } else if (ch == 0x03) {                        /* ctrl-C: abandon line */
            len = 0;
            echo("^C\r\n");
            echo(PROMPT);
        } else if (ch >= 0x20 && ch < 0x7F) {
            if (len + 1 < sizeof(line)) {
                line[len++] = (char)ch;
                uart_write_bytes(CONSOLE_UART, &ch, 1);   /* echo */
            }
        }
        /* anything else (escape sequences, etc.) is ignored */
    }
}

esp_err_t console_serial_start(void)
{
    /* Do not reconfigure baud rate or pins: the bootloader already set UART0
     * up for the USB bridge and stdout is still using it. */
    esp_err_t err = uart_driver_install(CONSOLE_UART, CONSOLE_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t ok = xTaskCreate(console_task, "console", CONSOLE_STACK, NULL,
                                CONSOLE_PRIO, NULL);
    if (ok != pdPASS) {
        uart_driver_delete(CONSOLE_UART);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
