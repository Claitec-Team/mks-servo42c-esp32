#include "console_tcp.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "servo_cmd.h"
#include "servo_ctl.h"

static const char *TAG = "tcp";

#define TCP_MAX_CLIENTS   4
#define TCP_LINE_MAX      128
#define TCP_RECV_CHUNK    64
#define TCP_CLIENT_STACK  4096
#define TCP_CLIENT_PRIO   4
#define TCP_LISTEN_STACK  3072
#define TCP_LISTEN_PRIO   4
#define TCP_SEND_TIMEOUT  2   /* seconds; keeps a stalled peer from blocking us */

/* Slots are never freed, only reused, so a sink can refer to one by index.
 * `generation` counts how many connections a slot has served, which is what
 * makes a stale reference detectable. */
typedef struct {
    int      sock;
    uint32_t generation;
    bool     in_use;
} tcp_client_t;

static tcp_client_t      g_clients[TCP_MAX_CLIENTS];
static SemaphoreHandle_t g_clients_mux;
static uint16_t          g_port;

/*
 * A command can still be queued when its client disconnects, and by the time
 * the servo task writes the reply the file descriptor may already belong to
 * somebody else - writing the reply there would leak it to the wrong client.
 *
 * So the sink does not carry a socket or a pointer. It carries the slot index
 * and the generation, packed into the ctx word, and the write callback drops
 * the text unless the slot still holds that same connection. Nothing has to
 * outlive the connection, which also rules out a dangling pointer.
 */
#define SLOT_BITS 4
_Static_assert(TCP_MAX_CLIENTS <= (1 << SLOT_BITS), "SLOT_BITS too small");

static void *pack_ref(int slot, uint32_t generation)
{
    return (void *)(uintptr_t)(((uintptr_t)generation << SLOT_BITS) |
                               (uintptr_t)slot);
}

static void tcp_write(void *ctx, const char *text)
{
    const uintptr_t packed = (uintptr_t)ctx;
    const int slot = (int)(packed & ((1u << SLOT_BITS) - 1));
    const uint32_t generation = (uint32_t)(packed >> SLOT_BITS);

    if (slot < 0 || slot >= TCP_MAX_CLIENTS) {
        return;
    }

    xSemaphoreTake(g_clients_mux, portMAX_DELAY);
    tcp_client_t *c = &g_clients[slot];
    int sock = (c->in_use && c->generation == generation) ? c->sock : -1;
    if (sock >= 0) {
        /* Held across send() so the socket cannot be closed under us. The send
         * timeout bounds how long that can last. */
        if (send(sock, text, strlen(text), 0) < 0) {
            ESP_LOGD(TAG, "send to slot %d failed: %d", slot, errno);
        }
    }
    xSemaphoreGive(g_clients_mux);
}

static void release_slot(int slot)
{
    xSemaphoreTake(g_clients_mux, portMAX_DELAY);
    int sock = g_clients[slot].sock;
    g_clients[slot].in_use = false;
    g_clients[slot].sock = -1;
    xSemaphoreGive(g_clients_mux);

    if (sock >= 0) {
        close(sock);
    }
}

static void client_task(void *arg)
{
    const int slot = (int)(intptr_t)arg;

    xSemaphoreTake(g_clients_mux, portMAX_DELAY);
    const int sock = g_clients[slot].sock;
    const uint32_t generation = g_clients[slot].generation;
    xSemaphoreGive(g_clients_mux);

    const cmd_sink_t sink = {
        .write  = tcp_write,
        .ctx    = pack_ref(slot, generation),
        .prompt = NULL,          /* no prompt over a socket */
    };

    ESP_LOGI(TAG, "client %d connected", slot);
    tcp_write(sink.ctx, "MKS SERVO42C. One command per line, 'help' for a list.\r\n");

    char line[TCP_LINE_MAX];
    size_t len = 0;

    for (;;) {
        char chunk[TCP_RECV_CHUNK];
        int n = recv(sock, chunk, sizeof(chunk), 0);
        if (n == 0) {
            break;                                  /* orderly close */
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }

        for (int i = 0; i < n; i++) {
            const char ch = chunk[i];
            if (ch == '\n' || ch == '\r') {
                if (len > 0) {
                    line[len] = '\0';
                    servo_ctl_submit(line, &sink);
                    len = 0;
                }
            } else if (ch >= 0x20 && ch < 0x7F) {
                if (len + 1 < sizeof(line)) {
                    line[len++] = ch;
                }
                /* Longer than the buffer: keep reading, drop the excess. The
                 * command will fail to parse, which is the honest outcome. */
            }
            /* Everything else - telnet negotiation, control codes - ignored. */
        }
    }

    ESP_LOGI(TAG, "client %d disconnected", slot);
    release_slot(slot);
    vTaskDelete(NULL);
}

/* Claims a free slot and returns its index, or -1 if all are busy. */
static int claim_slot(int sock)
{
    xSemaphoreTake(g_clients_mux, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
        if (!g_clients[i].in_use) {
            g_clients[i].in_use = true;
            g_clients[i].sock = sock;
            g_clients[i].generation++;
            slot = i;
            break;
        }
    }
    xSemaphoreGive(g_clients_mux);
    return slot;
}

static int open_listener(uint16_t port)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        return -1;
    }

    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() to port %u failed: %d", port, errno);
        close(sock);
        return -1;
    }
    if (listen(sock, TCP_MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        close(sock);
        return -1;
    }
    return sock;
}

static void listener_task(void *arg)
{
    (void)arg;

    for (;;) {
        int listener = open_listener(g_port);
        if (listener < 0) {
            /* Usually means the network stack is not ready yet. */
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "listening on port %u, up to %d clients",
                 g_port, TCP_MAX_CLIENTS);

        for (;;) {
            struct sockaddr_storage peer;
            socklen_t peer_len = sizeof(peer);
            int sock = accept(listener, (struct sockaddr *)&peer, &peer_len);
            if (sock < 0) {
                ESP_LOGW(TAG, "accept() failed: %d, restarting listener", errno);
                break;
            }

            const struct timeval send_timeout = { .tv_sec = TCP_SEND_TIMEOUT };
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout,
                       sizeof(send_timeout));

            int slot = claim_slot(sock);
            if (slot < 0) {
                static const char busy[] = "ERR too many clients\r\n";
                send(sock, busy, sizeof(busy) - 1, 0);
                close(sock);
                continue;
            }

            /* Wide enough for the longest %d the compiler must assume. */
            char name[20];
            snprintf(name, sizeof(name), "tcpcli%d", slot);
            if (xTaskCreate(client_task, name, TCP_CLIENT_STACK,
                            (void *)(intptr_t)slot, TCP_CLIENT_PRIO,
                            NULL) != pdPASS) {
                ESP_LOGE(TAG, "cannot start task for client %d", slot);
                release_slot(slot);
            }
        }
        close(listener);
    }
}

esp_err_t console_tcp_start(uint16_t port)
{
    if (g_clients_mux != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    g_clients_mux = xSemaphoreCreateMutex();
    if (g_clients_mux == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
        g_clients[i].sock = -1;
    }
    g_port = port;

    if (xTaskCreate(listener_task, "tcplisten", TCP_LISTEN_STACK, NULL,
                    TCP_LISTEN_PRIO, NULL) != pdPASS) {
        vSemaphoreDelete(g_clients_mux);
        g_clients_mux = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

unsigned console_tcp_clients(void)
{
    if (g_clients_mux == NULL) {
        return 0;
    }
    unsigned n = 0;
    xSemaphoreTake(g_clients_mux, portMAX_DELAY);
    for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
        if (g_clients[i].in_use) {
            n++;
        }
    }
    xSemaphoreGive(g_clients_mux);
    return n;
}
