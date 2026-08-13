#include "ota_update.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static const char *TAG = "ota";

#define OTA_CHUNK          4096   /* recv / flash-write granularity */
#define OTA_HEADER_MAX      64    /* "OTA <size>" and then some       */
#define OTA_TASK_STACK     4608
#define OTA_TASK_PRIO         4
#define OTA_RECV_TIMEOUT     20   /* s idle before a stalled push is abandoned */
#define OTA_SEND_TIMEOUT     10   /* s; keeps a dead peer from blocking a reply */

static uint16_t s_port;

/* Best-effort reply; an update is a one-shot exchange, so a failed send just
 * means the client will not see the message, not that state is left dangling. */
static void reply(int sock, const char *text)
{
    send(sock, text, strlen(text), 0);
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
    if (listen(sock, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        close(sock);
        return -1;
    }
    return sock;
}

/*
 * Reads the "OTA <size>\n" header. Any bytes that arrived in the same packet
 * after the newline are the first of the firmware, so they are handed back in
 * (*payload, *payload_len), which point into `chunk` and stay valid only until
 * the next recv() on `chunk`. Returns false on a closed socket or an
 * over-long header (a client that is not speaking this protocol).
 */
static bool read_header(int sock, char *hdr, size_t hdrcap,
                        uint8_t *chunk, size_t chunkcap,
                        uint8_t **payload, size_t *payload_len)
{
    size_t hlen = 0;
    for (;;) {
        int n = recv(sock, chunk, chunkcap, 0);
        if (n <= 0) {
            return false;
        }
        for (int i = 0; i < n; i++) {
            if (chunk[i] == '\n') {
                hdr[hlen] = '\0';
                *payload = &chunk[i + 1];
                *payload_len = (size_t)(n - (i + 1));
                return true;
            }
            if (chunk[i] == '\r') {
                continue;                       /* tolerate CRLF */
            }
            if (hlen + 1 >= hdrcap) {
                return false;                   /* not our protocol */
            }
            hdr[hlen++] = (char)chunk[i];
        }
        /* No newline yet; the header should never be this long, but keep
         * reading rather than misparse a fragmented one. */
    }
}

/*
 * Runs one update to completion. On success it sets the boot partition, closes
 * the socket and reboots - it does not return. On any failure it writes an ERR
 * line and returns, leaving the socket for the caller to close.
 */
static void do_update(int sock)
{
    /* Static so the 4 KB does not sit on the task stack. Safe because only one
     * update runs at a time: the listener handles connections one by one. */
    static uint8_t chunk[OTA_CHUNK];
    char hdr[OTA_HEADER_MAX];
    char msg[112];
    uint8_t *payload = NULL;
    size_t payload_len = 0;

    if (!read_header(sock, hdr, sizeof(hdr), chunk, sizeof(chunk),
                     &payload, &payload_len)) {
        reply(sock, "ERR bad or truncated header\r\n");
        return;
    }

    unsigned long size = 0;
    if (sscanf(hdr, "OTA %lu", &size) != 1 || size == 0) {
        reply(sock, "ERR expected 'OTA <size>'\r\n");
        return;
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL) {
        reply(sock, "ERR no OTA slot; flash the OTA partition table over USB "
                    "first\r\n");
        return;
    }
    if (size > target->size) {
        snprintf(msg, sizeof(msg), "ERR image %lu B exceeds slot %s (%lu B)\r\n",
                 size, target->label, (unsigned long)target->size);
        reply(sock, msg);
        return;
    }

    ESP_LOGI(TAG, "OTA -> %s, %lu bytes; erasing", target->label, size);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, size, &handle);
    if (err != ESP_OK) {
        snprintf(msg, sizeof(msg), "ERR esp_ota_begin: %s\r\n",
                 esp_err_to_name(err));
        reply(sock, msg);
        return;
    }

    /* Sent only after the erase, so the client streams once we can absorb it. */
    snprintf(msg, sizeof(msg), "OK begin %s %lu\r\n", target->label, size);
    reply(sock, msg);

    size_t remaining = size;

    /* Firmware bytes that rode in with the header, before the recv loop. */
    if (payload_len > 0) {
        size_t take = payload_len < remaining ? payload_len : remaining;
        if (esp_ota_write(handle, payload, take) != ESP_OK) {
            esp_ota_abort(handle);
            reply(sock, "ERR flash write failed\r\n");
            return;
        }
        remaining -= take;
    }

    while (remaining > 0) {
        int n = recv(sock, chunk, sizeof(chunk), 0);
        if (n <= 0) {
            esp_ota_abort(handle);
            ESP_LOGW(TAG, "transfer interrupted with %u bytes to go",
                     (unsigned)remaining);
            reply(sock, "ERR transfer interrupted\r\n");
            return;
        }
        size_t take = (size_t)n < remaining ? (size_t)n : remaining;
        err = esp_ota_write(handle, chunk, take);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            snprintf(msg, sizeof(msg), "ERR esp_ota_write: %s\r\n",
                     esp_err_to_name(err));
            reply(sock, msg);
            return;
        }
        remaining -= take;
    }

    /* Validates the image (magic, and the secure/anti-rollback checks). */
    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        snprintf(msg, sizeof(msg), "ERR image rejected: %s\r\n",
                 esp_err_to_name(err));
        reply(sock, msg);
        return;
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        snprintf(msg, sizeof(msg), "ERR set boot partition: %s\r\n",
                 esp_err_to_name(err));
        reply(sock, msg);
        return;
    }

    snprintf(msg, sizeof(msg), "OK done %s, rebooting\r\n", target->label);
    reply(sock, msg);
    ESP_LOGW(TAG, "OTA accepted; rebooting into %s", target->label);

    /* Flush the reply before the reset tears the connection down. */
    shutdown(sock, SHUT_RDWR);
    close(sock);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static void ota_task(void *arg)
{
    (void)arg;

    for (;;) {
        int listener = open_listener(s_port);
        if (listener < 0) {
            /* Network stack usually just isn't up yet. */
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "OTA update port %u ready", s_port);

        for (;;) {
            struct sockaddr_storage peer;
            socklen_t peer_len = sizeof(peer);
            int sock = accept(listener, (struct sockaddr *)&peer, &peer_len);
            if (sock < 0) {
                ESP_LOGW(TAG, "accept() failed: %d, restarting listener", errno);
                break;
            }

            const struct timeval rcv = { .tv_sec = OTA_RECV_TIMEOUT };
            const struct timeval snd = { .tv_sec = OTA_SEND_TIMEOUT };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv, sizeof(rcv));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));

            ESP_LOGI(TAG, "OTA client connected");
            do_update(sock);   /* returns only on failure; reboots on success */
            close(sock);
        }
        close(listener);
    }
}

esp_err_t ota_update_start(uint16_t port)
{
    s_port = port;
    if (xTaskCreate(ota_task, "ota", OTA_TASK_STACK, NULL,
                    OTA_TASK_PRIO, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
