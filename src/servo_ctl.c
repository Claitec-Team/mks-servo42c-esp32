#include "servo_ctl.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "servoctl";

#define CTL_LINE_MAX     128
#define CTL_QUEUE_NORMAL 8
#define CTL_QUEUE_URGENT 4
#define CTL_STACK        4096
#define CTL_PRIO         6      /* above the transports, so replies keep flowing */

typedef struct {
    char       line[CTL_LINE_MAX];
    cmd_sink_t sink;
} ctl_req_t;

static QueueHandle_t q_normal;
static QueueHandle_t q_urgent;
static mks_t        *g_servo;

/* Installed on every sink before dispatch. Long-running commands poll this and
 * bail out when something urgent is waiting. */
static bool ctl_abort_requested(void *ctx)
{
    (void)ctx;
    return q_urgent != NULL && uxQueueMessagesWaiting(q_urgent) > 0;
}

static void ctl_execute(ctl_req_t *req)
{
    cmd_sink_t sink = req->sink;
    sink.abort_requested = ctl_abort_requested;

    servo_cmd_execute_line(req->line, &sink);

    if (sink.prompt != NULL && sink.write != NULL) {
        sink.write(sink.ctx, sink.prompt);
    }
}

static void servo_task(void *arg)
{
    (void)arg;
    ctl_req_t req;

    for (;;) {
        /* Urgent commands always win, however long the normal queue is. */
        if (xQueueReceive(q_urgent, &req, 0) == pdTRUE) {
            ctl_execute(&req);
            continue;
        }
        /* Anything urgent that lands while a command runs is handled by the
         * command itself via ctl_abort_requested(), then served on the next
         * pass. Do not reorder it ahead of a command already dequeued: a stop
         * must never be followed by the move it was meant to cancel. */
        if (xQueueReceive(q_normal, &req, pdMS_TO_TICKS(50)) == pdTRUE) {
            ctl_execute(&req);
        }
    }
}

esp_err_t servo_ctl_start(mks_t *servo)
{
    if (servo == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (q_normal != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    q_normal = xQueueCreate(CTL_QUEUE_NORMAL, sizeof(ctl_req_t));
    q_urgent = xQueueCreate(CTL_QUEUE_URGENT, sizeof(ctl_req_t));
    if (q_normal == NULL || q_urgent == NULL) {
        ESP_LOGE(TAG, "queue allocation failed");
        goto fail;
    }

    g_servo = servo;
    servo_cmd_init(servo);

    if (xTaskCreate(servo_task, "servo", CTL_STACK, NULL, CTL_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "task creation failed");
        goto fail;
    }
    return ESP_OK;

fail:
    if (q_normal != NULL) { vQueueDelete(q_normal); q_normal = NULL; }
    if (q_urgent != NULL) { vQueueDelete(q_urgent); q_urgent = NULL; }
    g_servo = NULL;
    return ESP_ERR_NO_MEM;
}

bool servo_ctl_submit(const char *line, const cmd_sink_t *sink)
{
    if (line == NULL || sink == NULL) {
        return false;
    }
    if (q_normal == NULL) {
        cmd_printf(sink, "ERR servo control not started\r\n");
        return false;
    }

    ctl_req_t req;
    snprintf(req.line, sizeof(req.line), "%s", line);
    req.sink = *sink;

    bool urgent = servo_cmd_is_urgent(line);
    QueueHandle_t q = urgent ? q_urgent : q_normal;

    if (xQueueSend(q, &req, 0) != pdTRUE) {
        cmd_printf(sink, "ERR %s queue full, command dropped\r\n",
                   urgent ? "urgent" : "command");
        return false;
    }
    return true;
}

unsigned servo_ctl_pending(void)
{
    if (q_normal == NULL) {
        return 0;
    }
    return (unsigned)(uxQueueMessagesWaiting(q_normal) +
                      uxQueueMessagesWaiting(q_urgent));
}
