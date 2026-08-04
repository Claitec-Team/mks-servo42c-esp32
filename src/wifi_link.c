#include "wifi_link.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "servo_config.h"

static const char *TAG = "wifi";

/* Reconnect promptly a few times, then back off so a wrong password or an
 * absent access point does not spin the radio. */
#define WIFI_FAST_RETRIES   5
#define WIFI_BACKOFF_MS     5000

static bool s_connected;
static char s_ip[16] = "0.0.0.0";
static int  s_retries;

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *e = data;
        s_connected = false;
        snprintf(s_ip, sizeof(s_ip), "0.0.0.0");

        if (s_retries < WIFI_FAST_RETRIES) {
            s_retries++;
            ESP_LOGW(TAG, "disconnected (reason %d), retry %d",
                     e != NULL ? e->reason : -1, s_retries);
        } else {
            ESP_LOGW(TAG, "disconnected (reason %d), retrying every %d ms",
                     e != NULL ? e->reason : -1, WIFI_BACKOFF_MS);
            /* Runs on the event task, which has nothing else to do here. */
            vTaskDelay(pdMS_TO_TICKS(WIFI_BACKOFF_MS));
        }
        esp_wifi_connect();
        return;
    }

    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        s_retries = 0;
        s_connected = true;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "connected as %s, IP %s", SERVO_WIFI_HOSTNAME, s_ip);
        ESP_LOGW(TAG, "the command port is unauthenticated: anyone who can reach");
        ESP_LOGW(TAG, "%s:%d can drive the motor", s_ip, SERVO_TCP_PORT);
    }
}

esp_err_t wifi_link_start(void)
{
    if (SERVO_WIFI_SSID[0] == '\0') {
        ESP_LOGI(TAG, "WiFi off: set SERVO_WIFI_SSID in include/servo_config.h");
        return ESP_ERR_INVALID_STATE;
    }

    /* The WiFi driver keeps calibration data in NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (netif == NULL) {
        return ESP_FAIL;
    }
    esp_netif_set_hostname(netif, SERVO_WIFI_HOSTNAME);

    const wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL, NULL));

    wifi_config_t sta = { 0 };
    snprintf((char *)sta.sta.ssid, sizeof(sta.sta.ssid), "%s", SERVO_WIFI_SSID);
    snprintf((char *)sta.sta.password, sizeof(sta.sta.password), "%s",
             SERVO_WIFI_PASSWORD);
    /* An empty password means an open network; anything else needs at least
     * WPA2, which is what the default threshold asks for. */
    sta.sta.threshold.authmode =
        (SERVO_WIFI_PASSWORD[0] == '\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());

#if SERVO_WIFI_MAX_TX_POWER > 0
    /* Must come after esp_wifi_start(). Trades range for a smaller current
     * spike, which matters on a board with a marginal supply. */
    err = esp_wifi_set_max_tx_power(SERVO_WIFI_MAX_TX_POWER);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "tx power capped at %d.%02d dBm",
                 SERVO_WIFI_MAX_TX_POWER / 4,
                 (SERVO_WIFI_MAX_TX_POWER % 4) * 25);
    } else {
        ESP_LOGW(TAG, "esp_wifi_set_max_tx_power failed: %s", esp_err_to_name(err));
    }
#endif

    ESP_LOGI(TAG, "connecting to \"%s\"", SERVO_WIFI_SSID);
    return ESP_OK;
}

bool wifi_link_is_connected(void)
{
    return s_connected;
}

const char *wifi_link_ip(void)
{
    return s_ip;
}
