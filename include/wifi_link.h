/*
 * WiFi station connection.
 *
 * Reads SERVO_WIFI_SSID / SERVO_WIFI_PASSWORD from servo_config.h. If the SSID
 * is empty the radio is never started and wifi_link_start() reports
 * ESP_ERR_INVALID_STATE, which callers treat as "WiFi not wanted".
 *
 * Reconnects on its own, so callers do not have to watch the link.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

/* Brings up the station interface and starts connecting. Returns once the
 * attempt is under way, not once it has succeeded. */
esp_err_t wifi_link_start(void);

/* True while the station holds an IP address. */
bool wifi_link_is_connected(void);

/* Current address as a string, or "0.0.0.0" when not connected. Never NULL. */
const char *wifi_link_ip(void);
