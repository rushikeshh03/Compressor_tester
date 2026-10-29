#pragma once
/**
 * wifi_service.h  –  Factory-mode WiFi service UI + firmware updater
 *
 * Only used while running from the factory partition (CAN-flash mode). Raises a
 * hidden SoftAP (CONFIG_FACTORY_WIFI_* in app_config.h) and serves a small
 * password-protected web page to:
 *   • show the app0→app1 backup result + running partition,
 *   • upload a firmware .bin (flashed to app0 via the existing OTA path),
 *   • change runtime settings (CAN baud, KION mode) — persisted to NVS,
 *   • reboot the unit (exits factory mode; the AP is torn down on reboot).
 *
 * Started from can_bootloader_run_if_factory(); runs in its own task. Normal app
 * mode never calls this, so WiFi is off in normal operation.
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Raise the AP + web server and start the service task. Safe no-op if already
 *  started or if CONFIG_FACTORY_WIFI is 0. */
void wifi_service_start(void);

/** Stop the web server and bring the AP down. (Reboot also tears it down.) */
void wifi_service_stop(void);

/** True while a technician is logged in on the web page. Used by the factory
 *  reboot timer to switch from the 15 s CAN-idle recover to a 5-min
 *  web-inactivity window. */
bool wifi_service_logged_in(void);

/** Milliseconds since the last HTTP request (0xFFFFFFFF if none yet). */
uint32_t wifi_service_ms_since_activity(void);

#ifdef __cplusplus
}
#endif
