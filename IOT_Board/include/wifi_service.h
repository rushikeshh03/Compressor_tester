/******************************************************************************************************************************
 * @file    wifi_service.h
 * @brief   Factory-mode WiFi service UI + firmware updater (IoT device).
 *
 * Only used while running from the factory partition (OTA bootloader). Raises a hidden SoftAP
 * (CONFIG_FACTORY_WIFI_* in mqtt.h) and serves a small password-gated web page to:
 *   • show the app0→app1 backup result + running partition + firmware version,
 *   • upload a firmware .bin (flashed to app0 via ota_bl_web_flash_*),
 *   • change the CAN baud (persisted to NVS iot_cfg/can_baud, applied next boot),
 *   • reboot the device (exits factory mode; the AP is torn down on reboot).
 *
 * Started from ota_bootloader_run(); runs in its own task. Normal app mode never calls this,
 * so WiFi is off in normal operation. Bench/service tool — simple cookie auth over a hidden AP.
 ******************************************************************************************************************************/

#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/** Raise the AP + web server and start the service task. Safe no-op if already started or if
 *  CONFIG_FACTORY_WIFI is 0. */
void wifi_service_start(void);

/** Stop the web server and bring the AP down. (A reboot also tears it down.) */
void wifi_service_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_SERVICE_H */
