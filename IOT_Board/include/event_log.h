/******************************************************************************************************************************
 * @file    event_log.h
 * @brief   Lightweight IoT event log on the SD card (success/failure events only — NOT CAN data).
 *
 * Records what the device does and why: boot/partition, SD, CAN up/down, modem/network, MQTT
 * connect/publish, OTA download/flash, CAN self-update, rollback. One human-readable line per event,
 * appended to a per-day file:  /events/YYYYMMDD.log  (and /events/undated.log before time is known).
 *
 * Works from ALL partitions — app0, app1 AND the factory OTA bootloader — because all three run the
 * same firmware image. In app0 the SD is already mounted by sd_log_init(); in the factory bootloader
 * this module mounts it itself. Entirely BEST-EFFORT: any SD/mount/write failure is swallowed so
 * event logging can never crash or stall the application or the flasher.
 *
 * Line format:  <date> <time>  <LEVEL>  <PART>  <TAG>  <message>
 *   2026-08-04 16:02:07.026  OK    A0  MQTT  broker connected
 *   +000012.345s             FAIL  FC  OTA   sha256 mismatch      (pre-NTP: uptime instead of clock)
 ******************************************************************************************************************************/

#ifndef EVENT_LOG_H
#define EVENT_LOG_H

typedef enum { EV_INFO, EV_OK, EV_FAIL, EV_WARN } ev_level_t;

/**
 * @brief  Idempotent init: mount the SD card if not already mounted, create /events, create the
 *         write mutex, and resolve the running partition tag. Safe to call from any partition and
 *         safe to call more than once. Never blocks meaningfully; failures leave the logger inert.
 */
void event_log_init(void);

/** @brief  true once the SD card is mounted and the event log is usable. */
bool event_log_ready(void);

/** @brief  Append one event line (printf-style). No-op if the logger isn't ready. Thread-safe. */
void event_logf(ev_level_t lvl, const char *tag, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

#define EVLOG_OK(tag, ...)   event_logf(EV_OK,   tag, __VA_ARGS__)
#define EVLOG_FAIL(tag, ...) event_logf(EV_FAIL, tag, __VA_ARGS__)
#define EVLOG_INFO(tag, ...) event_logf(EV_INFO, tag, __VA_ARGS__)
#define EVLOG_WARN(tag, ...) event_logf(EV_WARN, tag, __VA_ARGS__)

#endif /* EVENT_LOG_H */
