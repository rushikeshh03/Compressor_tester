/***********************************************************************************************************************
 * @file   main.h
 * @brief  Defines and includes for all files
 ***********************************************************************************************************************/

#ifndef MAIN_H
#define MAIN_H
#define IOT_VERSION "1.0.0"
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <ArduinoJson.h>
#include "driver/uart.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "ota_bootloader.h"   // after the other includes
#include "frame_buffer.h"
#include "sd_log.h"
#include "event_log.h"
#include "das_sync.h"

static const char *CAN_TAG = "CAN";
static const char *UART_TAG = "UART";
static const char *LTE_TAG = "LTE";



/**
*  @brief  RTOS parameters
*/

#define RTOS_STACK_DEPTH                (2048)
#define INIT_TASK_COMPLETE_BIT          (1 << 0)
/* Set as soon as the CAN driver is up — BEFORE modem/MQTT init — so CAN
 * reception (and the 0x0C self-update trigger) works even with no network. */
#define CAN_READY_BIT                   (1 << 1)
    
extern QueueHandle_t uartSerialQueue;
extern QueueHandle_t responseQueue;
extern SemaphoreHandle_t atSemaphore;
extern uart_event_t event;
extern bool ntpTimeReceived;
extern bool gpsTimeReceived;

// Task handles
extern TaskHandle_t uartSerialTaskHandle;
extern TaskHandle_t mqttTaskHandle;
extern TaskHandle_t canToSerialTaskHandle;
extern TaskHandle_t canReceiveTaskHandle;
extern TaskHandle_t timestampSyncTaskHandle;
extern EventGroupHandle_t initTaskEventGroup;



/**
*  @brief  UART
*/

#define SERIAL_BAUD_RATE                (115200)

#define SERIAL_UART                     UART_NUM_0

#define SERIAL_BUFFER_SIZE              (256)

/* EC200 LTE modem removed — MQTT is WiFi-only. GSM UART / EC200 pins deleted. */

/**
*  @brief  Pin definitions
*/

#define CAN_RX                          5
#define CAN_TX                          4
#define SERIAL_RX                       44
#define SERIAL_TX                       43

/* SD card (SPI mode) — DM3AT-SF-PEJM5 socket on the board's SPI2 pins */
#define SD_CS                           10   /* CD/DAT3 */
#define SD_MOSI                         11   /* CMD      */
#define SD_SCK                          12   /* CLK      */
#define SD_MISO                         13   /* DAT0     */

/**
*  @brief  Runtime CAN baud configuration (single source of truth for ALL
*          partitions: app0 and the factory CAN-flash bootloader).
*
*  The baud is stored in NVS (namespace "iot_cfg", key "can_baud") and resolved
*  by can_resolve_baud_timing() in BOTH can_init() (app0) and can_init_bl()
*  (factory). CONFIG_CAN_BAUD_KBPS is the compile default / empty-NVS fallback.
*
*  A host changes it at runtime with IOT_BAUD_CFG_ID: DLC>=2, data[0]=magic,
*  data[1]=selector. The device ACKs on IOT_BAUD_ACK_ID, persists to NVS, and
*  re-tunes its TWAI immediately. The next factory entry / reboot then flashes
*  and runs at the same rate — app, factory, and bus always agree.
*
*  ⚠ DISTINCT ID + data from the LCD (LCD baud = 0x2CC, bare selector
*  0x01/0x02/0x05) so a baud command for one product never retunes the other on
*  a shared bus.
*/
#define CONFIG_CAN_BAUD_KBPS            (500)          /* default; 125 / 250 / 500 */
#define CAN_BAUD_NVS_NS                 "iot_cfg"
#define CAN_BAUD_NVS_KEY                "can_baud"

#define IOT_BAUD_CFG_ID                 0x2E0U         /* RX: change CAN baud       */
#define IOT_BAUD_ACK_ID                 0x2E1U         /* TX: ACK [magic, sel, 0]   */
#define IOT_BAUD_MAGIC                  0xB0U          /* data[0] must equal this   */
#define IOT_BAUD_SEL_125                0x11U
#define IOT_BAUD_SEL_250                0x22U
#define IOT_BAUD_SEL_500                0x55U

/* ── Restart command (LCD → IoT, standard 11-bit) ──────────────────────────
 *  The LCD's "Restart system" button TXes this; the IoT reboots on receipt (the
 *  LCD then reboots itself, so both restart together). Two-byte magic 'R''S' so
 *  stray traffic can't trigger a reboot. Intercepted in can_receive() before the
 *  DAS whitelist. ⚠ Must match IOT_RESTART_* in the LCD's ba100_can.h. */
#define IOT_RESTART_CMD_ID              0x2D0U
#define IOT_RESTART_MAGIC0              0x52U          /* 'R' */
#define IOT_RESTART_MAGIC1              0x53U          /* 'S' */

/**
*  @brief  DAS harness protocol (das_protocol.dbc, 11-bit STANDARD IDs)
*
*  This board is a PASSIVE listener on the harness CAN bus. The harness tester
*  TXes the scanned barcode on 0x600/0x601/0x602; the harness "master" broadcasts
*  status + cell voltages + temperatures + a heartbeat on 0x710/0x711/0x712/0x720.
*  We capture ALL of these raw frames into the lossless ring and upload them to
*  MQTT unchanged (wire format v:2) — the backend decodes them. These IDs are the
*  whitelist in handle_rx_message(). They must stay in lock-step with the harness
*  master (Harness testing/include/ba100_can.h) and the backend decoder.
*
*  ⚠ Kept clear of the flasher enter-trigger (0x0C) and the baud command (0x2E0),
*  which can_receive() intercepts BEFORE handle_rx_message() — do not add those here.
*/
#define DAS_BARCODE0_ID                 0x600U    /* barcode 0x600-0x602, confirmed by       */
#define DAS_BARCODE1_ID                 0x601U    /* harness_jig v5 ("ID changed from 0x700  */
#define DAS_BARCODE2_ID                 0x602U    /*  to 0x600"); frame3 chars 16-19 only    */
#define DAS_STATUS_ID                   0x710U    /* harness_jig v5 1808 Jig_ResultStatusBarcode */
#define DAS_CELLS_ID                    0x711U    /* 1809 Jig_ResultCells (mV, LE16)          */
#define DAS_TEMPS_ID                    0x712U    /* 1810 Jig_ResultTemps (SIGNED int16, 0.1 degC) */
#define DAS_FAULT_ID                    0x713U    /* 1811 Jig_HarnessFault (v5: 6 signals — avg masks + raw-sample detail) */
#define DAS_HEARTBEAT_ID                0x720U    /* 1824 Jig_Heartbeat (counter + CAN health)*/

/**
*  @brief  IoT → LCD status heartbeat (this board TX, standard 11-bit)
*
*  The IoT board TXes this every 1 s so the LCD/harness tester knows the IoT is powered
*  and on the bus, and whether a tablet is connected over BLE HID. Low priority (high
*  numeric ID) so it never wins arbitration over the DAS result frames.
*  ⚠ The LCD must add a matching RX handler for this ID to display IoT/tablet status.
*    DLC 4:  data[0]=magic 0xA5, data[1]=flags, data[2]=rolling counter, data[3]=reserved.
*    flags:  bit0 = tablet connected, bit1 = LOCAL SERVER reachable, bit2 = CAN bus OK.
*  ⚠ bit1 semantics CHANGED (2026-09-26): was "cloud MQTT broker up", now "LOCAL Data Server
*    sync link up" (das_sync control-MQTT). Same bit value → the LCD's uplink chip needs no
*    code change; just relabel it "Server"/"Uplink".
*/
#define IOT_HEARTBEAT_ID                0x7A0U
#define IOT_HEARTBEAT_MAGIC             0xA5U
#define IOT_HB_FLAG_TABLET              0x01U     /* tablet connected over BLE HID   */
#define IOT_HB_FLAG_SERVER              0x02U     /* LOCAL Data Server sync link reachable (was cloud MQTT) */
#define IOT_HB_FLAG_CAN                 0x04U     /* CAN bus active (harness frames)  */

/**
*  @brief  Extern variables
*/

typedef struct 
{
  uint32_t msgId;
  uint8_t dataLength;
  uint8_t data[8];
} CanData_t;
extern CanData_t CanMsgs[200];

typedef struct 
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} DateTime;

extern DateTime timestampData;

extern bool canStatus;
extern uint8_t pageStatus;

/* Set by handle_rx_message() when a test result completes (a temps 0x712 frame arrives).
   mqtt_service() publishes ONE message per result on this trigger and clears it — the
   board no longer streams continuously, and the heartbeat (0x720) never triggers a publish. */
extern volatile bool g_scan_ready;

/* True for the scan being published when ONLY the barcode arrived (no jig result frames) —
   serialized into the MQTT payload as "incomplete_read". Set/cleared in can.cpp. */
extern volatile bool g_scan_incomplete;

/* Publishes a barcode-only scan as incomplete after the result-wait window. Called from the
   CAN Rx task (defined in can.cpp) so it fires even when the bus goes quiet. */
void das_poll_result_timeout(void);

/* True once the SD card mounted at boot. When false, mqtt_service() falls back to the legacy
   RAM-only publish path so the device keeps uploading even without persistent SD logging. */
extern bool g_sdReady;

/* Tick of the last accepted (target-ID) CAN frame. Updated only by handle_rx_message,
   so it reflects master-frame activity only — other bus traffic never resets it. Used by
   mqtt_service() to detect the end of a master cycle (idle gap) and flush that cycle. */
extern TickType_t canReceivedTime;

/**
*  @brief  Public function prototypes
*/

/* OTA Bootloader partition */
bool ota_bootloader_check_and_run(void);
/* Called from app0 init_task after MQTT is up — checks NVS for pending ESP32 OTA result */
void ota_bootloader_check_esp32_ota_result(void);
void ota_bootloader_trigger_reboot(void);
void ota_bootloader_run(void);

/* Timestamp sync task */
void timestamp_sync_task(void *parameter);
void rtc_update_timestamp(void);   // advance software RTC — call before reading timestampData


/* RTOS */
void init_task(void *pvParameters);
void uart_serial_event_task(void *pvParameters);
void can_receive_task(void *parameter);
void sd_flush_task(void *parameter);
void send_can_to_serial_task(void *parameter);

/* CAN */
void can_init(void);
/* Resolve the CAN baud (from NVS, default CONFIG_CAN_BAUD_KBPS) into a TWAI
 * timing config. Shared by app0 (can_init) and the factory bootloader
 * (can_init_bl) so every partition runs at the same rate. Writes the chosen
 * kbps to *kbps_out when non-NULL. */
twai_timing_config_t can_resolve_baud_timing(uint32_t *kbps_out);
void get_can_status(void);
bool can_receive(void);
void send_to_serial(void);
void diagnostic_can_task(void *parameter);


/* Telemetry / storage (see analytics.cpp + das_sync.* + wifi_mqtt.*) */
void check_can_status(void);        /* CAN bus-liveness flag (defined in can.cpp) */
void sd_flush_service(void);        /* drain the RAM ring into the SD NDJSON log (no network) */
void uart_serial_event(void);
uint64_t now_epoch_ms(void);

/* Software-RTC anchor (defined in analytics.cpp). The WiFi SNTP sync (wifi_mqtt.cpp)
 * calls it to anchor the monotonic epoch clock now_epoch_ms() uses. */
void rtc_record_sync(void);

#endif  /* MAIN_H */