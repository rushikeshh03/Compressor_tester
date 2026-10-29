/**
 * main.cpp  --  Coulomb Harness Tester Display
 *
 * Hardware : Waveshare ESP32-S3-Touch-LCD-7 (800x480, ST7262 RGB + GT911 touch)
 * LVGL v8.3.x  |  ESP32_Display_Panel  |  Board class
 *
 * Board class reads esp_panel_board_custom_conf.h and initialises:
 *   - ST7262 RGB LCD  (via CH422G IO expander for RST)
 *   - GT911 touch     (I2C SDA=8 SCL=9, INT=GPIO4, RST=expander pin 1)
 *   - CH422G IO expander
 *   - Backlight       (via expander pin 2)
 *
 * ── CAN Bootloader ───────────────────────────────────────────────────────
 *
 * An ABMS-compatible CAN OTA bootloader is integrated.  The host tool that
 * flashes the ABMS unit will flash this display using the same commands.
 *
 *   CAN ID   : 0x00B  (11-bit standard)
 *   Baud     : CONFIG_CAN_BAUD_KBPS (app_config.h) — shared by app + flasher
 *   Protocol : identical to ABMS_BOOTLOADER v3.6
 *              Activate → UNLOCK → DATA × N → VERIFY → RESET
 *
 * The firmware binary to send is the .bin produced by PlatformIO:
 *   .pio/build/waveshare_esp32s3/firmware.bin
 *
 * Partition requirement: two OTA slots (ota_0 + ota_1) must exist in the
 * partition table.  Use default_16MB.csv or a custom CSV — see can_bootloader.h.
 *
 * Startup sequence:
 *   1. board->init() + board->begin()   -- LCD + touch + expander + backlight
 *   2. lvgl_port_init(lcd, touch)       -- LVGL with touch indev registered
 *   3. ba100_can_init()                 -- TWAI CAN driver (CONFIG_CAN_BAUD_KBPS, NORMAL)
 *   4. can_bootloader_start()           -- OTA bootloader task (monitors 0x00B)
 *   5. ui_init()                        -- build both LVGL screens
 *   6. Loop: poll CAN -> ui_apply_state() at ~25 Hz
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>

#include "app_config.h"        /* ← central hardware/feature selection */
#include "lvgl_v8_port.h"
#include "ba100_can.h"

/* Build-consistency guard: the native USB (GPIO19/20) is a HOST for the USB-HID
 * scanner, so USB-CDC MUST be off — an enabled CDC device would fight the host
 * stack for the OTG peripheral. platformio.ini pins ARDUINO_USB_CDC_ON_BOOT=0.
 * (CAN is on the external TJA1050 GPIO43/44, not these pins.) */
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#error "This board requires ARDUINO_USB_CDC_ON_BOOT=0 (set in platformio.ini)"
#endif
#include "can_bootloader.h"    /* ← OTA bootloader */
#include "sd_logger.h"         /* ← micro-SD event logging (isolated task) */
#include "ui/ui.h"
#include "harness_conn.h"      /* ← link manager (USB scanner + CAN check) */
#include "scanner.h"           /* ← USB scanner selector (CDC-ACM or HID) */

using namespace esp_panel::drivers;
using namespace esp_panel::board;

void ui_apply_state(const BA100_State_t &state);

static uint32_t g_ui_update_ms = 0;


/* ================================================================
 *  setup()
 * ================================================================ */
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Coulomb Li Tech Charger Display ===");

    /* ---- 1. Board init (LCD + Touch + Expander + Backlight) ---- */
    Serial.println("Init board...");
    Board *board = new Board();
    board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
    {
        auto lcd = board->getLCD();
        lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);

#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
        auto bus = lcd->getBus();
        if (bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
            static_cast<BusRGB *>(bus)->configRGB_BounceBufferSize(
                lcd->getFrameWidth() * 10
            );
        }
#endif
    }
#endif

    assert(board->begin());
    Serial.println("Board OK");

    /* ---- 2. LVGL port ------------------------------------------ */
    Serial.println("Init LVGL...");
    if (!lvgl_port_init(board->getLCD(), board->getTouch())) {
        Serial.println("LVGL init FAILED -- halting");
        while (true) delay(1000);
    }
    Serial.println("LVGL OK");

    /* ---- 3. USB / CAN routing ---------------------------------- */
    /*
     * Architecture (2026-09-10): the barcode scanner is a USB-HID device on the
     * native USB; CAN is reached over a UART link to an ESP32-WROOM bridge
     * (../CAN_Bridge). Separate pins, so both run at once.
     *   • USB_SEL (CH422G IO5) LOW → GPIO19/20 → USB-C port (scanner).
     *   • UART to the bridge = GPIO43/44 (app_config.h).
     * ⚠ GPIO43/44 are UART0: the LCD's USB-serial console is gone (logs → SD;
     *   the bridge's own USB is the CAN console). Set the UART1/2 DIP switch to
     *   UART2 so P1 carries these pins.
     */
    {
        constexpr int USB_SEL = 5;          /* CH422G IO5 — FSUSB42 USB/CAN-legacy mux */
        auto io_exp = board->getIO_Expander();
        if (io_exp) {
            io_exp->getBase()->digitalWrite(USB_SEL, LOW);    /* GPIO19/20 → USB-C (scanner) */
            Serial.println("USB_SEL=LOW (GPIO19/20 → USB-C scanner; CAN via UART bridge on GPIO43/44)");
        } else {
            Serial.println("WARN: no IO expander — cannot set USB_SEL");
        }
    }

    /* ---- CAN-flash bootloader (IOT parity) --------------------- */
    /* Dormant on this build: CAN-flash over the UART bridge is not wired up, and
     * in app0/app1 this returns false immediately (flash the LCD over USB). */
    if (can_bootloader_run_if_factory()) {
        while (true) delay(1000);   /* unreachable — factory loop runs forever */
    }

    Serial.println("Init CAN link (UART bridge → ../CAN_Bridge)...");
    if (!ba100_can_init()) {
        Serial.println("CAN bridge init FAILED");
    }

    /* ---- 3b. micro-SD logger (fully isolated task) -------------- */
    /* The SD chip-select sits on the CH422G expander, which the Board owns;
     * hand the logger that expander so it can hold CS low. Logging runs on its
     * own task and degrades silently if no card is present — it can never block
     * the display or CAN paths. */
    {
        auto io_exp = board->getIO_Expander();
        sd_logger_init(io_exp ? io_exp->getBase() : nullptr);
    }

    /* ---- 4. CAN bootloader (app-side) --------------------------- */
    /* IOT parity: the running app (app0) does NOT flash in place. Flashing
     * happens only after the desktop flasher's 0x0D "enter bootloader" trigger
     * backs up app0->app1 and reboots into the factory partition (handled in
     * ba100_can_poll -> can_bootloader_enter_notify). This call is a no-op. */
    can_bootloader_start();
    Serial.println("CAN bootloader: send 0x0D to enter (reboots into factory)");

    /* ---- 5. UI -------------------------------------------------- */
    Serial.println("Building UI...");
    lvgl_port_lock(-1);
    ui_init();
    lvgl_port_unlock();

    /* ---- 6. USB scanner + link manager ------------------------- */
    /* scanner_init() starts the selected USB-host driver (CDC-ACM or HID, per
     * CONFIG_SCANNER_USB_CDC); harness_conn drains completed barcodes. */
    Serial.println("Init USB scanner...");
    scanner_init();
    harness_conn_init();

    /* Pre-create the bootloader debug screen overlay so it is ready
     * before any CAN activation arrives.  The normal UI stays visible
     * until 0xB1 is received; the overlay is loaded at that point.   */
    can_bootloader_set_screen();

    Serial.println("Ready.\n");
}

/* ================================================================
 *  loop()
 * ================================================================ */
void loop()
{
    /* Poll the CAN bridge link (non-blocking) — drains frames relayed over UART. */
    ba100_can_poll();

    /* Refresh LVGL widgets at ~25 Hz */
    uint32_t now = millis();
    if (now - g_ui_update_ms >= 40) {
        g_ui_update_ms = now;

        BA100_State_t state = ba100_get_state();

        lvgl_port_lock(-1);
        ui_apply_state(state);
        harness_conn_poll();       /* drains the USB scanner + runs the CAN check */
        lvgl_port_unlock();
    }
}