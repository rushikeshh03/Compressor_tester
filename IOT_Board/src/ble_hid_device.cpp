/**
 * ble_hid_device.cpp  --  BLE HID keyboard peripheral (to the Android tablet)
 *
 * Adapted from the LCD project's ble_hid_device.cpp, but PERIPHERAL-ONLY: the IOT
 * board has no BLE scanner central, so this module owns the radio itself
 * (BLEDevice::init on the dev task) instead of relying on ble_hid_host.
 *
 * Advertises as a BLE HID keyboard; when the tablet connects, ble_hid_dev_type()
 * sends the string as keystrokes (press + release per character, Enter = 0x28).
 *
 * Ordering + stack notes (from the LCD, learned on hardware):
 *   1) The GATT server is built ONCE, at init, on a dedicated 16 KB task — never on
 *      the loop task (BLEHIDDevice's ctor allocates ~8 characteristics/descriptors
 *      and overflows a small stack), and never lazily on Advertise.
 *   2) Advertising start() can transiently fail until the host has finished its
 *      async sync, so the dev-task re-attempts every second (keep-alive), and
 *      re-advertises after a disconnect.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <BLECharacteristic.h>
#include <BLEAdvertising.h>
#include <BLESecurity.h>
#include <string.h>
#include <new>                /* std::nothrow */
#include "ble_hid_device.h"

/* Standard boot-keyboard report map, Report ID 1 (8-byte input report). */
static const uint8_t REPORT_MAP[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,
    0x85, 0x01,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x03,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
    0xC0
};

static BLEServer         *s_server = nullptr;
static BLEHIDDevice      *s_hid    = nullptr;
static BLECharacteristic *s_input  = nullptr;
static BLEAdvertising    *s_adv    = nullptr;

static volatile hiddev_state_t s_state     = HIDDEV_DOWN;
static volatile bool           s_want_adv   = false;
static volatile bool           s_advertising = false;   /* tracked ourselves: 2.0.x BLEAdvertising has no isAdvertising() */
static bool                    s_hid_built  = false;

/* command queue + task (all BLE-peripheral work happens here, off setup()/loop) */
enum DevCmd { DEV_ADV_ON, DEV_ADV_OFF, DEV_TYPE, DEV_RESTART };
typedef struct { DevCmd cmd; char s[130]; } DevMsg;   /* barcode + terminator */
static QueueHandle_t     s_q     = nullptr;
static SemaphoreHandle_t s_ready = nullptr;   /* signalled once the HID is built */

/* ── ASCII → HID usage ─────────────────────────────────────────────────────── */
static bool ascii_to_hid(char c, uint8_t *usage, bool *shift)
{
    *shift = false;
    if (c >= 'a' && c <= 'z') { *usage = 0x04 + (c - 'a'); return true; }
    if (c >= 'A' && c <= 'Z') { *usage = 0x04 + (c - 'A'); *shift = true; return true; }
    if (c >= '1' && c <= '9') { *usage = 0x1E + (c - '1'); return true; }
    if (c == '0') { *usage = 0x27; return true; }
    switch (c) {
        case '\n': case '\r': *usage = 0x28; return true;  /* Enter */
        case ' ':  *usage = 0x2C; return true;
        case '-':  *usage = 0x2D; return true;
        case '=':  *usage = 0x2E; return true;
        case '.':  *usage = 0x37; return true;
        case '/':  *usage = 0x38; return true;
        case '_':  *usage = 0x2D; *shift = true; return true;
        default:   return false;
    }
}

static void send_report(uint8_t mod, uint8_t usage)
{
    if (!s_input) return;
    uint8_t rep[8] = { mod, 0, usage, 0, 0, 0, 0, 0 };
    s_input->setValue(rep, sizeof(rep));
    s_input->notify();
}

static void type_string(const char *s)
{
    for (const char *p = s; *p; p++) {
        uint8_t usage; bool shift;
        if (!ascii_to_hid(*p, &usage, &shift)) continue;
        send_report(shift ? 0x02 : 0x00, usage);   /* key down */
        vTaskDelay(pdMS_TO_TICKS(12));
        send_report(0x00, 0x00);                    /* key up   */
        vTaskDelay(pdMS_TO_TICKS(12));
    }
}

/* ── Server callbacks ──────────────────────────────────────────────────────── */
class SrvCB : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        s_state = HIDDEV_CONNECTED;
        s_advertising = false;          /* Bluedroid stops advertising on connect */
        Serial.println("[BLE-HID] tablet connected");
    }
    void onDisconnect(BLEServer *) override {
        Serial.println("[BLE-HID] tablet disconnected");
        if (s_want_adv && s_adv) { s_adv->start(); s_advertising = true; s_state = HIDDEV_ADVERTISING; }
        else                     { s_advertising = false; s_state = HIDDEV_DOWN; }
    }
};
static SrvCB s_srv_cb;

/* ── BLE security ────────────────────────────────────────────────────────────
 * BLE HID REQUIRES a bonded, encrypted link — otherwise the tablet cannot enable
 * the HID report (GATT_INSUF_AUTHENTICATION) and no keystrokes flow. Just-Works
 * bonding (no passkey), Secure Connections, no MITM — mirrors the LCD's scanner/
 * tablet pairing, but with the Bluedroid (arduino-esp32 2.0.x) API. */
class SecCB : public BLESecurityCallbacks {
    uint32_t onPassKeyRequest() override { return 0; }
    void     onPassKeyNotify(uint32_t) override {}
    bool     onSecurityRequest() override { return true; }
    bool     onConfirmPIN(uint32_t) override { return true; }
    void     onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
        if (cmpl.success) Serial.println("[BLE-HID] pairing complete (bonded, encrypted)");
        else              Serial.printf("[BLE-HID] pairing FAILED (reason 0x%x)\n", cmpl.fail_reason);
    }
};
static SecCB s_sec_cb;

/* Configure Just-Works bonding + encryption. Call after BLEDevice::init(), before
 * advertising. Bluedroid instance API (the LCD's static NimBLE calls don't exist here).
 * HID requires a bonded/encrypted link or the tablet gets GATT_INSUF_AUTHENTICATION.
 * Bonds PERSIST across reboots, so the tablet stays paired after one manual connect. */
static void apply_security(void)
{
    BLEDevice::setSecurityCallbacks(&s_sec_cb);
    BLESecurity *sec = new BLESecurity();
    sec->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);   /* bonding + secure connections, no MITM */
    sec->setCapability(ESP_IO_CAP_NONE);                   /* Just Works (no passkey UI) */
    sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    sec->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
}

/* Build the HID peripheral — runs ONLY on the dev task (needs a big stack), and only
 * while NO connection is active. Returns true on success; guards every allocation so a
 * low-memory failure logs instead of crashing. Requires BLEDevice::init() done first. */
static bool build_hid(void)
{
    Serial.printf("[BLE-HID] building HID service… (free heap %u, largest %u)\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());

    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 30000) {
        Serial.printf("[BLE-HID] NOT building HID — only %u B free heap (BLE needs ~30 KB)\n",
                      (unsigned)freeHeap);
        return false;
    }

    s_server = BLEDevice::createServer();
    if (!s_server) { Serial.println("[BLE-HID] createServer FAILED (out of memory?)"); return false; }
    s_server->setCallbacks(&s_srv_cb);

    s_hid = new (std::nothrow) BLEHIDDevice(s_server);
    if (!s_hid) { Serial.println("[BLE-HID] new BLEHIDDevice FAILED (out of memory?)"); return false; }
    s_input = s_hid->inputReport(1);
    if (!s_input) { Serial.println("[BLE-HID] inputReport FAILED"); return false; }

    s_hid->pnp(0x02, 0xE502, 0xA111, 0x0210);
    s_hid->hidInfo(0x00, 0x01);
    s_hid->reportMap((uint8_t *)REPORT_MAP, sizeof(REPORT_MAP));
    s_hid->startServices();

    s_adv = BLEDevice::getAdvertising();
    if (!s_adv) { Serial.println("[BLE-HID] getAdvertising FAILED"); return false; }
    s_adv->setAppearance(0x03C1);                       /* HID keyboard */
    s_adv->addServiceUUID(BLEUUID((uint16_t)0x1812));   /* HID service  */
    s_adv->setScanResponse(true);

    Serial.println("[BLE-HID] HID service ready");
    return true;
}

/* Try once to (re)start advertising. Latches s_want_adv so onDisconnect and the
 * dev-task keep-alive both re-advertise. Returns true only when actually advertising. */
static bool start_advertising(void)
{
    if (!s_adv || !s_hid_built) {
        Serial.println("[BLE-HID] cannot advertise — HID not built");
        return false;
    }
    s_want_adv = true;
    if (s_advertising) { if (s_state != HIDDEV_CONNECTED) s_state = HIDDEV_ADVERTISING; return true; }

    s_adv->start();                     /* 2.0.x Bluedroid: returns void, synchronous */
    s_advertising = true;
    if (s_state != HIDDEV_CONNECTED) s_state = HIDDEV_ADVERTISING;
    Serial.printf("[BLE-HID] advertising as '%s'\n", CONFIG_BLE_HID_NAME);
    return true;
}

static void dev_task(void *)
{
    /* Peripheral-only: bring up the radio here (the LCD relied on ble_hid_init for this). */
    BLEDevice::init(CONFIG_BLE_HID_NAME);
    apply_security();      /* Just-Works bonding + encryption (required for HID) */

    s_hid_built = build_hid();
    if (s_ready) xSemaphoreGive(s_ready);

    /* Advertise from boot until a tablet connects; the first attempt often loses the
     * race with the host sync, so latch intent and let the keep-alive retry. */
    s_want_adv = true;
    start_advertising();

    DevMsg m;
    for (;;) {
        /* 1 s timeout doubles as an advertising keep-alive. */
        if (xQueueReceive(s_q, &m, pdMS_TO_TICKS(1000)) != pdTRUE) {
            if (s_want_adv && s_hid_built && s_adv &&
                s_state != HIDDEV_CONNECTED && !s_advertising)
                start_advertising();
            continue;
        }
        switch (m.cmd) {
            case DEV_ADV_ON:
                start_advertising();
                break;
            case DEV_ADV_OFF:
                s_want_adv = false;
                if (s_adv) s_adv->stop();
                s_advertising = false;
                if (s_server && s_server->getConnectedCount() > 0)
                    s_server->disconnect(s_server->getConnId());
                s_state = HIDDEV_DOWN;
                Serial.println("[BLE-HID] advertising stopped");
                break;
            case DEV_TYPE:
                if (s_state == HIDDEV_CONNECTED) type_string(m.s);
                break;
            case DEV_RESTART: {
                /* Peripheral-only restart on this 16 KB task: tear down, re-init, rebuild. */
                Serial.println("[BLE] restarting HID stack…");
                s_state = HIDDEV_DOWN; s_want_adv = false; s_advertising = false;
                s_server = nullptr; s_hid = nullptr; s_input = nullptr; s_adv = nullptr;
                BLEDevice::deinit(true);
                vTaskDelay(pdMS_TO_TICKS(150));
                BLEDevice::init(CONFIG_BLE_HID_NAME);
                apply_security();
                s_hid_built = build_hid();
                start_advertising();
                Serial.println("[BLE] HID stack restarted");
                break;
            }
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════ */
void ble_hid_dev_init(void)
{
    /* Bring up the radio + build the HID GATT server on a 16 KB task, and BLOCK here
     * until it is built (createServer() must run with no active connection). */
    s_q     = xQueueCreate(6, sizeof(DevMsg));
    s_ready = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(dev_task, "hid_dev", 16384, nullptr, 5, nullptr, 1);
    if (s_ready) xSemaphoreTake(s_ready, pdMS_TO_TICKS(8000));
}

void ble_hid_dev_advertise(bool on)
{
    if (!s_q) return;
    DevMsg m = {}; m.cmd = on ? DEV_ADV_ON : DEV_ADV_OFF;
    xQueueSend(s_q, &m, 0);
}

void ble_hid_restart_stack(void)
{
    if (!s_q) return;
    DevMsg m = {}; m.cmd = DEV_RESTART;
    xQueueSend(s_q, &m, 0);
}

void ble_hid_dev_type(const char *s)
{
    if (!s_q || !s || s_state != HIDDEV_CONNECTED) return;
    DevMsg m = {}; m.cmd = DEV_TYPE;
    strncpy(m.s, s, sizeof(m.s) - 1);
    xQueueSend(s_q, &m, 0);
}

hiddev_state_t ble_hid_dev_state(void) { return s_state; }
