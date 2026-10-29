/**
 * usb_cdc_scanner.cpp  --  USB CDC-ACM barcode-scanner reader (native USB host)
 *
 * See usb_cdc_scanner.h. Implemented directly on the ESP-IDF **raw USB Host**
 * API (usb/usb_host.h) — which IS bundled with arduino-esp32 3.x — rather than
 * the separate `usb_host_cdc_acm` managed component (that header is not fetched
 * in a plain arduino-framework build).
 *
 * What it does: brings up the USB host, waits for a device, finds the first
 * BULK-IN endpoint (the CDC data interface's read endpoint), claims that
 * interface, best-effort asserts DTR/RTS, then continuously submits IN transfers
 * and feeds the bytes into a CR/LF barcode accumulator — same completed-barcode
 * contract as the HID reader (usb_cdc_scanner_take()).
 *
 * The whole module is gated on CONFIG_SCANNER_USB_CDC; when the HID scanner is
 * selected (flag 0) it compiles to nothing.
 *
 * ⚠ UNVERIFIED on hardware. Works for a device that exposes a bulk-IN endpoint
 * (true CDC-ACM). USB-UART bridge chips (CP210x/FTDI/CH340) are vendor-specific
 * and won't be read correctly here.
 */

#include "app_config.h"

#if CONFIG_SCANNER_USB_CDC

#include "usb_cdc_scanner.h"

#include <Arduino.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"

#define BC_MAX  126

/* ── completed-code handoff (USB task → LVGL task) ───────────────────────── */
static portMUX_TYPE s_mux   = portMUX_INITIALIZER_UNLOCKED;
static char         s_asm[BC_MAX + 1] = {0};
static size_t       s_asm_len = 0;
static char         s_done[BC_MAX + 1] = {0};
static volatile bool s_done_ready = false;

/* ── device state ────────────────────────────────────────────────────────── */
static usb_host_client_handle_t s_client   = NULL;
static usb_device_handle_t       s_dev      = NULL;
static usb_transfer_t           *s_in_xfer  = NULL;
static uint8_t                   s_in_ep    = 0;
static int                       s_claimed_intf = -1;
static volatile bool             s_connected = false;
static volatile uint8_t          s_pending_addr = 0;   /* set by NEW_DEV event */

static void commit_barcode(void)
{
    if (s_asm_len == 0) return;
    s_asm[s_asm_len] = '\0';
    portENTER_CRITICAL(&s_mux);
    memcpy(s_done, s_asm, s_asm_len + 1);
    s_done_ready = true;
    portEXIT_CRITICAL(&s_mux);
    s_asm_len = 0;
}

static void feed_byte(uint8_t c)
{
    if (c == '\r' || c == '\n') { commit_barcode(); return; }
    if (c >= 0x20 && c < 0x7F && s_asm_len < BC_MAX) s_asm[s_asm_len++] = (char)c;
}

/* ── IN transfer completion: consume bytes, resubmit ─────────────────────── */
static void in_xfer_cb(usb_transfer_t *xfer)
{
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        for (int i = 0; i < xfer->actual_num_bytes; i++) feed_byte(xfer->data_buffer[i]);
    }
    /* Keep listening while the device is present (COMPLETED or a benign NAK/timeout). */
    if (s_connected &&
        (xfer->status == USB_TRANSFER_STATUS_COMPLETED ||
         xfer->status == USB_TRANSFER_STATUS_TIMED_OUT)) {
        usb_host_transfer_submit(xfer);
    }
}

/* ── best-effort CDC SET_CONTROL_LINE_STATE (DTR|RTS) on EP0 ─────────────── */
static void set_dtr_rts(uint8_t comm_intf)
{
    usb_transfer_t *ctrl = NULL;
    if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t), 0, &ctrl) != ESP_OK) return;
    usb_setup_packet_t *s = (usb_setup_packet_t *)ctrl->data_buffer;
    s->bmRequestType = 0x21;   /* host→device | class | interface */
    s->bRequest      = 0x22;   /* SET_CONTROL_LINE_STATE          */
    s->wValue        = 0x0003; /* DTR | RTS                       */
    s->wIndex        = comm_intf;
    s->wLength       = 0;
    ctrl->device_handle   = s_dev;
    ctrl->bEndpointAddress = 0;
    ctrl->num_bytes       = sizeof(usb_setup_packet_t);
    ctrl->callback        = NULL;
    ctrl->context         = NULL;
    if (usb_host_transfer_submit_control(s_client, ctrl) != ESP_OK)
        usb_host_transfer_free(ctrl);
    /* Note: control transfer completes asynchronously; we intentionally leak this
     * tiny one-shot transfer rather than track it (kept simple, best-effort). */
}

/* ── open a freshly-connected device: find bulk-IN, claim, start streaming ── */
static void open_device(uint8_t addr)
{
    if (s_dev) return;
    if (usb_host_device_open(s_client, addr, &s_dev) != ESP_OK) { s_dev = NULL; return; }

    const usb_config_desc_t *cfg = NULL;
    if (usb_host_get_active_config_descriptor(s_dev, &cfg) != ESP_OK || !cfg) {
        usb_host_device_close(s_client, s_dev); s_dev = NULL; return;
    }

    /* Walk the config descriptor for the first bulk-IN endpoint + its interface,
     * and the first Communications-class interface (for the DTR control request). */
    const uint8_t *p = (const uint8_t *)cfg;
    int total = cfg->wTotalLength, off = 0, cur_intf = -1, comm_intf = 0;
    uint8_t in_ep = 0; uint16_t in_mps = 64; int claim_intf = -1;
    while (off + 2 <= total) {
        uint8_t bLength = p[off];
        uint8_t bType   = p[off + 1];
        if (bLength == 0) break;
        if (bType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *id = (const usb_intf_desc_t *)(p + off);
            cur_intf = id->bInterfaceNumber;
            if (id->bInterfaceClass == 0x02) comm_intf = id->bInterfaceNumber;  /* CDC-Comm */
        } else if (bType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)(p + off);
            bool is_in   = (ep->bEndpointAddress & 0x80) != 0;
            bool is_bulk = (ep->bmAttributes & 0x03) == 0x02;
            if (is_in && is_bulk && claim_intf < 0) {
                in_ep = ep->bEndpointAddress;
                in_mps = ep->wMaxPacketSize;
                claim_intf = cur_intf;
            }
        }
        off += bLength;
    }

    if (claim_intf < 0) {
        Serial.println("[CDC] no bulk-IN endpoint — not a CDC-style device");
        usb_host_device_close(s_client, s_dev); s_dev = NULL; return;
    }
    if (usb_host_interface_claim(s_client, s_dev, claim_intf, 0) != ESP_OK) {
        Serial.printf("[CDC] interface %d claim failed\n", claim_intf);
        usb_host_device_close(s_client, s_dev); s_dev = NULL; return;
    }
    s_claimed_intf = claim_intf;
    s_in_ep = in_ep;

    if (usb_host_transfer_alloc(in_mps, 0, &s_in_xfer) != ESP_OK) {
        usb_host_interface_release(s_client, s_dev, s_claimed_intf); s_claimed_intf = -1;
        usb_host_device_close(s_client, s_dev); s_dev = NULL; return;
    }
    s_in_xfer->device_handle    = s_dev;
    s_in_xfer->bEndpointAddress = s_in_ep;
    s_in_xfer->callback         = in_xfer_cb;
    s_in_xfer->context          = NULL;
    s_in_xfer->num_bytes        = in_mps;
    s_in_xfer->timeout_ms       = 0;

    s_connected = true;
    set_dtr_rts((uint8_t)comm_intf);            /* best-effort; harmless if ignored */
    usb_host_transfer_submit(s_in_xfer);
    Serial.printf("[CDC] streaming from intf %d, EP 0x%02X (mps %u)\n",
                  claim_intf, in_ep, (unsigned)in_mps);
}

static void close_device(void)
{
    s_connected = false;
    if (s_in_xfer)  { usb_host_transfer_free(s_in_xfer); s_in_xfer = NULL; }
    if (s_dev) {
        if (s_claimed_intf >= 0) usb_host_interface_release(s_client, s_dev, s_claimed_intf);
        usb_host_device_close(s_client, s_dev);
    }
    s_claimed_intf = -1; s_dev = NULL; s_in_ep = 0;
}

/* ── USB client event callback (runs on the client task) ─────────────────── */
static void client_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    (void)arg;
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        s_pending_addr = msg->new_dev.address;
    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        close_device();
        Serial.println("[CDC] scanner disconnected");
    }
}

/* ── tasks ───────────────────────────────────────────────────────────────── */
static void usb_lib_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

static void usb_client_task(void *arg)
{
    (void)arg;
    for (;;) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(50));
        uint8_t addr = s_pending_addr;
        if (addr && !s_dev) { s_pending_addr = 0; open_device(addr); }
    }
}

/* ── API ─────────────────────────────────────────────────────────────────── */
void usb_cdc_scanner_init(void)
{
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    if (usb_host_install(&host_config) != ESP_OK) {
        Serial.println("[CDC] usb_host_install FAILED");
        return;
    }
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 5, NULL, 0);

    usb_host_client_config_t ccfg = {};
    ccfg.is_synchronous = false;
    ccfg.max_num_event_msg = 5;
    ccfg.async.client_event_callback = client_cb;
    ccfg.async.callback_arg = NULL;
    if (usb_host_client_register(&ccfg, &s_client) != ESP_OK) {
        Serial.println("[CDC] usb_host_client_register FAILED");
        return;
    }
    xTaskCreatePinnedToCore(usb_client_task, "cdc_scan", 4096, NULL, 4, NULL, 0);
    Serial.println("[CDC] USB CDC-ACM (raw-host) scanner started (USB_SEL must be LOW).");
}

bool usb_cdc_scanner_take(char *buf, size_t len)
{
    if (!buf || len == 0 || !s_done_ready) return false;
    portENTER_CRITICAL(&s_mux);
    strncpy(buf, s_done, len - 1);
    buf[len - 1] = '\0';
    s_done_ready = false;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

bool usb_cdc_scanner_connected(void)
{
    return s_connected;
}

#endif /* CONFIG_SCANNER_USB_CDC */
