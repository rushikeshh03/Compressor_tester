/******************************************************************************************************************************
 * @file    wifi_service.cpp
 * @brief   Factory-mode WiFi service UI + firmware updater (IoT device). See wifi_service.h.
 *
 * Hidden SoftAP + Arduino WebServer (built-in, synchronous). The firmware upload is streamed
 * straight into the bootloader's OTA path (ota_bl_web_flash_*), so a browser upload flashes app0
 * exactly the way a CAN flash does, then reboots into it.
 *
 * NOTE: bench/service tool on a hidden WPA2 AP — the login is a simple cookie gate
 * (CONFIG_FACTORY_WEB_USER/PASS), not hardened auth.
 ******************************************************************************************************************************/

#include "main.h"              /* IOT_VERSION, CAN_BAUD_NVS_*, CONFIG_CAN_BAUD_KBPS, mqtt.h (WiFi cfg), ota_bl_* */
#include "wifi_service.h"      /* extern "C" prototypes — must be in scope so the defs get C linkage */

#if CONFIG_FACTORY_WIFI

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "wifi_mqtt.h"        /* CONFIG_STA_WIFI_SSID default for the settings form */
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define WSVC_LOG(fmt, ...) Serial.printf("[WIFI] " fmt "\n", ##__VA_ARGS__)

static WebServer  s_server(80);
static bool       s_started   = false;
static String     s_sid;                 /* session token; empty = logged out  */
static bool       s_flash_ok  = false;   /* result of the last web flash        */
static String     s_flash_msg = "";

/* ── NVS: CAN baud, in the SAME store the app + factory read (iot_cfg/can_baud) ── */
static uint32_t nvs_get_baud(void)
{
    Preferences p; uint32_t v = CONFIG_CAN_BAUD_KBPS;
    if (p.begin(CAN_BAUD_NVS_NS, true)) { v = p.getUInt(CAN_BAUD_NVS_KEY, CONFIG_CAN_BAUD_KBPS); p.end(); }
    if (v != 125 && v != 250 && v != 500) v = CONFIG_CAN_BAUD_KBPS;
    return v;
}
static void nvs_set_baud(uint32_t baud)
{
    if (baud != 125 && baud != 250 && baud != 500) return;
    Preferences p;
    if (!p.begin(CAN_BAUD_NVS_NS, false)) { WSVC_LOG("NVS open (rw) failed"); return; }
    p.putUInt(CAN_BAUD_NVS_KEY, baud);
    p.end();
}

/* ── NVS: primary WiFi STA credentials (iot_cfg/sta_ssid,sta_pass) — slot 1 in wifi_mqtt.cpp ── */
static String nvs_get_wifi_ssid(void)
{
    Preferences p; String v = CONFIG_STA_WIFI_SSID;
    if (p.begin(CAN_BAUD_NVS_NS, true)) { v = p.getString("sta_ssid", CONFIG_STA_WIFI_SSID); p.end(); }
    return v;
}
static void nvs_set_wifi_ssid(const String &s)
{
    Preferences p;
    if (!p.begin(CAN_BAUD_NVS_NS, false)) { WSVC_LOG("NVS open (rw) failed"); return; }
    p.putString("sta_ssid", s);
    p.end();
}
static void nvs_set_wifi_pass(const String &s)
{
    Preferences p;
    if (!p.begin(CAN_BAUD_NVS_NS, false)) { WSVC_LOG("NVS open (rw) failed"); return; }
    p.putString("sta_pass", s);
    p.end();
}

/* ── Auth helpers ─────────────────────────────────────────────────────────── */
static bool authed(void)
{
    if (s_sid.isEmpty()) return false;
    if (!s_server.hasHeader("Cookie")) return false;
    String c = s_server.header("Cookie");
    int i = c.indexOf("SID=");
    if (i < 0) return false;
    String tok = c.substring(i + 4);
    int semi = tok.indexOf(';');
    if (semi >= 0) tok = tok.substring(0, semi);
    return tok == s_sid;
}

static const char *backup_str(void)
{
    switch (ota_bl_backup_status()) {
        case OTA_BL_BACKUP_PASS:    return "PASS";
        case OTA_BL_BACKUP_FAIL:    return "FAILED";
        case OTA_BL_BACKUP_SKIPPED: return "skipped (not entered from app0)";
        default:                    return "pending";
    }
}
static const char *running_part(void)
{
    const esp_partition_t *r = esp_ota_get_running_partition();
    return r ? r->label : "?";
}

/* ── Pages ────────────────────────────────────────────────────────────────── */
static const char *PAGE_HEAD =
    "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>ESP32 IoT Service</title><style>"
    "body{font-family:sans-serif;background:#0d1117;color:#e6edf3;margin:0;padding:16px}"
    "h1{font-size:1.2rem}h2{font-size:1rem;color:#7d8590;margin-top:24px}"
    ".card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin:12px 0;max-width:560px}"
    "table{width:100%;border-collapse:collapse}td{padding:4px 6px;border-bottom:1px solid #21262d}"
    ".k{color:#7d8590}input,button{font-size:1rem;padding:8px;margin:4px 0;border-radius:6px;border:1px solid #30363d;background:#0d1117;color:#e6edf3}"
    "button{background:#238636;border:0;cursor:pointer}button.r{background:#da3633}.ok{color:#3fb950}.bad{color:#f85149}"
    "</style></head><body>";
static const char *PAGE_FOOT = "</body></html>";

static void send_login(const char *err = nullptr)
{
    String h = PAGE_HEAD;
    h += "<h1>ESP32 IoT — Service Login</h1><div class=card><form method=POST action=/login>";
    if (err) { h += "<p class=bad>"; h += err; h += "</p>"; }
    h += "<p>Username<br><input name=username autocomplete=off></p>";
    h += "<p>Password<br><input name=password type=password></p>";
    h += "<button type=submit>Log in</button></form></div>";
    h += PAGE_FOOT;
    s_server.send(200, "text/html", h);
}

static void send_dashboard(void)
{
    uint32_t baud = nvs_get_baud();
    String h = PAGE_HEAD;
    h += "<h1>ESP32 IoT — Factory Service</h1>";

    /* Status card */
    h += "<div class=card><h2>Status</h2><table>";
    h += "<tr><td class=k>IoT firmware</td><td>" IOT_VERSION "</td></tr>";
    h += "<tr><td class=k>Running partition</td><td>"; h += running_part(); h += "</td></tr>";
    h += "<tr><td class=k>app0&rarr;app1 backup</td><td>";
    { int b = ota_bl_backup_status();
      h += (b == OTA_BL_BACKUP_FAIL) ? "<span class=bad>" : "<span class=ok>";
      h += backup_str(); h += "</span>"; }
    h += "</td></tr></table></div>";

    /* Last flash result */
    if (!s_flash_msg.isEmpty()) {
        h += "<div class=card><h2>Last firmware upload</h2><p class=";
        h += s_flash_ok ? "ok>" : "bad>"; h += s_flash_msg; h += "</p></div>";
    }

    /* Firmware upload */
    h += "<div class=card><h2>Update firmware (app0)</h2>"
         "<form method=POST action=/update enctype='multipart/form-data'>"
         "<input type=file name=fw accept='.bin'><br>"
         "<button type=submit>Upload &amp; flash</button>"
         "<p class=k>Flashes app0 via the OTA path and reboots into it.</p>"
         "</form></div>";

    /* CAN baud + primary WiFi */
    h += "<div class=card><h2>Settings</h2><form method=POST action=/config>";
    h += "<p class=k>CAN baud (applies next boot)</p>";
    for (uint32_t b : {125u, 250u, 500u}) {
        h += "<label><input type=radio name=baud value="; h += b;
        if (b == baud) h += " checked"; h += "> "; h += b; h += "k</label> &nbsp; ";
    }
    h += "<p class=k>Primary WiFi (applies next boot; two backup networks are compiled in)</p>";
    h += "SSID<br><input name=wifi_ssid value='"; h += nvs_get_wifi_ssid(); h += "'><br>";
    h += "Password<br><input name=wifi_pass type=password placeholder='(leave blank to keep current)'><br>";
    h += "<br><button type=submit>Save settings</button></form></div>";

    /* Reboot */
    h += "<div class=card><h2>Reboot</h2>"
         "<form method=POST action=/reboot onsubmit=\"return confirm('Reboot the device into the app?')\">"
         "<button class=r type=submit>Reboot device</button>"
         "<p class=k>Exits factory mode, boots the app, and closes WiFi.</p></form></div>";

    h += "<form method=POST action=/logout><button>Log out</button></form>";
    h += PAGE_FOOT;
    s_server.send(200, "text/html", h);
}

/* ── Handlers ─────────────────────────────────────────────────────────────── */
static void h_root(void)     { if (authed()) send_dashboard(); else send_login(); }

static void h_login(void)
{
    if (s_server.arg("username") == CONFIG_FACTORY_WEB_USER &&
        s_server.arg("password") == CONFIG_FACTORY_WEB_PASS) {
        s_sid = String((uint32_t)esp_random(), HEX) + String((uint32_t)esp_random(), HEX);
        s_server.sendHeader("Set-Cookie", "SID=" + s_sid + "; Path=/");
        s_server.sendHeader("Location", "/");
        s_server.send(303);
    } else {
        send_login("Invalid credentials");
    }
}

static void h_logout(void)
{
    s_sid = "";
    s_server.sendHeader("Set-Cookie", "SID=; Path=/; Max-Age=0");
    s_server.sendHeader("Location", "/");
    s_server.send(303);
}

static void h_config(void)
{
    if (!authed()) { s_server.send(403, "text/plain", "forbidden"); return; }
    uint32_t baud = s_server.arg("baud").toInt();
    nvs_set_baud(baud);

    /* Primary WiFi: SSID (if given) + password (only if given — blank keeps the current one). */
    String ssid = s_server.arg("wifi_ssid"); ssid.trim();
    if (ssid.length()) nvs_set_wifi_ssid(ssid);
    String pass = s_server.arg("wifi_pass");
    if (pass.length()) nvs_set_wifi_pass(pass);
    WSVC_LOG("config saved: baud=%u wifi_ssid=%s pass=%s",
             baud, ssid.c_str(), pass.length() ? "(updated)" : "(unchanged)");
    s_server.sendHeader("Location", "/");
    s_server.send(303);
}

static void h_reboot(void)
{
    if (!authed()) { s_server.send(403, "text/plain", "forbidden"); return; }
    String h = PAGE_HEAD;
    h += "<h1>Rebooting…</h1><div class=card><p>The device is rebooting into the app. "
         "WiFi will close.</p></div>"; h += PAGE_FOOT;
    s_server.send(200, "text/html", h);
    delay(600);
    ota_bl_reboot_to_app();   /* stops WiFi + boots app; never returns */
}

/* Firmware upload: main handler sends the result; the upload callback streams the bytes into the
 * bootloader's OTA path. */
static bool s_upload_authed = false;

static void h_update_done(void)
{
    if (!authed()) { s_server.send(403, "text/plain", "forbidden"); return; }
    String h = PAGE_HEAD;
    if (s_flash_ok) {
        h += "<h1 class=ok>Update OK</h1><div class=card><p>Wrote ";
        h += String(ota_bl_web_flash_bytes()); h += " bytes to app0. Rebooting…</p></div>";
    } else {
        h += "<h1 class=bad>Update FAILED</h1><div class=card><p>"; h += s_flash_msg;
        h += "</p><p><a href=/ style=color:#58a6ff>Back</a></p></div>";
    }
    h += PAGE_FOOT;
    s_server.send(200, "text/html", h);
    if (s_flash_ok) { delay(1200); ota_bl_reboot_to_app(); }
}

static void h_update_upload(void)
{
    HTTPUpload &up = s_server.upload();
    if (up.status == UPLOAD_FILE_START) {
        s_upload_authed = authed();
        s_flash_ok  = false;
        s_flash_msg = "";
        if (!s_upload_authed) { s_flash_msg = "not logged in"; return; }
        WSVC_LOG("upload start: %s", up.filename.c_str());
        if (!ota_bl_web_flash_begin()) {
            s_upload_authed = false; s_flash_msg = "flash begin failed (CAN flash active?)"; return;
        }
    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (!s_upload_authed) return;
        if (!ota_bl_web_flash_write(up.buf, up.currentSize)) {
            s_upload_authed = false; s_flash_msg = "write failed (bad image?)";
        }
    } else if (up.status == UPLOAD_FILE_END) {
        if (!s_upload_authed) { ota_bl_web_flash_abort(); return; }
        s_flash_ok  = ota_bl_web_flash_end();
        s_flash_msg = s_flash_ok ? "flash + verify OK" : "verify/end failed";
        WSVC_LOG("upload end: %s (%u B)", s_flash_ok ? "OK" : "FAIL", ota_bl_web_flash_bytes());
    } else if (up.status == UPLOAD_FILE_ABORTED) {
        ota_bl_web_flash_abort();
        s_upload_authed = false; s_flash_ok = false; s_flash_msg = "upload aborted";
    }
}

/* ── Service task ─────────────────────────────────────────────────────────── */
static void wifi_service_task(void *arg)
{
    (void)arg;
    for (;;) { s_server.handleClient(); vTaskDelay(pdMS_TO_TICKS(2)); }
}

void wifi_service_start(void)
{
    if (s_started) return;

    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(CONFIG_FACTORY_WIFI_SSID, CONFIG_FACTORY_WIFI_PASS,
                          CONFIG_FACTORY_WIFI_CHANNEL, CONFIG_FACTORY_WIFI_HIDDEN);
    WSVC_LOG("SoftAP '%s' (hidden=%d) %s, IP=%s", CONFIG_FACTORY_WIFI_SSID,
             CONFIG_FACTORY_WIFI_HIDDEN, ok ? "up" : "FAILED",
             WiFi.softAPIP().toString().c_str());
    if (!ok) { WiFi.mode(WIFI_OFF); return; }   /* AP failed → don't serve on a dead AP */

    static const char *hdrs[] = { "Cookie" };
    s_server.collectHeaders(hdrs, 1);

    s_server.on("/",       HTTP_GET,  h_root);
    s_server.on("/login",  HTTP_POST, h_login);
    s_server.on("/logout", HTTP_POST, h_logout);
    s_server.on("/config", HTTP_POST, h_config);
    s_server.on("/reboot", HTTP_POST, h_reboot);
    s_server.on("/update", HTTP_POST, h_update_done, h_update_upload);
    s_server.onNotFound([]() { s_server.sendHeader("Location", "/"); s_server.send(303); });
    s_server.begin();

    xTaskCreatePinnedToCore(wifi_service_task, "wifi_svc", 8192, nullptr, 2, nullptr, 0);
    s_started = true;
    WSVC_LOG("service UI ready at http://%s/", WiFi.softAPIP().toString().c_str());
}

void wifi_service_stop(void)
{
    if (!s_started) return;
    s_server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    s_started = false;
    WSVC_LOG("service stopped, WiFi off");
}

#else  /* CONFIG_FACTORY_WIFI == 0 — compile to no-ops */
void wifi_service_start(void) {}
void wifi_service_stop(void)  {}
#endif
