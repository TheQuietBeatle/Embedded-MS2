// web_control.c - Web page that controls the Neural Calibrator arm (Pico W, Pico SDK).
//
// Separate program from the MS2 driver test (main.c); both build from this folder.
//   Power-up : every joint homes to 90 deg, one at a time.
//   Wi-Fi    : joins WIFI_SSID (see wifi_config.example.h). If that fails twice (or
//              WIFI_SSID is ""), the Pico starts its OWN network "NeuralCalibrator"
//              (password robotarm123) - join it and open http://192.168.4.1.
//              The address is printed over USB serial; the onboard LED turns on
//              once the network is up.
//   Web page : live grip-pressure indicator (FSR) with the hold zone and safety
//              limit marked, mic test (sound level meter with peak marker), slider + buttons per joint, gripper Open / Close-step,
//              Home all, reward motor Forward / Reverse / Stop.
//   Motion   : non-blocking, at most STEP_DEG every STEP_MS, and only ONE joint moves
//              at a time so a 5 V 2 A adapter is not overloaded.
//
// Concurrency: lwIP callbacks (background interrupt) only record targets and
// requests; the main loop is the only place that drives the servos and motor.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/ip4_addr.h"

#include "pins.h"
#include "servo.h"
#include "motor.h"
#include "fsr.h"
#include "mic.h"
#include "dhcp_server.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#include "wifi_config.example.h"
#warning "wifi_config.h not found - copy wifi_config.example.h to wifi_config.h and set your Wi-Fi"
#endif

// ---- Tunable settings -------------------------------------------------------
#define HOME_ANGLE       90.0f
#define HOME_GAP_MS      400u
#define JOINT_MIN        5.0f
#define JOINT_MAX        175.0f
#define GRIPPER_OPEN     0.0f
#define GRIPPER_CLOSED   120.0f
#define GRIPPER_STEP     2
#define NUDGE_DEG        5
#define STEP_DEG         2.0f
#define STEP_MS          20u
#define MOTOR_MAX_MS     5000u     // a web motor command never runs longer than this

// Grip pressure indicator (FSR, 0-1000 units from fsr_read_units())
#define FSR_ZONE_LO      175       // hold zone, shown green on the page
#define FSR_ZONE_HI      185
#define FSR_SAFETY       400       // shown red: the gripper should back off above this
#define FSR_BAR_MAX      500       // full width of the page's bar
#define FSR_READ_MS      50u       // how often the main loop samples the FSR

// Mic test (INMP441 sound level meter on the page)
#define MIC_READ_MS      60u       // how often the main loop measures loudness
#define MIC_BURST        160u      // samples per measurement = 10 ms at 16 kHz (keeps servos smooth)
#define MIC_QUIET        500u      // level shown as an empty bar   (tune from what the page shows)
#define MIC_LOUD         60000u    // level shown as a full bar
#define HTTP_PORT        80
#define WIFI_JOIN_ATTEMPTS 2       // WPA2 + WPA2/WPA3, ~40 s, then own network

// The Pico's own network, used when WIFI_SSID can't be joined (or is "").
// Override in wifi_config.h if you like.
#ifndef AP_SSID
#define AP_SSID          "NeuralCalibrator"
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD      "robotarm123"      // at least 8 characters
#endif

// ---- Joints -------------------------------------------------------------------
enum { WAIST, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROLL, GRIPPER, NUM_JOINTS };

typedef struct {
    const char *key;
    const char *label;
    uint8_t     pin;
    float       lo, hi;
} joint_info_t;

static const joint_info_t joints[NUM_JOINTS] = {
    {"waist",       "Waist (MG996R, GP0)",      PIN_SERVO_WAIST,       JOINT_MIN,    JOINT_MAX},
    {"shoulder",    "Shoulder (MG996R, GP1)",   PIN_SERVO_SHOULDER,    JOINT_MIN,    JOINT_MAX},
    {"elbow",       "Elbow (MG996R, GP4)",      PIN_SERVO_ELBOW,       JOINT_MIN,    JOINT_MAX},
    {"wrist_pitch", "Wrist pitch (SG90, GP5)",  PIN_SERVO_WRIST_PITCH, JOINT_MIN,    JOINT_MAX},
    {"wrist_roll",  "Wrist roll (SG90, GP14)",  PIN_SERVO_WRIST_ROLL,  JOINT_MIN,    JOINT_MAX},
    {"gripper",     "Gripper (SG90, GP15)",     PIN_SERVO_GRIPPER,     GRIPPER_OPEN, GRIPPER_CLOSED},
};

static servo_t        servos[NUM_JOINTS];
static volatile float target[NUM_JOINTS];     // written by web requests, read by main loop

// Reward motor request from the web page (applied by the main loop)
static volatile bool     motor_req_pending;
static volatile int8_t   motor_req_speed;
static volatile uint32_t motor_req_ms;

// Latest FSR reading, sampled by the main loop (the ADC is only touched there)
static volatile uint16_t fsr_units;
static volatile uint16_t fsr_raw;

// Latest mic loudness, measured by the main loop
static bool              mic_ok;
static volatile uint32_t mic_level;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static int joint_by_key(const char *key) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (strcmp(joints[i].key, key) == 0) return i;
    }
    return -1;
}

static void set_target(int j, float deg) {
    target[j] = clampf(deg, joints[j].lo, joints[j].hi);
}

// ---- Web page -----------------------------------------------------------------
static const char PAGE[] =
"<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>Neural Calibrator Arm</title>"
"<style>"
"body{font-family:system-ui,sans-serif;background:#111;color:#eee;max-width:520px;margin:auto;padding:16px}"
".card{background:#1c1f26;border-radius:12px;padding:14px 16px;margin-bottom:10px}"
".row{display:flex;justify-content:space-between;align-items:baseline}"
".lbl{font-weight:600}.ang{font-size:26px;font-weight:bold;font-variant-numeric:tabular-nums}"
".btns{display:flex;gap:8px;margin-top:8px}"
"button{flex:1;padding:12px;font-size:15px;border:0;border-radius:10px;background:#2d3340;color:#eee}"
"button:active{background:#4f9cff}.stop{background:#4a2a2e}"
"input{width:100%;margin-top:10px;accent-color:#4f9cff}"
".bar{position:relative;height:20px;background:#2d3340;border-radius:10px;margin:10px 0 6px;overflow:hidden}"
".fill{position:absolute;left:0;top:0;bottom:0;width:0;background:#4f9cff;transition:width .2s,background .2s}"
".zone{position:absolute;top:0;bottom:0;background:#3ecf8e;opacity:.75;min-width:3px}"
".safe{position:absolute;top:0;bottom:0;right:0;background:#ff5d5d;opacity:.35}"
".st{font-weight:600}.meta{color:#888;font-size:13px}"
".peak{position:absolute;top:0;bottom:0;width:3px;background:#fff;opacity:.8;left:0}"
"#msg{color:#888;font-size:13px;margin-bottom:12px}"
"</style></head><body>"
"<h2>Neural Calibrator Arm</h2><div id=\"msg\">connecting...</div>"
"<div class=\"card\"><div class=\"row\"><span class=\"lbl\">Grip pressure (FSR)</span>"
"<span class=\"ang\" id=\"fsr\">--</span></div>"
"<div class=\"bar\"><div id=\"fsrbar\" class=\"fill\"></div><div id=\"zone\" class=\"zone\"></div>"
"<div id=\"safe\" class=\"safe\"></div></div>"
"<div class=\"row\"><span id=\"fsrst\" class=\"st\">--</span><span id=\"fsrinfo\" class=\"meta\"></span></div></div>"
"<div class=\"card\"><div class=\"row\"><span class=\"lbl\">Mic test (INMP441)</span>"
"<span class=\"ang\" id=\"mic\">--</span></div>"
"<div class=\"bar\"><div id=\"micbar\" class=\"fill\"></div><div id=\"micpk\" class=\"peak\"></div></div>"
"<div class=\"row\"><span id=\"micst\" class=\"st\">--</span><span class=\"meta\">talk or clap near the mic</span></div></div>"
"<div id=\"joints\"></div>"
"<div class=\"btns\" style=\"margin-bottom:10px\"><button onclick=\"go('/home')\">Home all (90&deg;)</button></div>"
"<div class=\"card\"><div class=\"row\"><span class=\"lbl\">Reward motor (L298N)</span>"
"<span class=\"ang\"><span id=\"mot\">0</span>%</span></div>"
"<div class=\"btns\"><button onclick=\"go('/motor?speed=70&ms=2000')\">Forward 2 s</button>"
"<button onclick=\"go('/motor?speed=-70&ms=2000')\">Reverse 2 s</button>"
"<button class=\"stop\" onclick=\"go('/motor?speed=0&ms=0')\">Stop</button></div></div>"
"<script>"
"const $=id=>document.getElementById(id);let built=false,drag=null;"
"function btn(t,p){return '<button onclick=\"go(\\''+p+'\\')\">'+t+'</button>'}"
"function build(d){let h='';for(const s of d.servos){const k=s.key;"
"h+='<div class=\"card\"><div class=\"row\"><span class=\"lbl\">'+s.label+'</span>'+"
"'<span class=\"ang\"><span id=\"v_'+k+'\"></span>&deg;</span></div>'+"
"'<input id=\"s_'+k+'\" type=\"range\" min=\"'+s.lo+'\" max=\"'+s.hi+'\" '+"
"'onpointerdown=\"drag=\\''+k+'\\'\" onchange=\"drag=null;go(\\'/set?servo='+k+'&angle=\\'+this.value)\">'+"
"'<div class=\"btns\">'+(k==='gripper'"
"?btn('Open','/set?servo=gripper&angle='+s.lo)+btn('Close step','/step?servo=gripper&delta='+d.gripper_step)+btn('Closed','/set?servo=gripper&angle='+s.hi)"
":btn('&minus;'+d.nudge,'/step?servo='+k+'&delta=-'+d.nudge)+btn('90&deg;','/set?servo='+k+'&angle=90')+btn('+'+d.nudge,'/step?servo='+k+'&delta='+d.nudge))+"
"'</div></div>'}$('joints').innerHTML=h;built=true}"
"function show(d){if(!built)build(d);for(const s of d.servos){$('v_'+s.key).textContent=s.angle;"
"if(drag!==s.key)$('s_'+s.key).value=s.target}$('mot').textContent=d.motor;"
"$('msg').textContent=d.moving?'moving...':'connected';if(d.fsr)fsr(d.fsr);if(d.mic)mic(d.mic)}"
"let pk=0;function mic(m){if(!m.ok){$('mic').textContent='--';$('micst').textContent='Mic not detected - check wiring';"
"$('micst').style.color='#ff5d5d';return}"
"const r=Math.log10(Math.max(m.level,m.quiet)/m.quiet)/Math.log10(m.loud/m.quiet),p=Math.min(1,Math.max(0,r))*100;"
"pk=Math.max(p,pk-4);$('mic').textContent=m.level;$('micbar').style.width=p+'%';$('micpk').style.left=pk+'%';"
"const t=p<5?'Quiet':p<60?'Sound detected':'Loud',c=p<5?'#888':p<60?'#3ecf8e':'#f5b942';"
"$('micst').textContent=t;$('micst').style.color=c;$('micbar').style.background=c}"
"function pct(v,f){return Math.min(100,Math.max(0,v*100/f.scale))+'%'}"
"function fsr(f){const u=f.units;$('fsr').textContent=u;$('fsrbar').style.width=pct(u,f);"
"$('zone').style.left=pct(f.zone_lo,f);$('zone').style.width=pct(f.zone_hi-f.zone_lo,f);"
"$('safe').style.left=pct(f.safety,f);"
"let t,c;if(u>=f.safety){t='SAFETY LIMIT - open the gripper';c='#ff5d5d'}"
"else if(u>f.zone_hi){t='Too hard';c='#f5b942'}else if(u>=f.zone_lo){t='IN ZONE';c='#3ecf8e'}"
"else{t=u<5?'No pressure':'Too light';c='#4f9cff'}"
"$('fsrst').textContent=t;$('fsrst').style.color=c;$('fsrbar').style.background=c;"
"$('fsrinfo').textContent='zone '+f.zone_lo+'-'+f.zone_hi+' | raw ADC '+f.raw}"
"async function go(p){try{show(await (await fetch(p)).json())}catch(e){$('msg').textContent='no response from Pico'}}"
"go('/state');setInterval(()=>go('/state'),300);"
"</script></body></html>";

// ---- HTTP server (lwIP raw TCP API) ---------------------------------------------
typedef struct {
    struct tcp_pcb *pcb;
    bool        responded;
    char        head[192];
    size_t      head_len, head_sent;
    const char *body;
    size_t      body_len, body_sent;
    char        json[1536];
} http_conn_t;

static size_t state_json(char *out, size_t cap) {
    size_t n = 0;
    bool moving = false;
    n += (size_t)snprintf(out + n, cap - n, "{\"servos\":[");
    for (int i = 0; i < NUM_JOINTS && n < cap; i++) {
        float a = servo_read(&servos[i]);
        if (a != target[i]) moving = true;
        n += (size_t)snprintf(out + n, cap - n,
                "%s{\"key\":\"%s\",\"label\":\"%s\",\"angle\":%d,\"target\":%d,\"lo\":%d,\"hi\":%d}",
                i ? "," : "", joints[i].key, joints[i].label,
                (int)(a + 0.5f), (int)(target[i] + 0.5f), (int)joints[i].lo, (int)joints[i].hi);
    }
    if (n < cap) {
        n += (size_t)snprintf(out + n, cap - n,
                "],\"motor\":%d,\"moving\":%s,\"nudge\":%d,\"gripper_step\":%d,"
                "\"fsr\":{\"units\":%u,\"raw\":%u,\"zone_lo\":%d,\"zone_hi\":%d,\"safety\":%d,\"scale\":%d},"
                "\"mic\":{\"ok\":%s,\"level\":%lu,\"quiet\":%u,\"loud\":%u}}",
                motor_get(), moving ? "true" : "false", NUDGE_DEG, GRIPPER_STEP,
                (unsigned)fsr_units, (unsigned)fsr_raw, FSR_ZONE_LO, FSR_ZONE_HI, FSR_SAFETY, FSR_BAR_MAX,
                mic_ok ? "true" : "false", (unsigned long)mic_level, MIC_QUIET, MIC_LOUD);
    }
    return n < cap ? n : cap - 1;
}

// Copies the value of `name` from a query string like "servo=elbow&angle=90".
static bool query_get(const char *query, const char *name, char *out, size_t cap) {
    size_t len = strlen(name);
    const char *p = query;
    while (p && *p) {
        if (strncmp(p, name, len) == 0 && p[len] == '=') {
            p += len + 1;
            size_t i = 0;
            while (*p && *p != '&' && *p != ' ' && i + 1 < cap) out[i++] = *p++;
            out[i] = '\0';
            return true;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    return false;
}

static void handle_request(http_conn_t *c, const char *req) {
    char path[128] = "";
    // "GET /set?servo=elbow&angle=90 HTTP/1.1"
    const char *sp = strchr(req, ' ');
    if (sp) {
        size_t i = 0;
        sp++;
        while (*sp && *sp != ' ' && i + 1 < sizeof path) path[i++] = *sp++;
        path[i] = '\0';
    }
    char *query = strchr(path, '?');
    if (query) *query++ = '\0';

    char key[16], val[16];
    int j = (query && query_get(query, "servo", key, sizeof key)) ? joint_by_key(key) : -1;

    if (strcmp(path, "/") == 0) {
        c->body = PAGE;
        c->body_len = sizeof PAGE - 1;
        c->head_len = (size_t)snprintf(c->head, sizeof c->head,
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %u\r\n"
            "Connection: close\r\n\r\n", (unsigned)c->body_len);
        return;
    }

    if (strcmp(path, "/set") == 0 && j >= 0 && query_get(query, "angle", val, sizeof val)) {
        set_target(j, (float)atoi(val));
    } else if (strcmp(path, "/step") == 0 && j >= 0 && query_get(query, "delta", val, sizeof val)) {
        set_target(j, target[j] + (float)atoi(val));
    } else if (strcmp(path, "/home") == 0) {
        for (int i = 0; i < NUM_JOINTS; i++) set_target(i, HOME_ANGLE);
    } else if (strcmp(path, "/motor") == 0 && query && query_get(query, "speed", val, sizeof val)) {
        int speed = atoi(val);
        int ms = query_get(query, "ms", val, sizeof val) ? atoi(val) : 0;
        if (speed > 100) speed = 100;
        if (speed < -100) speed = -100;
        if (ms < 0) ms = 0;
        if (ms > (int)MOTOR_MAX_MS) ms = (int)MOTOR_MAX_MS;
        motor_req_speed = (int8_t)speed;
        motor_req_ms = (uint32_t)ms;
        motor_req_pending = true;
    }
    // Every API call (including /state and unknown paths) answers with the current state.
    c->body = c->json;
    c->body_len = state_json(c->json, sizeof c->json);
    c->head_len = (size_t)snprintf(c->head, sizeof c->head,
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)c->body_len);
}

static void conn_free(http_conn_t *c) {
    if (c->pcb) {
        tcp_arg(c->pcb, NULL);
        tcp_recv(c->pcb, NULL);
        tcp_sent(c->pcb, NULL);
        tcp_err(c->pcb, NULL);
        tcp_poll(c->pcb, NULL, 0);
    }
    free(c);
}

static err_t conn_close(http_conn_t *c) {
    struct tcp_pcb *pcb = c->pcb;
    err_t result = ERR_OK;
    conn_free(c);
    if (pcb && tcp_close(pcb) != ERR_OK) {
        tcp_abort(pcb);
        result = ERR_ABRT;
    }
    return result;
}

static bool all_written(const http_conn_t *c) {
    return c->head_sent >= c->head_len && c->body_sent >= c->body_len;
}

// Queue as much of the response as the TCP send buffer allows.
static void send_more(http_conn_t *c) {
    while (!all_written(c)) {
        u16_t space = tcp_sndbuf(c->pcb);
        if (space == 0) break;
        bool in_head = c->head_sent < c->head_len;
        const char *src = in_head ? c->head + c->head_sent : c->body + c->body_sent;
        size_t left = in_head ? c->head_len - c->head_sent : c->body_len - c->body_sent;
        u16_t n = (u16_t)(left < space ? left : space);
        if (tcp_write(c->pcb, src, n, TCP_WRITE_FLAG_COPY) != ERR_OK) break;
        if (in_head) c->head_sent += n; else c->body_sent += n;
    }
    tcp_output(c->pcb);
}

static err_t on_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    (void)len;
    http_conn_t *c = arg;
    if (!c) return ERR_OK;
    send_more(c);
    // Close once everything is written and acknowledged.
    if (all_written(c) && tcp_sndbuf(pcb) == TCP_SND_BUF) {
        return conn_close(c);
    }
    return ERR_OK;
}

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    http_conn_t *c = arg;
    if (!c) {
        if (p) pbuf_free(p);
        return ERR_OK;
    }
    if (!p || err != ERR_OK) {            // client closed the connection
        if (p) pbuf_free(p);
        return conn_close(c);
    }
    tcp_recved(pcb, p->tot_len);
    if (!c->responded) {
        char req[256];
        u16_t n = pbuf_copy_partial(p, req, sizeof req - 1, 0);
        req[n] = '\0';
        c->responded = true;
        handle_request(c, req);
        send_more(c);
    }
    pbuf_free(p);
    return ERR_OK;
}

static void on_err(void *arg, err_t err) {
    (void)err;
    http_conn_t *c = arg;
    if (c) {
        c->pcb = NULL;                    // lwIP already freed the pcb
        free(c);
    }
}

static err_t on_poll(void *arg, struct tcp_pcb *pcb) {
    (void)pcb;
    http_conn_t *c = arg;
    return c ? conn_close(c) : ERR_OK;    // drop connections idle for ~5 s
}

static err_t on_accept(void *arg, struct tcp_pcb *pcb, err_t err) {
    (void)arg;
    if (err != ERR_OK || !pcb) return ERR_VAL;
    http_conn_t *c = calloc(1, sizeof *c);
    if (!c) {
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    c->pcb = pcb;
    tcp_arg(pcb, c);
    tcp_recv(pcb, on_recv);
    tcp_sent(pcb, on_sent);
    tcp_err(pcb, on_err);
    tcp_poll(pcb, on_poll, 10);           // poll interval is in 0.5 s units
    return ERR_OK;
}

static bool http_start(void) {
    cyw43_arch_lwip_begin();
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    bool ok = pcb && tcp_bind(pcb, IP_ANY_TYPE, HTTP_PORT) == ERR_OK;
    if (ok) {
        struct tcp_pcb *listener = tcp_listen_with_backlog(pcb, 4);
        ok = listener != NULL;
        if (ok) tcp_accept(listener, on_accept);
    }
    cyw43_arch_lwip_end();
    return ok;
}

// ---- Wi-Fi ----------------------------------------------------------------------
static const char *wifi_error_text(int err) {
    switch (err) {
    case PICO_ERROR_BADAUTH:        return "wrong password";
    case PICO_ERROR_TIMEOUT:        return "timed out - network not found (name typo? 5 GHz only? out of range?)";
    case PICO_ERROR_CONNECT_FAILED: return "connection refused by the network";
    default:                        return "unknown error";
    }
}

static const char *auth_name(uint8_t auth) {
    // Scan flags from the cyw43 driver: 4 = RSN (WPA2 or WPA3), 2 = WPA, 1 = WEP/privacy
    if (auth == 0)  return "open";
    if (auth & 0x4) return "WPA2/WPA3";
    if (auth & 0x2) return "WPA";
    return "WEP";
}

typedef struct {
    int  count;
    bool found;      // our WIFI_SSID was seen
} scan_summary_t;

static int scan_result(void *env, const cyw43_ev_scan_result_t *r) {
    scan_summary_t *sum = env;
    if (r && r->ssid_len > 0) {
        sum->count++;
        if (r->ssid_len == strlen(WIFI_SSID) && memcmp(r->ssid, WIFI_SSID, r->ssid_len) == 0) {
            sum->found = true;
        }
        printf("    %-32.*s  ch %2u  %4d dBm  %s\n", r->ssid_len, (const char *)r->ssid,
               r->channel, r->rssi, auth_name(r->auth_mode));
    }
    return 0;
}

// Print the 2.4 GHz networks the Pico W can see (it cannot see 5 GHz at all).
// Runs once at start-up, before any join attempt (a scan during a join stalls).
static void wifi_scan_print(void) {
    static scan_summary_t sum;
    cyw43_wifi_scan_options_t opts = {0};
    printf("Scanning for Wi-Fi networks (duplicates are normal):\n");
    cyw43_arch_lwip_begin();
    int err = cyw43_wifi_scan(&cyw43_state, &opts, &sum, scan_result);
    cyw43_arch_lwip_end();
    if (err != 0) {
        printf("    scan could not start (%d) - continuing\n", err);
        return;
    }
    absolute_time_t give_up = make_timeout_time_ms(10000);
    while (cyw43_wifi_scan_active(&cyw43_state) && !time_reached(give_up)) {
        sleep_ms(100);
    }
    printf("  %d networks seen.\n", sum.count);
    if (sum.found) {
        printf("  \"%s\" is visible - if joining fails, check the password.\n", WIFI_SSID);
    } else {
        printf("  \"%s\" was NOT seen: check the exact name (capitals, spaces, apostrophes),\n"
               "  switch the hotspot to 2.4 GHz (iPhone: Maximize Compatibility), or move closer.\n",
               WIFI_SSID);
    }
}

// Try to join WIFI_SSID: WPA2 first, then WPA2/WPA3 (for WPA3 phone hotspots).
// Returns false after WIFI_JOIN_ATTEMPTS failures so the caller can fall back
// to the Pico's own network.
static bool wifi_join(void) {
    static const uint32_t auth_modes[] = { CYW43_AUTH_WPA2_AES_PSK, CYW43_AUTH_WPA3_WPA2_AES_PSK };
    static const char *auth_names[]    = { "WPA2", "WPA2/WPA3" };

    if (strlen(WIFI_SSID) == 0) {
        printf("WIFI_SSID is empty - skipping straight to the Pico's own network.\n");
        return false;
    }
    wifi_scan_print();
    printf("Connecting to Wi-Fi \"%s\"...\n", WIFI_SSID);
    for (int attempt = 0; attempt < WIFI_JOIN_ATTEMPTS; attempt++) {
        int m = attempt % 2;
        int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, auth_modes[m], 20000);
        if (err == 0) {
            printf("Connected (%s).\n", auth_names[m]);
            return true;
        }
        printf("  %s attempt failed: %s (%d)\n", auth_names[m], wifi_error_text(err), err);
    }
    return false;
}

// Start the Pico's own Wi-Fi network with a DHCP server. Returns its interface.
static struct netif *start_own_network(void) {
    cyw43_arch_disable_sta_mode();
    cyw43_arch_enable_ap_mode(AP_SSID, AP_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
    struct netif *ap = &cyw43_state.netif[CYW43_ITF_AP];
    cyw43_arch_lwip_begin();
    bool ok = dhcp_server_start(ap);
    cyw43_arch_lwip_end();
    printf("Started own Wi-Fi network \"%s\" (password \"%s\")%s\n",
           AP_SSID, AP_PASSWORD, ok ? "" : " - DHCP server FAILED");
    return ap;
}

// ---- Main -----------------------------------------------------------------------
int main(void) {
    stdio_init_all();

    motor_init(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_ENA);   // motor off first
    fsr_init(PIN_FSR_ADC);
    for (int i = 0; i < NUM_JOINTS; i++) {
        servo_init(&servos[i], joints[i].pin, joints[i].lo, joints[i].hi);   // silent
    }

    // Wait up to 10 s for the serial monitor so the Wi-Fi scan below isn't missed.
    for (int i = 0; i < 100 && !stdio_usb_connected(); i++) {
        sleep_ms(100);
    }
    sleep_ms(500);
    printf("\n=== Neural Calibrator - web control ===\n");

    for (int i = 0; i < NUM_JOINTS; i++) {   // home one joint at a time
        printf("Homing %s\n", joints[i].key);
        servo_write(&servos[i], HOME_ANGLE);
        target[i] = servo_read(&servos[i]);
        sleep_ms(HOME_GAP_MS);
    }

    mic_ok = mic_init(PIN_MIC_SCK, PIN_MIC_WS, PIN_MIC_SD);
    printf("Mic: %s\n", mic_ok ? "I2S receiver running" : "INIT FAILED (check pins)");

    if (cyw43_arch_init()) {
        printf("Wi-Fi chip failed to start\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    struct netif *net = &cyw43_state.netif[CYW43_ITF_STA];
    bool own_network = !wifi_join();
    if (own_network) {
        printf("Could not join \"%s\" - switching to the Pico's own network.\n", WIFI_SSID);
        net = start_own_network();
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    if (!http_start()) {
        printf("Web server failed to start\n");
        return 1;
    }

    absolute_time_t next_step = make_timeout_time_ms(STEP_MS);
    absolute_time_t next_ip_print = get_absolute_time();
    absolute_time_t motor_stop_at = nil_time;
    absolute_time_t next_fsr = get_absolute_time();
    absolute_time_t next_mic = get_absolute_time();
    bool motor_timed = false;

    while (true) {
        // Remind the address every 10 s in case the serial monitor was opened late.
        if (time_reached(next_ip_print)) {
            if (own_network) {
                printf("Join Wi-Fi \"%s\" (password \"%s\"), then open http://%s\n",
                       AP_SSID, AP_PASSWORD, ip4addr_ntoa(netif_ip4_addr(net)));
            } else {
                printf("Open http://%s\n", ip4addr_ntoa(netif_ip4_addr(net)));
            }
            next_ip_print = make_timeout_time_ms(10000);
        }

        // Sample the FSR for the page's pressure indicator
        if (time_reached(next_fsr)) {
            next_fsr = make_timeout_time_ms(FSR_READ_MS);
            fsr_raw = fsr_read_raw();
            fsr_units = fsr_read_units();
        }

        // Measure mic loudness for the page's sound meter (10 ms burst)
        if (mic_ok && time_reached(next_mic)) {
            next_mic = make_timeout_time_ms(MIC_READ_MS);
            mic_level = mic_read_level(MIC_BURST);
        }

        // Reward motor commands from the page
        if (motor_req_pending) {
            motor_req_pending = false;
            motor_set(motor_req_speed);
            motor_timed = motor_req_speed != 0 && motor_req_ms > 0;
            if (motor_timed) motor_stop_at = make_timeout_time_ms(motor_req_ms);
            if (motor_req_speed == 0) motor_coast();
        }
        if (motor_timed && time_reached(motor_stop_at)) {
            motor_coast();
            motor_timed = false;
        }

        // Move ONE joint per tick (first one that is not at its target)
        if (time_reached(next_step)) {
            next_step = make_timeout_time_ms(STEP_MS);
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (servo_read(&servos[i]) != target[i]) {
                    servo_step_toward(&servos[i], target[i], STEP_DEG);
                    break;
                }
            }
        }

        sleep_ms(1);
    }
}
