// web_control.c - Web page that controls the Neural Calibrator arm (Pico W, Pico SDK).
//
// Separate program from the MS2 driver test (main.c); both build from this folder.
//   Power-up : every joint homes to 90 deg, one at a time.
//   Wi-Fi    : joins WIFI_SSID (see wifi_config.example.h); the address to open is
//              printed over USB serial and the onboard LED turns on when connected.
//   Web page : slider + buttons per joint, gripper Open / Close-step, Home all,
//              reward motor Forward / Reverse / Stop.
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
#define HTTP_PORT        80

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
"#msg{color:#888;font-size:13px;margin-bottom:12px}"
"</style></head><body>"
"<h2>Neural Calibrator Arm</h2><div id=\"msg\">connecting...</div>"
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
"$('msg').textContent=d.moving?'moving...':'connected'}"
"async function go(p){try{show(await (await fetch(p)).json())}catch(e){$('msg').textContent='no response from Pico'}}"
"go('/state');setInterval(()=>go('/state'),500);"
"</script></body></html>";

// ---- HTTP server (lwIP raw TCP API) ---------------------------------------------
typedef struct {
    struct tcp_pcb *pcb;
    bool        responded;
    char        head[192];
    size_t      head_len, head_sent;
    const char *body;
    size_t      body_len, body_sent;
    char        json[1024];
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
                "],\"motor\":%d,\"moving\":%s,\"nudge\":%d,\"gripper_step\":%d}",
                motor_get(), moving ? "true" : "false", NUDGE_DEG, GRIPPER_STEP);
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

static int scan_result(void *env, const cyw43_ev_scan_result_t *r) {
    (void)env;
    if (r && r->ssid_len > 0) {
        printf("    %-32.*s  ch %2u  %4d dBm  %s\n", r->ssid_len, (const char *)r->ssid,
               r->channel, r->rssi, auth_name(r->auth_mode));
    }
    return 0;
}

// Print the 2.4 GHz networks the Pico W can see (it cannot see 5 GHz at all).
static void wifi_scan_print(void) {
    cyw43_wifi_scan_options_t opts = {0};
    printf("  Networks the Pico can see (duplicates are normal):\n");
    cyw43_arch_lwip_begin();
    int err = cyw43_wifi_scan(&cyw43_state, &opts, NULL, scan_result);
    cyw43_arch_lwip_end();
    if (err != 0) {
        printf("    scan failed (%d)\n", err);
        return;
    }
    while (cyw43_wifi_scan_active(&cyw43_state)) {
        sleep_ms(100);
    }
    printf("  If \"%s\" is not listed: wrong name, 5 GHz only, or too far away.\n", WIFI_SSID);
}

// Keep trying until connected: WPA2 first, then WPA2/WPA3 (for WPA3 phone hotspots).
static void wifi_connect(void) {
    static const uint32_t auth_modes[] = { CYW43_AUTH_WPA2_AES_PSK, CYW43_AUTH_WPA3_WPA2_AES_PSK };
    static const char *auth_names[]    = { "WPA2", "WPA2/WPA3" };
    bool scanned = false;

    printf("Connecting to Wi-Fi \"%s\"...\n", WIFI_SSID);
    for (int attempt = 0; ; attempt++) {
        int m = attempt % 2;
        int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, auth_modes[m], 20000);
        if (err == 0) {
            printf("Connected (%s).\n", auth_names[m]);
            return;
        }
        printf("  %s attempt failed: %s (%d)\n", auth_names[m], wifi_error_text(err), err);
        if (!scanned) {
            wifi_scan_print();
            scanned = true;
        }
    }
}

// ---- Main -----------------------------------------------------------------------
int main(void) {
    stdio_init_all();

    motor_init(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_ENA);   // motor off first
    for (int i = 0; i < NUM_JOINTS; i++) {
        servo_init(&servos[i], joints[i].pin, joints[i].lo, joints[i].hi);   // silent
    }

    sleep_ms(2000);   // time to open the serial monitor
    printf("\n=== Neural Calibrator - web control ===\n");

    for (int i = 0; i < NUM_JOINTS; i++) {   // home one joint at a time
        printf("Homing %s\n", joints[i].key);
        servo_write(&servos[i], HOME_ANGLE);
        target[i] = servo_read(&servos[i]);
        sleep_ms(HOME_GAP_MS);
    }

    if (cyw43_arch_init()) {
        printf("Wi-Fi chip failed to start\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    wifi_connect();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    if (!http_start()) {
        printf("Web server failed to start\n");
        return 1;
    }

    absolute_time_t next_step = make_timeout_time_ms(STEP_MS);
    absolute_time_t next_ip_print = get_absolute_time();
    absolute_time_t motor_stop_at = nil_time;
    bool motor_timed = false;

    while (true) {
        // Remind the address every 10 s in case the serial monitor was opened late.
        if (time_reached(next_ip_print)) {
            printf("Open http://%s\n",
                   ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])));
            next_ip_print = make_timeout_time_ms(10000);
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
