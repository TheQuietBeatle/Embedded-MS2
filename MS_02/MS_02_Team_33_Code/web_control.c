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
//              limit marked, mic test (sound level meter with peak marker) and
//              3 s audio recording (play in the page or download as .wav), slider + buttons per joint, gripper Open / Close-step,
//              Home all, reward motor Forward / Reverse / Stop.
//   Motion   : non-blocking, at most STEP_DEG every STEP_MS, and only ONE joint moves
//              at a time so a 5 V 2 A adapter is not overloaded.
//
// Concurrency: lwIP callbacks (background interrupt) only record targets and
// requests; the main loop is the only place that drives the servos and motor.

#include <stdio.h>                 // printf / snprintf (USB serial output and text formatting)
#include <stdlib.h>                // atoi (text -> number), calloc / free (memory per connection)
#include <string.h>                // strcmp, strchr, strlen, memcpy ... (string helpers)

#include "pico/stdlib.h"           // Pico SDK basics: GPIO, sleep_ms, timers, stdio
#include "pico/cyw43_arch.h"       // Driver for the CYW43 Wi-Fi chip on the Pico W (also owns the onboard LED)
#include "lwip/tcp.h"              // lwIP raw TCP API, used to build the tiny web server
#include "lwip/ip4_addr.h"         // ip4addr_ntoa: turns an IP address into "192.168.x.x" text

#include "pins.h"                  // Which GPIO pin each part (servos, motor, FSR, mic) is wired to
#include "servo.h"                 // Servo driver: servo_init / servo_write / servo_read / servo_step_toward
#include "motor.h"                 // DC reward-motor driver (L298N): motor_set / motor_coast / motor_get
#include "fsr.h"                   // Force-sensitive resistor (grip pressure) driver
#include "mic.h"                   // INMP441 I2S microphone driver
#include "dhcp_server.h"           // Small DHCP server that hands out IPs on the Pico's own network

#if __has_include("wifi_config.h")                 // If the user created their private Wi-Fi file...
#include "wifi_config.h"                           // ...use it (holds WIFI_SSID and WIFI_PASSWORD)
#else                                              // Otherwise...
#include "wifi_config.example.h"                   // ...fall back to the example file in the repo
#warning "wifi_config.h not found - copy wifi_config.example.h to wifi_config.h and set your Wi-Fi"   // and warn at compile time
#endif                                             // end of the Wi-Fi config choice

// ---- Tunable settings -------------------------------------------------------
#define HOME_ANGLE       90.0f     // "Home" position for every joint, in degrees
#define HOME_GAP_MS      400u      // Pause between homing each joint at power-up (spreads the current draw)
#define JOINT_MIN        5.0f      // Lowest angle a normal joint may go to (keeps servos off their hard stop)
#define JOINT_MAX        175.0f    // Highest angle a normal joint may go to
#define GRIPPER_CLOSED   0.0f      // Gripper angle when fully closed (on this arm a LOWER angle closes)
#define GRIPPER_OPEN     120.0f    // Gripper angle when fully open
#define GRIPPER_STEP     2         // Degrees the "Close step" / "Open step" buttons move the gripper by
#define NUDGE_DEG        5         // Degrees the -/+ buttons move a joint by
#define STEP_DEG         2.0f      // Biggest move a servo makes in one motion tick (smooth, slow motion)
#define STEP_MS          20u       // Time between motion ticks (2 deg / 20 ms = 100 deg per second)
#define MOTOR_MAX_MS     5000u     // a web motor command never runs longer than this

// Grip pressure indicator (FSR, 0-1000 units from fsr_read_units())
#define FSR_ZONE_LO      175       // hold zone, shown green on the page
#define FSR_ZONE_HI      185       // upper edge of the green hold zone
#define FSR_SAFETY       400       // shown red: the gripper should back off above this
#define FSR_BAR_MAX      500       // full width of the page's bar
#define FSR_READ_MS      50u       // how often the main loop samples the FSR

// Mic test (INMP441 sound level meter on the page)
#define MIC_READ_MS      60u       // how often the main loop measures loudness
#define MIC_BURST        160u      // samples per measurement = 10 ms at 16 kHz (keeps servos smooth)
#define MIC_QUIET        500u      // level shown as an empty bar   (tune from what the page shows)
#define MIC_LOUD         60000u    // level shown as a full bar

// Audio recording (kept in RAM, served to the page as a .wav file)
#define REC_SECONDS      3u        // RAM limit: 3 s x 16 kHz x 2 bytes = 96 KB
#define REC_SAMPLES      (REC_SECONDS * MIC_SAMPLE_RATE_HZ)   // total samples in one recording (48 000)
#define REC_GAIN_SHIFT   4         // 24-bit mic -> 16-bit WAV: >>8 is 1x, >>4 is 16x louder
#define WAV_HEADER_LEN   44u       // size of a standard WAV file header in bytes
#define HTTP_PORT        80        // normal web port, so the browser URL needs no ":port"
#define WIFI_JOIN_ATTEMPTS 2       // WPA2 + WPA2/WPA3, ~40 s, then own network

// The Pico's own network, used when WIFI_SSID can't be joined (or is "").
// Override in wifi_config.h if you like.
#ifndef AP_SSID                              // only if wifi_config.h did not already choose a name
#define AP_SSID          "NeuralCalibrator"  // name of the Pico's own Wi-Fi network
#endif                                       // end of AP_SSID default
#ifndef AP_PASSWORD                          // only if wifi_config.h did not already choose a password
#define AP_PASSWORD      "robotarm123"      // at least 8 characters
#endif                                       // end of AP_PASSWORD default

// ---- Joints -------------------------------------------------------------------
enum { WAIST, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROLL, GRIPPER, NUM_JOINTS };   // joint numbers 0..5; NUM_JOINTS = 6 = how many

typedef struct {                   // fixed description of one joint
    const char *key;               // short id used in URLs, e.g. "elbow"
    const char *label;             // human-readable name shown on the page
    uint8_t     pin;               // GPIO pin the servo's signal wire is on
    float       lo, hi;            // allowed angle range for this joint
    const char *dn, *up;           // button names: dn = angle gets SMALLER, up = angle gets BIGGER
} joint_info_t;                    // name of the struct type

// If a button moves the wrong way on the real arm, just swap that joint's two
// button names (e.g. "&larr; Left" <-> "Right &rarr;"). Nothing else needs to change.
static const joint_info_t joints[NUM_JOINTS] = {                                                           // table of all 6 joints
    {"waist",       "Waist (MG996R, GP0)",      PIN_SERVO_WAIST,       JOINT_MIN,    JOINT_MAX,       "&larr; Left", "Right &rarr;"}, // base rotation
    {"shoulder",    "Shoulder (MG996R, GP1)",   PIN_SERVO_SHOULDER,    JOINT_MIN,    JOINT_MAX,       "&darr; Down", "Up &uarr;"},    // shoulder lift
    {"elbow",       "Elbow (MG996R, GP4)",      PIN_SERVO_ELBOW,       JOINT_MIN,    JOINT_MAX,       "Up &uarr;", "&darr; Down"},    // elbow bend (smaller angle raises the arm)
    {"wrist_pitch", "Wrist pitch (SG90, GP14)", PIN_SERVO_WRIST_PITCH, JOINT_MIN,    JOINT_MAX,       "&darr; Down", "Up &uarr;"},    // wrist up/down
    {"wrist_roll",  "Wrist roll (SG90, GP5)",  PIN_SERVO_WRIST_ROLL,  JOINT_MIN,    JOINT_MAX,       "&#8634; Left", "Right &#8635;"}, // wrist twist (curved arrows)
    {"gripper",     "Gripper (SG90, GP15)",     PIN_SERVO_GRIPPER,     GRIPPER_CLOSED, GRIPPER_OPEN,  "Close", "Open"},        // gripper range: lo = closed, hi = open (has its own buttons)
};                                                                                                         // end of joint table

static servo_t        servos[NUM_JOINTS];     // live state of each servo (driver data, current angle)
static volatile float target[NUM_JOINTS];     // written by web requests, read by main loop

// Reward motor request from the web page (applied by the main loop)
static volatile bool     motor_req_pending;   // true = the page sent a new motor command not yet applied
static volatile int8_t   motor_req_speed;     // requested speed, -100 (reverse) .. +100 (forward)
static volatile uint32_t motor_req_ms;        // how long to run it, in ms (0 = until told otherwise)

// Latest FSR reading, sampled by the main loop (the ADC is only touched there)
static volatile uint16_t fsr_units;           // pressure in 0-1000 units (shown on the page)
static volatile uint16_t fsr_raw;             // raw 12-bit ADC value (0-4095), shown for debugging

// Latest mic loudness, measured by the main loop
static bool              mic_ok;              // true if the mic driver started successfully
static volatile uint32_t mic_level;           // latest loudness value for the sound meter

// Recording: the main loop records when asked; /audio.wav serves the result.
enum { REC_IDLE, REC_REQUESTED, REC_RECORDING, REC_READY };   // the 4 stages a recording goes through
static volatile int rec_state = REC_IDLE;            // current stage (starts with nothing recorded)
static volatile uint32_t rec_id;                     // changes per recording (cache-buster)
static uint8_t wav[WAV_HEADER_LEN + REC_SAMPLES * 2]; // header + 16-bit mono samples

static float clampf(float v, float lo, float hi) {    // keep v inside [lo, hi]
    return v < lo ? lo : (v > hi ? hi : v);           // too low -> lo, too high -> hi, else v unchanged
}                                                     // end of clampf

static int joint_by_key(const char *key) {            // find a joint's number from its URL name
    for (int i = 0; i < NUM_JOINTS; i++) {            // check every joint in the table
        if (strcmp(joints[i].key, key) == 0) return i;   // names match -> return that joint's index
    }                                                 // end of search loop
    return -1;                                        // no joint has that name
}                                                     // end of joint_by_key

static void set_target(int j, float deg) {            // ask joint j to move to deg (main loop does the moving)
    target[j] = clampf(deg, joints[j].lo, joints[j].hi);   // store it, clamped to the joint's safe range
}                                                     // end of set_target

// ---- Web page -----------------------------------------------------------------
// The whole page (HTML + CSS + JavaScript) is one C string stored in flash.
// Each line below is one piece of that string; C glues adjacent "..." pieces together.
// The indentation is OUTSIDE the quotes, so it only helps you read the code -
// the browser receives the page exactly as before (no extra spaces are sent).
static const char PAGE[] =
"<!DOCTYPE html>"                                                                 // this is an HTML5 page
"<html>"
    "<head>"
        "<meta charset=\"utf-8\">"                                                // text is UTF-8
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">" // fit phone screens (no zoomed-out view)
        "<title>Neural Calibrator Arm</title>"                                    // browser tab title

        // ======================== CSS (how things look) ========================
        "<style>"
            "body{font-family:system-ui,sans-serif;background:#111;color:#eee;max-width:520px;margin:auto;padding:16px}" // dark page, centred, max 520 px wide
            ".card{background:#1c1f26;border-radius:12px;padding:14px 16px;margin-bottom:10px}"                         // rounded box around each section
            ".row{display:flex;justify-content:space-between;align-items:baseline}"                                     // label on the left, value on the right
            ".lbl{font-weight:600}"                                                                                     // bold section labels
            ".ang{font-size:26px;font-weight:bold;font-variant-numeric:tabular-nums}"                                   // big numbers that don't jiggle
            ".btns{display:flex;gap:8px;margin-top:8px}"                                                                // a row of buttons side by side
            "button{flex:1;padding:12px;font-size:15px;border:0;border-radius:10px;background:#2d3340;color:#eee}"       // big, finger-friendly buttons
            "button:active{background:#4f9cff}"                                                                         // blue while pressed
            ".stop{background:#4a2a2e}"                                                                                 // red-ish Stop button
            "input{width:100%;margin-top:10px;accent-color:#4f9cff}"                                                    // full-width blue sliders
            ".bar{position:relative;height:20px;background:#2d3340;border-radius:10px;margin:10px 0 6px;overflow:hidden}" // empty track of a level bar
            ".fill{position:absolute;left:0;top:0;bottom:0;width:0;background:#4f9cff;transition:width .2s,background .2s}" // coloured part of a bar (animated)
            ".zone{position:absolute;top:0;bottom:0;background:#3ecf8e;opacity:.75;min-width:3px}"                      // green "hold zone" marker on the FSR bar
            ".safe{position:absolute;top:0;bottom:0;right:0;background:#ff5d5d;opacity:.35}"                            // red "safety limit" area on the FSR bar
            ".st{font-weight:600}"                                                                                      // status word (IN ZONE, Loud ...)
            ".meta{color:#888;font-size:13px}"                                                                          // small grey info text
            ".peak{position:absolute;top:0;bottom:0;width:3px;background:#fff;opacity:.8;left:0}"                       // white peak-hold line on the mic bar
            "#msg{color:#888;font-size:13px;margin-bottom:12px}"                                                        // the "connected / moving" status line
        "</style>"
    "</head>"

    // ======================== HTML (what is on the page) ========================
    "<body>"
        "<h2>Neural Calibrator Arm</h2>"                                          // page heading
        "<div id=\"msg\">connecting...</div>"                                     // connection status (JS updates it)

        // ---- Card 1: grip pressure (FSR) ----
        "<div class=\"card\">"
            "<div class=\"row\">"
                "<span class=\"lbl\">Grip pressure (FSR)</span>"                  // card title
                "<span class=\"ang\" id=\"fsr\">--</span>"                        // big pressure number
            "</div>"
            "<div class=\"bar\">"
                "<div id=\"fsrbar\" class=\"fill\"></div>"                        // bar that grows with pressure
                "<div id=\"zone\" class=\"zone\"></div>"                          // green hold zone marker
                "<div id=\"safe\" class=\"safe\"></div>"                          // red safety area
            "</div>"
            "<div class=\"row\">"
                "<span id=\"fsrst\" class=\"st\">--</span>"                       // status word (Too light / IN ZONE / ...)
                "<span id=\"fsrinfo\" class=\"meta\"></span>"                     // zone range + raw ADC value
            "</div>"
        "</div>"

        // ---- Card 2: microphone test + recording ----
        "<div class=\"card\">"
            "<div class=\"row\">"
                "<span class=\"lbl\">Mic test (INMP441)</span>"                   // card title
                "<span class=\"ang\" id=\"mic\">--</span>"                        // big loudness number
            "</div>"
            "<div class=\"bar\">"
                "<div id=\"micbar\" class=\"fill\"></div>"                        // bar that grows with loudness
                "<div id=\"micpk\" class=\"peak\"></div>"                         // peak-hold marker
            "</div>"
            "<div class=\"row\">"
                "<span id=\"micst\" class=\"st\">--</span>"                       // status word (Quiet / Sound detected / Loud)
                "<span class=\"meta\">talk or clap near the mic</span>"           // hint
            "</div>"
            "<div class=\"btns\">"
                "<button id=\"recbtn\" onclick=\"go('/record')\">&#9679; Record</button>" // asks the Pico to record (/record)
            "</div>"
            "<div id=\"recst\" class=\"meta\" style=\"margin-top:8px\"></div>"   // recording status text
            "<audio id=\"player\" controls style=\"width:100%;margin-top:8px;display:none\"></audio>"          // audio player (hidden until ready)
            "<a id=\"dl\" download=\"recording.wav\" style=\"display:none;color:#4f9cff\">Download recording.wav</a>" // download link (hidden until ready)
        "</div>"

        // ---- Cards 3-8: one per joint (built by the JavaScript build() below) ----
        "<div id=\"joints\"></div>"

        "<div class=\"btns\" style=\"margin-bottom:10px\">"
            "<button onclick=\"go('/home')\">Home all (90&deg;)</button>"         // send every joint to 90 deg (/home)
        "</div>"

        // ---- Last card: reward motor ----
        "<div class=\"card\">"
            "<div class=\"row\">"
                "<span class=\"lbl\">Reward motor (L298N)</span>"                 // card title
                "<span class=\"ang\"><span id=\"mot\">0</span>%</span>"           // current motor speed in %
            "</div>"
            "<div class=\"btns\">"
                "<button onclick=\"go('/motor?speed=70&ms=1000')\">Forward 1 s</button>"           // 70% forward for 2 s
                "<button onclick=\"go('/motor?speed=-70&ms=1000')\">Reverse 1 s</button>"          // 70% reverse for 2 s
                "<button class=\"stop\" onclick=\"go('/motor?speed=0&ms=0')\">Stop</button>"       // stop now
            "</div>"
        "</div>"

        // ======================== JavaScript (what the page does) ========================
        "<script>"
            "const $=id=>document.getElementById(id);"                            // $('x') = find the element with id="x"
            "let built=false,drag=null;"                                          // built: joint cards made yet? drag: slider being dragged

            // btn(text, path): HTML for a button that calls go(path)
            "function btn(t,p){"
                "return '<button onclick=\"go(\\''+p+'\\')\">'+t+'</button>'"
            "}"

            // build(d): create one card per servo, using the list in the state JSON
            "function build(d){"
                "let h='';"                                                       // HTML text being built
                "for(const s of d.servos){"
                    "const k=s.key;"                                              // joint id, e.g. "elbow"
                    "h+='<div class=\"card\"><div class=\"row\"><span class=\"lbl\">'+s.label+'</span>'+"      // card + title
                      "'<span class=\"ang\"><span id=\"v_'+k+'\"></span>&deg;</span></div>'+"                // current angle (id v_<key>)
                      "'<input id=\"s_'+k+'\" type=\"range\" min=\"'+s.lo+'\" max=\"'+s.hi+'\" '+"           // slider in the joint's range (id s_<key>)
                      "'onpointerdown=\"drag=\\''+k+'\\'\" onchange=\"drag=null;go(\\'/set?servo='+k+'&angle=\\'+this.value)\">'+" // on release send /set
                      "'<div class=\"btns\">'+(k==='gripper'"                                                  // gripper gets its own buttons:
                        "?btn('Open','/set?servo=gripper&angle='+s.hi)"                                       // Open   = jump to the high angle
                        "+btn('Open step','/step?servo=gripper&delta='+d.gripper_step)"                       // Open step  = a few degrees UP                        
                        "+btn('Close step','/step?servo=gripper&delta=-'+d.gripper_step)"                     // Close step = a few degrees DOWN
                        "+btn('Closed','/set?servo=gripper&angle='+s.lo)"                                     // Closed = jump to the low angle
                        ":btn(s.dn,'/step?servo='+k+'&delta=-'+d.nudge)"                                      // others: e.g. "<- Left" = 5 deg smaller
                        "+btn('90&deg;','/set?servo='+k+'&angle=90')"                                         // centre
                        "+btn(s.up,'/step?servo='+k+'&delta='+d.nudge))+"                                     // e.g. "Right ->" = 5 deg bigger
                      "'</div></div>'"                                                                         // close buttons + card
                "}"
                "$('joints').innerHTML=h;"                                        // put all joint cards on the page
                "built=true"                                                      // remember the cards exist
            "}"

            // show(d): update the whole page from the state JSON
            "function show(d){"
                "if(!built)build(d);"                                             // first time: create the joint cards
                "for(const s of d.servos){"
                    "$('v_'+s.key).textContent=s.angle;"                          // show the current angle
                    "if(drag!==s.key)$('s_'+s.key).value=s.target"                // move the slider (unless you're dragging it)
                "}"
                "$('mot').textContent=d.motor;"                                   // motor speed
                "$('msg').textContent=d.moving?'moving...':'connected';"          // status line
                "if(d.fsr)fsr(d.fsr);"                                            // update the FSR card
                "if(d.mic)mic(d.mic);"                                            // update the mic card
                "if(d.rec)rec(d.rec)"                                             // update the recording part
            "}"

            // rec(r): Record button, status text, player and download link
            "let lastRec=-1;"                                                     // id of the recording already loaded
            "function rec(r){"
                "const b=$('recbtn');"
                "b.textContent=r.state==='recording'?'Recording...':'\\u25CF Record '+r.secs+' s';" // button text
                "b.disabled=r.state==='recording';"                               // grey out while recording
                "$('recst').textContent=r.state==='recording'?'Recording - the arm pauses for '+r.secs+' s':r.state==='ready'?'Recording ready':'';"
                "if(r.state==='ready'&&r.id!==lastRec){"                          // a NEW recording is ready
                    "lastRec=r.id;"
                    "const u='/audio.wav?id='+r.id;"                              // ?id= stops the browser using an old cached file
                    "$('player').src=u;"                                          // load it into the player
                    "$('player').style.display='block';"                          // show the player
                    "$('dl').href=u;"                                             // point the download link at it
                    "$('dl').style.display='inline-block'"                        // show the link
                "}"
            "}"

            // mic(m): sound level meter
            "let pk=0;"                                                           // peak-hold position (%)
            "function mic(m){"
                "if(!m.ok){"                                                      // mic failed to start
                    "$('mic').textContent='--';"
                    "$('micst').textContent='Mic not detected - check wiring';"
                    "$('micst').style.color='#ff5d5d';"                           // in red
                    "return"
                "}"
                "const r=Math.log10(Math.max(m.level,m.quiet)/m.quiet)/Math.log10(m.loud/m.quiet),p=Math.min(1,Math.max(0,r))*100;" // level -> 0-100% (log scale, like the ear)
                "pk=Math.max(p,pk-4);"                                            // peak jumps up, falls slowly
                "$('mic').textContent=m.level;"                                   // big number
                "$('micbar').style.width=p+'%';"                                  // bar length
                "$('micpk').style.left=pk+'%';"                                   // peak marker position
                "const t=p<5?'Quiet':p<60?'Sound detected':'Loud',c=p<5?'#888':p<60?'#3ecf8e':'#f5b942';" // status word + colour
                "$('micst').textContent=t;"
                "$('micst').style.color=c;"
                "$('micbar').style.background=c"
            "}"

            // pct(v, f): FSR units -> % of the bar width (0-100%)
            "function pct(v,f){"
                "return Math.min(100,Math.max(0,v*100/f.scale))+'%'"
            "}"

            // fsr(f): grip pressure card
            "function fsr(f){"
                "const u=f.units;"                                                // pressure in units
                "$('fsr').textContent=u;"                                         // big number
                "$('fsrbar').style.width=pct(u,f);"                               // bar length
                "$('zone').style.left=pct(f.zone_lo,f);"                          // green zone start
                "$('zone').style.width=pct(f.zone_hi-f.zone_lo,f);"               // green zone width
                "$('safe').style.left=pct(f.safety,f);"                           // red area start
                "let t,c;"                                                        // status text and colour
                "if(u>=f.safety){t='SAFETY LIMIT - open the gripper';c='#ff5d5d'}" // red: too much force
                "else if(u>f.zone_hi){t='Too hard';c='#f5b942'}"                  // orange: above the zone
                "else if(u>=f.zone_lo){t='IN ZONE';c='#3ecf8e'}"                  // green: just right
                "else{t=u<5?'No pressure':'Too light';c='#4f9cff'}"               // blue: below the zone
                "$('fsrst').textContent=t;"
                "$('fsrst').style.color=c;"
                "$('fsrbar').style.background=c;"
                "$('fsrinfo').textContent='zone '+f.zone_lo+'-'+f.zone_hi+' | raw ADC '+f.raw"
            "}"

            // go(path): call a Pico URL and show the JSON state it answers with
            "async function go(p){"
                "try{show(await (await fetch(p)).json())}"
                "catch(e){$('msg').textContent='no response from Pico'}"          // Pico unreachable
            "}"

            "go('/state');"                                                       // load the state once now...
            "setInterval(()=>go('/state'),300);"                                  // ...then refresh every 300 ms
        "</script>"
    "</body>"
"</html>";                                                                        // end of the page (and of the C string)

// ---- HTTP server (lwIP raw TCP API) ---------------------------------------------
typedef struct {                       // everything we keep for ONE browser connection
    struct tcp_pcb *pcb;               // lwIP's handle for this TCP connection
    bool        responded;             // true once we've handled the request (answer only once)
    char        head[192];             // the HTTP response header text
    size_t      head_len, head_sent;   // header length / how much of it is already sent
    const char *body;                  // points at the response body (page, JSON or WAV)
    size_t      body_len, body_sent;   // body length / how much of it is already sent
    char        json[1536];            // buffer where the state JSON is written
} http_conn_t;                         // name of the struct type

static size_t state_json(char *out, size_t cap) {      // write the arm's full state as JSON; returns its length
    size_t n = 0;                                      // characters written so far
    bool moving = false;                               // becomes true if any joint isn't at its target
    n += (size_t)snprintf(out + n, cap - n, "{\"servos\":[");   // start the JSON object and the servos list
    for (int i = 0; i < NUM_JOINTS && n < cap; i++) {  // one entry per joint (stop if the buffer is full)
        float a = servo_read(&servos[i]);              // the angle the servo is at right now
        if (a != target[i]) moving = true;             // still travelling -> arm is moving
        n += (size_t)snprintf(out + n, cap - n,        // append this joint's entry:
                "%s{\"key\":\"%s\",\"label\":\"%s\",\"dn\":\"%s\",\"up\":\"%s\",\"angle\":%d,\"target\":%d,\"lo\":%d,\"hi\":%d}",   // JSON template
                i ? "," : "", joints[i].key, joints[i].label, joints[i].dn, joints[i].up,               // comma before all but the first; name; label; button names
                (int)(a + 0.5f), (int)(target[i] + 0.5f), (int)joints[i].lo, (int)joints[i].hi);       // angle and target rounded; range
    }                                                  // end of joint loop
    if (n < cap) {                                     // only if there is still room in the buffer
        n += (size_t)snprintf(out + n, cap - n,        // append everything else:
                "],\"motor\":%d,\"moving\":%s,\"nudge\":%d,\"gripper_step\":%d,"                           // motor speed, moving flag, button step sizes
                "\"fsr\":{\"units\":%u,\"raw\":%u,\"zone_lo\":%d,\"zone_hi\":%d,\"safety\":%d,\"scale\":%d},"  // FSR reading and its markers
                "\"mic\":{\"ok\":%s,\"level\":%lu,\"quiet\":%u,\"loud\":%u},"                                // mic status, level and meter range
                "\"rec\":{\"state\":\"%s\",\"secs\":%u,\"id\":%lu}}",                                         // recording status (and close the JSON)
                motor_get(), moving ? "true" : "false", NUDGE_DEG, GRIPPER_STEP,                              // values for the motor/moving/step fields
                (unsigned)fsr_units, (unsigned)fsr_raw, FSR_ZONE_LO, FSR_ZONE_HI, FSR_SAFETY, FSR_BAR_MAX,    // values for the fsr fields
                mic_ok ? "true" : "false", (unsigned long)mic_level, MIC_QUIET, MIC_LOUD,                     // values for the mic fields
                rec_state == REC_READY ? "ready" :                                                            // recording state as text:
                rec_state == REC_IDLE  ? "idle"  : "recording",                                               // REQUESTED and RECORDING both show "recording"
                REC_SECONDS, (unsigned long)rec_id);                                                          // recording length and id
    }                                                  // end of "room left" check
    return n < cap ? n : cap - 1;                      // real length, or buffer size - 1 if the text was cut off
}                                                      // end of state_json

// Copies the value of `name` from a query string like "servo=elbow&angle=90".
static bool query_get(const char *query, const char *name, char *out, size_t cap) {   // returns true if found
    size_t len = strlen(name);                         // length of the name we look for
    const char *p = query;                             // p walks through the query string
    while (p && *p) {                                  // until the end of the string
        if (strncmp(p, name, len) == 0 && p[len] == '=') {   // found "name=" at this position
            p += len + 1;                              // skip past "name=" to the value
            size_t i = 0;                              // characters copied so far
            while (*p && *p != '&' && *p != ' ' && i + 1 < cap) out[i++] = *p++;   // copy until '&', space, end or buffer full
            out[i] = '\0';                             // end the copied text
            return true;                               // success
        }                                              // end of "found" block
        p = strchr(p, '&');                            // not here: jump to the next '&'
        if (p) p++;                                    // and step past it to the next "name=value"
    }                                                  // end of search loop
    return false;                                      // name not present
}                                                      // end of query_get

static void handle_request(http_conn_t *c, const char *req) {   // decide what to answer for one HTTP request
    char path[128] = "";                               // will hold e.g. "/set?servo=elbow&angle=90"
    // "GET /set?servo=elbow&angle=90 HTTP/1.1"
    const char *sp = strchr(req, ' ');                 // find the space after "GET"
    if (sp) {                                          // if the request looks valid
        size_t i = 0;                                  // characters copied so far
        sp++;                                          // move to the first character of the path
        while (*sp && *sp != ' ' && i + 1 < sizeof path) path[i++] = *sp++;   // copy the path up to the next space
        path[i] = '\0';                                // end the path text
    }                                                  // end of path extraction
    char *query = strchr(path, '?');                   // find where the "?query" part starts
    if (query) *query++ = '\0';                        // cut the path there; query now points after '?'

    char key[16], val[16];                             // small buffers for query values
    int j = (query && query_get(query, "servo", key, sizeof key)) ? joint_by_key(key) : -1;   // which joint "servo=" names (-1 if none)

    if (strcmp(path, "/audio.wav") == 0) {             // browser wants the recording
        if (rec_state == REC_READY) {                  // a finished recording exists
            c->body = (const char *)wav;               // body = the WAV buffer
            c->body_len = sizeof wav;                  // whole buffer (header + samples)
            c->head_len = (size_t)snprintf(c->head, sizeof c->head,                             // build the HTTP header:
                "HTTP/1.1 200 OK\r\nContent-Type: audio/wav\r\nCache-Control: no-store\r\n"     // OK, it's audio, don't cache it
                "Content-Disposition: inline; filename=\"recording.wav\"\r\n"                    // play in page; file name if saved
                "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)c->body_len);       // size, then close after sending
        } else {                                       // nothing recorded yet
            static const char none[] = "no recording yet";   // short error text
            c->body = none;                            // body = that text
            c->body_len = sizeof none - 1;             // its length (without the final '\0')
            c->head_len = (size_t)snprintf(c->head, sizeof c->head,                             // build a 404 header:
                "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nContent-Length: %u\r\n"   // not found, plain text, size
                "Connection: close\r\n\r\n", (unsigned)c->body_len);                             // close after sending
        }                                              // end of ready / not-ready
        return;                                        // done with this request
    }                                                  // end of /audio.wav

    if (strcmp(path, "/") == 0) {                      // browser wants the main page
        c->body = PAGE;                                // body = the HTML string above
        c->body_len = sizeof PAGE - 1;                 // its length (without the final '\0')
        c->head_len = (size_t)snprintf(c->head, sizeof c->head,                                 // build the header:
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %u\r\n"               // OK, it's HTML, size
            "Connection: close\r\n\r\n", (unsigned)c->body_len);                                 // close after sending
        return;                                        // done with this request
    }                                                  // end of "/"

    if (strcmp(path, "/set") == 0 && j >= 0 && query_get(query, "angle", val, sizeof val)) {         // /set?servo=X&angle=N
        set_target(j, (float)atoi(val));               // move joint X to angle N
    } else if (strcmp(path, "/step") == 0 && j >= 0 && query_get(query, "delta", val, sizeof val)) { // /step?servo=X&delta=D
        set_target(j, target[j] + (float)atoi(val));   // move joint X by D degrees from its target
    } else if (strcmp(path, "/record") == 0) {         // /record: start a recording
        if (mic_ok && (rec_state == REC_IDLE || rec_state == REC_READY)) {   // only if the mic works and none is in progress
            rec_state = REC_REQUESTED;                 // flag it; the main loop does the actual recording
        }                                              // end of record check
    } else if (strcmp(path, "/home") == 0) {           // /home: send every joint home
        for (int i = 0; i < NUM_JOINTS; i++) set_target(i, HOME_ANGLE);   // all targets = 90 deg
    } else if (strcmp(path, "/motor") == 0 && query && query_get(query, "speed", val, sizeof val)) { // /motor?speed=S&ms=T
        int speed = atoi(val);                         // requested speed
        int ms = query_get(query, "ms", val, sizeof val) ? atoi(val) : 0;   // requested run time (0 if missing)
        if (speed > 100) speed = 100;                  // limit speed to +100 %
        if (speed < -100) speed = -100;                // limit speed to -100 %
        if (ms < 0) ms = 0;                            // no negative times
        if (ms > (int)MOTOR_MAX_MS) ms = (int)MOTOR_MAX_MS;   // never longer than the safety maximum
        motor_req_speed = (int8_t)speed;               // hand the speed to the main loop
        motor_req_ms = (uint32_t)ms;                   // hand the time to the main loop
        motor_req_pending = true;                      // tell the main loop there's a new command
    }                                                  // end of command handling
    // Every API call (including /state and unknown paths) answers with the current state.
    c->body = c->json;                                 // body = this connection's JSON buffer
    c->body_len = state_json(c->json, sizeof c->json); // fill it with the current state
    c->head_len = (size_t)snprintf(c->head, sizeof c->head,                                     // build the header:
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"      // OK, it's JSON, don't cache
        "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)c->body_len);               // size, close after sending
}                                                      // end of handle_request

static void conn_free(http_conn_t *c) {                // forget a connection and free its memory
    if (c->pcb) {                                      // if lwIP's connection still exists
        tcp_arg(c->pcb, NULL);                         // stop passing our struct to callbacks
        tcp_recv(c->pcb, NULL);                        // unhook "data received" callback
        tcp_sent(c->pcb, NULL);                        // unhook "data sent" callback
        tcp_err(c->pcb, NULL);                         // unhook "error" callback
        tcp_poll(c->pcb, NULL, 0);                     // unhook periodic poll callback
    }                                                  // end of unhook
    free(c);                                           // release our struct
}                                                      // end of conn_free

static err_t conn_close(http_conn_t *c) {              // close a connection cleanly
    struct tcp_pcb *pcb = c->pcb;                      // remember lwIP's handle before freeing c
    err_t result = ERR_OK;                             // assume a normal close
    conn_free(c);                                      // free our struct and unhook callbacks
    if (pcb && tcp_close(pcb) != ERR_OK) {             // try a polite TCP close; if that fails...
        tcp_abort(pcb);                                // ...force-reset the connection
        result = ERR_ABRT;                             // tell lwIP we aborted it
    }                                                  // end of close attempt
    return result;                                     // lwIP needs to know if we aborted
}                                                      // end of conn_close

static bool all_written(const http_conn_t *c) {        // has the whole response been handed to TCP?
    return c->head_sent >= c->head_len && c->body_sent >= c->body_len;   // both header and body fully queued
}                                                      // end of all_written

// Queue as much of the response as the TCP send buffer allows.
static void send_more(http_conn_t *c) {
    while (!all_written(c)) {                          // repeat while something is left to send
        u16_t space = tcp_sndbuf(c->pcb);              // free space in TCP's send buffer
        if (space == 0) break;                         // buffer full: continue later (on_sent / on_poll)
        bool in_head = c->head_sent < c->head_len;     // still sending the header? (header goes first)
        const char *src = in_head ? c->head + c->head_sent : c->body + c->body_sent;     // where the next unsent byte is
        size_t left = in_head ? c->head_len - c->head_sent : c->body_len - c->body_sent; // how many bytes remain in that part
        u16_t n = (u16_t)(left < space ? left : space);                                  // send whichever is smaller
        if (tcp_write(c->pcb, src, n, TCP_WRITE_FLAG_COPY) != ERR_OK) break;             // queue n bytes (copied); stop on error
        if (in_head) c->head_sent += n; else c->body_sent += n;                          // record progress
    }                                                  // end of send loop
    tcp_output(c->pcb);                                // push queued data out onto Wi-Fi now
}                                                      // end of send_more

static err_t on_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {   // lwIP: the browser acknowledged some data
    (void)len;                                         // number of bytes acked - not needed
    http_conn_t *c = arg;                              // our connection struct
    if (!c) return ERR_OK;                             // already freed - nothing to do
    send_more(c);                                      // buffer has room again: send the next piece
    // Close once everything is written and acknowledged.
    if (all_written(c) && tcp_sndbuf(pcb) == TCP_SND_BUF) {   // all queued AND send buffer completely empty (all acked)
        return conn_close(c);                          // response finished - close
    }                                                  // end of finished check
    return ERR_OK;                                     // keep the connection open
}                                                      // end of on_sent

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {   // lwIP: data arrived from the browser
    http_conn_t *c = arg;                              // our connection struct
    if (!c) {                                          // connection already freed
        if (p) pbuf_free(p);                           // just drop the data
        return ERR_OK;                                 // done
    }                                                  // end of freed check
    if (!p || err != ERR_OK) {            // client closed the connection
        if (p) pbuf_free(p);                           // free any data we got
        return conn_close(c);                          // close our side too
    }                                                  // end of closed check
    tcp_recved(pcb, p->tot_len);                       // tell TCP we've taken the data (opens the receive window)
    if (!c->responded) {                               // only answer the first chunk of the request
        char req[256];                                 // room for the start of the request
        u16_t n = pbuf_copy_partial(p, req, sizeof req - 1, 0);   // copy up to 255 bytes out of lwIP's buffer
        req[n] = '\0';                                 // make it a proper C string
        c->responded = true;                           // remember we've handled it
        handle_request(c, req);                        // build the response
        send_more(c);                                  // start sending it
    }                                                  // end of first-chunk handling
    pbuf_free(p);                                      // give lwIP its buffer back
    return ERR_OK;                                     // all good
}                                                      // end of on_recv

static void on_err(void *arg, err_t err) {             // lwIP: the connection broke (reset, out of memory...)
    (void)err;                                         // reason - not needed
    http_conn_t *c = arg;                              // our connection struct
    if (c) {                                           // if we still have it
        c->pcb = NULL;                    // lwIP already freed the pcb
        free(c);                                       // free our struct (no unhooking - pcb is gone)
    }                                                  // end of cleanup
}                                                      // end of on_err

static err_t on_poll(void *arg, struct tcp_pcb *pcb) { // lwIP: called every 5 s for each connection
    (void)pcb;                                         // not needed
    http_conn_t *c = arg;                              // our connection struct
    if (!c) return ERR_OK;                             // already freed
    if (c->responded && !all_written(c)) {   // a long download (the .wav) is still going
        send_more(c);                                  // nudge it along
        return ERR_OK;                                 // keep the connection
    }                                                  // end of download check
    return conn_close(c);                    // drop connections idle for ~5 s
}                                                      // end of on_poll

static err_t on_accept(void *arg, struct tcp_pcb *pcb, err_t err) {   // lwIP: a browser opened a new connection
    (void)arg;                                         // not used
    if (err != ERR_OK || !pcb) return ERR_VAL;         // reject broken accepts
    http_conn_t *c = calloc(1, sizeof *c);             // new zeroed struct for this connection
    if (!c) {                                          // out of memory
        tcp_abort(pcb);                                // refuse the connection
        return ERR_ABRT;                               // tell lwIP we aborted it
    }                                                  // end of out-of-memory check
    c->pcb = pcb;                                      // remember lwIP's handle
    tcp_arg(pcb, c);                                   // lwIP will pass c to every callback below
    tcp_recv(pcb, on_recv);                            // call on_recv when data arrives
    tcp_sent(pcb, on_sent);                            // call on_sent when data is acknowledged
    tcp_err(pcb, on_err);                              // call on_err if the connection breaks
    tcp_poll(pcb, on_poll, 10);           // poll interval is in 0.5 s units
    return ERR_OK;                                     // connection accepted
}                                                      // end of on_accept

static bool http_start(void) {                         // start listening for browsers on port 80
    cyw43_arch_lwip_begin();                           // lock lwIP (it's also used from the background)
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);   // create a new TCP socket
    bool ok = pcb && tcp_bind(pcb, IP_ANY_TYPE, HTTP_PORT) == ERR_OK;   // bind it to port 80 on any address
    if (ok) {                                          // if that worked
        struct tcp_pcb *listener = tcp_listen_with_backlog(pcb, 4);   // turn it into a listener (up to 4 waiting)
        ok = listener != NULL;                         // check it worked
        if (ok) tcp_accept(listener, on_accept);       // call on_accept for each new connection
    }                                                  // end of setup
    cyw43_arch_lwip_end();                             // unlock lwIP
    return ok;                                         // true = web server is running
}                                                      // end of http_start

// ---- Wi-Fi ----------------------------------------------------------------------
static const char *wifi_error_text(int err) {          // turn a Wi-Fi error code into a helpful message
    switch (err) {                                     // check which error it is
    case PICO_ERROR_BADAUTH:        return "wrong password";                                    // password rejected
    case PICO_ERROR_TIMEOUT:        return "timed out - network not found (name typo? 5 GHz only? out of range?)";   // no answer
    case PICO_ERROR_CONNECT_FAILED: return "connection refused by the network";                 // router said no
    default:                        return "unknown error";                                     // anything else
    }                                                  // end of switch
}                                                      // end of wifi_error_text

static const char *auth_name(uint8_t auth) {           // turn a scan's security flags into text
    // Scan flags from the cyw43 driver: 4 = RSN (WPA2 or WPA3), 2 = WPA, 1 = WEP/privacy
    if (auth == 0)  return "open";                     // no flags = no password
    if (auth & 0x4) return "WPA2/WPA3";                // RSN bit set
    if (auth & 0x2) return "WPA";                      // old WPA bit set
    return "WEP";                                      // only the privacy bit
}                                                      // end of auth_name

typedef struct {                                       // running totals during a Wi-Fi scan
    int  count;                                        // how many networks were seen
    bool found;      // our WIFI_SSID was seen
} scan_summary_t;                                      // name of the struct type

static int scan_result(void *env, const cyw43_ev_scan_result_t *r) {   // called by the driver once per network found
    scan_summary_t *sum = env;                         // our running totals
    if (r && r->ssid_len > 0) {                        // skip hidden networks (no name)
        sum->count++;                                  // count it
        if (r->ssid_len == strlen(WIFI_SSID) && memcmp(r->ssid, WIFI_SSID, r->ssid_len) == 0) {   // same name as ours?
            sum->found = true;                         // yes - our network is in range
        }                                              // end of name check
        printf("    %-32.*s  ch %2u  %4d dBm  %s\n", r->ssid_len, (const char *)r->ssid,   // print: name,
               r->channel, r->rssi, auth_name(r->auth_mode));                              // channel, signal strength, security
    }                                                  // end of named-network block
    return 0;                                          // 0 = keep scanning
}                                                      // end of scan_result

// Print the 2.4 GHz networks the Pico W can see (it cannot see 5 GHz at all).
// Runs once at start-up, before any join attempt (a scan during a join stalls).
static void wifi_scan_print(void) {
    static scan_summary_t sum;                         // totals (static so it outlives the background scan)
    cyw43_wifi_scan_options_t opts = {0};              // default scan options (all channels)
    printf("Scanning for Wi-Fi networks (duplicates are normal):\n");   // header line
    cyw43_arch_lwip_begin();                           // lock the Wi-Fi driver
    int err = cyw43_wifi_scan(&cyw43_state, &opts, &sum, scan_result);   // start the scan; results go to scan_result
    cyw43_arch_lwip_end();                             // unlock
    if (err != 0) {                                    // scan didn't start
        printf("    scan could not start (%d) - continuing\n", err);   // say so
        return;                                        // and carry on without it
    }                                                  // end of error check
    absolute_time_t give_up = make_timeout_time_ms(10000);   // allow at most 10 s
    while (cyw43_wifi_scan_active(&cyw43_state) && !time_reached(give_up)) {   // until the scan ends or time runs out
        sleep_ms(100);                                 // wait a bit
    }                                                  // end of wait loop
    printf("  %d networks seen.\n", sum.count);        // print the total
    if (sum.found) {                                   // our network was seen
        printf("  \"%s\" is visible - if joining fails, check the password.\n", WIFI_SSID);   // so a failure would be the password
    } else {                                           // our network was NOT seen
        printf("  \"%s\" was NOT seen: check the exact name (capitals, spaces, apostrophes),\n"   // name probably wrong,
               "  switch the hotspot to 2.4 GHz (iPhone: Maximize Compatibility), or move closer.\n",   // or wrong band / too far
               WIFI_SSID);                             // the name we looked for
    }                                                  // end of found / not found
}                                                      // end of wifi_scan_print

// Try to join WIFI_SSID: WPA2 first, then WPA2/WPA3 (for WPA3 phone hotspots).
// Returns false after WIFI_JOIN_ATTEMPTS failures so the caller can fall back
// to the Pico's own network.
static bool wifi_join(void) {
    static const uint32_t auth_modes[] = { CYW43_AUTH_WPA2_AES_PSK, CYW43_AUTH_WPA3_WPA2_AES_PSK };   // security types to try, in order
    static const char *auth_names[]    = { "WPA2", "WPA2/WPA3" };                                     // their names for printing

    if (strlen(WIFI_SSID) == 0) {                      // no network configured
        printf("WIFI_SSID is empty - skipping straight to the Pico's own network.\n");   // explain
        return false;                                  // go straight to own network
    }                                                  // end of empty-SSID check
    wifi_scan_print();                                 // list nearby networks first (helps debugging)
    printf("Connecting to Wi-Fi \"%s\"...\n", WIFI_SSID);   // announce the attempt
    for (int attempt = 0; attempt < WIFI_JOIN_ATTEMPTS; attempt++) {   // try a limited number of times
        int m = attempt % 2;                           // alternate between WPA2 and WPA2/WPA3
        int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, auth_modes[m], 20000);   // try to join, wait up to 20 s
        if (err == 0) {                                // joined!
            printf("Connected (%s).\n", auth_names[m]);   // say which security worked
            return true;                               // success
        }                                              // end of success check
        printf("  %s attempt failed: %s (%d)\n", auth_names[m], wifi_error_text(err), err);   // explain the failure
    }                                                  // end of attempt loop
    return false;                                      // all attempts failed
}                                                      // end of wifi_join

// Start the Pico's own Wi-Fi network with a DHCP server. Returns its interface.
static struct netif *start_own_network(void) {
    cyw43_arch_disable_sta_mode();                     // stop trying to be a client of another network
    cyw43_arch_enable_ap_mode(AP_SSID, AP_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);   // become an access point (WPA2)
    struct netif *ap = &cyw43_state.netif[CYW43_ITF_AP];   // the network interface for that access point
    cyw43_arch_lwip_begin();                           // lock lwIP
    bool ok = dhcp_server_start(ap);                   // start handing out IP addresses to phones/laptops
    cyw43_arch_lwip_end();                             // unlock lwIP
    printf("Started own Wi-Fi network \"%s\" (password \"%s\")%s\n",   // announce the network
           AP_SSID, AP_PASSWORD, ok ? "" : " - DHCP server FAILED");   // and warn if DHCP failed
    return ap;                                         // caller uses this to print the IP address
}                                                      // end of start_own_network

// ---- Audio recording --------------------------------------------------------------
static void put_le16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }                     // store a 16-bit number, low byte first (WAV is little-endian)
static void put_le32(uint8_t *p, uint32_t v) { put_le16(p, v & 0xffff); put_le16(p + 2, v >> 16); }  // store a 32-bit number, low half first

// Standard 44-byte header for 16-bit mono PCM.
static void wav_write_header(void) {
    const uint32_t data_len = REC_SAMPLES * 2u;        // size of the audio data in bytes (2 bytes per sample)
    memcpy(wav, "RIFF", 4);                            // file type marker
    put_le32(wav + 4, 36u + data_len);                 // size of the rest of the file
    memcpy(wav + 8, "WAVEfmt ", 8);                    // "WAVE" format + start of the "fmt " chunk
    put_le32(wav + 16, 16);                          // fmt chunk size
    put_le16(wav + 20, 1);                           // PCM
    put_le16(wav + 22, 1);                           // mono
    put_le32(wav + 24, MIC_SAMPLE_RATE_HZ);          // samples per second (16 000)
    put_le32(wav + 28, MIC_SAMPLE_RATE_HZ * 2u);     // bytes per second
    put_le16(wav + 32, 2);                           // bytes per sample frame
    put_le16(wav + 34, 16);                          // bits per sample
    memcpy(wav + 36, "data", 4);                     // start of the "data" chunk
    put_le32(wav + 40, data_len);                    // size of the audio data
}                                                    // end of wav_write_header

// Blocking: records REC_SECONDS of audio into wav[]. Servos pause meanwhile.
static void record_audio(void) {
    rec_state = REC_RECORDING;                       // page will show "Recording..."
    printf("Recording %u s...\n", REC_SECONDS);      // log it over USB
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);   // LED off while recording

    for (int i = 0; i < 16; i++) {                   // drop stale samples waiting in the FIFO
        (void)mic_read_sample();                     // read and throw away
    }                                                // end of flush loop
    int32_t dc = 0;                                  // running average = the mic's constant offset (DC)
    for (int i = 0; i < 256; i++) {                  // estimate the DC offset first
        dc += (mic_read_sample() - dc) / 16;         // move the average 1/16 of the way toward each sample
    }                                                // end of DC warm-up
    int16_t *out = (int16_t *)(wav + WAV_HEADER_LEN);   // samples go right after the 44-byte header
    for (uint32_t i = 0; i < REC_SAMPLES; i++) {     // record every sample
        int32_t s = mic_read_sample();               // one 24-bit sample (waits for the next one)
        dc += (s - dc) / 1024;                       // slow tracking keeps the audio itself
        int32_t v = (s - dc) >> REC_GAIN_SHIFT;      // remove the offset and scale down to 16-bit size
        if (v > 32767)  v = 32767;                   // clip loud peaks to the 16-bit maximum
        if (v < -32768) v = -32768;                  // clip to the 16-bit minimum
        out[i] = (int16_t)v;                         // store the sample
    }                                                // end of recording loop
    wav_write_header();                              // fill in the WAV header
    rec_id++;                                        // new id so the browser fetches the new file, not a cached one
    rec_state = REC_READY;                           // /audio.wav may now serve it
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);   // LED back on
    printf("Recording ready: http://<pico>/audio.wav\n");   // log it
}                                                    // end of record_audio

// ---- Main -----------------------------------------------------------------------
int main(void) {                                     // program starts here
    stdio_init_all();                                // enable printf over USB serial

    motor_init(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_ENA);   // motor off first
    fsr_init(PIN_FSR_ADC);                           // set up the ADC pin for the pressure sensor
    for (int i = 0; i < NUM_JOINTS; i++) {           // for every joint...
        servo_init(&servos[i], joints[i].pin, joints[i].lo, joints[i].hi);   // silent
    }                                                // end of servo setup

    // Wait up to 10 s for the serial monitor so the Wi-Fi scan below isn't missed.
    for (int i = 0; i < 100 && !stdio_usb_connected(); i++) {   // 100 x 100 ms, stop early once USB is open
        sleep_ms(100);                               // wait 0.1 s
    }                                                // end of USB wait
    sleep_ms(500);                                   // small extra delay so the first lines aren't lost
    printf("\n=== Neural Calibrator - web control ===\n");   // start-up banner

    for (int i = 0; i < NUM_JOINTS; i++) {   // home one joint at a time
        printf("Homing %s\n", joints[i].key);        // log which joint
        servo_write(&servos[i], HOME_ANGLE);         // jump it to 90 deg (starts its PWM signal)
        target[i] = servo_read(&servos[i]);          // its target = where it is now, so it stays there
        sleep_ms(HOME_GAP_MS);                       // wait before the next one (avoids a current spike)
    }                                                // end of homing loop

    mic_ok = mic_init(PIN_MIC_SCK, PIN_MIC_WS, PIN_MIC_SD);   // start the I2S mic receiver
    printf("Mic: %s\n", mic_ok ? "I2S receiver running" : "INIT FAILED (check pins)");   // report the result

    if (cyw43_arch_init()) {                         // start the Wi-Fi chip (non-zero = failed)
        printf("Wi-Fi chip failed to start\n");      // report
        return 1;                                    // can't continue without Wi-Fi
    }                                                // end of Wi-Fi chip check
    cyw43_arch_enable_sta_mode();                    // "station" mode = join an existing network
    struct netif *net = &cyw43_state.netif[CYW43_ITF_STA];   // the interface we'll print the IP of
    bool own_network = !wifi_join();                 // try to join; if it fails we use our own network
    if (own_network) {                               // joining failed
        printf("Could not join \"%s\" - switching to the Pico's own network.\n", WIFI_SSID);   // explain
        net = start_own_network();                   // become an access point instead
    }                                                // end of fallback
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);   // LED on = network is up

    if (!http_start()) {                             // start the web server
        printf("Web server failed to start\n");      // report failure
        return 1;                                    // can't continue without it
    }                                                // end of server check

    absolute_time_t next_step = make_timeout_time_ms(STEP_MS);   // when the next servo motion tick is due
    absolute_time_t next_ip_print = get_absolute_time();          // print the address right away
    absolute_time_t motor_stop_at = nil_time;                     // when a timed motor run must stop (none yet)
    absolute_time_t next_fsr = get_absolute_time();               // read the FSR right away
    absolute_time_t next_mic = get_absolute_time();               // measure the mic right away
    bool motor_timed = false;                                     // true while a timed motor run is active

    while (true) {                                   // main loop - runs forever
        // Remind the address every 10 s in case the serial monitor was opened late.
        if (time_reached(next_ip_print)) {           // time to print the address?
            if (own_network) {                       // on the Pico's own network
                printf("Join Wi-Fi \"%s\" (password \"%s\"), then open http://%s\n",   // tell the user which network to join
                       AP_SSID, AP_PASSWORD, ip4addr_ntoa(netif_ip4_addr(net)));        // and the address to open
            } else {                                 // joined the normal Wi-Fi
                printf("Open http://%s\n", ip4addr_ntoa(netif_ip4_addr(net)));         // just the address
            }                                        // end of network type check
            next_ip_print = make_timeout_time_ms(10000);   // next reminder in 10 s
        }                                            // end of address reminder

        // Sample the FSR for the page's pressure indicator
        if (time_reached(next_fsr)) {                // time for a new reading?
            next_fsr = make_timeout_time_ms(FSR_READ_MS);   // schedule the next one (50 ms)
            fsr_raw = fsr_read_raw();                // raw ADC value
            fsr_units = fsr_read_units();            // value in 0-1000 units
        }                                            // end of FSR reading

        // Record when the page asked for it
        if (rec_state == REC_REQUESTED) {            // Record button was pressed
            record_audio();                          // record 3 s (blocks; the arm pauses)
            next_step = make_timeout_time_ms(STEP_MS);   // restart motion timing so it doesn't jump
        }                                            // end of recording

        // Measure mic loudness for the page's sound meter (10 ms burst)
        if (mic_ok && time_reached(next_mic)) {      // mic works and it's time to measure?
            next_mic = make_timeout_time_ms(MIC_READ_MS);   // schedule the next one (60 ms)
            mic_level = mic_read_level(MIC_BURST);   // loudness over 160 samples
        }                                            // end of mic measurement

        // Reward motor commands from the page
        if (motor_req_pending) {                     // a new motor command arrived
            motor_req_pending = false;               // mark it as handled
            motor_set(motor_req_speed);              // apply the speed
            motor_timed = motor_req_speed != 0 && motor_req_ms > 0;   // does it have a time limit?
            if (motor_timed) motor_stop_at = make_timeout_time_ms(motor_req_ms);   // yes: remember when to stop
            if (motor_req_speed == 0) motor_coast(); // speed 0 = Stop: let the motor spin down freely
        }                                            // end of motor command
        if (motor_timed && time_reached(motor_stop_at)) {   // timed run finished?
            motor_coast();                           // stop the motor
            motor_timed = false;                     // no timed run active anymore
        }                                            // end of motor timeout

        // Move ONE joint per tick (first one that is not at its target)
        if (time_reached(next_step)) {               // time for a motion tick?
            next_step = make_timeout_time_ms(STEP_MS);   // schedule the next tick (20 ms)
            for (int i = 0; i < NUM_JOINTS; i++) {   // look through the joints in order
                if (servo_read(&servos[i]) != target[i]) {   // this one isn't where it should be
                    servo_step_toward(&servos[i], target[i], STEP_DEG);   // move it up to 2 deg closer
                    break;                           // only one joint per tick (protects the power supply)
                }                                    // end of "needs to move" check
            }                                        // end of joint loop
        }                                            // end of motion tick

        sleep_ms(1);                                 // short rest so the loop doesn't spin flat out
    }                                                // end of main loop
}                                                    // end of main
