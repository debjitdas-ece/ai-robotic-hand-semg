// ═══════════════════════════════════════════════════════════
//  SENDER ESP32
//  Camera (MediaPipe UDP) = PRIMARY
//  EMG = FALLBACK (only when no hand detected)
//  ESP-NOW → Receiver ESP32
// ═══════════════════════════════════════════════════════════
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// ── ESP32 Access Point credentials ───────────────────────
const char* AP_SSID = "RoboticHand";
const char* AP_PASS = "12345678";
const uint16_t UDP_PORT = 5005;
WiFiUDP udp;

// ── ESP-NOW: Receiver MAC address ─────────────────────────
uint8_t RECEIVER_MAC[6] = {0x70, 0x4B, 0xCA, 0x58, 0x5D, 0xEC};

// ── EMG ───────────────────────────────────────────────────
#define EMG_PIN        35
#define SAMPLE_RATE    1000
#define WINDOW_SIZE    50
#define EMG_PRINT_MS   100

int  emgSamples[WINDOW_SIZE];
int  emgIdx       = 0;
bool emgFull      = false;
int  DC_OFFSET    = 2048;
int  BASELINE_RMS = 100;
int  MAX_RMS      = 690;

unsigned long lastEmgSample = 0;
unsigned long lastEmgPrint  = 0;

// ── MediaPipe finger curls (from UDP, 0-100) ──────────────
float mp_curl[5]  = {0, 0, 0, 0, 0};
unsigned long lastUdpMs   = 0;
bool  mpDataFresh = false;

// ── Hand-lost timeout: switch to EMG after this long ──────
#define HAND_TIMEOUT_MS  500

// ── Wrist fix: low-pass smoothing ─────────────────────────
float wrist_angle  = 50.0f;
#define WRIST_SMOOTH  0.15f

// ── Thumb fix: separate low-pass smoothing ────────────────
float thumb_smooth = 0.0f;
#define THUMB_SMOOTH  0.20f

// ── Bayesian fusion params (EMG fallback only) ────────────
#define PRIOR_FLEX    0.5f
#define EMG_HIGH_TH   70.0f
#define EMG_LOW_TH    20.0f
#define REST_PCT       5.0f
#define SIGMOID_K      0.12f

// ── ESP-NOW packet ────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint16_t servo[6];   // degrees x 10
} ServoPacket;

ServoPacket pkt;
bool espNowReady = false;

// ── Forward declarations ──────────────────────────────────
float computeEmgRMS();
float emgToPercent(float rms);
float bayesianConfidence(float emg_pct);
void  sampleEmg();
void  parseUdpPacket();
void  sendServos(float fused[6]);
void  onDataSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status);


// ─────────────────────────────────────────────────────────
//  EMG helpers
// ─────────────────────────────────────────────────────────
float computeEmgRMS() {
    int  n = emgFull ? WINDOW_SIZE : emgIdx;
    if (n == 0) return 0.0f;
    long sumSq = 0;
    for (int i = 0; i < n; i++)
        sumSq += (long)emgSamples[i] * emgSamples[i];
    return sqrtf((float)sumSq / n);
}

float emgToPercent(float rms) {
    int pct = map((int)rms, BASELINE_RMS, MAX_RMS, 0, 100);
    return (float)constrain(pct, 0, 100);
}

void sampleEmg() {
    unsigned long now = micros();
    if (now - lastEmgSample < (1000000UL / SAMPLE_RATE)) return;
    lastEmgSample = now;
    int raw     = analogRead(EMG_PIN);
    int centred = abs(raw - DC_OFFSET);
    emgSamples[emgIdx++] = centred;
    if (emgIdx >= WINDOW_SIZE) { emgIdx = 0; emgFull = true; }
}


// ─────────────────────────────────────────────────────────
//  Bayesian sigmoid gate (EMG fallback only)
// ─────────────────────────────────────────────────────────
float bayesianConfidence(float emg_pct) {
    float mid         = (EMG_HIGH_TH + EMG_LOW_TH) / 2.0f;
    float p_emg_flex  = 1.0f / (1.0f + expf(-SIGMOID_K * (emg_pct - mid)));
    float p_emg_relax = 1.0f - p_emg_flex;
    float numerator   = p_emg_flex * PRIOR_FLEX;
    float denominator = numerator + p_emg_relax * (1.0f - PRIOR_FLEX);
    if (denominator < 1e-6f) return 0.5f;
    return numerator / denominator;
}


// ─────────────────────────────────────────────────────────
//  UDP: parse "T,I,M,R,P\n" from Python
// ─────────────────────────────────────────────────────────
void parseUdpPacket() {
    int len = udp.parsePacket();
    if (len <= 0) return;

    char buf[64] = {0};
    udp.read(buf, sizeof(buf) - 1);

    float t, i, m, r, p;
    if (sscanf(buf, "%f,%f,%f,%f,%f", &t, &i, &m, &r, &p) == 5) {
        mp_curl[0] = constrain(t, 0, 100);
        mp_curl[1] = constrain(i, 0, 100);
        mp_curl[2] = constrain(m, 0, 100);
        mp_curl[3] = constrain(r, 0, 100);
        mp_curl[4] = constrain(p, 0, 100);
        mpDataFresh = true;
        lastUdpMs   = millis();
    }
}


// ─────────────────────────────────────────────────────────
//  ESP-NOW send callback
// ─────────────────────────────────────────────────────────
void onDataSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    // Uncomment to debug: Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}


// ─────────────────────────────────────────────────────────
//  Build & send servo packet
// ─────────────────────────────────────────────────────────
void sendServos(float fused[6]) {
    for (int i = 0; i < 6; i++) {
        float deg = fused[i] * 1.8f;
        deg = constrain(deg, 0.0f, 180.0f);
        pkt.servo[i] = (uint16_t)(deg * 10);
    }
    esp_now_send(RECEIVER_MAC, (uint8_t*)&pkt, sizeof(pkt));
}


// ═════════════════════════════════════════════════════════
//  Setup
// ═════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // ── Measure DC offset at boot ──────────────────────────
    Serial.println("Measuring DC offset — relax muscle...");
    delay(2000);
    long sum = 0; int cnt = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
        sum += analogRead(EMG_PIN); cnt++;
        delayMicroseconds(1000000 / SAMPLE_RATE);
    }
    DC_OFFSET = (cnt > 0 && (int)(sum/cnt) > 100) ? (int)(sum / cnt) : 2048;
    Serial.printf("DC_OFFSET = %d\n", DC_OFFSET);

    // ── Start ESP32 as Access Point ────────────────────────
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 1);   // channel 1 — fixed
    delay(500);

    uint8_t apChannel = 1;   // fixed AP channel — matches receiver
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("AP started! SSID: %s\n", AP_SSID);
    Serial.printf("AP Password: %s\n", AP_PASS);
    Serial.printf("ESP32 IP: %s\n", ip.toString().c_str());
    Serial.printf("AP Channel: %d (fixed)\n", apChannel);
    Serial.println(">>> Connect laptop to 'RoboticHand' WiFi <<<");
    Serial.printf(">>> Set ESP32_IP = \"%s\" in Python <<<\n", ip.toString().c_str());

    udp.begin(UDP_PORT);
    Serial.printf("UDP listening on port %d\n", UDP_PORT);

    // ── ESP-NOW ────────────────────────────────────────────
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED"); return;
    }
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = apChannel;    // use actual AP channel
    peer.ifidx   = WIFI_IF_AP;  // send via AP interface — REQUIRED in AP mode
    peer.encrypt  = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("ESP-NOW peer add FAILED"); return;
    }
    espNowReady = true;
    Serial.println("ESP-NOW ready");
    Serial.println("======= SENDER RUNNING =======");
    Serial.println("Mode: CAMERA PRIMARY / EMG FALLBACK");
}


// ═════════════════════════════════════════════════════════
//  Loop
// ═════════════════════════════════════════════════════════
void loop() {
    sampleEmg();
    parseUdpPacket();

    static unsigned long lastFuse = 0;
    unsigned long now = millis();
    if (now - lastFuse < 50) return;
    lastFuse = now;

    float fused[6];
    bool handVisible = mpDataFresh && (now - lastUdpMs < HAND_TIMEOUT_MS);

    if (handVisible) {
        // ════════════════════════════════════════
        //  CAMERA MODE — direct finger mapping
        // ════════════════════════════════════════

        // Thumb fix: apply low-pass smoothing to reduce jitter
        thumb_smooth += THUMB_SMOOTH * (mp_curl[0] - thumb_smooth);
        fused[0] = thumb_smooth;

        fused[1] = mp_curl[1];   // Index
        fused[2] = mp_curl[2];   // Middle
        fused[3] = mp_curl[3];   // Ring
        fused[4] = mp_curl[4];   // Pinky

        // Wrist fix: hold at neutral (50% = 90deg) and smooth transitions
        wrist_angle += WRIST_SMOOTH * (50.0f - wrist_angle);
        fused[5] = wrist_angle;

    } else {
        // ════════════════════════════════════════
        //  EMG FALLBACK MODE — hand not detected
        // ════════════════════════════════════════
        float rms     = computeEmgRMS();
        float emg_pct = emgToPercent(rms);
        float conf    = bayesianConfidence(emg_pct);

        // Thumb fix: smooth EMG-driven thumb too
        float thumb_target = conf * 100.0f + (1.0f - conf) * REST_PCT;
        thumb_smooth += THUMB_SMOOTH * (thumb_target - thumb_smooth);
        fused[0] = thumb_smooth;

        for (int i = 1; i < 5; i++) {
            fused[i] = conf * 100.0f + (1.0f - conf) * REST_PCT;
        }

        // Wrist fix: EMG confidence drives wrist, smoothed
        float wrist_target = conf * 90.0f + (1.0f - conf) * 45.0f;
        wrist_angle += WRIST_SMOOTH * (wrist_target - wrist_angle);
        fused[5] = wrist_angle;

        if (now - lastEmgPrint >= EMG_PRINT_MS) {
            lastEmgPrint = now;
            Serial.printf(
                "[EMG] rms:%4.0f  pct:%3.0f%%  conf:%4.2f | "
                "T:%4.1f I:%4.1f M:%4.1f R:%4.1f P:%4.1f W:%4.1f\n",
                rms, emg_pct, conf,
                fused[0], fused[1], fused[2], fused[3], fused[4], fused[5]
            );
        }
    }

    if (espNowReady) sendServos(fused);

    // Mode indicator every second
    static unsigned long lastModePrint = 0;
    if (now - lastModePrint >= 1000) {
        lastModePrint = now;
        Serial.printf("[%s] T:%4.1f I:%4.1f M:%4.1f R:%4.1f P:%4.1f W:%4.1f\n",
            handVisible ? "CAMERA" : "EMG   ",
            fused[0], fused[1], fused[2], fused[3], fused[4], fused[5]
        );
    }
}
