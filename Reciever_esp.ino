// ═══════════════════════════════════════════════════════════
//  RECEIVER ESP32
//  ESP-NOW → PCA9685 → 6x MG996R servos
//
//  FIXES APPLIED:
//  [Step 1] degreesToTick() — corrected tick formula (0–175° clamp)
//  [Step 1] loop() — constrain servo degrees to 0–175°
//  [Step 2] REMINDER: PCA9685 VCC must go to ESP32 3.3V (NOT buck converter)
//           ESP32 GND and buck converter GND must share common ground
//  [Step 3] 2000µF 16V cap already fitted across PCA9685 V+ and GND ✅
//  [Step 5] I²C health check added in loop() — watch Serial Monitor for errors
// ═══════════════════════════════════════════════════════════
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ── PCA9685 ───────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define PWM_FREQ       50        // Hz — standard servo frequency
#define SERVO_MIN_US   500       // µs pulse for 0°
#define SERVO_MAX_US   2500      // µs pulse for 180°

// Channel map — wire servos to PCA9685 channels 0-5
#define CH_THUMB   0
#define CH_INDEX   1
#define CH_MIDDLE  2
#define CH_RING    3
#define CH_PINKY   4
#define CH_WRIST   5

// ── CHANNEL FIX ───────────────────────────────────────────
// Must match what the sender prints as "AP Channel: X"
#define SENDER_WIFI_CHANNEL  1   // <<< change if sender prints a different channel

// ── ESP-NOW packet (must match sender) ───────────────────
typedef struct __attribute__((packed)) {
    uint16_t servo[6];   // degrees x 10
} ServoPacket;

ServoPacket lastPkt;
volatile bool newPacket = false;


// ─────────────────────────────────────────────────────────
//  degrees → PCA9685 tick count
//  [STEP 1 FIX] Corrected formula: maps µs pulse into 0–4095 range
//               using 20000µs period (50Hz). Clamp to 175° max.
// ─────────────────────────────────────────────────────────
uint16_t degreesToTick(float deg) {
    deg = constrain(deg, 0.0f, 175.0f);                          // [Step 1] was 180.0f
    float pulseUs = SERVO_MIN_US + (deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    return (uint16_t)((pulseUs / 20000.0f) * 4096.0f);           // [Step 1] was (us / TICK_US)
}

void setServo(uint8_t channel, float degrees) {
    pwm.setPWM(channel, 0, degreesToTick(degrees));
}


// ─────────────────────────────────────────────────────────
//  ESP-NOW receive callback (ISR context — keep short)
// ─────────────────────────────────────────────────────────
void onDataRecv(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
    if (len == sizeof(ServoPacket)) {
        memcpy(&lastPkt, data, len);
        newPacket = true;
    }
}


// ═════════════════════════════════════════════════════════
//  Setup
// ═════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);

    // ── PCA9685 ───────────────────────────────────────────
    // [Step 2 REMINDER] Check before powering on:
    //   PCA9685 VCC  → ESP32 3.3V pin  (NOT buck converter output)
    //   PCA9685 GND  → ESP32 GND
    //   PCA9685 V+   → Buck converter output (6V)
    //   PCA9685 GND  → Buck converter GND
    //   ESP32 GND and Buck GND must share a common wire
    Wire.begin();                          // SDA=21, SCL=22 (default ESP32)
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // calibrate if servos jitter
    pwm.setPWMFreq(PWM_FREQ);
    delay(10);
    Serial.println("PCA9685 ready");

    // Startup test: sweep thumb and wrist so you can verify they work
    Serial.println("Startup test: sweeping Thumb (CH0) and Wrist (CH5)...");
    for (int deg = 0; deg <= 175; deg += 10) {   // [Step 1] cap at 175°
        setServo(CH_THUMB, deg);
        setServo(CH_WRIST, deg);
        delay(30);
    }
    // Centre all servos
    for (int i = 0; i < 6; i++) setServo(i, 90.0f);
    delay(500);
    Serial.println("Startup test done — all servos at 90deg");

    // ── ESP-NOW (STA mode, forced channel) ────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_channel(SENDER_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.printf("Receiver MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("Locked to WiFi channel: %d\n", SENDER_WIFI_CHANNEL);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED"); return;
    }
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("ESP-NOW listening...");
    Serial.println("======= RECEIVER RUNNING =======");
}


// ═════════════════════════════════════════════════════════
//  Loop
// ═════════════════════════════════════════════════════════
void loop() {
    if (!newPacket) return;
    newPacket = false;

    // [STEP 5] I²C health check — confirm bus is alive before writing servos
    // If you see "I2C FAILED: 2" in Serial Monitor, your power separation
    // (Step 2 wiring) or capacitor (Step 3) still needs attention.
    Wire.beginTransmission(0x40);
    int err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("I2C FAILED: %d  — check VCC wiring and capacitor\n", err);
        return;
    }

    // [STEP 1 FIX] Clamp incoming degrees to safe range before driving servos
    float deg[6];
    for (int i = 0; i < 6; i++) {
        deg[i] = constrain(lastPkt.servo[i] / 10.0f, 0.0f, 175.0f);  // was just /10.0f
    }

    // All 6 channels explicitly set — thumb (0) and wrist (5) included
    setServo(CH_THUMB,  deg[0]);
    setServo(CH_INDEX,  deg[1]);
    setServo(CH_MIDDLE, deg[2]);
    setServo(CH_RING,   deg[3]);
    setServo(CH_PINKY,  deg[4]);
    setServo(CH_WRIST,  deg[5]);

    Serial.printf(
        "T:%.0f° I:%.0f° M:%.0f° R:%.0f° P:%.0f° W:%.0f°\n",
        deg[0], deg[1], deg[2], deg[3], deg[4], deg[5]
    );
}
