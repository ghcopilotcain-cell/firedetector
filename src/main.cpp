#define BLYNK_TEMPLATE_ID "TMPL6fNFvhHxH"
#define BLYNK_TEMPLATE_NAME "Fire Detector"
#define BLYNK_PRINT Serial

#include "WiFi.h"
#include "BlynkEdgent.h"
#include "Config.h"
#include "DHT22.h"
#include "AnalogSensor.h"

// Watchdog Vars
unsigned long lastConnectAttempt = 0;
unsigned long startupTime = 0;
int connectFailures = 0;
bool isResetting = false;
const unsigned long STARTUP_GRACE_PERIOD = 5000;  // 5 detik grace period saat startup

// State Vars
unsigned int dangerCount = 0;
bool lastDangerState = false;
bool lastWarningState = false;
float temp_value, smoke_value;

// Timer Intervals
unsigned long lastFastCheck = 0;
unsigned long lastSlowCheck = 0;

void setup() {
    Serial.begin(115200);
    startupTime = millis();  // Catat waktu startup untuk grace period

    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    // Setup LEDC for buzzer PWM (tone)
    ledcSetup(0, 5000, 8);  // Channel 0, 5kHz, 8-bit resolution
    ledcAttachPin(BUZZER, 0);

    setupDHT();
    initMQ2Sensor();
    BlynkEdgent.begin();
    lastConnectAttempt = millis();
}

void loop() {
    BlynkEdgent.run();
    unsigned long now = millis();

    // 1. WATCHDOG KONEKSI (IMPROVED)
    // Tracking koneksi WiFi dan Cloud secara terpisah
    // SKIP watchdog selama startup grace period
    if ((BlynkState::is(MODE_CONNECTING_NET) || BlynkState::is(MODE_CONNECTING_CLOUD)) && !isResetting) {
        // Grace period: jangan monitor watchdog dalam 5 detik pertama setelah startup
        bool inGracePeriod = (now - startupTime) < STARTUP_GRACE_PERIOD;

        if (!inGracePeriod && (now - lastConnectAttempt) > CONNECT_TIMEOUT_MS) {
            connectFailures++;
            lastConnectAttempt = now;

            if (BlynkState::is(MODE_CONNECTING_NET)) {
                Serial.printf("[WATCHDOG] WiFi timeout ke-%d (timeout %ldms)\n", connectFailures, CONNECT_TIMEOUT_MS);
            } else {
                Serial.printf("[WATCHDOG] Cloud timeout ke-%d (timeout %ldms)\n", connectFailures, CONNECT_TIMEOUT_MS);
            }

            // Beri lebih banyak kesempatan sebelum reset
            if (connectFailures >= MAX_FAILURES) {
                Serial.println("[WATCHDOG] !!! Max failures reached. Returning to config mode... !!!");
                Serial.println("[WATCHDOG] Please check your WiFi credentials and power supply stability.");

                // Reset ke MODE_WAIT_CONFIG daripada reset config (preservasi credentials)
                isResetting = true;
                BlynkState::set(MODE_WAIT_CONFIG);

                // Optional: Jika ingin reset credentials juga, uncomment:
                // configStore = configDefault;
                // config_save();
                // BlynkState::set(MODE_RESET_CONFIG);
            }
        }
    } else if (BlynkState::is(MODE_RUNNING)) {
        // Reset counter hanya saat berhasil terhubung ke cloud
        lastConnectAttempt = now;
        connectFailures = 0;
        isResetting = false;
    }
    // JANGAN reset counter saat MODE_WAIT_CONFIG atau MODE_CONFIGURING untuk tracking yang konsisten

    // 2. FAST CHECK (100ms): Respon cepat untuk API & ASAP
    if (now - lastFastCheck >= 100) {
        smoke_value = getMQ2PPM();
        int flameAnalog = getMaxIRValue();
        bool flameDetected = isFlameDetected();
        bool smokeDetected = smoke_value > THRESHOLD_SMOKE;

        bool dangerNow = flameDetected || (temp_value > THRESHOLD_TEMP && smokeDetected);
        bool warningNow = !dangerNow && (smokeDetected || temp_value > THRESHOLD_TEMP);

        if (dangerNow) {
            digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, LOW); digitalWrite(LED_YELLOW, LOW);
            ledcWriteTone(0, 1000);  // Channel 0, 1000 Hz
            if (!lastDangerState) {
                Blynk.logEvent("bahaya", "BAHAYA API!");
                dangerCount++;
            }
        } else if (warningNow) {
            digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_GREEN, LOW); digitalWrite(LED_RED, LOW);
            ledcWriteTone(0, 0);  // Turn off buzzer
            if (!lastWarningState) Blynk.logEvent("waspada", "Asap/Suhu Meningkat");
        } else {
            digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_RED, LOW);
            ledcWriteTone(0, 0);  // Turn off buzzer
        }

        lastDangerState = dangerNow;
        lastWarningState = warningNow;
        lastFastCheck = now;
    }

    // 3. SLOW CHECK (2000ms): Update DHT & Kirim ke Blynk + Serial Monitor
    if (now - lastSlowCheck >= 2000) {
        temp_value = readTemperatureSafe();
        int ir0 = getIRAnalogValue(0);
        int ir1 = getIRAnalogValue(1);
        int ir2 = getIRAnalogValue(2);
        int ir3 = getIRAnalogValue(3);
        int ir4 = getIRAnalogValue(4);
        int irMax = getMaxIRValue();

        String kondisi = lastDangerState ? "Bahaya" : (lastWarningState ? "Waspada" : "Aman");

        // Kirim ke Blynk
        Blynk.virtualWrite(V0, temp_value);
        Blynk.virtualWrite(V1, smoke_value);
        Blynk.virtualWrite(V2, irMax);  // Kirim nilai analog IR flame sensor
        Blynk.virtualWrite(V3, kondisi);
        Blynk.virtualWrite(V4, dangerCount);

        // Tampilkan di Serial Monitor untuk debug
        Serial.println("========== SENSOR READINGS ==========");
        Serial.printf("Status: %s\n", kondisi.c_str());
        Serial.printf("Temp: %.1f°C | Asap (MQ2): %.1f PPM\n", temp_value, smoke_value);
        Serial.printf("IR Flame Sensors (0-4): %4d | %4d | %4d | %4d | %4d\n", ir0, ir1, ir2, ir3, ir4);
        Serial.printf("Max IR Value: %d (Threshold: %d)\n", irMax, THRESHOLD_FLAME);
        Serial.printf("Danger Count: %d\n", dangerCount);
        Serial.println("====================================");

        lastSlowCheck = now;
    }
}
