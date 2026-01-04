#ifndef CONFIG_H
#define CONFIG_H

// SENSOR PINS
#define MQ2PIN 34
#define DHT22PIN 4
const int IR_PINS[] = {32, 33, 36, 39, 35};

// MQ-2 CALIBRATION CONSTANTS (MQUnifiedsensor)
#define MQ2_VOLTAGE_RESOLUTION 5      // ESP32 menggunakan 3.3V
#define MQ2_ADC_BIT_RESOLUTION 12       // ADC ESP32 adalah 12-bit
#define MQ2_A 3697.4                    // Koefisien untuk rumus PPM = a*ratio^b
#define MQ2_B -3.109                    // Eksponent untuk rumus PPM = a*ratio^b
#define MQ2_R0 32.95                    // Reference Resistance (SESUAIKAN DENGAN HASIL KALIBRASI ANDA!)

// OUTPUT PINS
#define LED_RED 17
#define LED_YELLOW 18
#define LED_GREEN 19
#define BUZZER 16

// THRESHOLDS
#define THRESHOLD_TEMP 40
#define THRESHOLD_SMOKE 51
#define THRESHOLD_FLAME 1500

// BLYNK WATCHDOG
#define CONNECT_TIMEOUT_MS 30000UL     // Timeout 30 detik (diperpanjang dari 15s untuk power adaptor)
#define MAX_FAILURES 1                 // Max 5 kali gagal (diperbanyak dari 2 untuk stabilitas)

#endif
