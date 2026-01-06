#include "AnalogSensor.h"
#include "Config.h"
#include <Arduino.h>
#include <MQUnifiedsensor.h>

// MQ2 Sensor Object Definition
MQUnifiedsensor MQ2("ESP32", MQ2_VOLTAGE_RESOLUTION, MQ2_ADC_BIT_RESOLUTION, MQ2PIN, "MQ-2");

int readAnalogDebounced(int pin) {
    long total = 0;
    for (int i = 0; i < 10; i++) { // Dikurangi ke 10 samples agar lebih cepat
        total += analogRead(pin);
        delayMicroseconds(500);   // Dikurangi ke 500us
    }
    return total / 10;
}

void initMQ2Sensor() {
    // Konfigurasi metode regresi untuk deteksi asap
    // Rumus: PPM = a * ratio^b
    MQ2.setRegressionMethod(1);
    MQ2.setA(MQ2_A);
    MQ2.setB(MQ2_B);

    // Inisialisasi sensor
    MQ2.init();

    // Set R0 dari hasil kalibrasi
    MQ2.setR0(MQ2_R0);

    Serial.println("[MQ2] Sensor initialized with MQUnifiedsensor library");
}

float getMQ2PPM() {
    // Update sensor dan baca nilai PPM asap
    MQ2.update();
    float smokePPM = MQ2.readSensor();

    // Validasi hasil (tidak boleh negatif)
    return (smokePPM < 0) ? 0 : smokePPM;
}

bool isFlameDetected() {
    for (int i = 0; i < 5; i++) {
        if (readAnalogDebounced(IR_PINS[i]) > THRESHOLD_FLAME) return true;
    }
    return false;
}

int getIRAnalogValue() {
    // Baca kelima sensor IR dan kembalikan nilai tertinggi
    int maxValue = 0;
    for (int i = 0; i < 5; i++) {
        int value = readAnalogDebounced(IR_PINS[i]);
        if (value > maxValue) {
            maxValue = value;
        }
    }
    return maxValue;
}
