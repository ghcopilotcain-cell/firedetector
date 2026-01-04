#include "AnalogSensor.h"
#include "Config.h"
#include <Arduino.h>
#include <MQUnifiedsensor.h>

// MQ2 Sensor Object Definition
MQUnifiedsensor MQ2("ESP32", MQ2_VOLTAGE_RESOLUTION, MQ2_ADC_BIT_RESOLUTION, MQ2PIN, "MQ-2");

// Minimal debounce - hanya 3 samples untuk respon cepat
int readAnalogFast(int pin) {
    long total = 0;
    for (int i = 0; i < 3; i++) {
        total += analogRead(pin);
        delayMicroseconds(100);  // Sangat minimal
    }
    return total / 3;
}

// Ambil nilai analog sensor ke-i
int getIRAnalogValue(int sensorIndex) {
    if (sensorIndex >= 0 && sensorIndex < 5) {
        return readAnalogFast(IR_PINS[sensorIndex]);
    }
    return 0;
}

// Ambil nilai maksimal dari semua IR sensor
int getMaxIRValue() {
    int maxVal = 0;
    for (int i = 0; i < 5; i++) {
        int val = readAnalogFast(IR_PINS[i]);
        if (val > maxVal) maxVal = val;
    }
    return maxVal;
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
    return getMaxIRValue() > THRESHOLD_FLAME;
}
