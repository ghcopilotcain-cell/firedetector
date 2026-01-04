#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

void initMQ2Sensor();
float getMQ2PPM();
bool isFlameDetected();
int getIRAnalogValue(int sensorIndex);  // Ambil nilai analog sensor ke-i
int getMaxIRValue();                     // Ambil nilai maksimal dari semua IR sensor

#endif
