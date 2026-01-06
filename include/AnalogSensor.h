#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

void initMQ2Sensor();
float getMQ2PPM();
bool isFlameDetected();
int getIRAnalogValue();

#endif
