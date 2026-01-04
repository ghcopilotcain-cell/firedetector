#include "DHT22.h"
#include "Config.h"
#include <DHT.h>

DHT dht(DHT22PIN, DHT22);

void setupDHT() { dht.begin(); }

float readTemperatureSafe() {
    float t = dht.readTemperature();
    return isnan(t) ? -999.0 : t;
}
