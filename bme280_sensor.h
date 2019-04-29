#include "bme280.h"

#ifndef DIGIBARO_BME280_SENSOR_H
#define DIGIBARO_BME280_SENSOR_H

class Bme280Sensor {
 public:
  Bme280Sensor(uint8_t addr);

  bool Begin();

  int8_t PerformMeasurement();

  uint32_t GetPressure();

  uint32_t GetTemperature();

  uint32_t GetHumidity();

 private:
  uint32_t pressure_;
  int32_t temperature_;
  struct bme280_dev device_;
  struct bme280_data data_;
};

#endif
