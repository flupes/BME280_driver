#include "bme280_sensor.h"

#include <Arduino.h>
#include <Wire.h>

#define Serial SerialUSB
// #define BME280_DEBUG

static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                        uint16_t len);

static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                       uint16_t len);

static void delay_msec(uint32_t ms) { delay(ms); };

void print_rslt(const char *str, int8_t ret) {
#ifdef BME280_DEBUG
  Serial.print(str);
  Serial.print(" = ");
  Serial.println(ret);
#endif
}

Bme280Sensor::Bme280Sensor(uint8_t addr) {
  device_.dev_id = addr;
  device_.intf = BME280_I2C_INTF;
  device_.read = &i2c_read;
  device_.write = &i2c_write;
  device_.delay_ms = &delay_msec;
}

bool Bme280Sensor::Begin() {
  int8_t rslt;
  bool ok = true;
  delay(5);  // sensor takes up to 2ms to start
  Serial.println("Bme280Sensor::Begin()");
  Wire.begin();
  // Wire.setClock(400000);
  delay(5);

  // uint8_t chip_id = read8(0x77, 0xD0);
  // print_rslt("chip id", chip_id);
  // if (chip_id != 0x60) return false;

  rslt = bme280_init(&device_);
  if (rslt) ok = false;
  print_rslt("bme280_init status", rslt);

  device_.settings.osr_h = BME280_OVERSAMPLING_1X;
  device_.settings.osr_p = BME280_OVERSAMPLING_4X;
  device_.settings.osr_t = BME280_OVERSAMPLING_2X;
  device_.settings.filter = BME280_FILTER_COEFF_2;

  uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL |
                         BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  rslt = bme280_set_sensor_settings(settings_sel, &device_);
  print_rslt(" bme280_set_sensor_settings", rslt);

  /* Always set the power mode after setting the configuration */
  // rslt = bme280_set_power_mode(bme280_SLEEP_MODE, &device_);
  // if (rslt) ok = false;
  // print_rslt("bme280_set_power_mode(sleep) status", rslt);

  return ok;
}

int8_t Bme280Sensor::PerformMeasurement() {
  int8_t rslt;
  // bme280_get_status(&status_, &device_);
  // Serial.print("Status before FORCED_MODE = ");
  // Serial.println(status_.measuring);

  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &device_);
  print_rslt("bme280_set_sensor_mode(forced) status", rslt);
  delay(40);

  // Waiting for measurement to be performed
  // while (status_.measuring == bme280_MEAS_ONGOING) {
  //   delay(1);
  //   bme280_get_status(&status_, &device_);
  // }
  // while (status_.measuring == bme280_MEAS_DONE) {
  //   delay(1);
  //   bme280_get_status(&status_, &device_);
  // }

  // Read the data now that the device is back to sleep
  rslt = bme280_get_sensor_data(BME280_ALL, &data_, &device_);
  print_rslt("bme280_get_sensor_data", rslt);
  return rslt;
}

uint32_t Bme280Sensor::GetPressure() {
  return data_.pressure;
}

uint32_t Bme280Sensor::GetTemperature() {
  return data_.temperature;
}

uint32_t Bme280Sensor::GetHumidity() {
  return data_.humidity;
}

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                uint16_t len) {
#ifdef BME280_DEBUG
  Serial.print("\tI2C $");
  Serial.print(reg_addr, HEX);
  Serial.print(" => ");
#endif

  Wire.beginTransmission((uint8_t)dev_id);
  Wire.write((uint8_t)reg_addr);
  Wire.endTransmission();
  if (len != Wire.requestFrom((uint8_t)dev_id, (byte)len)) {
#ifdef BME280_DEBUG
    Serial.print("Failed to read ");
    Serial.print(len);
    Serial.print(" bytes from ");
    Serial.println(dev_id, HEX);
#endif
    return 1;
  }
  while (len--) {
    *reg_data = (uint8_t)Wire.read();
#ifdef BM2680_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }
#ifdef BME280_DEBUG
  Serial.println("");
#endif
  return 0;
}

/**************************************************************************/
/*!
    @brief  Writes 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                 uint16_t len) {
#ifdef BME280_DEBUG
  Serial.print("\tI2C $");
  Serial.print(reg_addr, HEX);
  Serial.print(" <= ");
#endif
  Wire.beginTransmission((uint8_t)dev_id);
  Wire.write((uint8_t)reg_addr);
  while (len--) {
    Wire.write(*reg_data);
#ifdef BME280_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }
  Wire.endTransmission();
#ifdef BME280_DEBUG
  Serial.println("");
#endif
  return 0;
}
