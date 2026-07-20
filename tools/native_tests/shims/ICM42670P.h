#pragma once

#include <cstdint>

#include "SPI.h"

struct inv_imu_sensor_event_t {
  int16_t gyro[3] = {0, 0, 0};
  int16_t accel[3] = {0, 0, 0};
};

class ICM42670 {
public:
  ICM42670(SPIClass &, int) {}

  int begin() { return 0; }
  int startAccel(int, int) { return 0; }
  int startGyro(int, int) { return 0; }

  int getDataFromRegisters(inv_imu_sensor_event_t &event) {
    event = next_event;
    return read_status;
  }

  inv_imu_sensor_event_t next_event = {};
  int read_status = 0;
};
