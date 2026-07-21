#pragma once

#include <cstdint>

#include "SPI.h"

struct inv_imu_sensor_event_t {
  int16_t gyro[3] = {0, 0, 0};
  int16_t accel[3] = {0, 0, 0};
};

// Low-level driver surface the firmware's ICM42670WithLPF subclass reaches into.
// The host tests never touch the IMU, so these are inert stubs that only need to
// compile. `icm_driver` is protected in the real library; keep it protected here
// so the subclass can access it just as it does on target.
struct inv_imu_device_t {};

enum {
  GYRO_CONFIG1_GYRO_FILT_BW_121 = 0,
  ACCEL_CONFIG1_ACCEL_FILT_BW_25 = 0,
};

inline int inv_imu_set_gyro_ln_bw(inv_imu_device_t *, int) { return 0; }
inline int inv_imu_set_accel_ln_bw(inv_imu_device_t *, int) { return 0; }

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

protected:
  inv_imu_device_t icm_driver;
};
