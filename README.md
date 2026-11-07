# NeuralNetRobot

The sensor constructors configure their devices on the supplied I2C bus. Keep the
motoprs disabled during startup. Initialization errors use `ESP_ERROR_CHECK`;
runtime read errors return `false` and invalidate the cached sample.

```cpp
MPU6500 imu{bus};
VL53L0XV2 distance_sensor{bus};

// Keep the robot stationary while measuring the gyro bias.
if (!imu.calibrate_gyro()) {
  // Leave the motors disabled and handle the startup failure.
  return;
}
```

- `update()` returns `true` only when a new valid sample is available. A `false`
  return can also mean the sensor is not ready yet use `is_valid()` to check
  whether its cached reading is still usable. Both sensors default to a 100 ms
  maximum sample age, which can be overridden with `is_valid(max_age_ms)` if .
- MPU6500 raw getters keep returning signed counts.`get_ax_m_s2()` /
  `get_ay_m_s2()` / `get_az_m_s2()` getters return acceleration including gravity.
  `get_gx_rad_s()` / `get_gy_rad_s()` / `get_gz_rad_s()` return angular velocity
  with the calibrated gyro bias removed. Check validity eith `is_valid()` before using the values.
- The IMU uses a 100 Hz sample rate, low-pass filters, a +/-2 g accelerometer
  range, and a +/-250 degrees/s gyro range. 
- VL53L0XV2 starts continuous ranging after reference calibration.`update()` updates the sensor. 
`get_dist_mm()` must be used in order to get a distance measurement.

The distance sensor startup sequence is adapted from
[Pololu's VL53L0X library](https://github.com/pololu/vl53l0x-arduino), with notices
in `src/components/VL53L0X-LICENSE.txt`. IMU configuration follows the
[MPU6500 register map](https://www.dessy.ru/include/images/ware/pdf/m/MPU-6500-Register-Map.pdf).

main.cpp does not yet contain a finished control loop implementation.
