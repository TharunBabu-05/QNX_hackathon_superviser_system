#ifndef MPU6500_H
#define MPU6500_H

// Driver for an MPU6500 6-axis IMU (accel + gyro) over I2C, wired as:
//   SDA -> GPIO2  / physical pin 3  (Raspberry Pi's I2C1 bus -> /dev/i2c1)
//   SCL -> GPIO3  / physical pin 5
//   INT -> GPIO17 / physical pin 11
// The INT line signals "new sample ready" and would let this run
// interrupt-driven instead of polled, but wiring a GPIO edge to a QNX
// InterruptAttach() vector number is board/BSP-specific data this driver
// doesn't have -- so it polls on a timer instead. INT is left connected
// for whoever adds that later; it is not read by this code.
class Mpu6500 {
public:
    // Opens /dev/i2c1, confirms the WHO_AM_I register matches an MPU6500,
    // and wakes the device from its power-on sleep state. Returns false
    // (object left unusable) if any step fails -- wrong wiring, wrong
    // device on the bus, or the bus driver isn't running.
    bool connect();

    struct Sample {
        float accelG[3];  // +/-2g range (power-on default)
        float gyroDps[3]; // +/-250 deg/s range (power-on default)
    };

    // Reads one accel+gyro sample. Returns false on an I2C transaction
    // failure (bus error, NACK, device unplugged mid-run).
    bool read(Sample& out);

    ~Mpu6500();

private:
    int fd_ = -1;
};

#endif // MPU6500_H
