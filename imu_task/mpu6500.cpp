#include "mpu6500.h"

#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <hw/i2c.h>
#include <devctl.h>

namespace {
constexpr uint32_t MPU6500_ADDR = 0x68; // AD0 tied low; 0x69 if AD0 is tied high instead

constexpr uint8_t REG_WHO_AM_I     = 0x75;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B; // accel(6) + temp(2) + gyro(6) = 14 contiguous regs
constexpr uint8_t MPU6500_WHO_AM_I_VALUE = 0x70;

constexpr float ACCEL_LSB_PER_G  = 16384.0f; // for the +/-2g power-on default range
constexpr float GYRO_LSB_PER_DPS = 131.0f;   // for the +/-250dps power-on default range

// Writes one register: devctl(DCMD_I2C_SEND) with an i2c_send_t header
// immediately followed by the bytes to send -- here, [register, value].
bool i2cWriteReg(int fd, uint8_t reg, uint8_t value) {
    struct {
        i2c_send_t hdr;
        uint8_t    data[2];
    } msg;
    msg.hdr.slave.addr = MPU6500_ADDR;
    msg.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.hdr.len  = sizeof(msg.data);
    msg.hdr.stop = 1;
    msg.data[0] = reg;
    msg.data[1] = value;
    return devctl(fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL) == EOK;
}

// Reads `len` bytes starting at register `reg`. MPU6500 registers are read
// by writing the register address (no stop) then a repeated-start read of
// the value -- DCMD_I2C_SENDRECV does both halves as one bus transaction.
// The driver reuses the same data region for both directions: the register
// address is written at the offset right after the header, and the bytes
// read back overwrite that same offset (there's no reason to keep the sent
// byte once it's been clocked out), so the buffer only needs to be as big
// as the larger of send_len/recv_len, not their sum.
bool i2cReadRegs(int fd, uint8_t reg, uint8_t* out, unsigned len) {
    if (len > 32) return false; // driver used for small register blocks only

    uint8_t buf[sizeof(i2c_sendrecv_t) + 32];
    auto* hdr = reinterpret_cast<i2c_sendrecv_t*>(buf);
    hdr->slave.addr = MPU6500_ADDR;
    hdr->slave.fmt  = I2C_ADDRFMT_7BIT;
    hdr->send_len   = 1;
    hdr->recv_len   = len;
    hdr->stop       = 1;
    buf[sizeof(i2c_sendrecv_t)] = reg;

    const unsigned dataLen = (len > 1) ? len : 1;
    if (devctl(fd, DCMD_I2C_SENDRECV, buf, sizeof(i2c_sendrecv_t) + dataLen, NULL) != EOK) {
        return false;
    }
    memcpy(out, buf + sizeof(i2c_sendrecv_t), len);
    return true;
}

int16_t be16(const uint8_t* p) {
    return static_cast<int16_t>((p[0] << 8) | p[1]);
}
}

bool Mpu6500::connect() {
    fd_ = open("/dev/i2c1", O_RDWR);
    if (fd_ == -1) {
        perror("open(/dev/i2c1) failed");
        return false;
    }

    uint8_t whoAmI = 0;
    if (!i2cReadRegs(fd_, REG_WHO_AM_I, &whoAmI, 1)) {
        fprintf(stderr, "IMU: WHO_AM_I read failed -- check SDA/SCL wiring\n");
        return false;
    }
    if (whoAmI != MPU6500_WHO_AM_I_VALUE) {
        fprintf(stderr,
                "IMU: unexpected WHO_AM_I 0x%02X (expected 0x%02X) -- wrong "
                "device on the bus, or this isn't an MPU6500\n",
                whoAmI, MPU6500_WHO_AM_I_VALUE);
        return false;
    }

    // Clears the sleep bit the device powers up in and selects the
    // internal oscillator.
    if (!i2cWriteReg(fd_, REG_PWR_MGMT_1, 0x00)) {
        fprintf(stderr, "IMU: failed to wake device (PWR_MGMT_1 write)\n");
        return false;
    }
    return true;
}

bool Mpu6500::read(Sample& out) {
    uint8_t raw[14];
    if (!i2cReadRegs(fd_, REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        return false;
    }

    out.accelG[0] = be16(&raw[0]) / ACCEL_LSB_PER_G;
    out.accelG[1] = be16(&raw[2]) / ACCEL_LSB_PER_G;
    out.accelG[2] = be16(&raw[4]) / ACCEL_LSB_PER_G;
    // raw[6..7] is temperature -- not exposed by this driver.
    out.gyroDps[0] = be16(&raw[8])  / GYRO_LSB_PER_DPS;
    out.gyroDps[1] = be16(&raw[10]) / GYRO_LSB_PER_DPS;
    out.gyroDps[2] = be16(&raw[12]) / GYRO_LSB_PER_DPS;
    return true;
}

Mpu6500::~Mpu6500() {
    if (fd_ != -1) close(fd_);
}
