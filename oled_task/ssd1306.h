#ifndef SSD1306_H
#define SSD1306_H

#include <cstdint>
#include <cstddef>

// Driver for an SSD1306 128x64 I2C OLED, wired as:
//   SDA -> GPIO2 / physical pin 3 (shared with the MPU6500, Raspberry
//          Pi's I2C1 bus)
//   SCL -> GPIO3 / physical pin 5
// Originally wired to GPIO0/1 (I2C0), but a full-bus scan found nothing
// responds there -- unlike GPIO2/3, the Pi doesn't supply pull-up
// resistors on those pins (they're reserved for HAT EEPROM detection),
// so SDA/SCL just float. Sharing imu_task's bus works because I2C is
// multi-drop: the OLED (0x3C) and MPU6500 (0x68) coexist on the same two
// wires at different addresses. The device path is still a parameter
// rather than hardcoded, in case the wiring changes again.
class Ssd1306 {
public:
    static constexpr unsigned WIDTH  = 128;
    static constexpr unsigned HEIGHT = 64;
    static constexpr unsigned PAGES  = HEIGHT / 8; // 8 addressable text rows

    // Opens i2cDevicePath and runs the SSD1306 power-on init sequence.
    // Returns false if the device never ACKs -- wrong bus, wrong address,
    // or nothing plugged in -- checked explicitly here rather than assumed,
    // the same "connected or not" gate every other sensor in this project
    // goes through before anything tries to use it.
    bool connect(const char* i2cDevicePath, uint8_t i2cAddr = 0x3C);

    // Blanks the whole panel.
    void clear();

    // Draws ASCII text at character row `page` (0..7, 8px each) starting
    // at column `col` (0..127). Characters outside font5x7's supported
    // set render as a blank cell. Text is truncated, not wrapped, if it
    // would run past column 127.
    void drawText(uint8_t page, uint8_t col, const char* text);

    ~Ssd1306();

private:
    int     fd_   = -1;
    uint8_t addr_ = 0x3C;

    bool writeCommands(const uint8_t* cmds, size_t len);
    bool writeData(const uint8_t* data, size_t len);
    void setWindow(uint8_t pageStart, uint8_t pageEnd, uint8_t colStart, uint8_t colEnd);
};

#endif // SSD1306_H
