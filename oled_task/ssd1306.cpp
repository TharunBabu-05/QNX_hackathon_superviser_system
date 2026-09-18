#include "ssd1306.h"
#include "font5x7.h"

#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <hw/i2c.h>
#include <devctl.h>

namespace {
constexpr uint8_t CTRL_COMMAND = 0x00; // Co=0, D/C#=0 -- rest of payload is commands
constexpr uint8_t CTRL_DATA    = 0x40; // Co=0, D/C#=1 -- rest of payload is GDDRAM data

// One transaction covers a full 128-byte page write plus a small command
// sequence, so this needs to be at least WIDTH + a few bytes of margin.
constexpr size_t MAX_PAYLOAD = Ssd1306::WIDTH + 4;

// Standard SSD1306 128x64 power-on sequence: internal charge pump (this
// module has no external VCC supply for the panel), horizontal addressing
// so consecutive writes auto-advance column-then-page, and the COM/segment
// remap pair that matches how these modules are conventionally wired.
constexpr uint8_t INIT_CMDS[] = {
    0xAE,       // display off
    0xD5, 0x80, // clock divide ratio / oscillator frequency
    0xA8, 0x3F, // multiplex ratio = 64
    0xD3, 0x00, // display offset = 0
    0x40,       // display start line = 0
    0x8D, 0x14, // charge pump: enable
    0x20, 0x00, // memory addressing mode: horizontal
    0xA1,       // segment remap (column 127 = SEG0)
    0xC8,       // COM output scan direction: remapped
    0xDA, 0x12, // COM pins hardware config
    0x81, 0xCF, // contrast
    0xD9, 0xF1, // pre-charge period
    0xDB, 0x40, // VCOMH deselect level
    0xA4,       // entire display follows GDDRAM content (not all-on)
    0xA6,       // normal display (not inverted)
    0xAF,       // display on
};

bool i2cSend(int fd, uint8_t addr, uint8_t ctrl, const uint8_t* payload, size_t len) {
    if (len > MAX_PAYLOAD) return false;
    uint8_t buf[sizeof(i2c_send_t) + 1 + MAX_PAYLOAD];
    auto* hdr = reinterpret_cast<i2c_send_t*>(buf);
    hdr->slave.addr = addr;
    hdr->slave.fmt  = I2C_ADDRFMT_7BIT;
    hdr->len        = static_cast<uint16_t>(1 + len);
    hdr->stop       = 1;
    buf[sizeof(i2c_send_t)] = ctrl;
    memcpy(buf + sizeof(i2c_send_t) + 1, payload, len);
    return devctl(fd, DCMD_I2C_SEND, buf, sizeof(i2c_send_t) + 1 + len, NULL) == EOK;
}

const uint8_t* glyphFor(char c) {
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A'); // tolerate lowercase input
    for (unsigned i = 0; i < font5x7::TABLE_SIZE; ++i) {
        if (font5x7::TABLE[i].c == c) return font5x7::TABLE[i].bits;
    }
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    return blank; // unsupported character -- render as a blank cell, not garbage
}
}

bool Ssd1306::connect(const char* i2cDevicePath, uint8_t i2cAddr) {
    addr_ = i2cAddr;
    fd_ = open(i2cDevicePath, O_RDWR);
    if (fd_ == -1) {
        perror("open(OLED i2c device) failed");
        return false;
    }
    if (!writeCommands(INIT_CMDS, sizeof(INIT_CMDS))) {
        fprintf(stderr,
                "OLED: init sequence not ACKed on %s at 0x%02X -- check "
                "SDA/SCL wiring/power, or wrong I2C bus/address\n",
                i2cDevicePath, addr_);
        close(fd_);
        fd_ = -1;
        return false;
    }
    clear();
    return true;
}

bool Ssd1306::writeCommands(const uint8_t* cmds, size_t len) {
    return i2cSend(fd_, addr_, CTRL_COMMAND, cmds, len);
}

bool Ssd1306::writeData(const uint8_t* data, size_t len) {
    return i2cSend(fd_, addr_, CTRL_DATA, data, len);
}

void Ssd1306::setWindow(uint8_t pageStart, uint8_t pageEnd, uint8_t colStart, uint8_t colEnd) {
    const uint8_t cmds[] = {0x21, colStart, colEnd, 0x22, pageStart, pageEnd};
    writeCommands(cmds, sizeof(cmds));
}

void Ssd1306::clear() {
    uint8_t zeros[WIDTH] = {0};
    for (unsigned page = 0; page < PAGES; ++page) {
        setWindow(static_cast<uint8_t>(page), static_cast<uint8_t>(page), 0, WIDTH - 1);
        writeData(zeros, WIDTH);
    }
}

void Ssd1306::drawText(uint8_t page, uint8_t col, const char* text) {
    if (page >= PAGES || col >= WIDTH) return;

    // Always rewrites the *whole* rest of the row, not just the bytes the
    // new text needs. Drawing only `text`'s own glyph bytes left the tail
    // of whatever previously occupied this row on screen whenever the new
    // string was shorter (e.g. "US1:134CM" -> "US1: 87CM", or "VEHICLE
    // MOVING" -> "VEHICLE STOPPED" landing at a different length) -- stale
    // pixels that looked like flicker/ghosting on every update instead of
    // a clean redraw.
    const size_t rowLen = WIDTH - col;
    uint8_t      buf[WIDTH];
    size_t       used = 0;
    for (const char* p = text; *p != '\0' && used + 6 <= rowLen; ++p) {
        const uint8_t* glyph = glyphFor(*p);
        memcpy(buf + used, glyph, 5);
        buf[used + 5] = 0x00;
        used += 6;
    }
    memset(buf + used, 0x00, rowLen - used);

    setWindow(page, page, col, static_cast<uint8_t>(WIDTH - 1));
    writeData(buf, rowLen);
}

Ssd1306::~Ssd1306() {
    if (fd_ != -1) close(fd_);
}
