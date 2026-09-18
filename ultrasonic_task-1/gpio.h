#ifndef GPIO_H
#define GPIO_H

#include <cstdint>
#include <cstddef>

// Direct register-level GPIO access for the Raspberry Pi 4B (BCM2711).
//
// QNX is a microkernel: there is no GPIO driver built into the kernel.
// A normal user process has no hardware access by default -- this class
// must explicitly ask procnto for I/O privilege and then map the SoC's
// physical GPIO register block into its own address space. That request
// (see gpio.cpp) is the only "privileged" part of this program; everything
// else here is plain memory-mapped I/O.
class Gpio {
public:
    enum class Mode { Input, Output };

    // Requests I/O privilege and maps the GPIO register block.
    // Exits the process on failure: without the mapping nothing below
    // can work, and there is no degraded mode for a GPIO driver itself.
    Gpio();
    ~Gpio();

    Gpio(const Gpio&) = delete;
    Gpio& operator=(const Gpio&) = delete;

    void setMode(unsigned pin, Mode mode);
    void write(unsigned pin, bool high);
    bool read(unsigned pin) const;

private:
    volatile uint32_t* regs_;
    size_t regsLen_;
};

#endif // GPIO_H
