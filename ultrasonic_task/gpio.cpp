#include "gpio.h"

#include <sys/mman.h>
#include <sys/neutrino.h>
#include <cstdio>
#include <cstdlib>

namespace {
// BCM2711 (Raspberry Pi 4B) peripheral base is 0xFE000000; the GPIO
// controller sits at offset 0x200000 within it. This differs from the
// Pi 1/2/3 (BCM283x) base of 0x3F000000 -- these addresses are not
// portable across Pi models.
constexpr uintptr_t GPIO_PHYS_BASE = 0xFE200000;
constexpr size_t    GPIO_MAP_LEN   = 4096; // one page; covers all regs used below

// Register offsets in 32-bit words from GPIO_PHYS_BASE.
constexpr unsigned GPFSEL0 = 0x00 / 4; // GPFSEL0..GPFSEL5: function select, 3 bits/pin
constexpr unsigned GPSET0  = 0x1C / 4; // GPSET0/1: write 1 to drive a pin high
constexpr unsigned GPCLR0  = 0x28 / 4; // GPCLR0/1: write 1 to drive a pin low
constexpr unsigned GPLEV0  = 0x34 / 4; // GPLEV0/1: read current pin level
}

Gpio::Gpio() : regs_(nullptr), regsLen_(GPIO_MAP_LEN) {
    // Grants this thread I/O privilege. Requires the process to be run
    // with sufficient privilege on the target (root over qconn); without
    // it, mmap_device_memory() below fails with EPERM.
    if (ThreadCtl(_NTO_TCTL_IO, NULL) == -1) {
        perror("ThreadCtl(_NTO_TCTL_IO) failed -- run as root on the target");
        exit(EXIT_FAILURE);
    }

    void* addr = mmap_device_memory(NULL, GPIO_MAP_LEN,
                                     PROT_READ | PROT_WRITE | PROT_NOCACHE,
                                     0, GPIO_PHYS_BASE);
    if (addr == MAP_FAILED) {
        perror("mmap_device_memory(GPIO) failed");
        exit(EXIT_FAILURE);
    }
    regs_ = static_cast<volatile uint32_t*>(addr);
}

Gpio::~Gpio() {
    if (regs_) {
        munmap_device_memory(const_cast<uint32_t*>(regs_), regsLen_);
    }
}

void Gpio::setMode(unsigned pin, Mode mode) {
    // Each GPFSELn packs 10 pins x 3 bits; locate the register and the
    // bit field within it for this pin.
    const unsigned reg   = GPFSEL0 + (pin / 10);
    const unsigned shift = (pin % 10) * 3;

    uint32_t val = regs_[reg];
    val &= ~(0x7u << shift);           // 000 = input (also the power-on default)
    if (mode == Mode::Output) {
        val |= (0x1u << shift);        // 001 = output
    }
    regs_[reg] = val;
}

void Gpio::write(unsigned pin, bool high) {
    // GPSET/GPCLR are "write 1 to act" registers: writing 0 to other
    // bits is a no-op, so no read-modify-write is needed here.
    const unsigned reg = (high ? GPSET0 : GPCLR0) + (pin / 32);
    regs_[reg] = (1u << (pin % 32));
}

bool Gpio::read(unsigned pin) const {
    return (regs_[GPLEV0 + (pin / 32)] >> (pin % 32)) & 0x1u;
}
