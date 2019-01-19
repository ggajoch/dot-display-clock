#include "SoftwareSPI.h"

SoftSPI::SoftSPI(uint8_t mosi, uint8_t sck) {
    _mosi = mosi;
    _sck = sck;
    _delay = 2;
}

void SoftSPI::begin() {
    pinMode(_mosi, OUTPUT);
    pinMode(_sck, OUTPUT);
}

void SoftSPI::end() {
    pinMode(_mosi, INPUT);
    pinMode(_sck, INPUT);
}

void SoftSPI::setClockDivider(uint8_t div) {
    switch (div) {
        case SPI_CLOCK_DIV2:
            _delay = 2;
            break;
        case SPI_CLOCK_DIV4:
            _delay = 4;
            break;
        case SPI_CLOCK_DIV8:
            _delay = 8;
            break;
        case SPI_CLOCK_DIV16:
            _delay = 16;
            break;
        case SPI_CLOCK_DIV32:
            _delay = 32;
            break;
        case SPI_CLOCK_DIV64:
            _delay = 64;
            break;
        case SPI_CLOCK_DIV128:
            _delay = 128;
            break;
        default:
            _delay = 128;
            break;
    }
}

uint8_t SoftSPI::transfer(uint8_t val) {
    uint8_t del = _delay >> 1;

    uint8_t bval = 0;
    for (int8_t bit = 7; bit >= 0; bit--) {
        digitalWrite(_mosi, val & (1<<bit) ? HIGH : LOW);

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }

        digitalWrite(_sck, LOW);

        for (uint8_t i = 0; i < del; i++) {
            asm volatile("nop");
        }

        digitalWrite(_sck, HIGH);
    }
    return 0;
}
