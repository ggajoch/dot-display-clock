#ifndef _SOFTSPI_H
#define _SOFTSPI_H

#if (ARDUINO >= 100)
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

#include "SoftwareSPI.h"
#include <SPI.h>

class SoftSPI : public SPIClass {
    private:
        uint8_t _delay;
        uint8_t _mosi;
        uint8_t _sck;

    public:
        SoftSPI(uint8_t mosi, uint8_t sck);
        void begin();
        void end();
        void setClockDivider(uint8_t);
        uint8_t transfer(uint8_t);
};
#endif
