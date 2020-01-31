#ifndef ARDUINO_UTILS_HPP
#define ARDUINO_UTILS_HPP

#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/Timestamp.h>


#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

typedef uint8_t byte;

#define F(s) s

void delay(int ms);
void delayMicroseconds(int microseconds);
uint32_t millis();
int msToTicks(int ms);
int ticksToMs(int ticks);

class DebugStream {
    public:
        void putc(char c);

        void print(const char* s);
        void println(const char* s);

        void print(uint8_t u, int mode = DEC);
        void println(uint8_t u, int mode = DEC);

        void print(uint16_t u, int mode = DEC);
        void println(uint16_t u, int mode = DEC);

        void print(uint32_t u, int mode = DEC);
        void println(uint32_t u, int mode = DEC);

        void print(unsigned long u);
        void println(unsigned long u);

        void print(float f);
        void println(float f);

        void println();
};

#endif // ARDUINO_UTILS_HPP
