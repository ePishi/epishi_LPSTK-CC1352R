#include <GPS/ArduinoUtils.hpp>
#include <stdio.h>
#include <ti/sysbios/knl/Task.h>

void delay(int ms) {
    Task_sleep((ms * 1000) / Clock_tickPeriod);
}

void delayMicroseconds(int us) {
    Task_sleep((us * 1) / Clock_tickPeriod);
}

uint32_t millis() {
    return ((Clock_getTicks() * Clock_tickPeriod) / 1000);
}

int msToTicks(int ms) {
    return  ((ms * 1000) / Clock_tickPeriod);
}

int ticksToMs(int ticks) {
    return ((ticks * Clock_tickPeriod) / 1000);
}

void DebugStream::putc(char c) {
    putchar(c);
}

void DebugStream::print(const char* s) {
    printf(s);
}

void DebugStream::println(const char* s) {
    printf(s);
    printf("\n");
}

void DebugStream::print(uint8_t u, int mode) {
    switch (mode) {
        case HEX:
            printf("%02X", u);
        case DEC:
        default:
            printf("%u", u);
    }
}

void DebugStream::println(uint8_t u, int mode) {
    print(u, mode);
    printf("\n");
}

void DebugStream::print(uint16_t u, int mode) {
    switch (mode) {
        case HEX:
            printf("%04X", u);
        case DEC:
        default:
            printf("%u", u);
    }
}

void DebugStream::println(uint32_t u, int mode) {
    print(u, mode);
    printf("\n");
}

void DebugStream::print(uint32_t u, int mode) {
    switch (mode) {
        case HEX:
            printf("%08X", u);
        case DEC:
        default:
            printf("%u", u);
    }
}

void DebugStream::println(uint16_t u, int mode) {
    print(u, mode);
    printf("\n");
}

void DebugStream::print(unsigned long u) {
    printf("%lu", u);
}

void DebugStream::println(unsigned long u) {
    print(u);
    printf("\n");
}

void DebugStream::print(float f) {
    printf("%f", f);
}

void DebugStream::println(float f) {
    print(f);
    printf("\n");
}

void DebugStream::println() {
    printf("\n");
}
