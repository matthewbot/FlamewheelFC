#ifndef FC_DRIVERS_UART_H
#define FC_DRIVERS_UART_H

#include <stdlib.h>

void uart_init();
void uart_putch(char ch);
void uart_puts(const char *out);
void uart_putint(int i);
char uart_getch();
void uart_gets(char *buf, size_t len);
size_t uart_avail();

struct EndlObj { };
const EndlObj endl;

struct UARTObj {
    const UARTObj &operator<<(const char *out) const { uart_puts(out); return *this; }
    const UARTObj &operator<<(char ch) const { uart_putch(ch); return *this; }
    const UARTObj &operator<<(int i) const { uart_putint(i); return *this; }
    const UARTObj &operator<<(float f) const { uart_putint(static_cast<int>(f)); return *this; }
    const UARTObj &operator<<(const EndlObj &) const { uart_putch('\n'); return *this; }

    const UARTObj &operator>>(char &ch) const { ch = uart_getch(); return *this; }
    template <size_t N>
    const UARTObj &operator>>(char (&buf)[N]) const { uart_gets(buf, N); return *this; }
};

const UARTObj uart;

#endif
