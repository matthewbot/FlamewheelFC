#ifndef FC_DRIVERS_UART_H
#define FC_DRIVERS_UART_H

void uart_init();
void uart_puts(const char *out);
void uart_putint(int i);

struct EndlObj { };
const EndlObj endl;

struct UARTObj {
    const UARTObj &operator<<(const char *out) const { uart_puts(out); return *this; }
    const UARTObj &operator<<(int i) const { uart_putint(i); return *this; }
    const UARTObj &operator<<(const EndlObj &) const { uart_puts("\r\n"); return *this; }
};

const UARTObj uart;

#endif
