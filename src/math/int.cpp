#include "math/int.h"

char *itoan(int val, char *buf, size_t len) {
    size_t i = len;
    buf[--i] = '\0';

    bool neg = val < 0;
    if (neg)
        val = -val;

    if (val == 0) {
        buf[--i] = '0';
    } else {
        while (val && i) {
            buf[--i] = (val % 10) + '0';
            val /= 10;
        }
    }

    if (neg && i > 0)
        buf[--i] = '-';

    return buf + i;
}
