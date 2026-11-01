/*
 * miniprintf.c — Minimal printf for freestanding bare-metal firmware.
 * Supports %c, %s, %d, %u, %x, %X, %p, %%. No floating point.
 * Output goes through _write() (retargetable to USART).
 */

#include "miniprintf.h"
#include "usart.h"

#include <stdarg.h>
#include <stdint.h>

static void print_char(char c, int *count)
{
    _write(1, &c, 1);
    (*count)++;
}

static void print_string(const char *s, int *count)
{
    while (*s) {
        print_char(*s++, count);
    }
}

static void print_unsigned(unsigned long val, int base, int *count)
{
    char buf[32];
    int i = 0;
    const char *digits = "0123456789abcdef";

    if (val == 0) {
        print_char('0', count);
        return;
    }

    while (val > 0) {
        buf[i++] = digits[val % base];
        val /= base;
    }

    while (i > 0) {
        print_char(buf[--i], count);
    }
}

static void print_long(long val, int base, int *count)
{
    if (val < 0) {
        print_char('-', count);
        print_unsigned((unsigned long)(-(val + 1)) + 1, base, count);
    } else {
        print_unsigned((unsigned long)val, base, count);
    }
}

/* _write retarget — transmit via USART1 (polled) */
int _write(int file, char *ptr, int len)
{
    (void)file;
    usart_write(ptr, len);
    return len;
}

int miniprintf(const char *fmt, ...)
{
    va_list ap;
    int count = 0;

    va_start(ap, fmt);

    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            switch (*fmt) {
            case '%':
                print_char('%', &count);
                break;
            case 'c':
                print_char((char)va_arg(ap, int), &count);
                break;
            case 's':
                print_string(va_arg(ap, char *), &count);
                break;
            case 'd':
            case 'i':
                print_long(va_arg(ap, int), 10, &count);
                break;
            case 'u':
                print_unsigned(va_arg(ap, unsigned int), 10, &count);
                break;
            case 'x':
                print_unsigned(va_arg(ap, unsigned int), 16, &count);
                break;
            case 'X': {
                unsigned long val = va_arg(ap, unsigned long);
                char buf[32];
                int i = 0;
                const char *digits = "0123456789ABCDEF";
                if (val == 0) {
                    print_char('0', &count);
                } else {
                    while (val > 0) {
                        buf[i++] = digits[val % 16];
                        val /= 16;
                    }
                    while (i > 0) {
                        print_char(buf[--i], &count);
                    }
                }
                break;
            }
            case 'p':
                print_string("0x", &count);
                print_unsigned(va_arg(ap, unsigned long), 16, &count);
                break;
            default:
                print_char(*fmt, &count);
                break;
            }
        } else {
            print_char(*fmt, &count);
        }
        fmt++;
    }

    va_end(ap);
    return count;
}
