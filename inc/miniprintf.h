#ifndef MINIPRINTF_H
#define MINIPRINTF_H

#include <stdint.h>

/* Minimal printf for freestanding bare-metal (no libc) */
int miniprintf(const char *fmt, ...);

/* Redirects to _write (defined in miniprintf.c) */
extern int _write(int file, char *ptr, int len);

#endif /* MINIPRINTF_H */
