#include "User/share_buffer.h"

#include <stdint.h>

__attribute__((section(".bss.DTCM"))) int16_t shared_buffer[4800];
