#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UNSUPPORT_UNALIGN_ACCESS_REGION_NUM (2)

void *rec_dest;
const void *rec_src;
size_t rec_n;

const void *UNSUPPORT_UNALIGN_ACCESS_REGION[UNSUPPORT_UNALIGN_ACCESS_REGION_NUM][2] = {
    {(void *)0x30000000, (void *)0x30020000},
    {(void *)0x38000000, (void *)0x38010000},
};

void *my_memcpy(void *restrict dest, const void *const src, size_t n)
{
    rec_dest = dest;
    rec_src = src;
    rec_n = n;
    bool is_support_unalign_access = true;
    for (size_t i = 0; i < UNSUPPORT_UNALIGN_ACCESS_REGION_NUM; i++)
    {
        if ((UNSUPPORT_UNALIGN_ACCESS_REGION[i][0] <= dest && UNSUPPORT_UNALIGN_ACCESS_REGION[i][1] >= dest) ||
            (UNSUPPORT_UNALIGN_ACCESS_REGION[i][0] <= src && UNSUPPORT_UNALIGN_ACCESS_REGION[i][1] >= src))
        {
            is_support_unalign_access = false;
            break;
        }
    }
    if (is_support_unalign_access)
    {
        memcpy(dest, src, n);
    }
    else
    {
        uint8_t *d = dest;
        const uint8_t *s = src;
        while ((((uintptr_t)d & 3) || ((uintptr_t)s & 3)) && n--)
        {
            *d++ = *s++;
        }
        if (n == SIZE_MAX)
        {
            return dest;
        }
        size_t align_n = n & ~3U;
        memcpy(d, s, align_n);
        d += align_n;
        s += align_n;
        n -= align_n;
        while (n--)
        {
            *d++ = *s++;
        }
    }
    return dest;
}
