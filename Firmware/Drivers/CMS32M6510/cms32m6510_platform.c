#include "cms32m6510_platform.h"

void CMS32_PlatformIdle(void)
{
    for (;;)
    {
        __asm volatile("wfi");
    }
}
