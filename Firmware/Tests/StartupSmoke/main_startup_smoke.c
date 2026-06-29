#include <stdint.h>

#define STARTUP_SMOKE_BOOT_MAGIC 0x534D4F4BUL

typedef struct
{
    volatile uint32_t boot_magic;
    volatile uint32_t loop_count;
    volatile uint32_t main_magic;
} StartupSmokeState_t;

volatile StartupSmokeState_t g_startup_smoke = {
    STARTUP_SMOKE_BOOT_MAGIC,
    0U,
    0U,
};

int main(void)
{
    g_startup_smoke.main_magic = 0x4D41494EUL;

    while (1)
    {
        g_startup_smoke.loop_count++;
    }
}
