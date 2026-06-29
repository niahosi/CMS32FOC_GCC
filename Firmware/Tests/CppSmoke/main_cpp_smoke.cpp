#include <stdint.h>

#include "CMS32M6510_DataFlashLayout.h"

namespace {

volatile uint32_t g_ctor_probe_value;

class DynamicCtorProbe
{
public:
    explicit DynamicCtorProbe(uint32_t seed) : seed_(seed)
    {
        g_ctor_probe_value = value();
    }

    uint32_t value() const
    {
        return seed_ ^ 0x434F4B45UL;
    }

private:
    uint32_t seed_;
};

DynamicCtorProbe g_dynamic_probe(0x12345678UL);

} // namespace

typedef struct
{
    volatile uint32_t boot_magic;
    volatile uint32_t loop_count;
    volatile uint32_t ctor_value;
    volatile uint32_t data_flash_origin;
    volatile uint32_t data_flash_end;
    volatile uint32_t data_block_a;
    volatile uint32_t data_block_b;
    volatile uint32_t payload_size;
    volatile uint32_t block_size;
} CppSmokeState_t;

volatile CppSmokeState_t g_cpp_smoke;

int main()
{
    g_cpp_smoke.boot_magic = 0x43505031UL;
    g_cpp_smoke.ctor_value = g_ctor_probe_value;
    g_cpp_smoke.data_flash_origin = CMS32_DATA_FLASH_ORIGIN;
    g_cpp_smoke.data_flash_end = CMS32_DATA_FLASH_END;
    g_cpp_smoke.data_block_a = CMS32_DATA_BLOCK_A_ADDR;
    g_cpp_smoke.data_block_b = CMS32_DATA_BLOCK_B_ADDR;
    g_cpp_smoke.payload_size = sizeof(CMS32_DataFlashPayload_t);
    g_cpp_smoke.block_size = sizeof(CMS32_DataFlashBlock_t);

    while (1)
    {
        g_cpp_smoke.loop_count++;
    }
}
