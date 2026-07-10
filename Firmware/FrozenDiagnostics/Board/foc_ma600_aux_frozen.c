/**
 * @file foc_ma600_aux_frozen.c
 * @brief MA600 裸读角和 NVM store 历史辅助代码。
 *
 * 该文件不参与任何 CMake target，只保留已经从主驱动移除的调试流程。
 * 如需恢复，先把对应逻辑移回 `foc_ma600.c`，并确认 CS/xfer/delay 等私有
 * helper 的接口仍然一致。
 */

#if 0
/** @brief 旧调试流程的连续角度裸读；addr 当时未参与协议。 */
uint32_t ma600_read_data(uint32_t addr)
{
    uint32_t data;

    (void)addr;

    cs_low();
    data = (xfer(MA600_DUMMY_BYTE) << 8);
    data |= xfer(MA600_DUMMY_BYTE);
    cs_high();

    return data;
}

/**
 * @brief 按 MA600 store key 特殊帧序列触发 NVM block 存储。
 * @warning NVM 写入有寿命和误配置风险，默认主固件已移除该入口。
 */
void ma600_store_nvm_block(uint8_t block)
{
    cs_low();
    xfer(MA600_REG_WRITE_UNLOCK);
    xfer(MA600_REG_STORE_KEY);
    cs_high();

    m0_delay_us(1000);

    cs_low();
    xfer(MA600_REG_WRITE_UNLOCK);
    xfer(block);
    cs_high();

    m0_delay_us(10000);

    cs_low();
    xfer(MA600_DUMMY_BYTE);
    xfer(MA600_DUMMY_BYTE);
    cs_high();

    m0_delay_us(10000);
}
#endif
