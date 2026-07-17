#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Board UART 调试统计。
 *
 * rx_bytes/tx_bytes 是已处理/已写入 THR 的字节数；rx_overflow 当前保留，
 * 因为 RX 直接在中断中处理，没有软件 RX ring。
 */
typedef struct
{
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t rx_overflow;
    uint32_t line_errors;
} BoardUartStats;

/** @brief 初始化 UART0 调试通道；会等待 SWD 释放窗口后复用 P06/P07。 */
void BoardUart_Init(void);
/** @brief 从 TX ring 发送最多 1 字节，当前由 ADC_IRQHandler() 周期调用。 */
void BoardUart_TxTask(void);
/** @brief 写 UART END，供其他中断在访问非 UART 外设前调用。 */
void BoardUart_EndAccess(void);
/** @brief 将 1 个字符放入 TX ring。 */
void BoardUart_WriteChar(char ch);
/** @brief 将字符串放入 TX ring。 */
void BoardUart_WriteString(const char* text);
/** @brief 读取当前 UART 调试统计。 */
void BoardUart_GetStats(BoardUartStats* out);

#ifdef __cplusplus
}
#endif
