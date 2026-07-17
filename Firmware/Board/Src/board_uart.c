#include "board_uart.h"

/**
 * @file board_uart.c
 * @brief Board 层 UART0 调试通道。
 *
 * P06/P07 同时复用 SWDCLK/SWDIO 和 UART0 RX/TX。本模块上电先等待
 * BOARD_UART_SW_RELEASE_DELAY_MS，给 J-Link 留 connect-under-reset/烧录窗口；
 * 等待结束后主动关闭 SWD，再把引脚切到 UART。
 *
 * CMS32M65xx 的 UART 有 END 访问结束寄存器要求：访问 UART 寄存器后，
 * 再访问 UART 以外的外设寄存器前，需要写 UART0->END = 0。本文件所有
 * UART 寄存器访问序列结束处都调用 uart_end()；其他中断入口可以调用
 * BoardUart_EndAccess()，避免被 UART 访问状态影响。
 */

#include "BoardConfig.h"
#include "CMS32M6510.h"
#include "cgc.h"
#include "common.h"
#include "delay.h"
#include "gpio.h"
#include "uart.h"

#include <stddef.h>
#include <stdint.h>

#define BOARD_UART_TX_BUFFER_SIZE 256U
#define BOARD_UART_TX_BUFFER_MASK (BOARD_UART_TX_BUFFER_SIZE - 1U)

typedef struct
{
    uint8_t data[BOARD_UART_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t overflow_count;
} BoardUartTxBuffer;

static BoardUartTxBuffer s_tx;

static volatile uint8_t s_uart_ready;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_tx_bytes;
static volatile uint32_t s_line_errors;

static void uart_end(void);
static uint16_t tx_next(uint16_t value);
static uint8_t tx_push(uint8_t value);
static void tx_push_text(const char *text);
static void configure_uart_pins(void);
static void flush_rx(void);
static void handle_byte(uint8_t byte);
static void fault_write_text(const char *text);

static void uart_end(void)
{
#if (BOARD_UART_USE_END_LOCK != 0U)
    UART0->END = 0U;
    /* 等待 END 写入完成；这里不用关中断，只做总线写入同步。 */
    __asm volatile("dsb 0xF" ::: "memory");
#endif
}

void BoardUart_EndAccess(void)
{
#if (BOARD_UART_ENABLE != 0U)
    uart_end();
#endif
}

static uint16_t tx_next(uint16_t value)
{
    return (uint16_t)((value + 1U) & BOARD_UART_TX_BUFFER_MASK);
}

static uint8_t tx_push(uint8_t value)
{
    const uint16_t head = s_tx.head;
    const uint16_t next = tx_next(head);

    if (s_uart_ready == 0U)
    {
        return 0U;
    }

    if (next == s_tx.tail)
    {
        s_tx.overflow_count++;
        return 0U;
    }

    s_tx.data[head] = value;
    s_tx.head = next;
    return 1U;
}

/** @brief 把字符串放入 TX ring；真正写 THR 由 BoardUart_TxTask() 完成。 */
static void tx_push_text(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0')
    {
        (void)tx_push((uint8_t)*text);
        text++;
    }
}

/** @brief 将 P06 配成 UART0 RX，将 P07 配成 UART0 TX。 */
static void configure_uart_pins(void)
{
    GPIO_PinAFInConfig(UARTRXDCFG, IO_INCFG_P06_RXD);
    GPIO_Init(PORT0, PIN6, INPUT);
#if (BOARD_UART_RX_TTL_INPUT != 0U)
    PORT->P0TTLCFG |= (uint8_t)(1U << PIN6);
#else
    PORT->P0TTLCFG &= (uint8_t) ~(1U << PIN6);
#endif

    GPIO_PinAFOutConfig(P07CFG, IO_OUTCFG_P07_TXD);
    GPIO_Init(PORT0, PIN7, OUTPUT);
}

/** @brief 初始化前清掉 UART RBR 中可能残留的上电噪声字节。 */
static void flush_rx(void)
{
    uint8_t guard = 32U;

    while (guard-- != 0U)
    {
        uint8_t has_byte = 0U;
        uint32_t lsr;

        lsr = UART0->LSR;
        if ((lsr & UART_LSR_RDR_Msk) != 0U)
        {
            (void)UART0->RBR;
            has_byte = 1U;
        }
        uart_end();

        if (has_byte == 0U)
        {
            break;
        }
    }
}

/**
 * @brief UART RX 的轻量协议入口。
 *
 * 当前正式 bring-up 行为是回显：普通字符原样进入 TX ring，CR/LF 统一回
 * "\r\n"。不再对 '?' 做统计输出，避免调试逻辑影响真实收发路径。
 */
static void handle_byte(uint8_t byte)
{
    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
    {
        tx_push_text("\r\n");
        return;
    }

    (void)tx_push(byte);
}

/**
 * @brief 初始化 UART0 调试通道。
 *
 * 顺序很重要：
 * 1. 先等待 SWD 释放窗口；
 * 2. 开 UART 时钟并主动禁用 SWD；
 * 3. 配 UART 寄存器并写 END；
 * 4. 最后切 P06/P07 复用并打开 RX/line-status 中断。
 */
void BoardUart_Init(void)
{
#if (BOARD_UART_ENABLE != 0U)
    m0_delay_ms(BOARD_UART_SW_RELEASE_DELAY_MS);

    s_uart_ready = 0U;
    s_tx.head = 0U;
    s_tx.tail = 0U;
    s_tx.overflow_count = 0U;
    s_rx_bytes = 0U;
    s_tx_bytes = 0U;
    s_line_errors = 0U;

    CGC_PER12PeriphClockCmd(CGC_PER12Periph_UART, ENABLE);
    DBG->DBGSTOPCR |= DBG_DBGSTOPCR_SWDIS_Msk;

    NVIC_DisableIRQ(UART0_IRQn);
    UART0->IER = 0U;
    uart_end();

    /* RX 优先级高于 ADC；TX 不用 UART 发送中断，由 ADC 中断定时推进。 */
    NVIC_SetPriority(UART0_IRQn, BOARD_UART_IRQ_PRIORITY);
    NVIC_SetPriority(ADC_IRQn, BOARD_UART_ADC_IRQ_PRIORITY);

    UART_ConfigRunMode(UART0, BOARD_UART_BAUD, UART_WLS_8, UART_PARITY_NONE, UART_STOP_BIT_1);
    uart_end();

    configure_uart_pins();
    flush_rx();

    s_uart_ready = 1U;

    UART0->IER = UART_IER_RLSIE_Msk | UART_IER_RBREIE_Msk;
    uart_end();
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);

    tx_push_text("\r\nboard uart ready\r\n");
#endif
}

/**
 * @brief 从 TX ring 向 THR 推进 1 个字节。
 *
 * 这个函数设计给 ADC_IRQHandler() 调用：每次中断最多写一个字节，避免在
 * 主循环里阻塞，也避免在 UART RX 中断里做发送搬运。
 */
void BoardUart_TxTask(void)
{
#if (BOARD_UART_ENABLE != 0U)
    if (s_uart_ready == 0U)
    {
        return;
    }

    if (s_tx.head != s_tx.tail)
    {
        const uint16_t tail = s_tx.tail;
        const uint32_t lsr = UART0->LSR;

        if ((lsr & UART_LSR_THRE_Msk) != 0U)
        {
            UART0->THR = s_tx.data[tail];
            s_tx.tail = tx_next(tail);
            s_tx_bytes++;
        }
    }
    uart_end();
#endif
}

/** @brief 非中断上下文只负责入队，实际发送仍由 BoardUart_TxTask() 推进。 */
void BoardUart_WriteChar(char ch)
{
#if (BOARD_UART_ENABLE != 0U)
    (void)tx_push((uint8_t)ch);
#else
    (void)ch;
#endif
}

/** @brief 写调试字符串到 TX ring；ring 满时会丢弃后续字节并计 overflow。 */
void BoardUart_WriteString(const char *text)
{
#if (BOARD_UART_ENABLE != 0U)
    tx_push_text(text);
#else
    (void)text;
#endif
}

/** @brief 读取基础统计；rx_overflow 当前保留为 0，因为 RX 没有软件队列。 */
void BoardUart_GetStats(BoardUartStats *out)
{
    if (out == NULL)
    {
        return;
    }

    out->rx_bytes = s_rx_bytes;
    out->tx_bytes = s_tx_bytes;
    out->rx_overflow = 0U;
    out->line_errors = s_line_errors;
}

/**
 * @brief HardFault 紧急输出。
 *
 * 只在异常场景使用阻塞式 THR 写入，不依赖 ADC 中断继续运行。
 */
static void fault_write_text(const char *text)
{
    while ((text != NULL) && (*text != '\0'))
    {
        uint32_t guard = 1000000U;
        while (((UART0->LSR & UART_LSR_TEMT_Msk) == 0U) && (guard-- != 0U))
        {
            uart_end();
        }
        uart_end();

        UART0->THR = (uint8_t)*text;
        uart_end();
        text++;
    }
}

/** @brief 故障时尽量从 UART 打印最短标记，随后停机等待调试器接管。 */
void HardFault_Handler(void)
{
    fault_write_text("\r\n!HF\r\n");
    while (1)
    {
    }
}

/**
 * @brief UART0 RX/line-status 中断。
 *
 * 这里只处理接收和错误状态：RBR 中有字节就读出并交给 handle_byte()。
 * 发送不在这里做，避免 RX ISR 被 TX ring 搬运拖长。
 */
void UART0_IRQHandler(void)
{
#if (BOARD_UART_ENABLE != 0U)
    uint32_t int_id;
    uint8_t guard = 0U;

    uart_end();

    do
    {
        int_id = UART0->IIR & 0x0FU;
        uart_end();

        if ((int_id & UART_IIR_INTSTATUS_Msk) != 0U)
        {
            break;
        }

        switch ((int_id & UART_IIR_INTID_Msk) >> UART_IIR_INTID_Pos)
        {
        case 0x02U:
        case 0x06U:
            while (1)
            {
                uint8_t has_byte = 0U;
                uint8_t byte = 0U;
                const uint32_t lsr = UART0->LSR;

                if ((lsr & UART_LSR_RDR_Msk) != 0U)
                {
                    byte = (uint8_t)UART0->RBR;
                    has_byte = 1U;
                }
                uart_end();

                if ((lsr & (UART_LSR_BI_Msk | UART_LSR_FE_Msk | UART_LSR_PE_Msk)) != 0U)
                {
                    s_line_errors++;
                }

                if (has_byte == 0U)
                {
                    break;
                }

                s_rx_bytes++;
                handle_byte(byte);
            }
            break;

        default:
        {
            const uint32_t lsr = UART0->LSR;
            uart_end();
            if ((lsr & (UART_LSR_BI_Msk | UART_LSR_FE_Msk | UART_LSR_PE_Msk)) != 0U)
            {
                s_line_errors++;
            }
            break;
        }
        }

        guard++;
    } while (guard < 20U);

    uart_end();
    NVIC_ClearPendingIRQ(UART0_IRQn);
#endif
}
