#include "hc06.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

#define HC06_UART   &huart2   /* PD5=TX, PD6=RX */

static uint8_t  rx_byte;
static uint8_t  rx_buf[HC06_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* --- ISR callback (called from HAL_UART_RxCpltCallback) --- */

void HC06_RxCallback(uint8_t byte)
{
    uint16_t next = (rx_head + 1) & (HC06_RX_BUF_SIZE - 1);
    if (next != rx_tail) {
        rx_buf[rx_head] = byte;
        rx_head = next;
    }
}

/* --- init --- */

void HC06_Init(void)
{
    /*
     * HC06 模块内置 MCU，上电后需要自检/初始化蓝牙协议栈，实际耗时
     * 因个体差异在 300ms~1s 之间波动。STM32H7 启动远快于 HC06，
     * 若不等待：
     *   1. HC06 RX 未就绪 → 信标字节丢失/错位 → "HCO6"/"H\nC06"
     *   2. HC06 TX 引脚在上电期间浮空/抖动 → STM32 收到噪声字节
     *
     * 先延时 1s 等 HC06 完全就绪，再启动 RX 中断和发送信标。
     * 延时期间不启动 RX IT，避免捕获 HC06 上电噪声。
     */
    HAL_Delay(1000);

    /* 清空硬件 RX 寄存器中的残留噪声 */
    __HAL_UART_FLUSH_DRREGISTER(HC06_UART);

    /* 启动 HAL 中断接收 */
    HAL_UART_Receive_IT(HC06_UART, &rx_byte, 1);

    HC06_SendString("HC06 Ready\r\n");
}

/* --- get byte from IT buffer + restart RX IT --- */

static uint8_t HC06_GetRxByte(void)
{
    uint8_t data = rx_byte;
    HAL_UART_Receive_IT(HC06_UART, &rx_byte, 1);
    return data;
}

/* --- send --- */

void HC06_SendByte(uint8_t data)
{
    HAL_UART_Transmit(HC06_UART, &data, 1, HC06_TIMEOUT);
}

void HC06_SendString(const char *str)
{
    HAL_UART_Transmit(HC06_UART, (uint8_t *)str, (uint16_t)strlen(str), HC06_TIMEOUT);
}

void HC06_SendData(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(HC06_UART, (uint8_t *)buf, len, HC06_TIMEOUT);
}

/* --- receive (ring buffer, polling) --- */

uint16_t HC06_Available(void)
{
    return (rx_head - rx_tail) & (HC06_RX_BUF_SIZE - 1);
}

uint8_t HC06_ReceiveByte(void)
{
    while (HC06_Available() == 0);
    uint8_t data = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & (HC06_RX_BUF_SIZE - 1);
    return data;
}

/* --- AT commands --- */

HAL_StatusTypeDef HC06_AT_Test(void)
{
    HC06_SendString("AT");
    return HAL_OK;
}

HAL_StatusTypeDef HC06_AT_SetName(const char *name)
{
    char buf[32];
    int len = sprintf(buf, "AT+NAME%s", name);
    if (len < 0 || len > 31) return HAL_ERROR;
    HC06_SendString(buf);
    return HAL_OK;
}

HAL_StatusTypeDef HC06_AT_SetBaud(uint32_t baud)
{
    char buf[16];
    int len = sprintf(buf, "AT+BAUD%lu", (unsigned long)baud);
    if (len < 0 || len > 15) return HAL_ERROR;
    HC06_SendString(buf);
    return HAL_OK;
}

HAL_StatusTypeDef HC06_AT_SetPIN(const char *pin)
{
    char buf[16];
    int len = sprintf(buf, "AT+PIN%s", pin);
    if (len < 0 || len > 15) return HAL_ERROR;
    HC06_SendString(buf);
    return HAL_OK;
}

HAL_StatusTypeDef HC06_AT_GetVersion(void)
{
    HC06_SendString("AT+VERSION");
    return HAL_OK;
}

HAL_StatusTypeDef HC06_AT_Reset(void)
{
    HC06_SendString("AT+RESET");
    return HAL_OK;
}

/* --- HAL callback --- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == HC06_UART) {
        HC06_RxCallback(HC06_GetRxByte());
    }
}

/* ================================================================
 * HC06_Process — 排空接收缓冲
 *
 * 原实现把字节交给 Cargo 解析, cargo 模块已随云台精简一起移除。
 * 保留这个排空动作: 只要 HC06_Init() 开了 RX 中断, 环形缓冲就会被
 * 持续写入, 不排空的话写满后新字节会被直接丢掉(丢弃逻辑见
 * HC06_RxCallback), 更重要的是调用方无从判断链路是否还活着。
 * 将来要用蓝牙收指令, 在这里换成自己的解析即可。
 * ================================================================ */
void HC06_Process(void)
{
    while (HC06_Available() > 0) {
        (void)HC06_ReceiveByte();
    }
}
