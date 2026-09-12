#ifndef __HC06_H__
#define __HC06_H__

#include "main.h"

/* HC06 Bluetooth 2.0 SPP module
 * Interface: USART2 (PD5=TX, PD6=RX)
 * Default: 9600-8N1, slave mode, no flow control
 * AT mode: enter when module is not paired (no need for \r\n)
 * Transparent mode: auto-enter when paired with master device
 */

#define HC06_TIMEOUT  500
#define HC06_RX_BUF_SIZE 128

void HC06_Init(void);
void HC06_SendByte(uint8_t data);
void HC06_SendString(const char *str);
void HC06_SendData(const uint8_t *buf, uint16_t len);
uint8_t HC06_ReceiveByte(void);
uint16_t HC06_Available(void);
void HC06_RxCallback(uint8_t byte);
void HC06_Process(void);       /* 帧解析+指令分发 */

/* AT commands - only work when HC06 is not paired (LED blinking) */
HAL_StatusTypeDef HC06_AT_Test(void);
HAL_StatusTypeDef HC06_AT_SetName(const char *name);
HAL_StatusTypeDef HC06_AT_SetBaud(uint32_t baud);
HAL_StatusTypeDef HC06_AT_SetPIN(const char *pin);
HAL_StatusTypeDef HC06_AT_GetVersion(void);
HAL_StatusTypeDef HC06_AT_Reset(void);

#endif
