/**
  ******************************************************************************
  * @file    can2.c
  * @brief   CAN 从机 (Slave) 应用层
  ******************************************************************************
  */

#include "can2.h"
#include "can.h"
#include <stdio.h>

void CAN2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();   /* gpio.c 已开过 GPIOA, 重复使能无害, 双保险 */

    /* 先把输出锁存器写 0 再配成输出, 这样使能输出的瞬间不会先蹦一下高电平。
     * 下面 HAL_GPIO_Init() 之后再写一次, 确保初值确实是低。 */
    HAL_GPIO_WritePin(CAN2_LED_PORT, CAN2_LED_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = CAN2_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN2_LED_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(CAN2_LED_PORT, CAN2_LED_PIN, GPIO_PIN_RESET);
}

void CAN2_SendAck(void)
{
    uint8_t payload[1] = { (uint8_t)CAN_ID_ACK };

    if (CAN_SendStd(CAN_ID_ACK, payload, 1U) != HAL_OK)
    {
        printf("[SLAVE] reply ID=0x%02X failed (TX FIFO full?)\r\n", CAN_ID_ACK);
    }
}

void CAN2_Process(void)
{
    CAN_FrameTypeDef frame;

    while (CAN_Receive(&frame) != 0U)
    {
        if (frame.ext || frame.id != CAN_ID_REQ) continue;   /* 不是请求帧, 丢 */

        CAN2_SendAck();                                      /* 先回 0x22 */
        HAL_GPIO_TogglePin(CAN2_LED_PORT, CAN2_LED_PIN);     /* 再翻转 PA1 */

        /* ★ printf 是阻塞发送 (见 usart.c 的 __io_putchar), 115200 下约 6 ms。
         *   控制环在 SysTick 中断里不受影响, 但会拖慢主循环响应。 */
        printf("[SLAVE] RX ID=0x%02X -> sent 0x%02X, PA1 toggled to %d\r\n",
               CAN_ID_REQ,
               CAN_ID_ACK,
               (int)HAL_GPIO_ReadPin(CAN2_LED_PORT, CAN2_LED_PIN));
    }
}
