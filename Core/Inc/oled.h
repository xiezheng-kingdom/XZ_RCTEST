#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"

#define OLED_ADDR   0x78    // SSD1306 7-bit:0x3C -> 8-bit:0x78
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_PAGES  8

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowString8(uint8_t x, uint8_t y, const char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len);
void OLED_ShowNum8(uint8_t x, uint8_t y, int32_t num, uint8_t len);
void OLED_ShowFloat8(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_Refresh(void);
uint8_t OLED_GetAddr(void);
extern uint8_t oled_buf[128][8];
extern uint8_t dirty_page[8];

#endif
