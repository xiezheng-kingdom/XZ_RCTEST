/**
 ******************************************************************************
 * @file    ultrasonic.c
 * @brief   HC-SR04 超声波测距模块实现 (TIM5 CH2 输入捕获)
 *
 * 测距流程:
 *   1. US_Trigger() 发 10μs TRIG 脉冲
 *   2. TIM5 捕获 ECHO 上升沿 → 记录 CNT
 *   3. 切换为下降沿捕获
 *   4. TIM5 捕获 ECHO 下降沿 → CNT 差值 = 脉宽 μs
 *   5. distance = pulse_us * US_SPEED_CM_PER_US
 *
 * TIM5: APB1 定时器, TIMCLK=200MHz, PSC=199 → 1MHz(1μs)
 ******************************************************************************
 */
#include "ultrasonic.h"
#include "tim.h"

TIM_HandleTypeDef htim5;   /* TIM5 — 专用于超声波输入捕获 */

/* ====================== 内部状态 ====================== */
static volatile uint32_t capture_rising  = 0;
static volatile uint32_t capture_falling = 0;
static volatile uint8_t  capture_done    = 0;  /* 1=本次测量完成 */
static volatile uint8_t  capture_rising_edge = 1; /* 1=等上升沿 0=等下降沿 */

static UltrasonicTypeDef us_data;

/* 微秒级延时 (粗略, 用于 10μs TRIG 脉冲) */
static void delay_us(uint32_t us)
{
    /* SysTick 1ms/tic, 用空循环:
     * 400MHz CPU, 每条循环约 3 周期 → 133 循环/μs */
    uint32_t count = us * 133;
    while (count--) { __NOP(); }
}

/* ====================== 初始化 ====================== */

void US_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* --- GPIO --- */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TRIG: 推挽输出 */
    GPIO_InitStruct.Pin   = US_TRIG_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(US_TRIG_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);

    /* ECHO: 复用为 TIM5 CH2 */
    GPIO_InitStruct.Pin       = US_ECHO_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;  /* PA1 = AF2 → TIM5_CH2 */
    HAL_GPIO_Init(US_ECHO_PORT, &GPIO_InitStruct);

    /* --- TIM5: 200MHz → PSC=199 → 1MHz = 1μs --- */
    __HAL_RCC_TIM5_CLK_ENABLE();

    US_TIM->Instance           = TIM5;
    US_TIM->Init.Prescaler     = 199;   /* 200MHz / 200 = 1MHz */
    /* ⚠️ 时钟: HSE 25MHz → SYSCLK=400MHz → TIM5_CLK=200MHz
     * PSC=199 → 200MHz/(199+1)=1MHz=1μs ✓ */
    US_TIM->Init.CounterMode   = TIM_COUNTERMODE_UP;
    US_TIM->Init.Period        = 65535;
    US_TIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    US_TIM->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(US_TIM);

    /* 输入捕获: CH2, 上升沿, 不分频, 不滤波 */
    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(US_TIM, &sConfigIC, US_TIM_CHANNEL);

    /* 启动输入捕获中断 */
    HAL_TIM_IC_Start_IT(US_TIM, US_TIM_CHANNEL);

    /* 使能 NVIC */
    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);

    /* 初始状态 */
    us_data.distance_cm = 0.0f;
    us_data.valid       = 0;
    us_data.busy        = 0;
    us_data.last_trig_tick = 0;
}

/* ====================== 触发测距 ====================== */

void US_Trigger(void)
{
    uint32_t now = HAL_GetTick();
    if (now - us_data.last_trig_tick < US_TRIG_INTERVAL_MS) return;

    us_data.last_trig_tick = now;
    us_data.valid = 0;
    us_data.busy  = 1;
    capture_done  = 0;
    capture_rising_edge = 1;   /* 先等上升沿 */

    /* TRIG: >=10μs 高电平 */
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET);
    delay_us(12);
    HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
}

/* ====================== TIM5 捕获中断处理 ====================== */

void US_IRQ_Handler(void)
{
    if (!__HAL_TIM_GET_FLAG(US_TIM, TIM_FLAG_CC2)) return;
    __HAL_TIM_CLEAR_FLAG(US_TIM, TIM_FLAG_CC2);

    if (capture_rising_edge) {
        /* 上升沿: 记录并切换到下降沿 */
        capture_rising = HAL_TIM_ReadCapturedValue(US_TIM, US_TIM_CHANNEL);
        __HAL_TIM_SET_CAPTUREPOLARITY(US_TIM, US_TIM_CHANNEL,
                                       TIM_INPUTCHANNELPOLARITY_FALLING);
        capture_rising_edge = 0;
    } else {
        /* 下降沿: 计算脉宽 */
        capture_falling = HAL_TIM_ReadCapturedValue(US_TIM, US_TIM_CHANNEL);
        __HAL_TIM_SET_CAPTUREPOLARITY(US_TIM, US_TIM_CHANNEL,
                                       TIM_INPUTCHANNELPOLARITY_RISING);

        uint32_t pulse_us;
        if (capture_falling >= capture_rising) {
            pulse_us = capture_falling - capture_rising;
        } else {
            /* 定时器溢出回绕 */
            pulse_us = (65536U - capture_rising) + capture_falling;
        }

        us_data.busy = 0;
        if (pulse_us > US_TIMEOUT_US || pulse_us == 0) {
            us_data.valid = 0;
            us_data.distance_cm = 0.0f;
        } else {
            float dist = (float)pulse_us * US_SPEED_CM_PER_US;
            if (dist >= US_DIST_MIN_CM && dist <= US_DIST_MAX_CM) {
                us_data.distance_cm = dist;
                us_data.valid = 1;
            } else {
                us_data.valid = 0;
            }
        }
        capture_done = 1;
        capture_rising_edge = 1;
    }
}

/* ====================== 查询 ====================== */

float US_Read(void)
{
    return us_data.valid ? us_data.distance_cm : -1.0f;
}

int US_IsValid(void)
{
    return (int)us_data.valid;
}

float US_GetDistance(void)
{
    return US_Read();
}

/* 阻塞式测距: 触发后等完成 (最长 ~65ms) */
void US_StartMeasurement(void)
{
    US_Trigger();
    uint32_t timeout = HAL_GetTick() + 100;  /* 100ms 超时 */
    while (us_data.busy && HAL_GetTick() < timeout) {
        __NOP();
    }
}
