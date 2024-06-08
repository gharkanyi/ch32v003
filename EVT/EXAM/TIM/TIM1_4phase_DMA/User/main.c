/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 * This example demonstrates using DMA to output 4-phases pulses on
 * TIM1_CH1 .. TIM1_CH4. (Pin mapping used: see '==>' mark below ...)
 * (28-BYJ-48 stepper motor wave drive mode)
 */

/*
 * Remap bits for timer 1 (TIM1). These bits can be read and
 * written by the user.
 * It controls the mapping of channels 1 to 4, 1N to
 * 3N, external trigger (ETR) and brake input
 * (BKIN) of timer 1 to the GPIO ports.
 * -------------------
 * 00: Default mapping
 * ETR/PC5,
 * CH1/PD2, CH2/PA1, CH3/PC3, CH4/PC4,
 * BKIN/PC2,
 * CH1N/PD0, CH2N/PA2, CH3N/PD1
 * ==> ---------------
 * 01: Partial mapping
 * ETR/PC5
 * CH1/PC6, CH2/PC7, CH3/PC0, CH4/PD3,
 * BKIN/PC1,
 * CH1N/PC3, CH2N/PC4, CH3N/PD1
 * -------------------
 * 10: Partial mapping
 * ETR/PD4,
 * CH1/PD2, CH2/PA1, CH3/PC3, CH4/PC4,
 * BKIN/PC2,
 * CH1N/PD0, CH2N/PA2, CH3N/PD1
 * --------------------
 * 11: Complete mapping
 * ETR/PC2,
 * CH1/PC4, CH2/PC7, CH3/PC5, CH4/PD4,
 * BKIN/PC1,
 * CH1N/PC3, CH2N/PD2, CH3N/PC6
 */


#include "debug.h"

#define DIM(a) (sizeof(a)/sizeof(a[0]))

typedef enum {
    SLOW,
    MEDIUM,
    FAST,
    SPEEDS
} SPEED;

typedef enum {
    CH1_IN1A,
    CH2_IN2B,
    CH3_IN3C,
    CH4_IN4D,
    REVERSE = CH4_IN4D,
    STEPS
} STEP;

typedef enum {
    CW,
    CCW
} DIR;

typedef enum {
    STEP_OUT,
    STEP_IN,
    STEP_INIT = STEP_IN,
    EDGE_CNT,
    GPIO_PORT = EDGE_CNT,
    GPIO_PIN,
    DMA_CHN,
    EDGES
} EDGE;

typedef void (*TIM_OCXINIT)(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *TIM_OCInitStruct);
static TIM_OCXINIT tim_ocxinit[STEPS] = {
    TIM_OC1Init, TIM_OC2Init, TIM_OC3Init, TIM_OC4Init
};

typedef void (*TIM_OCXPRELOADCONFIG)(TIM_TypeDef *TIMx, uint16_t TIM_OCPreload);
static TIM_OCXPRELOADCONFIG tim_ocxpreloadconfig[STEPS] = {
    TIM_OC1PreloadConfig, TIM_OC2PreloadConfig, TIM_OC3PreloadConfig, TIM_OC4PreloadConfig
};

static u16 dma_ccrsrc[STEPS] = {
    TIM_DMA_CC1, TIM_DMA_CC2, TIM_DMA_CC3, TIM_DMA_CC4
};
static u32 dma_ccraddr[STEPS] = {
    (u32)&TIM1->CH1CVR, (u32)&TIM1->CH2CVR, (u32)&TIM1->CH3CVR, (u32)&TIM1->CH4CVR
};

static DMA_Channel_TypeDef *dma1_channelx[] = {
        DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6, DMA1_Channel7
};

typedef struct {
	u16 arr;
	u16 psc;
} CLOCK;
static CLOCK clock[SPEEDS] = {
		{10000-1, 4800-1},
		{10000-1,  480-1},
		{ 8000-1,   48-1}
};

typedef u16 DRIVE[SPEEDS][STEPS][EDGES];
static DRIVE waves = { // resolution: 10 us
    // SLOW (resolution: 100 us): step = 250 ms (4 Hz), period = 1 s (1 Hz)
	{{2500,    0, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_6, 2},
	 {5000, 2500, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_7, 3},
	 {7500, 5000, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_0, 6},
	 {   0, 7500, (u16)RCC_APB2Periph_GPIOD, GPIO_Pin_3, 4}},
    // MEDIUM (resolution: 10 us): step = 25 ms (40 Hz), period = 100 ms (10 Hz)
    {{2500,    0, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_6, 2},
     {5000, 2500, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_7, 3},
     {7500, 5000, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_0, 6},
     {   0, 7500, (u16)RCC_APB2Periph_GPIOD, GPIO_Pin_3, 4}},
    // FAST (resolution: 1 us):  step = 2 ms (500 Hz), period: 8 ms (125 Hz)
	{{2000,    0, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_6, 2},
	 {4000, 2000, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_7, 3},
	 {6000, 4000, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_0, 6},
	 {   0, 6000, (u16)RCC_APB2Periph_GPIOD, GPIO_Pin_3, 4}}
};

void TIM1_Drive_DMA_Init(DRIVE drive, SPEED speed, DIR dir);

/* ************************************************* */
void TIM1_Drive_Init(DRIVE drive, SPEED speed, DIR dir)
{
	u16 arr = clock[speed].arr, psc = clock[speed].psc;

    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1 |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    // RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM2, ENABLE);

    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    for (STEP step = CH1_IN1A; step < STEPS; step++) {
        GPIO_InitStructure.GPIO_Pin = drive[speed][step][GPIO_PIN];
        GPIO_Init(drive[speed][step][GPIO_PORT] == RCC_APB2Periph_GPIOC ? GPIOC : GPIOD, &GPIO_InitStructure);
    }

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // TIM_OCPolarity_Low;
    for (STEP step = CH1_IN1A; step < STEPS; step++) {
        // TIM_OCInitStructure.TIM_OCMode = step == CH2_IN2B ? TIM_OCMode_Inactive : TIM_OCMode_Toggle; // DMA collision ...
        TIM_OCInitStructure.TIM_Pulse = drive[speed][dir ? (REVERSE - step) : step][STEP_INIT];
        tim_ocxinit[step](TIM1, &TIM_OCInitStructure);
        tim_ocxpreloadconfig[step](TIM1, TIM_OCPreload_Disable);
    }

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM1_Drive_DMA_Init(drive, speed, dir);
}

/*********************************************************************
 * @fn      TIM1_DMA_Init
 *
 * @brief   Initializes the TIM DMAy Channelx configuration.
 *
 * @param   DMA_CHx -
 *            x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void TIM1_DMA_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

    DMA_Cmd(DMA_CHx, ENABLE);
}

/* ***************************************************** */
void TIM1_Drive_DMA_Init(DRIVE drive, SPEED speed, DIR dir)
{
    for (STEP step = CH1_IN1A; step < STEPS; step++) {
        // if (step == CH2_IN2B) continue; // DMA collision ...
        TIM1_DMA_Init(dma1_channelx[drive[speed][step][DMA_CHN]-1], dma_ccraddr[step],
            (u32)drive[speed][dir ? (REVERSE - step) : step], EDGE_CNT);
        TIM_DMACmd(TIM1, dma_ccrsrc[step], ENABLE);
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    SystemCoreClockUpdate();
    Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    TIM1_Drive_Init(waves, FAST, CW); // FAST: 1 us resolution, step = 2 ms (125 Hz)

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    while(1);
}
