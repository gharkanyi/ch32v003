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
 * First step: Port 'Output_Compare_Mode' to TIM2_CH3
 * TIM2_CH3 (remap_f: PD6)
 * Finally:
 * This example demonstrates using DMA to output 4-phases pulses on
 * TIM2_CH1 .. TIM2_CH4. (Pin mapping used: see '==>' mark below ...)
 * (28-BYJ-48 stepper motor wave drive mode)
 * FATAL: Because of the DMA_Channel7 collision of TIM2_CH2 and
 * TIM2_CH4, the 4-phases pulses cannot be realized in this way ... :-(
 * So TIM2_CH2 deactivated now, dry test-only implementation :-) ...
 * TODO: this project should be ported to TIM1!
 */
/*
 *  Timer 2 pin mappings by AFIO->PCFR1
    00  AFIO_PCFR1_TIM2_REMAP_NOREMAP
        D4      T2CH1ETR
        D3      T2CH2
        C0      T2CH3
        D7      T2CH4  --note: requires disabling nRST in opt
==> 01  AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1
        C5      T2CH1ETR_
        C2      T2CH2_
        D2      T2CH3_
        C1      T2CH4_
    10  AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2
        C1      T2CH1ETR_
        D3      T2CH2
        C0      T2CH3
        D7      T2CH4  --note: requires disabling nRST in opt
    11  AFIO_PCFR1_TIM2_REMAP_FULLREMAP
        C1      T2CH1ETR_
        C7      T2CH2_
        D6      T2CH3_
        D5      T2CH4_
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
    (u32)&TIM2->CH1CVR, (u32)&TIM2->CH2CVR, (u32)&TIM2->CH3CVR, (u32)&TIM2->CH4CVR
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
	{{2500,    0, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_5, 5},
	 {5000, 2500, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_2, 7},
	 {7500, 5000, (u16)RCC_APB2Periph_GPIOD, GPIO_Pin_2, 1},
	 {   0, 7500, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_1, 7}},
    // MEDIUM (resolution: 10 us): step = 25 ms (40 Hz), period = 100 ms (10 Hz)
    {{2500,    0, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_5, 5},
     {5000, 2500, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_2, 7},
     {7500, 5000, (u16)RCC_APB2Periph_GPIOD, GPIO_Pin_2, 1},
     {   0, 7500, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_1, 7}},
    // FAST (resolution: 1 us):  step = 2 ms (500 Hz), period: 8 ms (125 Hz)
	{{2000,    0, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_5, 5},
	 {4000, 2000, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_2, 7},
	 {6000, 4000, (u16)RCC_APB2Periph_GPIOD, GPIO_Pin_2, 1},
	 {   0, 6000, (u16)RCC_APB2Periph_GPIOC, GPIO_Pin_1, 7}}
};

/* Output Compare Mode Definition */
#define OutCompare_Timing   0
#define OutCompare_Active   1
#define OutCompare_Inactive 2
#define OutCompare_Toggle   3

/* Output Compare Mode Selection */
//#define OutCompare_MODE OutCompare_Timing
//#define OutCompare_MODE OutCompare_Active
//#define OutCompare_MODE OutCompare_Inactive
#define OutCompare_MODE OutCompare_Toggle

void TIM2_Drive_DMA_Init(DRIVE drive, SPEED speed, DIR dir);

/* ************************************************* */
void TIM2_Drive_Init(DRIVE drive, SPEED speed, DIR dir)
{
	u16 arr = clock[speed].arr, psc = clock[speed].psc;

    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | 
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

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
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    // TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // TIM_OCPolarity_Low;
    for (STEP step = CH1_IN1A; step < STEPS; step++) {
        TIM_OCInitStructure.TIM_OCMode = step == CH2_IN2B ? TIM_OCMode_Inactive : TIM_OCMode_Toggle; // DMA collision ...
        TIM_OCInitStructure.TIM_Pulse = drive[speed][dir ? (REVERSE - step) : step][STEP_INIT];
        tim_ocxinit[step](TIM2, &TIM_OCInitStructure);
        tim_ocxpreloadconfig[step](TIM2, TIM_OCPreload_Disable);
    }

    TIM_ARRPreloadConfig( TIM2, ENABLE );

    TIM2_Drive_DMA_Init(drive, speed, dir);
}

/*********************************************************************
 * @fn      TIM2_OutCompare_Init
 *
 * @brief   Initializes TIM2 output compare.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void TIM2_OutCompare_Init(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

#if (OutCompare_MODE == OutCompare_Timing)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;

#elif (OutCompare_MODE == OutCompare_Active)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;

#elif (OutCompare_MODE == OutCompare_Inactive)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;

#elif (OutCompare_MODE == OutCompare_Toggle)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;

#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // TIM_OCPolarity_Low;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);

    // TIM_CtrlPWMOutputs( TIM2, ENABLE );
    TIM_OC3PreloadConfig( TIM2, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM2, ENABLE );
    // TIM_Cmd( TIM2, ENABLE );
}

/* Private variables */
static u16 pbuf[] = {25, 0};

// u16 pbuf[3] = {10, 50, 80};

/*********************************************************************
 * @fn      TIM2_DMA_Init
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
void TIM2_DMA_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
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
void TIM2_Drive_DMA_Init(DRIVE drive, SPEED speed, DIR dir)
{
    for (STEP step = CH1_IN1A; step < STEPS; step++) {
        if (step == CH2_IN2B) continue; // DMA collision ...
        TIM2_DMA_Init(dma1_channelx[drive[speed][step][DMA_CHN]-1], dma_ccraddr[step],
            (u32)drive[speed][dir ? (REVERSE - step) : step], EDGE_CNT);
        TIM_DMACmd(TIM2, dma_ccrsrc[step], ENABLE);
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

    // TIM2_OutCompare_Init( 100-1, 48000-1, 50 );
    // TIM2_OutCompare_Init( 100-1, 48000-1, pbuf[1] );
    // TIM2_Drive_Init(waves, MEDIUM, CW, 100-1, 48000-1); // 1 ms resolution, step = 25 ms (40 Hz)

    TIM2_Drive_Init(waves, FAST, CW); // 10 us resolution, step = 25 ms (40 Hz)

/*
    // TIM2_DMA_Init(DMA1_Channel1, (u32)&TIM2->CH3CVR, (u32)pbuf, DIM(pbuf));

    // TIM2_DMA_Init(DMA1_Channel5, (u32)&TIM2->CH1CVR, (u32)waves[MEDIUM][CH1_IN1A], EDGE_CNT);
    TIM2_DMA_Init(dma1_channelx[waves[MEDIUM][CH1_IN1A][DMA_CHN]-1], dma_ccraddr[CH1_IN1A], (u32)waves[MEDIUM][CH1_IN1A], EDGE_CNT);
    // TIM2_DMA_Init(DMA1_Channel7, (u32)&TIM2->CH2CVR, (u32)waves[MEDIUM][CH2_IN2B], EDGE_CNT);
    TIM2_DMA_Init(DMA1_Channel1, (u32)&TIM2->CH3CVR, (u32)waves[MEDIUM][CH3_IN3C], EDGE_CNT);
    TIM2_DMA_Init(DMA1_Channel7, (u32)&TIM2->CH4CVR, (u32)waves[MEDIUM][CH4_IN4D], EDGE_CNT);

    TIM_DMACmd(TIM2, dma_ccrsrc[CH1_IN1A], ENABLE);
    // TIM_DMACmd(TIM2, TIM_DMA_CC2, ENABLE);
    TIM_DMACmd(TIM2, TIM_DMA_CC3, ENABLE);
    TIM_DMACmd(TIM2, TIM_DMA_CC4, ENABLE);
*/
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
/**/

    while(1);
}
