/* [https://stm32f4-discovery.net/2015/01/properly-set-clock-speed-stm32f4xx-devices/]
 * STM32f4xx discovery clock setup:
 * 1.
 * startup_stm32f4xx.c
 * -uncomment SystemInit()
 * -add function call before main()
 * 2.
 * stm32f4xx.h
 * -change HSE_VALUE to (uint32_t)8000000
 * 3.
 * system_stm32f4xx.c
 * -change PLL_M to 8
 *
 *
 */

//includes------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

//defines-------------------------------------------------------
#define  LED_QTY    8
#define  LED_MAX    8

/*
 * --------------- WS2812 manual ----------------
 * power supply voltage: +3.5 ~ +5.3 V
 *
 * data transfer time = TH + TL = 1.25us +/-150ns
 * 		T0H - 0, high voltage - 0.35us - +/-150ns
 * 		T0L - 0, low voltage  - 0.9us  - +/-150ns
 * 		T1H - 1, high voltage - 0.9us  - +/-150ns
 * 		T1L - 1, low voltage  - 0.35us - +/-150ns
 * 		RES - low voltage     - above 50us
 *
 * 		0 code - T0H + T0L
 * 		1 code - T1H + T1L
 *
 * data order is GRB !
 * 3 * 8 bit = 24 bit per LED =~ 30us
 * Transmission pause =~ 2 * LED
 *
 * TIM3 = 2*APB1 => TIM_CLK = 84MHz
 * PWM 	= TIM_CLK/ (period + 1)/(prescale + 1)
 * ----------------------------------------------
 *
 *   PRESCALER    0	84 MHz (11.9ns)
 *   PERIOD     104	80 kHz (1.25us)
 *   LO_TIME     29	29 * 11.9ns = 0.34us
 *   HI_TIME     76	76 * 11.9ns = 0.90us
 */
#define  PRESCALER    0
#define  PERIOD		104
#define  LO_TIME	 29
#define  HI_TIME	 76

#define  TIMER_BUF_LENGTH1   (LED_QTY + 2) * 24
#define  TIMER_BUF_LENGTH    (LED_MAX + 2) * 24

typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
}RGB_t;

/*
 * 	Colour codes
 * 	------------------------------
 * 	3 x 8 bit, RGB
 */
#define RGB_COLOUR_OFF      (RGB_t) {0x00,0x00,0x00}
#define RGB_COLOUR_BLUE     (RGB_t) {0x00,0x00,0xFF}
#define RGB_COLOUR_GREEN    (RGB_t) {0x00,0xFF,0x00}
#define RGB_COLOUR_RED      (RGB_t) {0xFF,0x00,0x00}
#define RGB_COLOUR_WHITE    (RGB_t) {0xFF,0xFF,0xFF}
#define RGB_COLOUR_CYAN     (RGB_t) {0x00,0xFF,0xFF}
#define RGB_COLOUR_MAGENTA  (RGB_t) {0xFF,0x00,0xFF}
#define RGB_COLOUR_YELLOW   (RGB_t) {0xFF,0xFF,0x00}

//global variables----------------------------------------------
uint32_t dma_status;
uint16_t TIMER_BUF[TIMER_BUF_LENGTH];
uint8_t  channel;
uint32_t ledmax;
uint32_t pos;
RGB_t *str;
RGB_t LED_BUF[LED_QTY];


/*
 *
 */
void Init_gpio(void)
{
	GPIO_InitTypeDef gpio_struct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	gpio_struct.GPIO_Pin = GPIO_Pin_6;
	gpio_struct.GPIO_Mode = GPIO_Mode_AF;
	gpio_struct.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_struct.GPIO_OType = GPIO_OType_PP;
	gpio_struct.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &gpio_struct);

	GPIOC->BSRRH = GPIO_Pin_6;

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
}

/*
 *
 */
void Init_tim(void)
{
	TIM_TimeBaseInitTypeDef  time_base_struct;
	TIM_OCInitTypeDef  oc_struct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	time_base_struct.TIM_Period = PERIOD;
	time_base_struct.TIM_Prescaler = PRESCALER;
	time_base_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	time_base_struct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &time_base_struct);

	oc_struct.TIM_OCMode = TIM_OCMode_PWM1;
	oc_struct.TIM_OutputState = TIM_OutputState_Enable;
	oc_struct.TIM_Pulse = 0;
	oc_struct.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &oc_struct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);	//	enable timer
}

/*
 *
 */
void Init_dma(void)
{
	DMA_InitTypeDef dma_struct;

    DMA_Cmd(DMA1_Stream4, DISABLE);
    DMA_DeInit(DMA1_Stream4);

    dma_struct.DMA_Channel = DMA_Channel_5;
    dma_struct.DMA_PeripheralBaseAddr = (uint32_t) &(TIM3->CCR1);
    dma_struct.DMA_Memory0BaseAddr = (uint32_t)TIMER_BUF;
    dma_struct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dma_struct.DMA_BufferSize = 8*24;
    dma_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16bit
    dma_struct.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
    dma_struct.DMA_Mode = DMA_Mode_Normal;
    dma_struct.DMA_Priority = DMA_Priority_VeryHigh;
    dma_struct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma_struct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    dma_struct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma_struct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream4, &dma_struct);
}

/*
 *
 */
void DMA1_Stream4_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))	//	check interrupt flag, did dma finish transfer
	{
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);	//	 reset interrupt flag

		TIM_Cmd(TIM3, DISABLE);	//	disable timer
		DMA_Cmd(DMA1_Stream4, DISABLE);	//	disable dma

		dma_status = 0;	//	set dma status 'ready'
	}
}

/*
 *
 */
void DMA_Start(void)
{
	dma_status = 1;	//	set dma status 'busy'

	Init_dma();

    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);	//	Transfer_Complete Interrupt enable
    DMA_Cmd(DMA1_Stream4, ENABLE);	//	enable dma

    TIM_Cmd(TIM3, ENABLE);	//	enable timer
}

/*
 *
 */
void Init_nvic(void)
{
	NVIC_InitTypeDef nvic_struct;

    TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);

    nvic_struct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
    nvic_struct.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_struct.NVIC_IRQChannelSubPriority = 0;
    nvic_struct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_struct);
}


/*
 * SetChannel(uint8_t ch)
 *
 * Set channel for active LED string
 */
void SetChannel(uint8_t ch)
{
    if(ch == 1) {
      channel = 1;
      str = &LED_BUF[0];
      ledmax = LED_QTY;
    }
    else	return;
}

/*
 * Fill_Timer_Buf(void)
 *
 * Fill in Timer buffer with RGB values of active LEDs
 */
void Fill_Timer_Buf(void)
{
	uint32_t no;
	//uint32_t pos = 0;
	uint8_t mask = 0x80;	//	mask = 0x80 = 1000 0000
	RGB_t led;

	for(no = 0; no < ledmax; no++) // Set up time for LEDs
	{
		led = str[no];

		mask = 0x80;
		while(mask != 0)	//	green 8-bit
		{
			TIMER_BUF[pos] = LO_TIME;
			if((led.green&mask) != 0) TIMER_BUF[pos] = HI_TIME;
			pos++;
			mask >>= 1;
		}
		mask = 0x80;

		while(mask != 0)	//	red 8-bit
        {
        	TIMER_BUF[pos] = LO_TIME;
        	if((led.red&mask) != 0) TIMER_BUF[pos] = HI_TIME;
        	pos++;
        	mask >>= 1;
        }
        mask = 0x80;

        while(mask != 0)	//	blue 8-bit
        {
        	TIMER_BUF[pos] = LO_TIME;
        	if((led.blue&mask) != 0) TIMER_BUF[pos] = HI_TIME;
        	pos++;
        	mask >>= 1;
        }

        // po usart wyswietlic pos
        // stmstudio !

	}

	/*for(no = 0; no < (2*24); no++)
	{	//	add pause between LEDs
		TIMER_BUF[pos] = 0;
		pos++;
	}*/
	pos = 0;
}


/*
 * Refresh(void)
 *
 * Update LEDs
 */
void Refresh(void)
{
	while(dma_status != 0); // DMA status busy?

	Fill_Timer_Buf();	//	Fill Timer buffer with 'high' and 'low' periods

	DMA_Start();
}

/*
 * All_LED_RGB(RGB_t rgb, uint8_t refresh)
 * rgb 		- rgb code
 * refresh	- 1 if refresh active led string
 *
 * Set all LEDs
 */
void All_LED_RGB(RGB_t rgb, uint8_t refresh)
{
	uint32_t n;

	for(n = 0; n < ledmax; n++)
	{
		str[n] = rgb;
	}
	if(refresh == 1) Refresh();
}

/*
 * One_LED_RGB(uint32_t on, RGB_t rgb, uint8_t refresh)
 * on 		- ordinal number
 * rgb 		- rgb code
 * refresh	- 1 if refresh active led string
 *
 * Set one LED
 */
void One_LED_RGB(uint32_t on, RGB_t rgb, uint8_t refresh)
{
	if(on < ledmax)
	{
		str[on] = rgb;
		if(refresh == 1) Refresh();
	}
}

/*
 * Reset_LEDs(void)
 *
 * Reset all LED string
 */
void Reset_LEDs(void)
{
    SetChannel(1);

    while(dma_status != 0);
    DMA_Start();

    SetChannel(1);
    All_LED_RGB(RGB_COLOUR_OFF, 1);

}

/*
 * Init(void)
 *
 * Initialize peripherals, timer and LED arrays
 */
void Init(void)
{
	uint32_t n;

	dma_status = 0;
	channel = 0;
	ledmax = 0;

	for(n = 0; n < TIMER_BUF_LENGTH; n++)
	{	//	Init timer array and fill in with '0'
		TIMER_BUF[n] = 0;
	}

    for(n = 0; n < LED_QTY; n++)
    {	//	Init all LED array
    	LED_BUF[n] = RGB_COLOUR_OFF;
    }
    channel = 1;

    SetChannel(channel);	//	Select connected channel (TIM3 CH1 - PC6)

    Init_gpio();
    Init_tim();
    Init_nvic();
    Init_dma();

    int c;
    Reset_LEDs();  // Reset all LEDs in string
}

void Delay(uint32_t ms)
{
	while(ms--);
}


int main(void)
{
	RGB_t led1,led2,led3,led4,led5,led6,led7,led8;

	SystemInit();

	Init();

	int a;
	led1 = RGB_COLOUR_OFF;
	led2 = RGB_COLOUR_RED;
	/*led3 = RGB_COLOUR_RED;
	led4 = RGB_COLOUR_BLUE;
	led5 = RGB_COLOUR_BLUE;
	led6 = RGB_COLOUR_BLUE;
	led7 = RGB_COLOUR_BLUE;
	led8 = RGB_COLOUR_BLUE;*/

	pos = 0;

	One_LED_RGB(0, led1, 0);
	One_LED_RGB(1, led2, 1);
	//One_LED_RGB(2, led3, 1);

	while(1)
	{

	//	One_LED_RGB(0, led1, 0);
	//	One_LED_RGB(1, led2, 1);
	//	One_LED_RGB(2, led3, 1);
		Delay(20);
	}
}
