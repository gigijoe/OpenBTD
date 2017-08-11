#include "pwm.h"
#include "bool.h"

void Pwm_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA and GPIOC clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = PwmPulseMax - 1;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	//TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;       //初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

static bool pwm1_reverse = false;

void Pwm1_Reverse()
{
	pwm1_reverse = true;
}

void Pwm1_Pulse(uint16_t pulse)
{
	if(pulse > PwmPulseMax)	
		pulse = PwmPulseMax;

	if(pwm1_reverse)
		pulse = PwmPulseMax - pulse;
	
	TIM2->CCR1 = pulse;
}

void Pwm_Pulse(uint8_t channel, uint16_t pulse)
{
	if(pulse > PwmPulseMax)	
		pulse = PwmPulseMax;

	if(pwm1_reverse)
		pulse = PwmPulseMax - pulse;

	switch(channel) {
		case 1: TIM2->CCR1 = pulse;
			break;
		default: return;
	}
}
