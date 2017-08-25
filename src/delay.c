/*
 * delay.c
 *
 *  Created on: 31 Oct 2016
 *      Author: Steve Chang
 */

#include "delay.h"
	 
static u8  fac_us = 0;
static u16 fac_ms = 0;

void delay_init(u8 SYSCLK)
{
	SysTick->CTRL &= 0xfffffffb; //bit2清空,选择外部时钟  HCLK/8
	fac_us = SYSCLK / 8;		    
	fac_ms = (u16)fac_us * 1000;
}				
				    
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD = (u32)nms * fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL = 0x00;           //清空计数器
	SysTick->CTRL = 0x01;          //开始倒数  
	do
		temp = SysTick->CTRL;
	while(temp & 0x01 && !(temp & (1<<16)));//等待时间到达   
	SysTick->CTRL = 0x00;       //关闭计数器
	SysTick->VAL = 0X00;       //清空计数器	  	    
}   

void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01;       //开始倒数 	 
	do
		temp = SysTick->CTRL;
	while(temp & 0x01 && !(temp & (1<<16)));//等待时间到达   
	SysTick->CTRL = 0x00;       //关闭计数器
	SysTick->VAL = 0X00;       //清空计数器	 
}

void delay(u32 x)
{
    u32 i,j;
	for(i=0;i<x;i++)
	   for(j=0;j<500;j++);
}
