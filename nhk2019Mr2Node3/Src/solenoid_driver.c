/*
 * solenoid_driver.c
 *
 *  Created on: May 30, 2018
 *      Author: yusaku
 */
//������CarrierNode.cpp���瑗��ꂽ�M����d���قɑ���v���O����
//CarrierNode.hpp�̕��ł�8�r�b�g�����ׂď�����Ă��Ȃ��������A���炭�󂫂̕����͍�����0����[�����Ǝv��
#include "solenoid_driver.h"

#include "stm32f1xx_hal.h"

void solenoid_drive(uint8_t pattern)
{
	GPIO_PORT->BSRR = GPIO_PIN_SRCLK << 16;			// reset srclk //B15 reset
	GPIO_PORT->BSRR = GPIO_PIN_RCLK << 16;			// reset srclk //B13 reset

	int i = 0;
	while(i < 8)
	{
		GPIO_PORT->BSRR = GPIO_PIN_SRCLK << 16;			// reset srclk
		if(pattern & 0x01 != 0x00)
		{
			GPIO_PORT->BSRR = GPIO_PIN_SER;			//   set ser
		}
		else
		{
			GPIO_PORT->BSRR = GPIO_PIN_SER << 16;	//   set ser
		}
		pattern >>= 1;
		i++;
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		GPIO_PORT->BSRR = GPIO_PIN_SRCLK;			//   set srclk
	}

	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	GPIO_PORT->BSRR = GPIO_PIN_RCLK;				//   set srclk
}

void solenoid_enable(void)
{
	GPIO_PORT->BSRR = GPIO_PIN_nOD;					//   set nOD
}

void solenoid_disable(void)
{
	GPIO_PORT->BSRR = GPIO_PIN_nOD << 16;			// reset nOD
}


