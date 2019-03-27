/*
 * solenoid_driver.h
 *
 *  Created on: May 30, 2018
 *      Author: yusaku
 */

#ifndef SOLENOID_DRIVER_H_
#define SOLENOID_DRIVER_H_

#include "stdint.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define GPIO_PORT      GPIOB
#define GPIO_PIN_nOD   GPIO_PIN_12
#define GPIO_PIN_RCLK  GPIO_PIN_13
#define GPIO_PIN_SER   GPIO_PIN_14
#define GPIO_PIN_SRCLK GPIO_PIN_15

void solenoid_drive(uint8_t pattern);
void solenoid_enable(void);
void solenoid_disable(void);

#ifdef __cplusplus
 }
#endif

#endif /* SOLENOID_DRIVER_H_ */
