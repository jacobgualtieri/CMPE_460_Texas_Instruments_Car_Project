/*
 * i2c.h
 *
 *  Created on: 9/4/21
 *      Author: LJBeato
 */

#ifndef I2C_H_
#define I2C_H_
#include "Common.h"

void i2c0_Init(int listenerAddress);
void i2c0_put(BYTE *data, unsigned int len);

void i2c0_putchar(BYTE ch);

#endif /* I2C_H_ */
