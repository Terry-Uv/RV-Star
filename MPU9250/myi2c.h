/*
 * myi2c.h
 *
 *  Created on: 2021年10月16日
 *      Author: chen
 */
#ifndef APPLICATION_MYI2C_H_
#define APPLICATION_MYI2C_H_
#include <stdint.h>

void I2C_Configuration(void);
void I2C_WriteByte(uint8_t device,uint8_t addr, uint8_t data);
uint8_t I2C_ReadByte(uint8_t device,uint8_t addr);
void I2C_ReadBytes(uint8_t device,uint8_t addr,int length,uint8_t* buf);

#endif /* APPLICATION_MYI2C_H_ */
