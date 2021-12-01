/*
 * myi2c.c
 *
 *  Created on: 2021年10月16日
 *      Author: chen
 */

#include "myi2c.h"
#include "mpu9250.h"
#include "nuclei_sdk_soc.h"
void I2C_Configuration(void)
{
    uint32_t GPIO_SDA, GPIO_SCL;
    uint32_t GPIO_PIN_SDA, GPIO_PIN_SCL;

    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C1);

    GPIO_SDA = GPIOB;
    GPIO_PIN_SDA = GPIO_PIN_11;
    GPIO_SCL = GPIOB;
    GPIO_PIN_SCL = GPIO_PIN_10;

    gpio_init(GPIO_SCL, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_SCL);
    gpio_init(GPIO_SDA, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_SDA);

    i2c_disable(I2C1);
    const uint32_t NORMAL_SPEED =100000;
    i2c_clock_config(I2C1, NORMAL_SPEED, I2C_DTCY_2);
    i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x01 << 1);

    i2c_ackpos_config(I2C1, I2C_ACKPOS_CURRENT);
    i2c_ack_config(I2C1, I2C_ACK_DISABLE);

    i2c_enable(I2C1);
}

void I2C_WriteByte(uint8_t device,uint8_t addr, uint8_t data)
{
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C1, I2C_FLAG_I2CBSY));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C1);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C1, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus*/
    i2c_master_addressing(I2C1, device, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set*/
    while(!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);

    /* send a addr byte */
    i2c_data_transmit(I2C1, addr);
    /* wait until the transmission data register is empty*/
    while(!i2c_flag_get(I2C1, I2C_FLAG_TBE));

    /* send a data byte */
    i2c_data_transmit(I2C1, data);
    /* wait until the transmission data register is empty*/
    while(!i2c_flag_get(I2C1, I2C_FLAG_TBE));

    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2C1);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C1)&0x0200);
}

uint8_t I2C_ReadByte(uint8_t device,uint8_t addr) {
	/* wait until I2C bus is idle */
	while(i2c_flag_get(I2C1, I2C_FLAG_I2CBSY));

    /* config write device and addr */
	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C1);
	/* wait until SBSEND bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_SBSEND));
	/* send slave address to I2C bus*/
	i2c_master_addressing(I2C1, device, I2C_TRANSMITTER);
	/* wait until ADDSEND bit is set*/
	while(!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND));
	/* clear ADDSEND bit */
	i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);

	/* send a addr byte */
	i2c_data_transmit(I2C1, addr);
	/* wait until the transmission data register is empty*/
	while(!i2c_flag_get(I2C1, I2C_FLAG_TBE));


	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C1);
	/* wait until SBSEND bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_SBSEND));
	/* send slave address to I2C bus */
	i2c_master_addressing(I2C1,device,I2C_RECEIVER);
	/*　wait until ADDSEND bit is set */
	while (!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND));
	/* N=1,reset ACKEN bit before clearing ADDRSEND bit */
	i2c_ack_config(I2C1, I2C_ACK_DISABLE);
	/* clear ADDSEND bit */
	i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);
	/* N=1,send stop condition after clearing ADDRSEND bit */
	i2c_stop_on_bus(I2C1);
	/* wait until the RBNE bit is set */
	while (!i2c_flag_get(I2C1, I2C_FLAG_RBNE));

	uint8_t res=i2c_data_receive(I2C1);
	while (I2C_CTL0(I2C1)&0x0200);
	/* Enable Acknowledge */
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	return res;
}

void I2C_ReadBytes(uint8_t device,uint8_t addr,int length,uint8_t* buf) {
	for (int i=0;i<length;i++) {
		buf[i]=I2C_ReadByte(device,addr+i);
	}
}
