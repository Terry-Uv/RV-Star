#include "myi2c.h"
#include "mpu9250.h"
#include <stdio.h>
#include "gd32vf103.h"

static float acc[3],gyro[3];
static short mag[3];

int main(void)
{
	I2C_Configuration();
	int res=MPU9250_Init();
	if (res==1) return -1;
	while (1) {
		GetAccelDataApi(acc);
		printf("acceleration: x: %.2f m/s^2 y: %.2f m/s^2 z: %.2f m/s^2\n",acc[0],acc[1],acc[2]);
		delay_1ms(1000);

		GetGyroDataApi(gyro);
		printf("gyroscope:    x: %.2f   y: %.2f  z: %.2f\n",gyro[0],gyro[1],gyro[2]);
		delay_1ms(1000);

		GetMagDataApi(mag);
		printf("magnet: x: %d y: %d z: %d\n", (int)mag[0],(int)mag[1],(int)mag[2]);
		delay_1ms(1000);
	}

	return 0;
}
