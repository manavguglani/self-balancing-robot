/*
? * Team Id: eYRC-BB#3144
? * Author List: Manav Guglani
? * Filename: gyro_L3G.c
? * Theme: Balance Bot
? * Functions: gyro_init(), get_omega_x(), get_omega_y(), get_omega_z(), get_omega_all()
? * Global Variables:none
? */
#define F_CPU 14745600
#include <avr/io.h>
#include <util/delay.h>
#include "../I2C/i2c_lib.h"



/*
? * Function Name: gyro_init()
? * Input: none
? * Output: initializes the gyroscope
? * Logic: send data into the control register using i2c
? * Example Call: gyro_init()
? */
void gyro_init()
{	//initialize the gyroscope in normal mode and 100Hz data rate
	i2c_sendbyte(0x69<<1,0x20,0x0F);
	_delay_ms(20);
}



/*
? * Function Name: get_omega_x()
? * Input: none
? * Output: angular velocity along x axis in degrees/sec as floating point.
? * Logic: get lower and higher byte using i2c and concatenate both. 
? * Convert to degrees per second by multiplying with resolution.
? * Example Call: get_omega_x()
? */
float get_omega_x()
{
	INT8 higher_byte,lower_byte;
	float omega_x;
	//fetch lower byte and then higher byte
	i2c_getbyte(0x69<<1,0x28,&lower_byte);
	i2c_getbyte(0x69<<1,0x29,&higher_byte);
	//concatenate both and convert to degree per second
	omega_x=0.00875*((higher_byte<<8)|(UINT8)lower_byte);
	return omega_x;
}

/*
? * Function Name: get_omega_y()
? * Input: none
? * Output: angular velocity along y axis in degrees/sec as floating point.
? * Logic: get lower and higher byte using i2c and concatenate both.
? * Convert to degrees per second by multiplying with resolution.
? * Example Call: get_omega_y()
? */
float get_omega_y()
{
	INT8 higher_byte,lower_byte;
	float omega_y;
	//fetch lower byte and then higher byte
	i2c_getbyte(0x69<<1,0x2A,&lower_byte);
	i2c_getbyte(0x69<<1,0x2B,&higher_byte);
	//concatenate both and convert to degree per second
	omega_y=0.00875*((higher_byte<<8)|(UINT8)lower_byte);
	return omega_y;
}


/*
? * Function Name: get_omega_z()
? * Input: none
? * Output: angular velocity along z axis in degrees/sec as floating point.
? * Logic: get lower and higher byte using i2c and concatenate both.
? * Convert to degrees per second by multiplying with resolution.
? * Example Call: get_omega_z()
? */
float get_omega_z()
{
	INT8 higher_byte,lower_byte;
	float omega_z;
	//fetch lower byte and then higher byte
	i2c_getbyte(0x53<<1,0x2C,&lower_byte);
	i2c_getbyte(0x53<<1,0x2D,&higher_byte);
	//concatenate both and convert to degree per second
	omega_z=0.00875*((higher_byte<<8)|(UINT8)lower_byte);
	return omega_z;
}




/*
? * Function Name: get_omega_all()
? * Input: Address of three floating point variables.
? * Output: angular velocity along x,y and z axis in degrees/sec as floating point.
? * Logic: get lower and higher bytes of all 3 axis using i2c and concatenate both bytes of each axis
? * Convert to degrees per second by multiplying with resolution.
? * Example Call: get_omega_all(&a,&b,&c)
? */
void get_omega_all(float *omega_x, float *omega_y, float *omega_z)
{	INT8 all_omega[6];
	//fetch all the bytes at once in an array
	i2c_read_multi_byte(0x69<<1,0x32,6,all_omega);
	//concatenate lower and higher byte of each of the three axis rotation and convert to angular velocity
	*omega_x=0.00875*((all_omega[1]<<8)|(UINT8)all_omega[0]);
	*omega_y=0.00875*((all_omega[3]<<8)|(UINT8)all_omega[2]);
	*omega_z=0.00875*((all_omega[5]<<8)|(UINT8)all_omega[4]);
}