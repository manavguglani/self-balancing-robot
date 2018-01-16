/*
? * Team Id: eYRC-BB#3144
? * Author List: Manav Guglani
? * Filename: accelero_ADCL.c
? * Theme: Balance Bot
? * Functions: accelero_init(), get_acc_x(), get_acc_y(), get_acc_z(), get_acc_all()
? * Global Variables:none
? */

#define F_CPU 14745600
#include <avr/io.h>
#include <util/delay.h>
#include "../I2C/i2c_lib.h"




/*
? * Function Name: accerlero_init()
? * Input: none
? * Output: initializes the accelerometer
? * Logic: send data into Bandwidth rate, power control and data format register using i2c
? * Example Call: accelero_init()
? */

void accelero_init()
{	
	//set 100Hz data rate and normal power mode
	i2c_sendbyte(0x53<<1,0x2C,0x0A);
	_delay_ms(20);
	//set in measure mode and normal mode
	i2c_sendbyte(0x53<<1,0x2D,0x08);
	_delay_ms(20);
	//set 2g range and data format
	i2c_sendbyte(0x53<<1,0x31,0x00);
	_delay_ms(20);
}




/*
? * Function Name: get_acc_x()
? * Input: none
? * Output: acceleration along x axis in units of g as floating point.
? * Logic: get lower and higher byte using i2c and concatenate both.
? * Convert to g units by multiplying with resolution.
? * Example Call: get_acc_x()
? */
float get_acc_x()
{
	INT8 higher_byte,lower_byte;
	float acc_x;
	//fetch lower byte and then higher byte
	i2c_getbyte(0x53<<1,0x32,&lower_byte);
	i2c_getbyte(0x53<<1,0x33,&higher_byte);
	//concatenate both and convert into units of g
	acc_x=0.0039*((higher_byte<<8)|(UINT8)lower_byte);
	return acc_x;
}



/*
? * Function Name: get_acc_y()
? * Input: none
? * Output: acceleration along y axis in units of g as floating point.
? * Logic: get lower and higher byte using i2c and concatenate both.
? * Convert to g units by multiplying with resolution.
? * Example Call: get_acc_y()
? */
float get_acc_y()
{
	INT8 higher_byte,lower_byte;
	float acc_y;
	//fetch lower byte and then higher byte
	i2c_getbyte(0x53<<1,0x34,&lower_byte);
	i2c_getbyte(0x53<<1,0x35,&higher_byte);
	//concatenate both and convert into units of g
	acc_y=0.0039*((higher_byte<<8)|(UINT8)lower_byte);
	return acc_y;
}



/*
? * Function Name: get_acc_z()
? * Input: none
? * Output: acceleration along z axis in units of g as floating point.
? * Logic: get lower and higher byte using i2c and concatenate both.
? * Convert to g units by multiplying with resolution.
? * Example Call: get_acc_z()
? */
float get_acc_z()
{
	INT8 higher_byte,lower_byte;
	float acc_z;
	//fetch lower byte and then higher byte
	i2c_getbyte(0x53<<1,0x36,&lower_byte);
	i2c_getbyte(0x53<<1,0x37,&higher_byte);
	//concatenate both and convert into units of g
	acc_z=0.0039*((higher_byte<<8)|(UINT8)lower_byte);
	return acc_z;
}




/*
? * Function Name: get_acc_all()
? * Input: Address of three floating point variables.
? * Output: acceleration along x,y and z axis in units of as floating point.
? * Logic: get lower and higher bytes of all 3 axis using i2c and concatenate both bytes of each axis
? * Convert to g units per second by multiplying with resolution.
? * Example Call: get_acc_all(&a,&b,&c)
? */
void get_acc_all(float *acc_x, float *acc_y, float *acc_z)
{	INT8 all_acc[6];
	//fetch all axis acceleration in an array
	i2c_read_multi_byte(0x53<<1,0x32,6,all_acc);
	//concatenate lower and higher bytes of each and convert into units of g
	*acc_x=0.0039*((all_acc[1]<<8)|(UINT8)all_acc[0]);
	*acc_y=0.0039*((all_acc[3]<<8)|(UINT8)all_acc[2]);
	*acc_z=0.0039*((all_acc[5]<<8)|(UINT8)all_acc[4]);
}