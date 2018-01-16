#ifndef I2C_LIB_H
#define I2C_LIB_H

/********typedef for data types***********/
typedef unsigned char 		UINT8;
typedef signed char			INT8;
typedef unsigned short int 	UINT16;
typedef signed short int 	INT16;
typedef enum status_check_cond{ START_ERR=-7, SLAVEW_ERR,SLAVER_ERR,WRITE_ERR,READ_ERR,REPSTART_ERR,ACK_ERR,OK }STAT;

/**********i2c related terms*************/
#define done 		 	(1<<7)			
#define eack 		 	(1<<6)
#define start		 	(1<<5)
#define stop 		 	(1<<4)
#define i2cen			(1<<2)
#define write			 0x00
#define read1			 0x01

/**********function declaration**********/
void i2c_init();
void i2c_start();
void clear_twint();
void wait();
void i2c_stop();
UINT8 i2c_getstatus();
void i2c_write(UINT8 data);
void i2c_get(INT8 *data);
STAT i2c_sendbyte(UINT8 dev_add, UINT8 int_add,UINT8 data);
STAT i2c_getbyte(UINT8 dev_add,UINT8 int_add,INT8 *data);
STAT i2c_read_multi_byte(UINT8 dev_add,UINT8 int_add,UINT16 n, INT8 *data);

#endif