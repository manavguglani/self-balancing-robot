/*Begining of Auto generated code by Atmel studio */
#define F_CPU 14745600
#include <Arduino.h>
#include "I2C/i2c_lib.h"
#include "UART/UART.h"
#include "SCILAB/SCILAB.h"
#include "accelero_ADXL/accelero_ADXL.h"
#include "gyro_L3G/gyro_L3G.h"
//#include <avr/interrupt.h>

volatile long elapsed_time=0; //for noting time elapsed
volatile unsigned char state1=0,state2=0;	//present state of pins
long previous_count=0;	//for storing last time calculated count of motor
float tilt_angle=0;	//for storing tilt angle
float tilt_angle_error1=0,tilt_angle_error2=0;		//for storing error in tilt angle
float integral_tilt_angle1_error=0,integral_tilt_angle2_error=0;			//for storing integral of error
float derivative_tilt_angle_error1=0,derivative_tilt_angle_error2=0;	//for storing derivative of error
float previous_tilt_angle_error1=0,previous_tilt_angle_error2=0,previous_tilt_angle=0;;		//for storing previously calculated tilt angle
float set_angle1=0,set_angle2=0;				//angle set-points of for left and right wheel
unsigned char kp=10,ki=64,kd=22,offset=32;		//PID constants for balance control
volatile long count1=0,previous_count1=0;			// encoder 1 count and previous encoder 1 count
volatile long count2=0,previous_count2=0;			//encoder2  count and previous encoder 1 count
float motor1_speed=0,motor2_speed=0,previous_motor1_speed=0,previous_motor2_speed=0;	//storing present and previous speed of motors
float motor1_speed_error=0,motor2_speed_error=0,previous_motor1_speed_error=0,previous_motor2_speed_error=0;	//for storing motor speed errors
float integral_motor1_speed_error=0,integral_motor2_speed_error=0;	// for storing integral of speed errors
const INT8 a[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};	//lookup table for encoder count calculation
INT8 rotational_velocity=0;		//for storing rotation velocity
INT8 translational_velocity=0;		//for storing translational velocity
unsigned char ki_velocity_control=26,kp_velocity_control=53,kd__velocity_control=0;	//PID constants for velocity control 53, 26, 0
struct rxFrame rxf;
struct txFrame txf;
float angle_offset=0;
void uart1_tx(char data)
{
	while(!(UCSR1A & TE));						//waiting to transmit
	UDR1 = data;
}
void uart1_init()
{
	UCSR1B = 0x00;							//disable while setting baud rate

	UBRR1L = 95; 							// for the clock frequency 14745600 and the baud rate 9600, value of UBRR is 95
	UBRR1H = 0x00;

	UCSR1C = 3<<1;							//setting 8-bit character and 1 stop bit
	UCSR1B = RX | TX;						//enabling receiver and transmit
}

char uart1_rx()
{
	while(!(UCSR1A & RE));						//waiting to transmit
	return UDR1;
}

/*
? * Function Name: motor_pin_config()
? * Input: none
? * Output: configures motor pin
? * Logic: set PH4,PH6,PH7 as output and their value as low,set PK0 and PK1 as output,enable pullup
? * Example Call: motor_pin_config()
? */
void motor_pin_config(void)
{	
	DDRH=DDRH|0xDE;		//set PH1,PH2,PH3,PH4,PH6,PH7 as output
	PORTH = PORTH & 0x21;	//set above pin low
	DDRK=DDRK&0xF0;		//set PK0,PK1,PK2 and PK3 as input	(encoder pins)
	PORTK=PORTK|0x0F;	//enable pull-up
}
/*
? * Function Name: encoder_config()
? * Input: none
? * Output: configures quadrature encoder
? * Logic: clear global interrupt, enables Pin Change Interrupt 2, set interrupt for PK0 and PK1 in pin change mask register,
? * sets initial state and enable global interrupt
? * Example Call: encoder_config()
? */
void encoder_config()
{
	cli();		//clear global interrupt
	PCICR=0x04;	//enables Pin Change Interrupt 2
	PCMSK2=0x0F;//set interrupt for PK0 PK1 in pin change mask register
	state1=PINK&0x03;	// sets initial state
	state2=(PINK&0x0C)>>2;	//sets initial state
	sei();		//enable global interrupt
}



/*
? * Function Name: timer0_init()
? * Input: none
? * Output: initializes the timer0
? * Logic: clear global interrupt, select mode and prescale, enable output compare interrupt,
? * give timer initial value as 0, set output compare register value to 57 and enable global interrupt
? * Example Call: timer0_init()
? */

void timer0_init(){
	cli();			//clear global interrupts
	TCCR0A=0x02;	//mode selection
	TCCR0B=0x04;	//prescale=256, mode=CTC
	TIMSK0=0x02;	//enable output compare interrupt
	TCNT0=0;		//initial value of timer
	OCR0A=57;		//reset timer when it matches with 57 (1ms)
	sei();			//enable global interrupt
}



/*
? * Function Name: timer4_init()
? * Input: none
? * Output: initializes the timer4
? * Logic: clear global interrupt, select mode and prescale, enable pwm mode,enable global interrupt
? * Example Call: timer4_init()
? */

void timer4_init()
{	cli();
	TCCR4B = 0x00;	//Stop
	TCNT4H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT4L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR4BH = 0x00;	//Output compare register high value for Right Motor
	OCR4BL = 0xFF;	//Output compare register low value for Right Motor
	TCCR4A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR4B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
	OCR4BH = 0x00;
	OCR4AL = 0xFF;
	
	sei();
}

/*
? * Function Name: velocity()
? * Input: unsigned char velocity for setting motor speed
? * Output: sets motor speed
? * Logic: sets output compare register to velocity value and hence changing duty cycle
? * Example Call: velocity(100)
? */

void velocity (unsigned char left, unsigned char right)
{	
	OCR4AL = (unsigned char)left;	//sets output compare register to velocity value
	OCR4BL = (unsigned char)right;	//sets output compare register to velocity value
}

/*
? * Function Name: back()
? * Input: none
? * Output: rotates motor in forward direction
? * Logic: sets PH7 and PH2 to high and PH6 and PH1 to low
? * Example Call: back()
? */
void back (void) //both wheels forward
{
	PORTH=PORTH|0x84;	//sets PH7 and PH2 to high
	PORTH=PORTH&0xBD;	//set PH6 and PH1 to low

}


/*
? * Function Name: back()
? * Input: none
? * Output: rotates motor in backward direction
? * Logic: sets PH6 and PH1 to high and PH7 and PH2 to low
? * Example Call: back()
? */
void forward (void) //both wheels backward
{
	PORTH=PORTH|0x42;	//sets PH6 and PH1 to high
	PORTH=PORTH&0x7B;	//sets PH7 and PH2 to low
}


/*
? * Function Name: stop()
? * Input: none
? * Output: stops motor
? * Logic: sets PH7,PH6,PH2,PH1 to low
? * Example Call: stop()
? */
void stop_b (void)
{
	PORTH=PORTH&0x39;	//sets PH7, PH6, PH2, PH1 to low
	
}



/*
? * Function Name: buzzer_pin_config()
? * Input: none
? * Output: configure pins of buzzer
? * Logic: sets DDRF0 as 1 and enable pull-up 
? * Example Call: buzzer_pin_config
? */
void buzzer_pin_config()
{
	DDRF|=0x01;		//Set pin PF0 as output 
	PORTF|=0x01;	//off buzzer
}

/*
? * Function Name: init_devices()
? * Input: none
? * Output: initializes all the hardware
? * Logic: clear global interrupt, calls all the initialization functions of the respective hardwares and enable global interrupt
? * Example Call: init_devices()
? */
void init_devices (void) //use this function to initialize all devices
{
	cli();					//disable all interrupts
	buzzer_pin_config();
	motor_pin_config();			//configure motor ports
	timer0_init();			//initialize timer0
	timer4_init();			//initialize timer4
	encoder_config();		//configure encoder
	Serial.begin(9600);
//	uart0_init();
	uart1_init();
	i2c_init();
	accelero_init();
	gyro_init();
	sei();					//re-enable interrupts
}


/*
? * Function Name: update_encoder_count()
? * Input: none
? * Output: calculates count for motor quadrature encoder
? * Logic: Uses a lookup table to count value. The table takes care of both rotation magnitude and sense of rotation.
? * Example Call: update_encoder_count()
? */

void update_encoder_count()
{
	state1<<=2;			//shifts state1 left by 2
	state1|=(PINK&0x03);		//getting new state from PINK0 and PINK1
	state1&=0x0F;				//mask higher nibble
	state2<<=2;				//shifts state2 left by 2
	state2|=((PINK&0x0C)>>2);	//getting new state from PINK2 and PINK3
	state2&=0x0F;					//mask higher nibble
	count1+=a[state1];				//calculate new count1
	count2+=a[state2];			//calculate new count1
}

/*
? * Function Name: ISR(PCINT2_vect)
? * Input: none
? * Output: updates encoder count
? * Logic: gets executed whenever a pin of PK0 and PK1 toggles. calls update_encoder_count();
? * Example Call: automatically called
? */
ISR(PCINT2_vect)
{
	update_encoder_count(); // updates count value
}

/*
? * Function Name: ISR(TIMER0_COMPA_vect)
? * Input: none
? * Output: increments time elapsed by 1
? * Logic: Interrupt is fired after every 1ms. Counter for elapsed time is incremented by 1
? * Example Call: none. It is automatically called
? */
ISR(TIMER0_COMPA_vect)
{
	elapsed_time++; // increment the counter indicating 1ms time
}

/*
? * Function Name: epoch()
? * Input: none
? * Output: unsigned long elapsed_time variable giving time in ms
? * Logic: Just return the elapsed_time variable which stores time elapsed in ms
? * Example Call: epoch()
? */
unsigned long epoch()
{	uint8_t oldSREG = SREG;
	unsigned long t;
	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	t = elapsed_time;
	SREG = oldSREG;
	
	return t;
}


/*
? * Function Name: tilt_angles_update()
? * Input: none
? * Output: tilt angle in degrees as float
? * Logic: gets acceleration in x-axis and z-axis, calculate tilt angle by accelerometer readings, gets angular velocity along y-axis,
? *			calculate tilt angle using accelerometer reading and combine both angles using complementary filter to get accurate tilt angle
? * Example Call: tilt_angles_update()
? */
void tilt_angles_update()
{

	float acc_x,acc_z,omega_y;
	float tilt_angle_accelero,tilt_angle_gyro;
	
	// get z axis and x axis acceleration
	acc_z=get_acc_z();
	acc_x=get_acc_x();
	//convert the data into degrees giving pitch
	tilt_angle_accelero=-57.29*atan2(acc_x,acc_z);
	

	/***code for measuring tilt angle using gyroscope here**/
	//get angular velocity along y axis
	omega_y=get_omega_y();
	//calculate the angle by integrating (summation) angular velocity with time elapsed in each cycle
	tilt_angle_gyro = omega_y*.05+tilt_angle;
	/***end of code for measuring tilt angle using gyroscope**/
	
	
	//using complementary filter for fusion of both the sensors
	tilt_angle = tilt_angle_gyro*0.95 + tilt_angle_accelero*0.05;
	

}

/*
? * Function Name: velocity_correct()
? * Input: int left for left motor velocity, int right for right motor velocity and int velocity_offset for adding motor offset
? * Output: gives velocity and direction to motors according to sign of inputs and limits the velocity.
? * Logic: if left is positive add offset to it and move left motor forward. if it is negative, add offset to negative to it and move left motor backward.
? *			limit left to 255. Apply this velocity to left motor. Same logic for right motor
? * Example Call: velocity_correct(100,100,30);
? */

void velocity_correct(int left, int right, int velocity_offset)
{
	if(left>=0)				//if left is positive then
	{
		left=left+velocity_offset;		//add offset
		PORTH=PORTH|0x40;	//sets PH6 to high
		PORTH=PORTH&0x7F;	// and PH7 to low
	}
	else						//if left is negative then
	{
		left=velocity_offset-left;	//add offset to negative of velocity
		PORTH=PORTH|0x80;	//sets PH7 to high
		PORTH=PORTH&0xBF;	//and PH6 to low
	}
	if(left>255)					//bound left to 255
	left=255;
	OCR4BL=(unsigned char)left;		//give velocity to left motor
	if(right>=0)		//if right is positive then
	{
		right=right+velocity_offset;		//add offset to right
		PORTH=PORTH|0x02;			//set PH1 to high
		PORTH=PORTH&0xFB;			//set PH2 to low
	}
	else						//if right is negative then
	{
		right=velocity_offset-right;			//add negative of it to offset
		PORTH=PORTH|0x04;						//set PH2 high
		PORTH=PORTH&0xFD;					//set PH1 to low
	}
	if(right>255)					//bound right velocity
	right=255;						
	OCR4AL=(unsigned char)right;		//apply velocity to right motor
}

void constrain_value(float *value, float lower_limit,float upper_limit)
{
	if(*value>upper_limit)
	*value=upper_limit;
	if(*value<lower_limit)
	*value=lower_limit;
}
/*
? * Function Name: pid_angle()
? * Input: none
? * Output: update PID value and motor velocity
? * Logic: Calculate proportional term, Integral term and derivative term and evaluate motor speed using this values
? * Example Call: pid_angle()
? */
void pid_angle()
{
	float P1,D1,P2,D2,vel1,vel2;		//P,D and output velocity for left and right motor
	tilt_angle_error1=tilt_angle-set_angle1;	//error in tilt angle for motor1
	tilt_angle_error2=tilt_angle-set_angle2;	//error in tilt angle for motor2
	derivative_tilt_angle_error1=tilt_angle-previous_tilt_angle;	//derivative of error in tilt angle for motor1
	derivative_tilt_angle_error2=tilt_angle-previous_tilt_angle;	//derivative of error in tilt angle for motor2
	previous_tilt_angle_error1=tilt_angle_error1;				//set previous error to present error for motor1
	previous_tilt_angle_error2=tilt_angle_error2;				//set previous error to present error for motor1
	previous_tilt_angle=tilt_angle;							//set previous tilt angle to present tilt angle
	P1=kp*tilt_angle_error1;			//proportional term calculation for motor1
	P2=kp*tilt_angle_error2;			//proportional term calculation for motor2
	integral_tilt_angle1_error+=(ki/10.0)*(tilt_angle_error1);			// integral term calculation for motor1
	constrain_value(&integral_tilt_angle1_error,-250,250);
	integral_tilt_angle2_error+=(ki/10.0)*(tilt_angle_error2);			// integral term calculation for motor2
	constrain_value(&integral_tilt_angle2_error,-250,250);			//bounding the integral
	D1=kd*derivative_tilt_angle_error1;		//calculating derivative term for motor1
	D2=kd*derivative_tilt_angle_error2;		//calculating derivative term for motor2
	constrain_value(&D1,-150,150);
	constrain_value(&D2,-150,150);
	vel1=P1 + integral_tilt_angle1_error + D1;				//calculate net PID velocity for motor1
	vel2=P2 + integral_tilt_angle1_error + D2;				//calculate net PID velocity for motor2
	velocity_correct(vel2,vel1,offset);		//apply the calculated velocities to motors
}

void pid_velocity()
{	motor1_speed=count1-previous_count1;		//calculate motor1 speed
	motor2_speed=count2-previous_count2;		//calculate motor2 speed
	motor1_speed_error=motor1_speed-translational_velocity+rotational_velocity;	//calculate error in motor1 speed
	motor2_speed_error=motor2_speed-translational_velocity-rotational_velocity; //calculate error in motor2 speed
	integral_motor1_speed_error+=ki_velocity_control*motor1_speed_error;	//calculate integral error of motor1 speed
	integral_motor2_speed_error+=ki_velocity_control*motor2_speed_error;	//calculate integral error of motor2 speed
	constrain_value(&integral_motor1_speed_error,-80000,50000);
	constrain_value(&integral_motor2_speed_error,-80000,50000);
	set_angle1=angle_offset+integral_motor1_speed_error/5000.0 + kp_velocity_control*(motor1_speed_error)/1000.0+kd__velocity_control*(motor1_speed-previous_motor1_speed)/1000.0;
	//calculate PID output for motor1
	set_angle2=angle_offset+integral_motor2_speed_error/5000.0 + kp_velocity_control*(motor2_speed_error)/1000.0+kd__velocity_control*(motor2_speed-previous_motor2_speed)/1000.0;
	//calculate PID output for motor1
	previous_count1=count1;		//set previous_count1 as present count1
	previous_count2=count2;		//set previous_count2 as present count2
	previous_motor1_speed_error=motor1_speed_error; //set previous_motor1_speed error as present motor1 speed error
	previous_motor2_speed_error=motor2_speed_error;	//set previous_motor2_speed error as present motor2 speed error
	previous_motor1_speed=motor1_speed;				//set previous_motor1_speed as present motor1 speed
	previous_motor2_speed=motor2_speed;				//set previous_motor2_speed as present motor2 speed
}
/*
? * Function Name: buzzer_on()
? * Input: none
? * Output: on buzzer
? * Logic: set PF0 to low
? * Example Call: buzzer_on();
? */

void buzzer_on()
{
	PORTF&=0xFE;
}

/*
? * Function Name: buzzer_off()
? * Input: none
? * Output: off buzzer
? * Logic: set PF0 to high
? * Example Call: buzzer_off();
? */
void buzzer_off()
{
	PORTF|=0x01;
}

/*
? * Function Name: reset_pids()
? * Input: none
? * Output: reset pid parameters
? * Logic: set integral values to zero and previous values to present values
? * Example Call: reset_pids();
? */

void reset_pids()
{
	integral_tilt_angle1_error=0;
	integral_tilt_angle2_error=0;
	integral_motor1_speed_error=tilt_angle;
	integral_motor2_speed_error=tilt_angle;
	previous_motor1_speed=0;
	previous_motor2_speed=0;
	previous_count1=0;
	previous_count2=0;
	count1=0;
	count2=0;
	previous_tilt_angle=tilt_angle;
	previous_tilt_angle=tilt_angle;
}
char prbzr=1;
/*
? * Function Name: receive_joystick_data()
? * Input: none
? * Output: turn buzzer on or off according to joystick switch state and get rotational and translational velocity
? * Logic: read from buffer until 0x7E. After that discard 11 bytes, read next byte and set state of buzzer according to it,
? *			read two bytes and concatenate to get translational velocity, same for rotational velocity and finally discard 1 byte
? * Example Call: receive_joystick_data();
? */
float previous_rotational_velocity=0,present_rotational_velocity=0;
float previous_translational_velocity=0,present_translational_velocity=0;
void receive_joystick_data()
{	int i=0,m;
	UINT8 a=0,b=0;
	float x;
	while(Serial.read()!=0x7E); 			//read from buffer until 0x7E
		for(i=1;i<=9;i++)				//discard 11 bytes
		Serial.read();	
		a=Serial.read();
		b=Serial.read();
		if(a==12 && b==0){
		a=Serial.read();
		if(a&4)			//change buzzer state according to switch on joystick
		{
			if(prbzr==1) 
			{buzzer_off();}	
			prbzr=1;
		}
		else{
			if(prbzr==0)
		{buzzer_on();}
		prbzr=0;			
		}
		if(a&8)
		{
			kp_velocity_control=26;
			ki_velocity_control=53;
		}
		else
		{
			kp_velocity_control=53;
			ki_velocity_control=16;
		}
		a=Serial.read();		//read byte
		b=Serial.read();		//read byte
		x=(a<<8)|b;				//concatenate above 2
		present_translational_velocity=(.116630*x -64.31);		//convert to equivalent translational velocity
		if(present_translational_velocity==previous_translational_velocity)
		translational_velocity=previous_translational_velocity;
		previous_translational_velocity=present_translational_velocity;

		a=Serial.read();		//read byte
		b=Serial.read();		//read byte
		x=(a<<8)|b;				//concatenate above 2
		present_rotational_velocity=.5*(.116630*x -65.31);	//convert to equivalent rotational velocity
		if(previous_rotational_velocity==present_rotational_velocity)
		rotational_velocity=present_rotational_velocity;
		previous_rotational_velocity=present_rotational_velocity;
		Serial.read();				//discard 1 byte
		}
		else
		{
			for(i=0;i<=6;i++)
			Serial.read();
		}
}



/*
? * Function Name: main()
? * Input: none
? * Output: int to inform the caller that the program exited correctly or incorrectly
? * Logic: Initialize devices, after every 50ms update tilt angle,if absolute value of tilt angle is greater than 45 than stop bot and reset PID
? *			otherwise call pid controller for velocity control and angle control. Receive joystick data every 300ms.
? * Example Call: Called automatically by system
? */

int main()
{	
	init_devices();		//initialize devices
	uart1_init();
	long present_time=0;	//for noting present time
	long previous_tilt_angle_update=0;				//for storing previous tilt angle update time
	long previous_rx_time=0;
	stop_b();
	//velocity_correct(200,200,0);
	char tempr;
//	_delay_ms(2000);
/*	kp=uart1_rx();
	//	_delay_ms(500);
	uart1_tx(kp);
	//		_delay_ms(500);
	ki=uart1_rx();
	//		_delay_ms(500);
	uart1_tx(ki);
	//		_delay_ms(500);
	kd=uart1_rx();
	//		_delay_ms(500);
	uart1_tx(kd);
	//		_delay_ms(500);
	//offset=uart_rx();
	//uart1_tx(offset);


	ki_velocity_control=uart1_rx();
	//		_delay_ms(500);
	uart1_tx(ki_velocity_control);
	//		_delay_ms(500);
	kp_velocity_control=uart1_rx();
	//		_delay_ms(500);
	uart1_tx(kp_velocity_control);
	//		_delay_ms(500);
	kd__velocity_control=uart1_rx();
	//		_delay_ms(500);
	uart1_tx(kd__velocity_control);
	//		_delay_ms(500);
	tempr=uart1_rx();
	uart1_tx(tempr);
	angle_offset=-.1*tempr;
	//tsp=uart1_rx();*/
	//uart_tx(set_angle);
	Serial1.begin(9600);
	char temp;
    while(1) 
		{
		present_time = epoch();						// get present time
		
		if ((present_time-previous_tilt_angle_update)>=50)			//after every 50ms
		{	
			PCICR=0x00;				//disable encoder interrupt 2
			tilt_angles_update();			//update tilt angles
			PCICR=0x04;	//enables Pin Change Interrupt 2
			previous_tilt_angle_update=present_time;		//Previous tilt angle is now present tilt angle
			if(fabs(tilt_angle) > 45)				//if absolute value of tilt angle is greater than 45 than stop bot and reset PID
			{
				stop_b();			//stop bot
				reset_pids();		//reset pid values
			}
			else 					//otherwise
			{
				pid_velocity();			//call	PID controller for velocity		
				pid_angle();		//call PID controller for angle 
			}
		}
		if( (present_time-previous_rx_time)>=80)	//after every 80ms
		{	
			
			if(Serial.available()>=18){				//if data in buffer is greater than 18 bytes then
			receive_joystick_data();				//receive joystick data
			if(translational_velocity!=0 || rotational_velocity!=0){
	//		Serial1.print(translational_velocity);
	//		Serial1.print(" ");
	//		Serial1.println(rotational_velocity);
				}
			}
			
			previous_rx_time=present_time;		//set previous receive time to present time
		}	
		
	
		}
		}
