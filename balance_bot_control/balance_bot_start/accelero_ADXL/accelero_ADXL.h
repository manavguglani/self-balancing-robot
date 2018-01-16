/*
 * accelero_ADXL.h
 *
 * Created: 06-12-2016 08:43:30
 *  Author: Manav Guglani
 */ 


#ifndef ADXL345_H_
#define ADXL345_H_
void accelero_init();
float get_acc_x();
float get_acc_y();
float get_acc_z();
void get_acc_all(float *acc_x, float *acc_y, float *acc_z);





#endif 