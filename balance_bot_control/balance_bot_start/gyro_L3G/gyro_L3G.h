/*
 * gyro_L3G.h
 *
 * Created: 06-12-2016 09:55:37
 *  Author: Manav Guglani
 */ 


#ifndef L3G4200D_H_
#define L3G4200D_H_

void gyro_init();
float get_omega_x();
float get_omega_y();
float get_omega_z();
void get_omega_all(float *omega_x, float *omega_y, float *omega_z);


#endif 