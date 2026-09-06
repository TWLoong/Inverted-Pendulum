#ifndef __PID_H
#define __PID_H

typedef struct
{
	float Kp; 
	float Ki; 
	float Kd;
	
	float Error0; 
	float Error1; 
	float ErrorInt;
	
	float Target; 
	float Actural; 
	float Out;
	
	float OutMax;
	float OutMin;
	
}	PID_t;

void PID_Update(PID_t *p);

#endif
