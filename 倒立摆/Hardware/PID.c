#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "math.h"

void PID_Update(PID_t *p)
{
	p->Error1=p->Error0;
	p->Error0=p->Target-p->Actural;
	
	if(fabs(p->Ki)!=0.000001)
	{	
		p->ErrorInt+=p->Error0;
	}
	else
	{
		p->ErrorInt=0;
	}
	
	p->Out=p->Kp*p->Error0+p->Ki*p->Error1+p->Kd*(p->Error0-p->Error1);
	
	if(p->Out>=p->OutMax){p->Out=p->OutMax;}
	if(p->Out<=p->OutMin){p->Out=p->OutMin;}
}


//float LocKp=0.6,LocKi,LocKd=4;
//float LocError0,LocError1,LocErrorInt;
//float LocTarget,LocActural,LocOut;

//float AngleKp=0.3,AngleKi=0.01,AngleKd=0.5;
//float AngleError0,AngleError1,AngleErrorInt;
//float AngleTarget,AngleActural,AngleOut;



//PID_t AnglePID=
//{
//	AnglePID.Kp=0.3,
//	AnglePID.Ki=0.01,
//	AnglePID.Kd=0.5,
//	
//	AnglePID.Target=CENTER_ANGLE,
//	
//	AnglePID.OutMax=100,
//	AnglePID.OutMin=-100,
//}

//PID_t LocationPID=
//{
//	LocationPID.Kp=0.6,
//	LocationPID.Ki=0,
//	LocationPID.Kd=4,
//	
//	LocationPID.OutMax=50,
//	LocationPID.OutMin=-50,
//}
