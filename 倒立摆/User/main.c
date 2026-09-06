#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Serial.h"
#include "RP.h"
#include "Motor.h"
#include "Encoder.h"
#include "Key.h"
#include "LED.h"
#include "AD.h"
#include "math.h"

#define CENTER_ANGLE		2085
#define ANGLE_RANGE			600


float LocKp=0.6,LocKi=0,LocKd=4;
float LocError0,LocError1,LocErrorInt;
float LocTarget,LocActural,LocOut;

float AngleKp=0.3,AngleKi=0.01,AngleKd=0.6;
float AngleError0,AngleError1,AngleErrorInt;
float AngleTarget,AngleActural,AngleOut;

uint8_t KeyNum,RunState,StateLock;
uint16_t EncoderCount,Angle,Angle0,Angle1,Angle2;

int main(void)
{
	OLED_Init();
	Timer_Init();
	Serial_Init();
	RP_Init();
	Motor_Init();
	Encoder_Init();
	Key_Init();
	LED_Init();
	AD_Init();
	
	while (1)
	{	
			KeyNum=Key_GetNum();
		
			if(KeyNum==1)
			{
				RunState=1;
				StateLock=1;
			}
			if(KeyNum==2)
			{
				RunState=0;
				StateLock=0;
			}			
			
			LocKp=RP_GetValue(1)/4095.0*2;
			LocKi=RP_GetValue(2)/4095.0;
			LocKd=RP_GetValue(3)/4095.0*5;
			LocTarget=RP_GetValue(4)/4095.0*420;
			
			OLED_ShowString(0,0,"              ",OLED_8X16);
			OLED_Printf(0,0,OLED_8X16,"State=%d",RunState);
			
			OLED_Printf(0,16,OLED_8X16,"Kp=%4.2f",LocKp);
			OLED_Printf(0,32,OLED_8X16,"Ki=%4.2f",LocKi);
			OLED_Printf(0,48,OLED_8X16,"Kd=%4.2f",LocKd);
			
			OLED_ShowString(64,16,"        ",OLED_8X16);
			OLED_Printf(64,16,OLED_8X16,"Tar=%d",(uint16_t)LocTarget);
			OLED_ShowString(64,32,"        ",OLED_8X16);
			OLED_Printf(64,32,OLED_8X16,"Act=%d",(uint16_t)LocActural);
			OLED_ShowString(64,48,"        ",OLED_8X16);
			OLED_Printf(64,48,OLED_8X16,"out=%d",TIM_GetCapture1(TIM2));
			
			OLED_Update();
		
			Serial_Printf("%d,%f,%f,%d\r\n",Angle,LocActural,AngleOut,RunState);	
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0, Count1, Count2;
	
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Key_Tick();
		
		Angle=AD_GetValue();
		EncoderCount+=Encoder_Get();
		
		Count0++;
		
		if(Count0==40)		//每隔40个中断周期进行一次状态判断
		{
			Count0=0;
			
			Angle2=Angle1;
			Angle1=Angle0;
			Angle0=Angle;
			
			if(RunState!=2&&StateLock==1)		//避免在稳摆情况下的状态转移
			{
				if(Angle<=CENTER_ANGLE-ANGLE_RANGE&&Angle>=200){RunState=11;}
				if(Angle<=3895&&Angle>=CENTER_ANGLE+ANGLE_RANGE){RunState=12;}
			}
			
			if(StateLock==1)
			{
				if(Angle>=CENTER_ANGLE-ANGLE_RANGE&&Angle<=CENTER_ANGLE+ANGLE_RANGE){RunState=2;}
			}
		}
		
		if(RunState==0)
		{
			LED_OFF();
			StateLock=0;
			Motor_SetPWM(0);
		}
		
		if(RunState==1)
		{
			LED_ON();
			Motor_SetPWM(-50);
		}
		
		if(RunState==11)
		{
			if(Angle1>Angle0&&Angle1>Angle2)
			{
				Motor_SetPWM(50);
			}
		}
			
		if(RunState==12)
		{
			if(Angle1<Angle0&&Angle1<Angle2)
			{
				Motor_SetPWM(-50);
			}
		}
		
		if(RunState==2)
		{
			Count1++;
			Count2++;
			
			if (! (Angle > CENTER_ANGLE - ANGLE_RANGE&& Angle < CENTER_ANGLE + ANGLE_RANGE))
				{
					RunState = 0;	//恢复停止状态，防止失控
					KeyNum=0;		//同步更新KeyNum，防止状态跳转
				}
			
			/**************************位置环调节**************************/
			if(Count1>=50)
			{
				Count1=0;

				LocActural=EncoderCount;
				
				LocError1=LocError0;
				LocError0=LocTarget-LocActural;
				
				/*未开启i项调节时不进行误差累加*/
				if(fabs(LocKi)>=0.00001)
				{
					LocErrorInt+=LocError0;		
				}		
				else
				{
					LocErrorInt=0;
				}
				
				LocOut=LocKp*LocError0+LocKi*LocErrorInt+LocKd*(LocError0-LocError1);
				
				if(LocOut>50){LocOut=50;}
				if(LocOut<-50){LocOut=-50;}
				
				AngleTarget=CENTER_ANGLE-LocOut;
			}
					
			/**************************角度环调节**************************/
			if(Count2>=5)
			{
				Count2=0;
			
				AngleActural=Angle;
			
				AngleError1=AngleError0;
				AngleError0=AngleTarget-AngleActural;
				
				/*未开启i项调节时不进行误差累加*/
				if(fabs(AngleKi)>=0.00001)
				{
					AngleErrorInt+=AngleError0;		
				}		
				else
				{
					AngleErrorInt=0;
				}
				
				AngleOut=AngleKp*AngleError0+AngleKi*AngleErrorInt+AngleKd*(AngleError0-AngleError1);
				
				if(AngleOut>=100){AngleOut=100;}
				if(AngleOut<=-100){AngleOut=-100;}
				
				Motor_SetPWM(AngleOut);					
			}			
		}		
	}
}

