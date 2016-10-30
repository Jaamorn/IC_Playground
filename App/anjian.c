#include "include.h"

#define kp_speed 160
#define ki_speed 40
#define kd_speed 40
int init_speed=0;
int speed;
int error_speed[3]={0};
int flaga;
void speed_set(int speed_set)
{
    int adjust_speed;

    speed = DMA_count_get(DMA_CH2);     //计数寄存器   DMA累加计数
    if(gpio_get(PTD3)==1)speed=-speed;
    DMA_count_reset(DMA_CH2);
    flaga=gpio_get(PTD3);

   // LCD_Show_Number(6,6,speed);
    error_speed[2]=error_speed[1];
    error_speed[1]=error_speed[0];
    error_speed[0]=speed_set-speed;                                //原先是float  (int)(ki_speed*error_speed[0])


    if(error_speed[0]>30)
    {
      init_speed=9500;
    }
   if(error_speed[0]<-30)
    {
      init_speed=-9500;
    }
    if(error_speed[0]>=-30&&error_speed[0]<=30)
    {
      adjust_speed=kp_speed*(error_speed[0]-error_speed[1])+(int)(ki_speed*error_speed[0])+kd_speed*(error_speed[0]-2*error_speed[1]+error_speed[2]);
      init_speed=init_speed+adjust_speed;
    }
    if(init_speed>9500)        //保护程序              //limit the max PWM for motor
    {
      init_speed=9500;
    }
    else if(init_speed<-9500)                    //limit the min PWM for motor
    {
      init_speed=-9500;
    }

  //
  //   if(init_speed<0)
  //   {
  //
  // 
  //
  //
  //  //  ftm_pwm_duty(FTM0, FTM_CH4, 0);
  // //   ftm_pwm_duty(FTM0, FTM_CH3, -init_speed);
  //    ftm_pwm_duty(FTM0, FTM_CH2, 0);
  //    ftm_pwm_duty(FTM0, FTM_CH1, -init_speed);
  //
  //
  //   }
  //   else
  //   {
  //
  //  //  ftm_pwm_duty(FTM0, FTM_CH3, 0);
  //   // ftm_pwm_duty(FTM0, FTM_CH4, init_speed);
  //    ftm_pwm_duty(FTM0, FTM_CH2, init_speed);
  //    ftm_pwm_duty(FTM0, FTM_CH1, 0);
  //
  //   }



}
