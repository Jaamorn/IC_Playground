#include "include.h"

#define kp_speed 160
#define ki_speed 40 //80
#define kd_speed 40
int init_speed=0;
int speed;
int error_speed[3]={0};
int speedset=0;

uint8 key1;
uint8 key2;
uint8 key3;
uint8 key4;
uint8 lastkey1;
uint8 lastkey2;
uint8 lastkey3;
uint8 lastkey4;

void speed_set(int speed_set)
{
    int adjust_speed;

    speed = DMA_count_get(DMA_CH2);     //
    if(gpio_get(PTD3)==1)speed=-speed;
    DMA_count_reset(DMA_CH2);


   // LCD_Show_Number(6,6,speed);
    error_speed[2]=error_speed[1];
    error_speed[1]=error_speed[0];
    error_speed[0]=speed_set-speed;                                //ԭ����float  (int)(ki_speed*error_speed[0])


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
    if(init_speed>9500)                    //limit the max PWM for motor
    {
      init_speed=9500;
    }
    else if(init_speed<-9500)                    //limit the min PWM for motor
    {
      init_speed=-9500;
    }


    if(init_speed<0)
    {




   //  ftm_pwm_duty(FTM0, FTM_CH4, 0);
  //   ftm_pwm_duty(FTM0, FTM_CH3, -init_speed);
     ftm_pwm_duty(FTM0, FTM_CH2, 0);
     ftm_pwm_duty(FTM0, FTM_CH1, -init_speed);


    }
    else
    {

   //  ftm_pwm_duty(FTM0, FTM_CH3, 0);
    // ftm_pwm_duty(FTM0, FTM_CH4, init_speed);
     ftm_pwm_duty(FTM0, FTM_CH2, init_speed);
     ftm_pwm_duty(FTM0, FTM_CH1, 0);

    }



}

void key_scan()
{
  key1=gpio_get(PTB1);
  key2=gpio_get(PTB3);
  key3=gpio_get(PTB5);
  key4=gpio_get(PTB7);
}

void setting()
{
 int speedtemp;
 int stage_flag;
 speedset=0;
 key_scan();
 while (gpio_get(PTB7)==1)
 {
   key_scan();
//   LCD_P8x16Str(5,0,1);
//   LCD_P8x16Str(0,1,'S');
//   LCD_P8x16Str(0,2,'T');
   key_scan();
   if (key1 == 1 && lastkey1 == 0)
   {
     OLED_Refresh_Gram();
     while (gpio_get(PTB7)==1)
     {
       key_scan();

       if (key1 == 1 && lastkey1 == 0)
       {
         speedset=speedset+5;
       }

       if (key2==1&&lastkey2==0)
       {
         speedset=speedset-5;
       }
       LCD_Show_Number(10,2,speedset);
       lastkey1=key1;
       lastkey2=key2;
     }
     lastkey1=key1;
     lastkey2=key2;
     lastkey3=key3;
     lastkey4=key4;
   }

   // while (!key3)
   // {
   //   stage_flag=2;
   // }
   // while (!key4 && stage_flag==2)
   // {
   //   while (!key1)
   //   {
   //     speedtemp=speedtemp+10;
   //   }
   //   while (!key2)
   //   {
   //     speedtemp=speedtemp-10;
   //   }
   // }
 }
}