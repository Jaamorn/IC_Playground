#include "include.h"

int speed_left,speed_right;

int error_speed_right[3]={0};
#define kp_speed_right 160//  120
#define ki_speed_right 80//8  
#define kd_speed_right 40  //10
int init_speed_right=0;

void speed_set_right(int speed_set)
{
    int adjust_speed_right;
    
    speed_right = DMA_count_get(DMA_CH1);   //返回累加计数值
 
    if(gpio_get(PTD0)==1)speed_right=-speed_right;
    DMA_count_reset(DMA_CH1);
    // LCD_Show_Number(6,5,speed_right);
    error_speed_right[2]=error_speed_right[1];
    error_speed_right[1]=error_speed_right[0];
    error_speed_right[0]=-speed_set-speed_right;
    
if(    error_speed_right[0]>30)
{
   init_speed_right=9500;
}


 if(    error_speed_right[0]<-30)
   
   
{
  init_speed_right=-9500;
}



if  (  error_speed_right[0]<=30&&error_speed_right[0]>=-30)
{
    adjust_speed_right=kp_speed_right*(error_speed_right[0]-error_speed_right[1])+(int)(ki_speed_right*error_speed_right[0])+kd_speed_right*(error_speed_right[0]-2*error_speed_right[1]+error_speed_right[2]);
    init_speed_right=init_speed_right+adjust_speed_right;
}
    if(init_speed_right>9500)                      //limit the max PWM for motor                     
    {
      init_speed_right=9500;
    }
    else if(init_speed_right<-9500)                    //limit the min PWM for motor 
    {
      init_speed_right=-9500;
    }
    
    
    if(init_speed_right<0)
    {
     //  ftm_pwm_duty(FTM0, FTM_CH1,-init_speed_right);
      //ftm_pwm_duty(FTM0, FTM_CH2 ,0);
      ftm_pwm_duty(FTM0, FTM_CH3,-init_speed_right);
      ftm_pwm_duty(FTM0, FTM_CH4 ,0 );
      
    }
    else
    {
      ftm_pwm_duty(FTM0, FTM_CH3, 0);
     ftm_pwm_duty(FTM0, FTM_CH4, init_speed_right);//init_speed_right
       // ftm_pwm_duty(FTM0, FTM_CH1, 0);
     // ftm_pwm_duty(FTM0, FTM_CH2, init_speed_right);
    }    
}
  int error_speed_left[3]={0};
#define kp_speed_left 160
#define ki_speed_left 80    //这些值从哪里来？自己测
#define kd_speed_left 40
int init_speed_left=0;

void speed_set_left(int speed_set)
{
    int adjust_speed_left;
    
    speed_left = DMA_count_get(DMA_CH2);     //计数寄存器   DMA累加计数
    

    if(gpio_get(PTD3)==1)speed_left=-speed_left;   //什么意思   高电平意味着什么？？？
    DMA_count_reset(DMA_CH2);
   // LCD_Show_Number(6,6,speed_left);
    error_speed_left[2]=error_speed_left[1];
    error_speed_left[1]=error_speed_left[0];
    error_speed_left[0]=speed_set-speed_left;                                //原先是float  (int)(ki_speed_left*error_speed_left[0])
    
    
    if(error_speed_left[0]>30)
    { 
      init_speed_left=9500;
    }
   if(error_speed_left[0]<-30)
    {
      init_speed_left=-9500;
    }
    if(error_speed_left[0]>=-30&&error_speed_left[0]<=30)
    { 
      adjust_speed_left=kp_speed_left*(error_speed_left[0]-error_speed_left[1])+(int)(ki_speed_left*error_speed_left[0])+kd_speed_left*(error_speed_left[0]-2*error_speed_left[1]+error_speed_left[2]);
      init_speed_left=init_speed_left+adjust_speed_left;
    }
    if(init_speed_left>9500)        //保护程序              //limit the max PWM for motor                     
    {
      init_speed_left=9500;
    }
    else if(init_speed_left<-9500)                    //limit the min PWM for motor 
    {
      init_speed_left=-9500;
    }
    
    
    if(init_speed_left<0)
    {
      
     


   //  ftm_pwm_duty(FTM0, FTM_CH4, 0);
  //   ftm_pwm_duty(FTM0, FTM_CH3, -init_speed_left);
     ftm_pwm_duty(FTM0, FTM_CH1, 0);
     ftm_pwm_duty(FTM0, FTM_CH2, -init_speed_left);
    
    
    }
    else
    {

   //  ftm_pwm_duty(FTM0, FTM_CH3, 0);
    // ftm_pwm_duty(FTM0, FTM_CH4, init_speed_left);
     ftm_pwm_duty(FTM0, FTM_CH1, init_speed_left);
     ftm_pwm_duty(FTM0, FTM_CH2, 0);

    }    


    
}


