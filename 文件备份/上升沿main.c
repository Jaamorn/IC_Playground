/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,野火科技
 *     All rights reserved.
 *     技术讨论：野火初学论坛 http://www.chuxue123.com
 *
 *     除注明出处外，以下所有内容版权均属野火科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留野火科技的版权声明。
 *
 * @file       main.c
 * @brief      野火K60 平台主程序
 * @author     野火科技
 * @version    v5.0
 * @date       2013-12-19
 */

#include "common.h"
#include "include.h"     


extern float Gyro_Now,g_fCarAngle;

extern float OutData[4];                              //SCI示波器参数
extern float Gyro_Now,angle_offset_vertical;          //陀螺仪转化后的角速度，转化后的加速度角度
extern float g_fCarAngle,g_fGyroscopeAngleIntegral;   //融合后的角度
extern volatile int     MMA7361 ,ENC03,real_angle;    //加速度计AD ,陀螺仪AD，模块输出的角度
extern int16 count_flag;
extern int16 temp_Left, temp_Right;
extern int16 nLeftVal, nRightVal;
extern uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];	
//extern uint8 time; 

extern int8 count_1ms;
extern int8 nSpeedControlPeriod;
extern int8 nSpeedControlCount;
extern int8 nDirectionControlPeriod;
extern int8 nDirectionControlCount;

extern uint8 left_side;
extern uint8 right_side;
extern int8 dismid;
extern uint8 shizi;

void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);

extern   void OutPut_Data(void);                              //SCI采参数
 extern   void SpeedGet(void);
 extern   void SpeedControl(void);
 extern   void SpeedControlOutput(void);
 extern   void MotorOutput(void);
 extern   void SetMotorVoltage(void);
 // void PIT1_IRQHandler(void);
 
extern   void CarDircAdjust(void);
extern   void CCD_Get(void);
extern   void vcan_sendccd(uint8 *ccdaddr, uint32 ccdsize);
extern   void DirectionControlOutput(void);
extern   void distinguish(void);

void main() 
{
  
    DisableInterrupts;//禁止总中断
    

    
    ftm_quad_init(FTM1);                                    //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）  zuo
    ftm_quad_init(FTM2);                                    //FTM2 正交解码初始化（所用的管脚可查 port_cfg.h ）  you

    // 在control.h定义了相关宏
    adc_init (ZOUT);         //MMA7361 Z轴
    adc_init (Gyro1);        // ENC03角速度
    //adc_init (Ang);          //角度

    uart_init (UART3, 9600);
    //初始化 PWM 输出
    //FTM 的管脚 可在  fire_port_cfg.h
    //宏定义FTM0_PRECISON   改为  1000u
    //PWM数值反转。
    ftm_pwm_init(FTM0, FTM_CH4,10*1000,0);
    ftm_pwm_init(FTM0, FTM_CH5,10*1000,0);
    ftm_pwm_init(FTM0, FTM_CH6,10*1000,0);
    ftm_pwm_init(FTM0, FTM_CH3,10*1000,0);
        //初始化 线性CCD
    tsl1401_set_addrs(TSL1401_MAX,(uint8 *)&CCD_BUFF[0],(uint8 *)&CCD_BUFF[1],(uint8 *)&CCD_BUFF[2]);
    tsl1401_init(time);                         //初始化 线性CCD ，配置 中断时间为 time

    gpio_init(PTE0,GPO,0);
    //LCD_Init();                                 //OLED初始化
    //gpio_init(PTE2,GPO,0);
    DELAY_MS(500);
    /*
    //4 左 反转
    //6 左 正转
    //5 右 正转
    //3 右 反转
     */
    //led_init(LED0);                                         //初始化LED0，PIT0中断用到LED0
    
/*
    NVIC_SetPriorityGrouping(0);                            //配置中断优先级分组
    NVIC_SetPriority(PIT1_VECTORn, 0);                     
    NVIC_SetPriority(PIT0_VECTORn, 1);

    */
    
    //pit_init_ms(PIT0, 1);                                
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断复位函数为 PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断
    
    
    pit_init_ms(PIT1, 1);                                //初始化PIT1，定时时间为： 1ms
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT1的中断复位函数为 PIT1_IRQHandler
    enable_irq (PIT1_IRQn);                                 //使能PIT1中断

    EnableInterrupts;//中断允许 n.;

    while(1)
   {
        gpio_turn (PTE0);
        CCD_Get();
        //distinguish();
        
        //gpio_turn (PTE0);
        /*
        Display(left_side,10,0);
        Display(right_side,60,0);
        Display(dismid,10,2);
        //LCD_P6x8Str(10,4,"shizi");
        Display(shizi,60,4);
        */
        
        
     
        //vcan_sendccd((uint8 *)&CCD_BUFF[0],TSL1401_SIZE);
        //vcan_sendccd((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],TSL1401_SIZE);     
        //OutData[0] = ENC03;
        //OutData[1] = MMA7361;//Gyro_Now;
        //OutData[2] = angle_offset_vertical ;
        //OutData[3] = g_fCarAngle;
        //OutPut_Data();

        
   }
}


/**********************中断服务程序*******************/
void PIT1_IRQHandler(void)
{
   count_1ms ++;
   
   if(count_1ms >= 5)
   {
      SpeedGet();
      count_1ms = 0;
   }
   else if(count_1ms == 1)
   {
         //CCD_Get();                             //CCD
     AD_Calculate();
     Speed_Calculate(g_fCarAngle,Gyro_Now);      //速度计算
   }
   else if(count_1ms == 2)
   {
   nSpeedControlPeriod ++;
   nDirectionControlPeriod ++;
   
   SpeedControlOutput();                               //缓慢输出
   DirectionControlOutput();
   MotorOutput();
   }
   else if(count_1ms == 3)
   {
     nSpeedControlCount ++;
     if(nSpeedControlCount >= SPEED_CONTROL_COUNT)
     {
//gpio_turn (PTE0);
       SpeedControl();
       nSpeedControlCount = 0;
       nSpeedControlPeriod = 0;
       temp_Left = 0;
       temp_Right = 0;

     }
   }

    else if(count_1ms == 4)
     {
        nDirectionControlCount ++;
        if(nDirectionControlCount >= DIRECTION_CONTROL_COUNT)
        { 
          //gpio_turn (PTE0);
          //CCD_Get();
          CarDircAdjust();
          nDirectionControlCount = 0;
          nDirectionControlPeriod = 0;
          }
     
       }
     
    
    PIT_Flag_Clear(PIT1);                       //清中断标志位
     
}     
     

	                                        //事件计数清零
     


void PIT0_IRQHandler()
{
    tsl1401_time_isr();
    PIT_Flag_Clear(PIT0);
}
