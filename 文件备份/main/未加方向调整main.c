/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,Ұ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��Ұ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      Ұ��K60 ƽ̨������
 * @author     Ұ��Ƽ�
 * @version    v5.0
 * @date       2013-12-19
 */

#include "common.h"
#include "include.h"     


extern float Gyro_Now,g_fCarAngle;

extern float OutData[4];                              //SCIʾ��������
extern float Gyro_Now,angle_offset_vertical;          //������ת����Ľ��ٶȣ�ת����ļ��ٶȽǶ�
extern float g_fCarAngle,g_fGyroscopeAngleIntegral;   //�ںϺ�ĽǶ�
extern volatile int     MMA7361 ,ENC03,real_angle;    //���ٶȼ�AD ,������AD��ģ������ĽǶ�
extern int16 count_flag;
extern int16 temp_Left, temp_Right;
extern int16 nLeftVal, nRightVal;
extern uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];	
extern uint8 time; 

extern int8 count_1ms;
extern int8 nSpeedControlPeriod;
extern int8 nSpeedControlCount;
extern int8 nDirectionControlPeriod;
extern int8 nDirectionControlCount;



extern void PIT0_IRQHandler(void);
extern   void OutPut_Data(void);                              //SCI�ɲ���
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
 
void main() 
{
  
    DisableInterrupts;//��ֹ���ж�
    

    
    ftm_quad_init(FTM1);                                    //FTM1 ���������ʼ�������õĹܽſɲ� port_cfg.h ��  zuo
    ftm_quad_init(FTM2);                                    //FTM2 ���������ʼ�������õĹܽſɲ� port_cfg.h ��  you

    // ��control.h��������غ�
    adc_init (ZOUT);         //MMA7361 Z��
    adc_init (Gyro1);        // ENC03���ٶ�
    //adc_init (Ang);          //�Ƕ�

    //uart_init (UART3, 9600);
    //��ʼ�� PWM ���
    //FTM �Ĺܽ� ����  fire_port_cfg.h
    //�궨��FTM0_PRECISON   ��Ϊ  1000u
    //PWM��ֵ��ת��
    ftm_pwm_init(FTM0, FTM_CH4,10*1000,0);
    ftm_pwm_init(FTM0, FTM_CH5,10*1000,0);
    ftm_pwm_init(FTM0, FTM_CH6,10*1000,0);
    ftm_pwm_init(FTM0, FTM_CH3,10*1000,0);
        //��ʼ�� ����CCD
    tsl1401_set_addrs(TSL1401_MAX,(uint8 *)&CCD_BUFF[0],(uint8 *)&CCD_BUFF[1],(uint8 *)&CCD_BUFF[2]);
    tsl1401_init(time);                         //��ʼ�� ����CCD ������ �ж�ʱ��Ϊ time

    gpio_init(PTE0,GPO,0);
    //gpio_init(PTE2,GPO,0);
    DELAY_MS(500);
    /*
    //4 �� ��ת
    //6 �� ��ת
    //5 �� ��ת
    //3 �� ��ת
     */
    //led_init(LED0);                                         //��ʼ��LED0��PIT0�ж��õ�LED0
    
/*
    NVIC_SetPriorityGrouping(0);                            //�����ж����ȼ�����
    NVIC_SetPriority(PIT1_VECTORn, 0);                     
    NVIC_SetPriority(PIT0_VECTORn, 1);

    */
    
    pit_init_ms(PIT0, 1);                                //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϸ�λ����Ϊ PIT0_IRQHandler
    enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�
    /*
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //����PIT1���жϸ�λ����Ϊ PIT1_IRQHandler
    enable_irq (PIT1_IRQn);                                 //ʹ��PIT0�ж�
*/
    EnableInterrupts;//�ж����� n.;

    while(1)
   {
        //vcan_sendccd((uint8 *)&CCD_BUFF[0],TSL1401_SIZE);
        //vcan_sendccd((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],TSL1401_SIZE);     
        //OutData[0] = ENC03;
        //OutData[1] = MMA7361;//Gyro_Now;
        //OutData[2] = angle_offset_vertical ;
        //OutData[3] = g_fCarAngle;
        //OutPut_Data();

        
   }
}


/**********************�жϷ������*******************/
void PIT0_IRQHandler(void)
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
     Speed_Calculate(g_fCarAngle,Gyro_Now);      //�ٶȼ���
   }
   else if(count_1ms == 2)
   {
   nSpeedControlPeriod ++;
   nDirectionControlPeriod ++;
   
   SpeedControlOutput();                               //�������    
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
          CCD_Get();
          CarDircAdjust();
          nDirectionControlCount = 0;
          nDirectionControlPeriod = 0;
          }
     
       } 
     
    
    PIT_Flag_Clear(PIT0);                       //���жϱ�־λ
     
}     
     

	                                        //�¼���������
     
 /*    
     
gpio_set(PTE0,1);
    AD_Calculate();                             //AD
    Speed_Calculate(g_fCarAngle,Gyro_Now);      //�ٶȼ���
    count_flag++;

    SpeedGet();
    if(count_flag>=20)
    {

      //now_speed_L = temp_Left;
      //now_speed_R = temp_Right;


      SpeedControl();
      temp_Left = 0;
      temp_Right = 0;
      count_flag = 0;

      
    }
    
    SpeedControlOutput();
    CCD_Get();
    CarDircAdjust();
    MotorOutput();
    
gpio_turn (PTE0);
        
    PIT_Flag_Clear(PIT0);                       //���жϱ�־λ

}
}
     */
/*
void PIT1_IRQHandler()
{
    tsl1401_time_isr();
    PIT_Flag_Clear(PIT1);
}
*/