/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,Ұ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ������������������ҵ��;��
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






extern int sum;
int vl;
int vr;
extern uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];	
extern  void ccd1_deall(uint8*ccd1_array);
extern int R2_flag;
extern int L2_flag;
extern int lv;
extern int rv;
extern int zhongxian;
extern int zhongxian1,zhongxianpre1;

extern int error;
extern int DPWM;
extern int errorpre;

extern int speed_left,speed_right;
extern float OutData[4]; 
extern int qipao;
extern int ZA_flag;
extern int ZA;
int zhangaicount=0;
extern void OutPut_Data(void); 
extern void speed_set_left(int speed_set);
extern void speed_set_right(int speed_set);
void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void Datainit(void);
void DMA_CH1_Handler(void);    //DMA��һ�ֲ�����CPU��ֱ�Ӵ��ڴ��˴�ȡ���ݵ����ݽ���ģʽ
void DMA_CH2_Handler(void);
extern int wide;
extern int pd;
extern int podao;
extern int change1;
int qipaocount=0;


 void nihe()
 {
     
   ccd2_deall(CCD_BUFF[0]); 
  
zhongxian=zhongxian1;



}
  
void main()
{
   DisableInterrupts;//��ֹ���ж� 
  
   
   uart_init (UART4, 57600);
   LCD_Init();
   tsl1401_set_addrs(2,(uint8 *)&CCD_BUFF[0],(uint8 *)&CCD_BUFF[1],(uint8 *)&CCD_BUFF[2]);
   tsl1401_init(time);      //��ʼ�� ����CCD ������ �ж�ʱ��Ϊ time
   set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϸ�λ����Ϊ PIT0_IRQHandler
   enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�
   DMA_count_Init(DMA_CH1, PTA19, 0x7FFF, 0xA2u);
   DMA_count_Init(DMA_CH2, PTB18, 0x7FFF, 0xA2u);
   gpio_init (PTD0, GPI,0);      
   gpio_init (PTD3, GPI,0);  
        
    
   ftm_pwm_init(FTM0, FTM_CH1,10000, 0); 
   ftm_pwm_init(FTM0, FTM_CH2,10000, 0); 
   ftm_pwm_init(FTM0, FTM_CH3,10000, 0); 
   ftm_pwm_init(FTM0, FTM_CH4,10000, 0);  
   ftm_pwm_init(FTM1, FTM_CH0,50, 0); 
    
   EnableInterrupts;//�ж�����
   
   while(1)
     
   {

       tsl1401_get_img();

   
 
        nihe();
      //  RUNspeed();
        shuchu();

 
//vcan_sendccd((uint8 *)&CCD_BUFF[0],TSL1401_SIZE);
/* OutData[0] = a;
  OutData[1] = 10;
   OutData[2] = 20 ;
    OutData[3] = 40;
    OutPut_Data();*/
   }
}

void PIT0_IRQHandler()
{
     PIT_Flag_Clear(PIT0);  
     tsl1401_time_isr();

    

  sum++;
if (sum>500)
{vl=lv;
 vr=rv;
}


if (qipao==1)
{
qipaocount++;

}
if (qipaocount>100)
{


qipao=2;

}
if(ZA_flag==1)
{
  zhangaicount++;
  
}


if(zhangaicount>100)
{
  
  
  ZA_flag=0;
 
  qipao=3;


}

   
  




  speed_set_right(vl); 
   
 speed_set_left(vr);
  
}
 