//#include "VCAN_TSL1401.h"
#ifndef _CONTROL_H
#define _CONTROL_H

#define XOUT    ADC0_SE8   // PTB0
#define YOUT    ADC0_SE9   // PTB1
#define ZOUT    ADC1_SE10  // PTB4

#define Gyro1   ADC1_SE11  // PTB5
#define Gyro2   ADC1_SE12  // PTB6
#define Ang     ADC1_SE13  // PTB7

/***********************ֱ�����Ʋ���********************/
#define MMA7361_vertical             2123           //2110   
#define GYRO_VAL                     2250           //2245   //��������ֵ        //�Ӵ���󣬼�����ǰ
#define Gyro_ratio                   0.55           //0.4    
#define GRAVITY_ADJUST_TIME_CONSTANT 2
#define DT                           0.005
#define MMA7361_ratio                0.3468      //0.305
#define P_ANGLE                55         //55       
#define D_ANGLE                0.6         //0.4         //0.6
#define P_SPEED                5         //6,55                 //4.5          
//#define I_SPEED                0.05         //0.05,1.5                 //0.25                 //0.7          //0.15         
 //������������5���������Լ���е�ṹ
 
#define Speed_Set      100   //300

#define MOTOR_DEAD_VAL_L   15     //15    //  ������ѹ
#define MOTOR_DEAD_VAL_R   10     //15

#define MOTOR_MAX_Z  600
#define MOTOR_MIN_Z  0

#define MOTOR_MAX_F  0
#define MOTOR_MIN_F  -600

#define MOTOR_MAX_I 600
#define MOTOR_MIN_I -600



#define SPEED_CONTROL_COUNT  20
#define DIRECTION_CONTROL_COUNT   2

/***********************������Ʋ���********************/

#define DIRC_P    6//5   //6  //����Pֵ
#define BIN_MAX 0x80


//#define diff_threshold    ((maxdif> 12) ? ((maxdif*80)/100) :10)     // �����ֵ
//#define diff_threshold    10
//#define safe_isolation    3
#define CENTERADJ_1 80                     //��CCD�߶��й�
/***********************��������********************/
 extern   void Rd_Ad_Value(void);                              //AD�ɼ�
 extern   void AD_Calculate(void);                              //AD�ɼ��ͼ���
 extern   void Speed_Calculate(float angle,float angle_dot);   //�ٶȼ���
 static   unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
 extern   void OutPut_Data(void);                              //SCI�ɲ���
 extern   void SEND(float a,float b,float c,float d);
 extern   void SpeedGet(void);
 extern   void SpeedControl(void);
 extern   void SpeedControlOutput(void);
 extern   void MotorOutput(void);
 extern   void SetMotorVoltage(void);
  extern   void vcan_sendccd(uint8 *ccdaddr, uint32 ccdsize);
 
extern   void CarDircAdjust(void);

extern   void CCD_Get(void);

extern   void PIT1_IRQHandler(void);
 
extern   void DirectionControlOutput(void);
extern uint8 ccd1_array[138]; //128�����ص�ѹ
extern uint8 ccd2_array[138]; //128�����ص�ѹ
extern uint8 breakpoint1;     //���䷧ֵ
extern uint8 leftbreak1;      //��������
extern uint8 rightbreak1;     //�Ҳ������
extern uint8 leftlose1;       //�����߶�ʧ��־λ
extern uint8 rightlose1;      //�Ҳ���߶�ʧ��־λ
extern uint8 breakpoint2;	   //���䷧ֵ
extern uint8 leftbreak2;      //��������
extern uint8 rightbreak2;     //�Ҳ������
extern uint8 leftlose2;       //�����߶�ʧ��־λ
extern uint8 rightlose2;      //�Ҳ���߶�ʧ��־λ
extern void ccd1_sample(void);        //ccd1����
extern void ccd2_sample(void);        //ccd2����
extern void ccd3_sample(void);
extern void ccd1_deal(void);          //ccd1���ش���
extern void ccd2_deal(void);          //ccd2���ش���
extern void ccd_init(void);
//void ccd_senddata(uint8*);        //ccd��������
//void ccd_sendhex(uint8);
//extern void ccd_init(void);
//extern void tsl1401_restet();
//extern void tsl1401_si_out(uint8 data);
//extern void tsl1401_clk_out(uint8 data);

#endif