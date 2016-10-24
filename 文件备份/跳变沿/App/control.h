//#include "VCAN_TSL1401.h"
#ifndef _CONTROL_H
#define _CONTROL_H

#define XOUT    ADC0_SE8   // PTB0
#define YOUT    ADC0_SE9   // PTB1
#define ZOUT    ADC1_SE10  // PTB4

#define Gyro1   ADC1_SE11  // PTB5
#define Gyro2   ADC1_SE12  // PTB6
#define Ang     ADC1_SE13  // PTB7

/***********************直立控制参数********************/
#define MMA7361_vertical             2123           //2110   
#define GYRO_VAL                     2250           //2245   //陀螺仪中值        //加大向后，减少向前
#define Gyro_ratio                   0.55           //0.4    
#define GRAVITY_ADJUST_TIME_CONSTANT 2
#define DT                           0.005
#define MMA7361_ratio                0.3468      //0.305
#define P_ANGLE                55         //55       
#define D_ANGLE                0.6         //0.4         //0.6
#define P_SPEED                5         //6,55                 //4.5          
//#define I_SPEED                0.05         //0.05,1.5                 //0.25                 //0.7          //0.15         
 //反复调整以上5个参数，以及机械结构
 
#define Speed_Set      100   //300

#define MOTOR_DEAD_VAL_L   15     //15    //  死区电压
#define MOTOR_DEAD_VAL_R   10     //15

#define MOTOR_MAX_Z  600
#define MOTOR_MIN_Z  0

#define MOTOR_MAX_F  0
#define MOTOR_MIN_F  -600

#define MOTOR_MAX_I 600
#define MOTOR_MIN_I -600



#define SPEED_CONTROL_COUNT  20
#define DIRECTION_CONTROL_COUNT   2

/***********************方向控制参数********************/

#define DIRC_P    6//5   //6  //方向P值
#define BIN_MAX 0x80


//#define diff_threshold    ((maxdif> 12) ? ((maxdif*80)/100) :10)     // 差分阈值
//#define diff_threshold    10
//#define safe_isolation    3
#define CENTERADJ_1 80                     //与CCD高度有关
/***********************函数声明********************/
 extern   void Rd_Ad_Value(void);                              //AD采集
 extern   void AD_Calculate(void);                              //AD采集和计算
 extern   void Speed_Calculate(float angle,float angle_dot);   //速度计算
 static   unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
 extern   void OutPut_Data(void);                              //SCI采参数
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
extern uint8 ccd1_array[138]; //128个像素电压
extern uint8 ccd2_array[138]; //128个像素电压
extern uint8 breakpoint1;     //跳变阀值
extern uint8 leftbreak1;      //左侧跳变点
extern uint8 rightbreak1;     //右侧跳变点
extern uint8 leftlose1;       //左侧黑线丢失标志位
extern uint8 rightlose1;      //右侧黑线丢失标志位
extern uint8 breakpoint2;	   //跳变阀值
extern uint8 leftbreak2;      //左侧跳变点
extern uint8 rightbreak2;     //右侧跳变点
extern uint8 leftlose2;       //左侧黑线丢失标志位
extern uint8 rightlose2;      //右侧黑线丢失标志位
extern void ccd1_sample(void);        //ccd1采样
extern void ccd2_sample(void);        //ccd2采样
extern void ccd3_sample(void);
extern void ccd1_deal(void);          //ccd1像素处理
extern void ccd2_deal(void);          //ccd2像素处理
extern void ccd_init(void);
//void ccd_senddata(uint8*);        //ccd发送数据
//void ccd_sendhex(uint8);
//extern void ccd_init(void);
//extern void tsl1401_restet();
//extern void tsl1401_si_out(uint8 data);
//extern void tsl1401_clk_out(uint8 data);

#endif