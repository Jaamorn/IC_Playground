#include "include.h"

float OutData[4] = { 0 };                                    //SCI示波器参数
float Gyro_Now,angle_offset_vertical;  //陀螺仪转化后的角速度，转化后的加速度角度
float g_fCarAngle,g_fGyroscopeAngleIntegral; //融合后的角度
float Speed_Old = 0, Speed_New = 0, Speed_Keep = 0;
//int8 count_flag = 0;
int16 nSpeedControlOut = 0, nAngleControlOut = 0, nDirControlOut = 0;
float nSpeedControlOut1 = 0,De = 0;         //检测是否分20份
int16 temp_Left = 0, temp_Right = 0;
float Speed_L = 0,Speed_R = 0,speed_Start = 0,Speed_L_Last = 0,Speed_R_Last = 0;  //左右轮速度 ，最终速度

volatile int16     MMA7361 ,ENC03,real_angle;                             //加速度计AD ,陀螺仪AD，模块输出的角度
int16 nLeftVal = 0, nRightVal = 0;
int8 SpeedStepCount = 0;
float WantSpeed = 0.0;
uint8 Iflag = 0;


float I_SPEED = 0.0;
//uint8 zhijiao = 0;


int16 nDirControlOutNew = 0,nDirControlOutOld = 0;
uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];							//定义存储接收CCD图像的数组
uint8 prebin[TSL1401_SIZE];//,prebin2[TSL1401_SIZE];
uint8 predismid[3] = {0};
uint8 lossline_Left = 0,lossline_Right = 0;
uint8 shizi = 0;
uint8 shizicount = 3;


uint8 left_side;
uint8 right_side;
int8 dismid = 0;
//uint8 time = 6;                             //设置曝光时间 


int8 count_1ms = 0;
int8 nSpeedControlPeriod = 0;
int8 nSpeedControlCount = 0;
//int8 SpeedStartCount = 0;

int8 nDirectionControlPeriod = 0;
int8 nDirectionControlCount = 0;

float nP = 0, nI = 0;
float nSpeed = 0, nSpeedChange = 0;
uint8 repairline=0;

  uint8  max[TSL1401_SIZE];
  uint8  avg[TSL1401_SIZE];
  
  
//**************************************************************************
/*
*  功能说明：AD采集
*  参数说明： 无
*  函数返回：无符号结果值
*  修改时间：2013-2-10
*/
//**************************************************************************
void Rd_Ad_Value(void)
{

    MMA7361 = adc_once(ZOUT, ADC_12bit);   //Z
    ENC03= adc_once(Gyro1,ADC_12bit);    // gyro1

    //由于使用软件滤波，因此不再用 硬件融合角度
    // real_angle = adc_once(Ang,ADC_12bit); //ang

#if 0
    OutData[0] = MMA7361;
    OutData[1] = ENC03;
    //OutData[2] = gyro2 ;
    //OutData[3] = real_angle;
    OutPut_Data();
#endif

}

//**************************************************************************
/*
*  功能说明：SCI示波器CRC校验
内部调用函数
*  参数说明： 无
*  函数返回：无符号结果值
*  修改时间：2013-2-10
*/
//**************************************************************************
static unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

//************************************************
//
/*
*  功能说明：SCI示波器发送函数

*  参数说明：
OutData[]  需要发送的数值赋予该数组
*  函数返回：无符号结果值
*  修改时间：2013-2-10
*/
//****************************************************
void OutPut_Data(void)
{
    int temp[4] = {0};
    unsigned int temp1[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;
    for(i=0;i<4;i++)
    {

        temp[i]  = (int)OutData[i];
        temp1[i] = (unsigned int)temp[i];

    }

    for(i=0;i<4;i++)
    {
        databuf[i*2]   = (unsigned char)(temp1[i]%256);
        databuf[i*2+1] = (unsigned char)(temp1[i]/256);
    }

    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16%256;
    databuf[9] = CRC16/256;

    for(i=0;i<10;i++)
    {
        uart_putchar (UART3,(char)databuf[i]);
    }
}

//**************************************************************************
//   Kalman滤波
//**************************************************************************

float angle, angle_dot;         //外部需要引用的变量
//-------------------------------------------------------
// 0.00015     //0.0001
const float Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.005;
//0.0001         //0.00015        //1.2
//注意：dt的取值为kalman滤波器采样时间;         //0.8
static float P[2][2] = {
    { 1, 0 },
    { 0, 1 }
};

static float Pdot[4] ={0,0,0,0};

static const char C_0 = 1;

static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m)          //gyro_m:gyro_measure
{
    angle+=(gyro_m-q_bias) * dt;
    
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]=- P[1][1];
    Pdot[2]=- P[1][1];
    Pdot[3]=Q_gyro;
    
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    
    
    angle_err = angle_m - angle;
    
    

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    
    
    angle   += K_0 * angle_err;
    q_bias  += K_1 * angle_err;
    angle_dot = gyro_m-q_bias;
}
//**************************************************************************
//   清华角度滤波方案
//*************************************************************************
/*
*  功能说明：清华角度滤波
*  参数说明：G_angle                       加速度计角度0-90内
*            Gyro                         陀螺仪角速度转花后的数值
*            GRAVITY_ADJUST_TIME_CONSTANT  时间校正系数
DT                             定时器时间 单位s
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：参考清华源码
*/
//
//*************************************************************************
void QingHua_AngleCalaulate(float G_angle,float Gyro)
{
    float fDeltaValue;

    g_fCarAngle = g_fGyroscopeAngleIntegral;   //最终融合角度
    fDeltaValue = (G_angle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;  //时间系数矫正
    g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;                //融合角度
}

//**************************************************************************
/*
*  功能说明：直立角度计算
*  参数说明：

*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：参考清华源码
*/
//**************************************************************************
void AD_Calculate(void)
{
    

    Rd_Ad_Value();                          //采集 AD
    
    Gyro_Now = (GYRO_VAL - ENC03)* Gyro_ratio;                            //陀螺仪采集到的角速度归一化
    angle_offset_vertical = (MMA7361_vertical - MMA7361) * MMA7361_ratio;  //将加速度计采集到的角度归一化，乘上0.375是为了归一化到0~90°
    if(angle_offset_vertical > 90)angle_offset_vertical = 90;               //防止加速度角度溢出
    if(angle_offset_vertical < -90)angle_offset_vertical = -90;

    //计算融合后的角度
    QingHua_AngleCalaulate(angle_offset_vertical,Gyro_Now);                 //清华角度滤波方案
    

    /*****************************串口看波形（选择使用）****************************/
#if 0                           //宏条件编译 选择是否使用 虚拟示波器
    OutData[0] = ENC03;
    OutData[1] = MMA7361;//Gyro_Now;
    OutData[2] = angle_offset_vertical ;
    OutData[3] = g_fCarAngle;
    OutPut_Data();
#elif  0
    OutData[0] = angle_dot;
    OutData[1] = Gyro_Now;
    OutData[2] = angle_offset_vertical ;
    OutData[3] = angle;
    OutPut_Data();
#endif	
}

/***********************************************************************/
/*
*  功能说明：直立速度计算
*  参数说明：angle                融合后最终角度
*            angle_dot            陀螺仪角速度
*
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：参考清华源码
*/
/******************************************************************************/
void Speed_Calculate(float angle,float angle_dot)
{
    /***********************************速度计算************************************/
    speed_Start = angle * P_ANGLE  + angle_dot * D_ANGLE ;  //直立时所要的速度

    //P_ANGLE  P_GYRO  宏定义 直立所需要的PD参数

    Speed_L = speed_Start;//左轮总速度
    Speed_R = speed_Start;//右轮总速度
    /***********************将最大速度限制在985个PWM内******************************/
    if(Speed_L > MOTOR_MAX_Z)  Speed_L = MOTOR_MAX_Z;
    if(Speed_L < MOTOR_MIN_F) Speed_L = MOTOR_MIN_F;
    if(Speed_R > MOTOR_MAX_Z)  Speed_R = MOTOR_MAX_Z;
    if(Speed_R < MOTOR_MIN_F) Speed_R = MOTOR_MIN_F;
    
    nAngleControlOut = Speed_L = Speed_R;
    
}
    /***************因为驱动部分加了反相器，所以需对速度进行一个最终的处理***************
        Speed_L_Last = Speed_L;  //正相
    else
        Speed_L_Last = - Speed_L;

    if(Speed_R > 0)     
        Speed_R_Last = Speed_R;  //正相
    else
        Speed_R_Last = - Speed_R;

   **********用所得到的对应角度的速度进行PWM控制********************/
    /*
    if(Speed_L >= 0)    //angle大于0，向前，小于0，向后
    {
        ftm_pwm_duty(FTM0,FTM_CH4,0);
        ftm_pwm_duty(FTM0,FTM_CH6,(uint32)(Speed_L - MOTOR_DEAD_VAL_L));    //加入死区电压
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH6,0);
        ftm_pwm_duty(FTM0,FTM_CH4,(uint32)(-Speed_L - MOTOR_DEAD_VAL_L));    //加入死区电压
    }

    if(Speed_R >= 0)    //angle大于0，向前，小于0，向后
    {
        ftm_pwm_duty(FTM0,FTM_CH3,0);
        ftm_pwm_duty(FTM0,FTM_CH5,(uint32)(Speed_R - MOTOR_DEAD_VAL_R));    //加入死区电压
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH5,0);
        ftm_pwm_duty(FTM0,FTM_CH3,(uint32)(-Speed_R - MOTOR_DEAD_VAL_R));   //加入死区电压
    }
}
*/
/***********************************************************************/
/*
*  功能说明：速度控制
*  参数说明：
*
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：
*/
/******************************************************************************/

void SpeedControl(void) 
{
     float nLeftSpeed = 0, nRightSpeed = 0;
     //int16 nP = 0, nI = 0;
     //int16 nSpeed = 0, nSpeedChange = 0;
     
     nLeftSpeed = temp_Left;
     nRightSpeed = temp_Right;
     nSpeed = (nLeftSpeed + nRightSpeed)/2;
     /*
     if (SpeedStartCount<=30)
     {
       SpeedStartCount++;
       nSpeedChange = - nSpeed;
     }
     else 
     {
     */
        /*                   
      if(Speed_Set-WantSpeed<1)      //缓慢起步   
      {
      WantSpeed = Speed_Set;
      }
      else 
      {
      WantSpeed+=1; 
      }
     */
      
       if(SpeedStepCount<18)                               //18
      {
      SpeedStepCount++;
      }
      
      nSpeedChange =( Speed_Set/10.0+Speed_Set*SpeedStepCount/20.0) - nSpeed ;                //10,20
      //nSpeedChange = nSpeed - ( Speed_Set/5.0+Speed_Set*SpeedStepCount/10.0);
     
     //nSpeedChange = Speed_Set - nSpeed;
     //}
      
     if (Iflag == 0 && nSpeed>=0.9*Speed_Set)                                   //积分隔离
     {
       Iflag = 1;
       I_SPEED = 0.005;
     }

      
     nP = nSpeedChange * P_SPEED;
     nI = nSpeedChange * I_SPEED;
     

     
     
if(temp_Left == 0 && temp_Right == 0)
{
  nP = 0;
}
     
     Speed_Keep += nI;
     Speed_Old = Speed_New;
     Speed_New = nP + Speed_Keep;
     
     //Speed_Keep -= nI;
     //Speed_New = Speed_Keep - nP;
     
    
     if(Speed_Keep > MOTOR_MAX_I)
       Speed_Keep = MOTOR_MAX_I;
     if(Speed_Keep < MOTOR_MIN_I)
       Speed_Keep = MOTOR_MIN_I;
 
     
}
void SpeedControlOutput(void)       //将速度均分20份
{
     
     
     nSpeedControlOut1 = Speed_New - Speed_Old;
     nSpeedControlOut = nSpeedControlOut1*nSpeedControlPeriod/20 + Speed_Old;
     De = nSpeedControlOut1*nSpeedControlPeriod/20;
     //Cal_Left_Speed = Cal_Right_Speed = nSpeedControlOut;
}


/***********************************************************************/
/*
*  功能说明：正交解码
*  参数说明：
*
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：
*/
/******************************************************************************/


void SpeedGet(void)
{
    int16 temp_L = 0, temp_R = 0;
    
    temp_Left = ftm_quad_get(FTM1);          //获取FTM 正交解码 的脉冲数(负数表示反方向)zuo
    temp_Left = -temp_Left;
    temp_Left+= temp_L;
      
    ftm_quad_clean(FTM1);
/*
    if(temp_Left<=0)
    {
        printf("\n左正转：%d",-temp_Left);
    }
    else
    {
        printf("\n左反转：%d",temp_Left);
    }
*/
    
    temp_R = ftm_quad_get(FTM2);          //获取FTM 正交解码 的脉冲数(负数表示反方向)you
    temp_Right+= temp_R;
    ftm_quad_clean(FTM2);
    //count_flag++;
/*
    if(temp_Right>=0)
    {
        printf("\n右正转：%d",temp_Right);
    }
    else
    {
        printf("\n右反转：%d",-temp_Right);
    }
*/
}

/***********************************************************************/
/*
*  功能说明：电机输出
*  参数说明：
*
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：
*/
/******************************************************************************/



void MotorOutput(void) 
{
 //int16 nLeft = 0, nRight = 0;
 //int16 nLeftVal = 0, nRightVal = 0;
 
 nLeftVal = nAngleControlOut - nSpeedControlOut - nDirControlOut;
 nRightVal = nAngleControlOut - nSpeedControlOut + nDirControlOut;
 
 //nLeftMotorOut = nLeft;
 //nRightMotorOut = nRight;
/*
 MotorSpeedOut();
}

void MotorSpeedOut(void)
 { 
 */
 

//nLeftVal = nLeft;
//nRightVal = nRight;

 if(nLeftVal > 0)
 {
 nLeftVal += MOTOR_DEAD_VAL_L;
 
 if(nLeftVal > MOTOR_MAX_Z) nLeftVal = MOTOR_MAX_Z;     
 if(nLeftVal < MOTOR_MIN_Z) nLeftVal = MOTOR_MIN_Z;     
 
 }
else if(nLeftVal <= 0)
{
nLeftVal -= MOTOR_DEAD_VAL_L;

if( nLeftVal < MOTOR_MIN_F) nLeftVal = MOTOR_MIN_F;    
if( nLeftVal > MOTOR_MAX_F) nLeftVal = MOTOR_MAX_F;    

}
 if(nRightVal > 0)
 {
   nRightVal += MOTOR_DEAD_VAL_R;
   
 if(nRightVal > MOTOR_MAX_Z) nRightVal = MOTOR_MAX_Z;     
 if(nRightVal < MOTOR_MIN_Z) nRightVal = MOTOR_MIN_Z;     
 }
 else if(nRightVal <= 0)
 {
   nRightVal -= MOTOR_DEAD_VAL_R; 
   
if( nRightVal < MOTOR_MIN_F) nRightVal = MOTOR_MIN_F;     
if( nRightVal > MOTOR_MAX_F) nRightVal = MOTOR_MAX_F;     

 }
 
 

SetMotorVoltage();
} 


void SetMotorVoltage(void)
{

    
  if(nLeftVal >= 0)   
    {
        ftm_pwm_duty(FTM0,FTM_CH4,0);
        ftm_pwm_duty(FTM0,FTM_CH6,nLeftVal);    
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH6,0);
        ftm_pwm_duty(FTM0,FTM_CH4,-nLeftVal);    
    }

    if(nRightVal >= 0)    
    {
        ftm_pwm_duty(FTM0,FTM_CH3,0);
        ftm_pwm_duty(FTM0,FTM_CH5,nRightVal);    
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH5,0);
        ftm_pwm_duty(FTM0,FTM_CH3,-nRightVal);   
    }
}




/***********************************************************************/
/*
*  功能说明：方向
*  参数说明：
*
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：
*/
/******************************************************************************/



void maxvar(uint8 *buf,uint16 len,uint8  maxval)
{
    while(len--)
    {
        if(buf[len] > maxval)
        {
            buf[len]= maxval;
        }
    }

}


/*!
 *  @brief      计算差分绝对值
 *  @since      v5.0
 *  @note       山外差分法补充说明：差分最大值maxval 和 差分平均值avgval 这两个
                参数是为了便于定义确定阈值而加入的，可删除。差分平均值，一般返回结果
                都非常小，因此顶层用不上，建议删掉（此处保留是为了给大家验证）
 */
void abs_diff(uint8 *dst,uint8 *src,uint16 len,uint8 * maxval,uint8 * avgval)
{
    int8 tmp;
    uint8 max = 0;
    uint32 sum = 0;
    uint16 lentmp = len;
    while(--lentmp)                 //仅循环 len-1 次
    {
        tmp = *(src+1)- *src;
        tmp = ABS(tmp) ;
        if(tmp > max )
        {
             max = tmp;
        }

        sum += tmp;
        *dst = tmp;
        src++;
        dst++;
    }
    *dst = 0;               // 最后一个 点配置为 0
    *maxval = max;           // 返回最大绝对差值
    *avgval = (uint8)(sum/(len-1));  //前 len -1 个数的平均值
    
}

/*!
 *  @brief      简单的一个二值化 算法（不稳定,仅测试）
 *  @since      v5.0
 */
// diff_threshold 差分阈值 ,不同的角度，不同的环境而有所不同
//可根据 maxdif 最大差分值来配置，或者直接固定阈值
//#define diff_threshold    ((maxdif> 12) ? ((maxdif*80)/100) :10)     // 差分阈值
//#define diff_threshold    10
//#define safe_isolation    3
void bin(uint8 *bin,uint8 * img,uint8 * difimg,uint16 len,uint8 maxdif)
{
    uint16 tmplen = len;
    uint8  thldnum = 0;        //阈值次数
    uint8  thresholdimg;
    uint8  tmpnum;
    
  //uint8 len_ahead_n = 5,len_ahead_1 = 5;
  //uint8 len_hinder_n = 123,len_hinder_1 = 123;
  //uint8 lossflag1_Left = 0,lossflag2_Left = 0;
  //uint8 lossflag1_Right = 0,lossflag2_Right = 0;

    memset(bin,0xFF,len);  //全部当作

    while(tmplen--)
    {
        if((tmplen == 0)|| (tmplen > len))
        {
            return;
        }

        if(difimg[tmplen] > diff_threshold)                  //找到 差分阈值
        {
            thldnum++;

            //寻找最大差分阈值
            while(tmplen--)
            {
                if((tmplen == 0)|| (tmplen > len))
                {
                    return;
                }

                if(difimg[tmplen] < difimg[tmplen+1] )    //tmplen+1 为最大阈值
                {
                     break;
                }
            }

            //tmplen + 1 是 差分最大值 ，切换到 颜色扫描
            if((img[tmplen] <= img[tmplen+1]) ||(img[tmplen+1] <= img[tmplen+2]) )  // 前面 黑色 ，后面 白色
            {
                //选择 差分值最大值的前一个 作为 阈值
                thresholdimg = (img[tmplen+1] + img[tmplen+2])/2;

                //扫描下一个 高于 此阈值 (比此点更白)
                while(img[tmplen] <= thresholdimg)
                {
                    bin[tmplen] = 0;                //黑色
                    tmplen--;
                    if(tmplen == 0)      //结尾了 ,直接退出
                    {
                        if(img[tmplen] <= thresholdimg)
                        {
                             bin[tmplen] = 0;                //黑色
                        }
                        return ;
                    }
                    else if (tmplen > len)
                    {
                         return;
                    }
                }
                tmplen -= safe_isolation;
                if((tmplen == 0)|| (tmplen > len))
                {
                    return;
                }

                //等待差分值降低
                while(difimg[tmplen] > diff_threshold)
                {
                    tmplen--;
                    if((tmplen == 0)|| (tmplen > len))
                    {
                        return;
                    }
                }
            }
            else
            {
                //前白 后 黑
                if(thldnum == 1)
                {
                    //后面的 内容都是 黑色的
                    tmpnum  = tmplen + 1;
                    while(tmpnum < len)
                    {
                        bin[tmpnum] = 0;                //黑色
                        tmpnum ++;
                    }
                }
            }


        }
    }
    

    
}



/*************************补线*******************/


void distinguish(void)
{
  uint8 len_ahead_n = 3;//,len_ahead_1 = 3;
  uint8 len_hinder_n = 125;//,len_hinder_1 = 125;
  uint8 lossflag1_Left = 0;//,lossflag2_Left = 0;
  uint8 lossflag1_Right = 0;//,lossflag2_Right = 0;
  //uint8 i = 0;
  //uint8 blacknum1 = 0,blacknum2 = 0;
  //uint//8 mayzhijiao = 0;

  

  
  while (CCD_BUFF[2*TSL1401_MAX+0][len_ahead_n] != 0 && len_ahead_n<=64)
  {
    
    len_ahead_n++;
    if (len_ahead_n >= 60)
    {
      lossflag1_Left = 1;
      break;
    }
    
    
  }
  
  /*

  while (prebin[len_ahead_1] != 0 && len_ahead_1<=64)
  {
    
    len_ahead_1++;
    if (len_ahead_1 >= 60)
    {
      lossflag2_Left = 1;
      break;
    }
    
  }
  */
  //if (lossflag1_Left == 0 && lossflag2_Left == 0)
  //shizi = 1;
  //if (lossflag1_Left == 0 && lossflag2_Left == 1)
    //lossline_Left = 1;
  
  
  

  while (CCD_BUFF[2*TSL1401_MAX+0][len_hinder_n] != 0 && len_hinder_n>=64)
  {
    len_hinder_n--;
    if (len_hinder_n <= 70)
    {
      lossflag1_Right = 1;
      break;
    }
  }
 
  
  
  
   /*  

  while (prebin[len_hinder_1] != 0 && len_hinder_1>=64)
  {
    len_hinder_1--;
    if (len_hinder_1 <= 70)
    {
      lossflag2_Right = 1;
      break;
    }
  }
*/
/*
    if (lossflag1_Left == 1)
  {
    lossline_Left = 1;
  }
    if (lossflag1_Right == 1)
  {
    lossline_Right = 1;
  }
  */
    if (lossflag1_Left == 1 && lossflag1_Right == 1 )
  {
    shizi = 1;
    lossflag1_Left = 0;
    lossflag1_Right = 0;
  }
  /********************************直角判断********************************************/
  /*
  for (i = 0;i<=128;i++)                                //检测有多少个像素点为黑
  {
       if (CCD_BUFF[2*TSL1401_MAX+0][i] == 0)
    {
      blacknum1++;
    }
    if (prebin[i] == 0)
    {
      blacknum2++;
    }
    
  }
  
  if (blacknum1 > 100&&blacknum2 > 100)                 //连续两次黑点数大于100，认为可能是标志位
  {
    zhijiao = 1;
  }
  */
  
  /*
  if (mayzhijiao == 1&&(lossline_Left == 1||lossline_Right == 1))               //确定为直角
  {
    zhijiao = 1;
  }
    */
  
}



void vcan_sendccd(uint8 *ccdaddr, uint32 ccdsize)
{
	#define CMD_CCD     2
    uint8 cmdf[2] = {CMD_CCD, ~CMD_CCD};    //开头命令
    uint8 cmdr[2] = {~CMD_CCD, CMD_CCD};    //结尾命令

    uart_putbuff(UART3, cmdf, sizeof(cmdf));    //先发送命令

    uart_putbuff(UART3, ccdaddr, ccdsize); //再发送图像

    uart_putbuff(UART3, cmdr, sizeof(cmdr));    //再发送命令
}

void CarDircAdjust(void)
{
  uint8 i = 0;
  uint8 j = 0;
  uint8 leftbreaknum = 0,rightbreaknum = 0;
  uint8 centerflag=0;
  //uint8 losslineflag = 0;

  
  for(i=0;i<=64;i++)
          {
          if(CCD_BUFF[1*TSL1401_MAX+0][64-i]>=diff_threshold)
                  {
                  left_side=64-i;
                  break;
                  }

                  leftbreaknum = 64-i;
          }
  

          
          for(i=0;i<64;i++)
                  {
                  if(CCD_BUFF[1*TSL1401_MAX+0][64+i]>=diff_threshold)
                          {
                          right_side=64+i;
                          break;
                          }

                          rightbreaknum = 64+i;
                  }

  if (leftbreaknum == 0)
  {
    lossline_Left = 1;
    
  }
  if (rightbreaknum == 127)
  {
    lossline_Right = 1;
    
  }
  if (leftbreaknum == 0 && rightbreaknum == 127)
  {
    shizi = 1;
    lossline_Right = 0;
    lossline_Left = 0;
  }

    /*
         if (left_side == right_side)                                   //左或右超过64
         {
        for(i=0;i<64;i++)
          {
          if(CCD_BUFF[1*TSL1401_MAX+0][64-i]<=diff_threshold)
                  {
                  right_side=64-i;
                  break;
                  }
          else
                  right_side=127;
          }
  
  
          
          for(i=0;i<64;i++)
                  {
                  if(CCD_BUFF[1*TSL1401_MAX+0][64+i]!=diff_threshold)
                          {
                          left_side=64+i;
                          break;
                          }	
                  else
                          left_side=0;
                  }
  */
           /*
           for(i=0;i<128;i++)
          {
          if(CCD_BUFF[2*TSL1401_MAX+0][i]==0)
                  {
                  right_side=i-1;
                  break;
                  }
          }
          
          
          for(i=127;i<=0;i--)
          {
          if(CCD_BUFF[2*TSL1401_MAX+0][i]==0)
                  {
                  left_side=i+1;
                  break;
                  }
          }
           
   }
*/

    
    if (shizi == 1)                                     //十字分期处理
    {
      
      //dismid = predismid[j-1];
      dismid=64-(left_side+right_side)/2;
      if (shizicount == 0)
      {
      shizi = 0;
      shizicount = 3;
      }
      
    }
  /*
    else if(zhijiao == 1)                                       //直角处理
    {
      dismid = -10;
      zhijiao = 0;
    }
  */
  
    else  if(lossline_Left == 1)
    {
      dismid = 64 - (right_side - repairline);
      lossline_Left = 0;
    }
    else if(lossline_Right == 1)
    {
      dismid = 64 - (left_side + repairline);
      lossline_Right = 0;
    }
  
    else
      
    dismid=64-(left_side+right_side)/2;
    
    for (j = 0;j<2;j++)                         //保存前三次偏差
    {
    predismid[j] = predismid[j+1];
    }
    predismid[j] = dismid;
    
    if (dismid == 0 && centerflag == 0)
    {
      repairline = (64-left_side+127-right_side)/2;
        centerflag = 1;
    }
    
    if (dismid >=30)
      dismid = 30;
    if (dismid <= -30)
      dismid = -30;
    
    nDirControlOutNew = nDirControlOutOld;
    nDirControlOutNew = dismid*DIRC_P;

  
  
  }
  

/*!
*	@since		v5.0
*	@note		山外 线性CCD 测试实验
                              修改 PIT0 的定时时间即可修改曝光时间
*/
void  CCD_Get(void)
{
      //uint8 time =6;                             //设置曝光时间        
    //uint8 copynum = 0;

  //uint8  max[TSL1401_SIZE];
  //uint8  avg[TSL1401_SIZE];
 


  //LCD_init();                                 //初始化
  //uart_init (UART3,115200);

  //初始化 线性CCD
  //tsl1401_set_addrs(TSL1401_MAX,(uint8 *)&CCD_BUFF[0]);
  //tsl1401_init(time);                         //初始化 线性CCD ，配置 中断时间为 time
  //tsl1401_led_en(TSL1401_MAX);                //开启补光灯
  //tsl1401_led_dis(TSL1401_MAX);             //关闭补光灯



     ///采集 线性CCD 图像
  
      //tsl1401_restet();
      tsl1401_get_img();



      //限制最大值
      maxvar((uint8 *)&CCD_BUFF[0],TSL1401_SIZE,BIN_MAX);
      //maxvar((uint8 *)&CCD_BUFF[1],TSL1401_SIZE,BIN_MAX);
      //maxvar((uint8 *)&CCD_BUFF[2],TSL1401_SIZE,BIN_MAX);

      //求波形差分
      abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+0],(uint8 *)&CCD_BUFF[0],TSL1401_SIZE,&max[0],&avg[0]);
      //abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+1],(uint8 *)&CCD_BUFF[1],TSL1401_SIZE,&max[1],&avg[1]);
      //abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+2],(uint8 *)&CCD_BUFF[2],TSL1401_SIZE,&max[2],&avg[2]);
      
      /*
      if (shizi == 1 && shizicount != 0)
      {
        for (;copynum < 128;copynum++)
       {
        CCD_BUFF[2*TSL1401_MAX+0][copynum] = prebin[copynum];
       }
       
       shizicount--;
        
      }
      
      
      
      else
      {
        
      for (copynum = 0;copynum < 128;copynum++)
      {
        prebin[copynum] = CCD_BUFF[2*TSL1401_MAX+0][copynum];
      }
      */
      /*
      for (copynum = 0;copynum < 128;copynum++)
      {
        prebin2[copynum] = prebin[copynum];
      }
      */
      //根据差分波形二值化图像
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],(uint8 *)&CCD_BUFF[0],(uint8 *)&CCD_BUFF[TSL1401_MAX+0],TSL1401_SIZE,max[0]);
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+1],(uint8 *)&CCD_BUFF[1],(uint8 *)&CCD_BUFF[TSL1401_MAX+1],TSL1401_SIZE,max[1]);
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+2],(uint8 *)&CCD_BUFF[2],(uint8 *)&CCD_BUFF[TSL1401_MAX+2],TSL1401_SIZE,max[2]);
      
            
      //vcan_sendccd((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],TSL1401_SIZE);       //这里仅发送一个CCD图像数据到上位机，因此上位机需要选择一个CCD   
      }
              
void DirectionControlOutput(void) 
{ 
    int16 Value = 0;
    Value = nDirControlOutNew - nDirControlOutOld;
    nDirControlOut = Value * nDirectionControlPeriod /2 + nDirControlOutOld;  
                          
}
