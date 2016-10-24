#include "include.h"

float OutData[4] = { 0 };                                    //SCIʾ��������
float Gyro_Now,angle_offset_vertical;  //������ת����Ľ��ٶȣ�ת����ļ��ٶȽǶ�
float g_fCarAngle,g_fGyroscopeAngleIntegral; //�ںϺ�ĽǶ�
float Speed_Old = 0, Speed_New = 0, Speed_Keep = 0;
//int8 count_flag = 0;
int16 nSpeedControlOut = 0, nAngleControlOut = 0, nDirControlOut = 0;
float nSpeedControlOut1 = 0,De = 0;         //����Ƿ��20��
int16 temp_Left = 0, temp_Right = 0;
float Speed_L = 0,Speed_R = 0,speed_Start = 0,Speed_L_Last = 0,Speed_R_Last = 0;  //�������ٶ� �������ٶ�

volatile int16     MMA7361 ,ENC03,real_angle;                             //���ٶȼ�AD ,������AD��ģ������ĽǶ�
int16 nLeftVal = 0, nRightVal = 0;
int8 SpeedStepCount = 0;
float WantSpeed = 0.0;
uint8 Iflag = 0;


float I_SPEED = 0.0;
//uint8 zhijiao = 0;


int16 nDirControlOutNew = 0,nDirControlOutOld = 0;
uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];							//����洢����CCDͼ�������
uint8 prebin[TSL1401_SIZE];//,prebin2[TSL1401_SIZE];
uint8 predismid[3] = {0};
uint8 lossline_Left = 0,lossline_Right = 0;
uint8 shizi = 0;
uint8 shizicount = 3;


uint8 left_side;
uint8 right_side;
int8 dismid = 0;
//uint8 time = 6;                             //�����ع�ʱ�� 


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
*  ����˵����AD�ɼ�
*  ����˵���� ��
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
*/
//**************************************************************************
void Rd_Ad_Value(void)
{

    MMA7361 = adc_once(ZOUT, ADC_12bit);   //Z
    ENC03= adc_once(Gyro1,ADC_12bit);    // gyro1

    //����ʹ������˲�����˲����� Ӳ���ںϽǶ�
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
*  ����˵����SCIʾ����CRCУ��
�ڲ����ú���
*  ����˵���� ��
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
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
*  ����˵����SCIʾ�������ͺ���

*  ����˵����
OutData[]  ��Ҫ���͵���ֵ���������
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
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
//   Kalman�˲�
//**************************************************************************

float angle, angle_dot;         //�ⲿ��Ҫ���õı���
//-------------------------------------------------------
// 0.00015     //0.0001
const float Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.005;
//0.0001         //0.00015        //1.2
//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��;         //0.8
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
//   �廪�Ƕ��˲�����
//*************************************************************************
/*
*  ����˵�����廪�Ƕ��˲�
*  ����˵����G_angle                       ���ٶȼƽǶ�0-90��
*            Gyro                         �����ǽ��ٶ�ת�������ֵ
*            GRAVITY_ADJUST_TIME_CONSTANT  ʱ��У��ϵ��
DT                             ��ʱ��ʱ�� ��λs
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע���ο��廪Դ��
*/
//
//*************************************************************************
void QingHua_AngleCalaulate(float G_angle,float Gyro)
{
    float fDeltaValue;

    g_fCarAngle = g_fGyroscopeAngleIntegral;   //�����ںϽǶ�
    fDeltaValue = (G_angle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;  //ʱ��ϵ������
    g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;                //�ںϽǶ�
}

//**************************************************************************
/*
*  ����˵����ֱ���Ƕȼ���
*  ����˵����

*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע���ο��廪Դ��
*/
//**************************************************************************
void AD_Calculate(void)
{
    

    Rd_Ad_Value();                          //�ɼ� AD
    
    Gyro_Now = (GYRO_VAL - ENC03)* Gyro_ratio;                            //�����ǲɼ����Ľ��ٶȹ�һ��
    angle_offset_vertical = (MMA7361_vertical - MMA7361) * MMA7361_ratio;  //�����ٶȼƲɼ����ĽǶȹ�һ��������0.375��Ϊ�˹�һ����0~90��
    if(angle_offset_vertical > 90)angle_offset_vertical = 90;               //��ֹ���ٶȽǶ����
    if(angle_offset_vertical < -90)angle_offset_vertical = -90;

    //�����ںϺ�ĽǶ�
    QingHua_AngleCalaulate(angle_offset_vertical,Gyro_Now);                 //�廪�Ƕ��˲�����
    

    /*****************************���ڿ����Σ�ѡ��ʹ�ã�****************************/
#if 0                           //���������� ѡ���Ƿ�ʹ�� ����ʾ����
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
*  ����˵����ֱ���ٶȼ���
*  ����˵����angle                �ںϺ����սǶ�
*            angle_dot            �����ǽ��ٶ�
*
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע���ο��廪Դ��
*/
/******************************************************************************/
void Speed_Calculate(float angle,float angle_dot)
{
    /***********************************�ٶȼ���************************************/
    speed_Start = angle * P_ANGLE  + angle_dot * D_ANGLE ;  //ֱ��ʱ��Ҫ���ٶ�

    //P_ANGLE  P_GYRO  �궨�� ֱ������Ҫ��PD����

    Speed_L = speed_Start;//�������ٶ�
    Speed_R = speed_Start;//�������ٶ�
    /***********************������ٶ�������985��PWM��******************************/
    if(Speed_L > MOTOR_MAX_Z)  Speed_L = MOTOR_MAX_Z;
    if(Speed_L < MOTOR_MIN_F) Speed_L = MOTOR_MIN_F;
    if(Speed_R > MOTOR_MAX_Z)  Speed_R = MOTOR_MAX_Z;
    if(Speed_R < MOTOR_MIN_F) Speed_R = MOTOR_MIN_F;
    
    nAngleControlOut = Speed_L = Speed_R;
    
}
    /***************��Ϊ�������ּ��˷���������������ٶȽ���һ�����յĴ���***************
        Speed_L_Last = Speed_L;  //����
    else
        Speed_L_Last = - Speed_L;

    if(Speed_R > 0)     
        Speed_R_Last = Speed_R;  //����
    else
        Speed_R_Last = - Speed_R;

   **********�����õ��Ķ�Ӧ�Ƕȵ��ٶȽ���PWM����********************/
    /*
    if(Speed_L >= 0)    //angle����0����ǰ��С��0�����
    {
        ftm_pwm_duty(FTM0,FTM_CH4,0);
        ftm_pwm_duty(FTM0,FTM_CH6,(uint32)(Speed_L - MOTOR_DEAD_VAL_L));    //����������ѹ
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH6,0);
        ftm_pwm_duty(FTM0,FTM_CH4,(uint32)(-Speed_L - MOTOR_DEAD_VAL_L));    //����������ѹ
    }

    if(Speed_R >= 0)    //angle����0����ǰ��С��0�����
    {
        ftm_pwm_duty(FTM0,FTM_CH3,0);
        ftm_pwm_duty(FTM0,FTM_CH5,(uint32)(Speed_R - MOTOR_DEAD_VAL_R));    //����������ѹ
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH5,0);
        ftm_pwm_duty(FTM0,FTM_CH3,(uint32)(-Speed_R - MOTOR_DEAD_VAL_R));   //����������ѹ
    }
}
*/
/***********************************************************************/
/*
*  ����˵�����ٶȿ���
*  ����˵����
*
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע��
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
      if(Speed_Set-WantSpeed<1)      //������   
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
      
     if (Iflag == 0 && nSpeed>=0.9*Speed_Set)                                   //���ָ���
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
void SpeedControlOutput(void)       //���ٶȾ���20��
{
     
     
     nSpeedControlOut1 = Speed_New - Speed_Old;
     nSpeedControlOut = nSpeedControlOut1*nSpeedControlPeriod/20 + Speed_Old;
     De = nSpeedControlOut1*nSpeedControlPeriod/20;
     //Cal_Left_Speed = Cal_Right_Speed = nSpeedControlOut;
}


/***********************************************************************/
/*
*  ����˵������������
*  ����˵����
*
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע��
*/
/******************************************************************************/


void SpeedGet(void)
{
    int16 temp_L = 0, temp_R = 0;
    
    temp_Left = ftm_quad_get(FTM1);          //��ȡFTM �������� ��������(������ʾ������)zuo
    temp_Left = -temp_Left;
    temp_Left+= temp_L;
      
    ftm_quad_clean(FTM1);
/*
    if(temp_Left<=0)
    {
        printf("\n����ת��%d",-temp_Left);
    }
    else
    {
        printf("\n��ת��%d",temp_Left);
    }
*/
    
    temp_R = ftm_quad_get(FTM2);          //��ȡFTM �������� ��������(������ʾ������)you
    temp_Right+= temp_R;
    ftm_quad_clean(FTM2);
    //count_flag++;
/*
    if(temp_Right>=0)
    {
        printf("\n����ת��%d",temp_Right);
    }
    else
    {
        printf("\n�ҷ�ת��%d",-temp_Right);
    }
*/
}

/***********************************************************************/
/*
*  ����˵����������
*  ����˵����
*
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע��
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
*  ����˵��������
*  ����˵����
*
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע��
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
 *  @brief      �����־���ֵ
 *  @since      v5.0
 *  @note       ɽ���ַ�����˵����������ֵmaxval �� ���ƽ��ֵavgval ������
                ������Ϊ�˱��ڶ���ȷ����ֵ������ģ���ɾ�������ƽ��ֵ��һ�㷵�ؽ��
                ���ǳ�С����˶����ò��ϣ�����ɾ�����˴�������Ϊ�˸������֤��
 */
void abs_diff(uint8 *dst,uint8 *src,uint16 len,uint8 * maxval,uint8 * avgval)
{
    int8 tmp;
    uint8 max = 0;
    uint32 sum = 0;
    uint16 lentmp = len;
    while(--lentmp)                 //��ѭ�� len-1 ��
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
    *dst = 0;               // ���һ�� ������Ϊ 0
    *maxval = max;           // ���������Բ�ֵ
    *avgval = (uint8)(sum/(len-1));  //ǰ len -1 ������ƽ��ֵ
    
}

/*!
 *  @brief      �򵥵�һ����ֵ�� �㷨�����ȶ�,�����ԣ�
 *  @since      v5.0
 */
// diff_threshold �����ֵ ,��ͬ�ĽǶȣ���ͬ�Ļ�����������ͬ
//�ɸ��� maxdif �����ֵ�����ã�����ֱ�ӹ̶���ֵ
//#define diff_threshold    ((maxdif> 12) ? ((maxdif*80)/100) :10)     // �����ֵ
//#define diff_threshold    10
//#define safe_isolation    3
void bin(uint8 *bin,uint8 * img,uint8 * difimg,uint16 len,uint8 maxdif)
{
    uint16 tmplen = len;
    uint8  thldnum = 0;        //��ֵ����
    uint8  thresholdimg;
    uint8  tmpnum;
    
  //uint8 len_ahead_n = 5,len_ahead_1 = 5;
  //uint8 len_hinder_n = 123,len_hinder_1 = 123;
  //uint8 lossflag1_Left = 0,lossflag2_Left = 0;
  //uint8 lossflag1_Right = 0,lossflag2_Right = 0;

    memset(bin,0xFF,len);  //ȫ������

    while(tmplen--)
    {
        if((tmplen == 0)|| (tmplen > len))
        {
            return;
        }

        if(difimg[tmplen] > diff_threshold)                  //�ҵ� �����ֵ
        {
            thldnum++;

            //Ѱ���������ֵ
            while(tmplen--)
            {
                if((tmplen == 0)|| (tmplen > len))
                {
                    return;
                }

                if(difimg[tmplen] < difimg[tmplen+1] )    //tmplen+1 Ϊ�����ֵ
                {
                     break;
                }
            }

            //tmplen + 1 �� ������ֵ ���л��� ��ɫɨ��
            if((img[tmplen] <= img[tmplen+1]) ||(img[tmplen+1] <= img[tmplen+2]) )  // ǰ�� ��ɫ ������ ��ɫ
            {
                //ѡ�� ���ֵ���ֵ��ǰһ�� ��Ϊ ��ֵ
                thresholdimg = (img[tmplen+1] + img[tmplen+2])/2;

                //ɨ����һ�� ���� ����ֵ (�ȴ˵����)
                while(img[tmplen] <= thresholdimg)
                {
                    bin[tmplen] = 0;                //��ɫ
                    tmplen--;
                    if(tmplen == 0)      //��β�� ,ֱ���˳�
                    {
                        if(img[tmplen] <= thresholdimg)
                        {
                             bin[tmplen] = 0;                //��ɫ
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

                //�ȴ����ֵ����
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
                //ǰ�� �� ��
                if(thldnum == 1)
                {
                    //����� ���ݶ��� ��ɫ��
                    tmpnum  = tmplen + 1;
                    while(tmpnum < len)
                    {
                        bin[tmpnum] = 0;                //��ɫ
                        tmpnum ++;
                    }
                }
            }


        }
    }
    

    
}



/*************************����*******************/


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
  /********************************ֱ���ж�********************************************/
  /*
  for (i = 0;i<=128;i++)                                //����ж��ٸ����ص�Ϊ��
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
  
  if (blacknum1 > 100&&blacknum2 > 100)                 //�������κڵ�������100����Ϊ�����Ǳ�־λ
  {
    zhijiao = 1;
  }
  */
  
  /*
  if (mayzhijiao == 1&&(lossline_Left == 1||lossline_Right == 1))               //ȷ��Ϊֱ��
  {
    zhijiao = 1;
  }
    */
  
}



void vcan_sendccd(uint8 *ccdaddr, uint32 ccdsize)
{
	#define CMD_CCD     2
    uint8 cmdf[2] = {CMD_CCD, ~CMD_CCD};    //��ͷ����
    uint8 cmdr[2] = {~CMD_CCD, CMD_CCD};    //��β����

    uart_putbuff(UART3, cmdf, sizeof(cmdf));    //�ȷ�������

    uart_putbuff(UART3, ccdaddr, ccdsize); //�ٷ���ͼ��

    uart_putbuff(UART3, cmdr, sizeof(cmdr));    //�ٷ�������
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
         if (left_side == right_side)                                   //����ҳ���64
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

    
    if (shizi == 1)                                     //ʮ�ַ��ڴ���
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
    else if(zhijiao == 1)                                       //ֱ�Ǵ���
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
    
    for (j = 0;j<2;j++)                         //����ǰ����ƫ��
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
*	@note		ɽ�� ����CCD ����ʵ��
                              �޸� PIT0 �Ķ�ʱʱ�伴���޸��ع�ʱ��
*/
void  CCD_Get(void)
{
      //uint8 time =6;                             //�����ع�ʱ��        
    //uint8 copynum = 0;

  //uint8  max[TSL1401_SIZE];
  //uint8  avg[TSL1401_SIZE];
 


  //LCD_init();                                 //��ʼ��
  //uart_init (UART3,115200);

  //��ʼ�� ����CCD
  //tsl1401_set_addrs(TSL1401_MAX,(uint8 *)&CCD_BUFF[0]);
  //tsl1401_init(time);                         //��ʼ�� ����CCD ������ �ж�ʱ��Ϊ time
  //tsl1401_led_en(TSL1401_MAX);                //���������
  //tsl1401_led_dis(TSL1401_MAX);             //�رղ����



     ///�ɼ� ����CCD ͼ��
  
      //tsl1401_restet();
      tsl1401_get_img();



      //�������ֵ
      maxvar((uint8 *)&CCD_BUFF[0],TSL1401_SIZE,BIN_MAX);
      //maxvar((uint8 *)&CCD_BUFF[1],TSL1401_SIZE,BIN_MAX);
      //maxvar((uint8 *)&CCD_BUFF[2],TSL1401_SIZE,BIN_MAX);

      //���β��
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
      //���ݲ�ֲ��ζ�ֵ��ͼ��
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],(uint8 *)&CCD_BUFF[0],(uint8 *)&CCD_BUFF[TSL1401_MAX+0],TSL1401_SIZE,max[0]);
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+1],(uint8 *)&CCD_BUFF[1],(uint8 *)&CCD_BUFF[TSL1401_MAX+1],TSL1401_SIZE,max[1]);
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+2],(uint8 *)&CCD_BUFF[2],(uint8 *)&CCD_BUFF[TSL1401_MAX+2],TSL1401_SIZE,max[2]);
      
            
      //vcan_sendccd((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],TSL1401_SIZE);       //���������һ��CCDͼ�����ݵ���λ���������λ����Ҫѡ��һ��CCD   
      }
              
void DirectionControlOutput(void) 
{ 
    int16 Value = 0;
    Value = nDirControlOutNew - nDirControlOutOld;
    nDirControlOut = Value * nDirectionControlPeriod /2 + nDirControlOutOld;  
                          
}
