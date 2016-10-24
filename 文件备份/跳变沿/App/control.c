#include "include.h"
#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "MK60_adc.h"



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



int16 nDirControlOutNew = 0,nDirControlOutOld = 0;
uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];							//����洢����CCDͼ�������
uint8 left_side;
uint8 right_side;
int8 dismid = 0;
uint8 time = 6;                             //�����ع�ʱ�� 


int8 count_1ms = 0;
int8 nSpeedControlPeriod = 0;
int8 nSpeedControlCount = 0;
//int8 SpeedStartCount = 0;

int8 nDirectionControlPeriod = 0;
int8 nDirectionControlCount = 0;

float nP = 0, nI = 0;
float nSpeed = 0, nSpeedChange = 0;


//-----------------------ccd1----------------------//

uint8 breakpoint1;     //���䷧ֵ

uint8 leftbreak1=0;      //��������
uint8 rightbreak1=127;     //�Ҳ������

uint8 leftlose1;       //�����߶�ʧ��־λ
uint8 rightlose1;      //�Ҳ���߶�ʧ��־λ
int yuzhi;
int Ryuzhi;
//-----------------------ccd2----------------------//

uint8 breakpoint2;		//���䷧ֵ

uint8 leftbreak2;      //��������
uint8 rightbreak2;     //�Ҳ������

uint8 leftlose2;       //�����߶�ʧ��־λ
uint8 rightlose2;      //�Ҳ���߶�ʧ��־λ
//-----------------------ccd3----------------------//
uint8 m1,n1;
uint8 n2=0,m2=0;
uint8 breakpoint3;		//���䷧ֵ
//extern STEERPID steerpid;
//float ccd_center1[6];   //ccd1��ʵ������ 
float ccd1_center;
float ccd2_center;   //ccd2��ʵ������

float direction_err=0;



/*********************************************************************************/

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
		  if(Speed_Set-WantSpeed<1) 	 //������   
		  {
		  WantSpeed = Speed_Set;
		  }
		  else 
		  {
		  WantSpeed+=1; 
		  }
		 */
		  
		   if(SpeedStepCount<18)							   //18
		  {
		  SpeedStepCount++;
		  }
		  
		  nSpeedChange =( Speed_Set/10.0+Speed_Set*SpeedStepCount/20.0) - nSpeed ;				  //10,20
		  //nSpeedChange = nSpeed - ( Speed_Set/5.0+Speed_Set*SpeedStepCount/10.0);
		 
		 //nSpeedChange = Speed_Set - nSpeed;
		 //}
		  
		 if (Iflag == 0 && nSpeed>=0.9*Speed_Set)									//���ָ���
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

/*
*****************************************************************************/


/*
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

*/
/*!
 *  @brief      �����־���ֵ
 *  @since      v5.0
 *  @note       ɽ���ַ�����˵����������ֵmaxval �� ���ƽ��ֵavgval ������
                ������Ϊ�˱��ڶ���ȷ����ֵ������ģ���ɾ�������ƽ��ֵ��һ�㷵�ؽ��
                ���ǳ�С����˶����ò��ϣ�����ɾ�����˴�������Ϊ�˸������֤��

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
 */
/*!
 *  @brief      �򵥵�һ����ֵ�� �㷨�����ȶ�,�����ԣ�
 *  @since      v5.0

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
 */
void vcan_sendccd(uint8 *ccdaddr, uint32 ccdsize)
{
	#define CMD_CCD     2
    uint8 cmdf[2] = {CMD_CCD, ~CMD_CCD};    //��ͷ����
    uint8 cmdr[2] = {~CMD_CCD, CMD_CCD};    //��β����

    uart_putbuff(UART3, cmdf, sizeof(cmdf));    //�ȷ�������

    uart_putbuff(UART3, ccdaddr, ccdsize); //�ٷ���ͼ��

    uart_putbuff(UART3, cmdr, sizeof(cmdr));    //�ٷ�������
}
/*
void CarDircAdjust(void)
{
  int8 i = 0;
  
  for(i=0;i<64;i++)
          {
          if(CCD_BUFF[2*TSL1401_MAX+0][64-i]==0)
                  {
                  left_side=64-i;
                  break;
                  }	
          else
                  left_side=0;
          }
  
  
          
          for(i=0;i<64;i++)
                  {
                  if(CCD_BUFF[2*TSL1401_MAX+0][64+i]==0)
                          {
                          right_side=64+i;
                          break;
                          }	
                  else
                          right_side=127;
                  }
  
          dismid=64-(left_side+right_side)/2;
          
          nDirControlOutNew = nDirControlOutOld;
          nDirControlOutNew = dismid*DIRC_P;
  
  
  
  }
*/
/*!
*	@since		v5.0
*	@note		ɽ�� ����CCD ����ʵ��
                              �޸� PIT0 �Ķ�ʱʱ�伴���޸��ع�ʱ��

void  CCD_Get(void)
{
      //uint8 time =6;                             //�����ع�ʱ��        
/* Site_t site1={0,0};                         //��ʾͼ�����Ͻ�λ��
  Site_t site1b={0,0+30};                     //��ʾͼ�����Ͻ�λ��
  Site_t site1max={TSL1401_SIZE+20,0};        //��ʾ�����ֵλ��

  Site_t site2={0,70};                        //��ʾͼ�����Ͻ�λ��
  Site_t site2b={0,70+30};                    //��ʾͼ�����Ͻ�λ��

  Site_t site3={0,140};                        //��ʾͼ�����Ͻ�λ��
  Site_t site3b={0,140+30};                    //��ʾͼ�����Ͻ�λ��


  Size_t imgsize={TSL1401_SIZE,1};            //ͼ���С
  Size_t size={TSL1401_SIZE,30};              //��ʾ�����С
  
  uint8  max[TSL1401_SIZE];
  uint8  avg[TSL1401_SIZE];
*/ 


  //LCD_init();                                 //��ʼ��
  //uart_init (UART3,115200);

  //��ʼ�� ����CCD
  //tsl1401_set_addrs(TSL1401_MAX,(uint8 *)&CCD_BUFF[0]);
  //tsl1401_init(time);                         //��ʼ�� ����CCD ������ �ж�ʱ��Ϊ time
  //tsl1401_led_en(TSL1401_MAX);                //���������
  //tsl1401_led_dis(TSL1401_MAX);             //�رղ����

    /*

     ///�ɼ� ����CCD ͼ��
  
      //tsl1401_restet();
      tsl1401_get_img();

//LCD ��ʾͼ��
      LCD_Img_gray_Z(site1,size,(uint8 *)&CCD_BUFF[0],imgsize);
      LCD_Img_gray_Z(site2,size,(uint8 *)&CCD_BUFF[1],imgsize);
      LCD_Img_gray_Z(site3,size,(uint8 *)&CCD_BUFF[2],imgsize);

      //LCD ��ʾ����ͼ
      LCD_wave_display(site1,size,(uint8 *)&CCD_BUFF[0],BIN_MAX,BLUE );
      LCD_wave_display(site2,size,(uint8 *)&CCD_BUFF[1],BIN_MAX,BLUE );
      LCD_wave_display(site3,size,(uint8 *)&CCD_BUFF[2],BIN_MAX,BLUE );
    

      //�������ֵ
      maxvar((uint8 *)&CCD_BUFF[0],TSL1401_SIZE,BIN_MAX);
      //maxvar((uint8 *)&CCD_BUFF[1],TSL1401_SIZE,BIN_MAX);
      //maxvar((uint8 *)&CCD_BUFF[2],TSL1401_SIZE,BIN_MAX);

      //���β��
      abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+0],(uint8 *)&CCD_BUFF[0],TSL1401_SIZE,&max[0],&avg[0]);
      //abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+1],(uint8 *)&CCD_BUFF[1],TSL1401_SIZE,&max[1],&avg[1]);
      //abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+2],(uint8 *)&CCD_BUFF[2],TSL1401_SIZE,&max[2],&avg[2]);
  */
      /*LCD ��ʾ���ͼ
      LCD_wave_display(site1,size,(uint8 *)&CCD_BUFF[TSL1401_MAX+0],max[0],GREEN);
      LCD_wave_display(site2,size,(uint8 *)&CCD_BUFF[TSL1401_MAX+1],max[1],GREEN);
      LCD_wave_display(site3,size,(uint8 *)&CCD_BUFF[TSL1401_MAX+2],max[2],GREEN);
   

      //���ݲ�ֲ��ζ�ֵ��ͼ��
      bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],(uint8 *)&CCD_BUFF[0],(uint8 *)&CCD_BUFF[TSL1401_MAX+0],TSL1401_SIZE,max[0]);
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+1],(uint8 *)&CCD_BUFF[1],(uint8 *)&CCD_BUFF[TSL1401_MAX+1],TSL1401_SIZE,max[1]);
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+2],(uint8 *)&CCD_BUFF[2],(uint8 *)&CCD_BUFF[TSL1401_MAX+2],TSL1401_SIZE,max[2]);
    

      /*
    //LCD ��ʾ��ֵ��ͼ
      LCD_Img_gray_Z(site1b,size,(uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],imgsize);
      LCD_Img_gray_Z(site2b,size,(uint8 *)&CCD_BUFF[2*TSL1401_MAX+1],imgsize);
      LCD_Img_gray_Z(site3b,size,(uint8 *)&CCD_BUFF[2*TSL1401_MAX+2],imgsize);


      //��ʾ�����ֵ
      LCD_num_BC(site1max, max[0],3,FCOLOUR,BCOLOUR);

              
      //vcan_sendccd((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],TSL1401_SIZE);       //���������һ��CCDͼ�����ݵ���λ���������λ����Ҫѡ��һ��CCD   

              }
 */


void ccd1_deal(void)
{
  
        //leftbreak1=0;      //�������� 
        //rightbreak1=127;     //�Ҳ������
        uint8 leftbreakshuzu1[]={0},rightbreakshuzu1[]={127,127,127,127,127,127,127};
        m1=0,n1=0;
	leftlose1 = 0;
	rightlose1 = 0;
	uint8 i, j, k;
	uint8 ccd_data[128];
	uint8 temp;
	    yuzhi=45;
        Ryuzhi=20;
	
	for(k=0; k<128; k++)
	{
		ccd_data[k] = ccd1_array[k];
	}
	
	//-------------------------���3�����ֵ--------------------------//
	
	for(i=0; i<5; i++)
	{
		for(j=i+1; j<128; j++)
		{
			if(ccd_data[j] > ccd_data[i])
			{
				temp = ccd_data[i];
				ccd_data[i] = ccd_data[j];
				ccd_data[j] = temp;
			}
		}
	}
	
	//-------------------------���3����Сֵ--------------------------//
	
	for(i=5; i<10; i++)
	{
		for(j=i+1; j<128; j++)
		{
			if(ccd_data[j] < ccd_data[i])
			{
				temp = ccd_data[i] ;
				ccd_data[i] = ccd_data[j];
				ccd_data[j] = temp;
			}
		}
	}
	
	breakpoint1 = (uint8)(((float)Ryuzhi/100)*(ccd_data[3]+ccd_data[4]+ccd_data[8]+ccd_data[9])/4.0);
       // if(breakpoint1<4) breakpoint1=4;
		
	//-------------------------����������-------------------------//
	
	for(i=0; i<122; i++)       //0-127
        {
            if( ccd1_array[i+6]-ccd1_array[i] >= breakpoint1 )
            {
               leftbreakshuzu1[n1]=i;
               if(n1>0)
               {
                 if(leftbreakshuzu1[n1]==leftbreakshuzu1[n1-1]+1)
                 {
                   leftbreakshuzu1[n1-1]=leftbreakshuzu1[n1];
                   n1--;
                 }
               }
               n1++;
            }
        }
        leftbreak1=leftbreakshuzu1[0];
	
	//-------------------------�Ҳ��������-------------------------//
	
	for(j=127; j>5; j--)     //127-0
        {
		if( ccd1_array[j-6]-ccd1_array[j] >= breakpoint1 )
		{
                  rightbreakshuzu1[m1] = j;
                  if(m1>0)
                    {
                      if(rightbreakshuzu1[m1]==rightbreakshuzu1[m1-1]-1)
                      {
                        rightbreakshuzu1[m1-1]=rightbreakshuzu1[m1];
                        m1--;
                      }
                    }
                  m1++;
                }
        }
    rightbreak1=rightbreakshuzu1[0];
   /* if((leftbreak1 > rightbreak1))  
    {
      if(steerpid.pwm>60)
         rightbreak1=127;
      else if((steerpid.pwm<-60))
        leftbreak1=0;
    }
    */
        if(rightbreak1==127)          //�Ҳ���߶�ʧ���Ҳ൥��ѭ��
	{
		rightlose1 = 1;   //��λ�Ҳ���߶�ʧ��־λ
	}
      //else if(steerpid.pwm>70) rightlose1 = 1;
      if(leftbreak1==0)           //�����߶�ʧ����൥��ѭ��
      {
        leftlose1 = 1;     //��λ�����߶�ʧ��־λ
      }
      //else if(steerpid.pwm<-70) leftlose1 = 1;
    
}

/*********************************************************************************/

void direction_deal(void)
{  
  ////////////..........�ϰ�.............///////////////
  /*if(slope_flag==0&&(((rightbreak1-leftbreak1<60)&&(rightbreak1-leftbreak1>35))&&(((rightbreak1+leftbreak1)/2-64)<30)&&(((rightbreak2+leftbreak2)/2-64)<40)&&(((rightbreak1+leftbreak1)/2-64)>-30)&&(((rightbreak2+leftbreak2)/2-64)>-40)&&(m1==1&&n1==1))) 
     {
       if(obstacle==0)
         obstacle=1;
       if(obstacle_flag==0)
         obstacle_flag=1;
     }
     else obstacle_flag=0;
   */
    //////////////////////////////////////////////////////////////////////////
    ///////////////////////////////CCD1///////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

	//------------------��൥��ѭ��---------------------//
    
	
	if(leftlose1 == 0 && rightlose1 == 1)
	{
		ccd1_center = (leftbreak1 + leftbreak1+CENTERADJ_1) / 2;
    }
	
	//------------------�Ҳ൥��ѭ��---------------------//
	
	else if(leftlose1 == 1 && rightlose1 == 0)
	{
		ccd1_center = (rightbreak1 + rightbreak1-CENTERADJ_1) / 2;
  }
	
	//-------------------˫��ѭ��------------------------//
	
	else if(leftlose1 == 0 && rightlose1 == 0)
	{
		if(leftbreak1 < rightbreak1)      //���������С���������ʱ�Ÿı�
			ccd1_center = (leftbreak1 + rightbreak1)/2.0;
		else
			ccd1_center = 64;
	}
		//-------------------˫��ѭ��------------------------//
	
	else if(leftlose1 == 1 && rightlose1 == 1)
	{
		if(leftbreak1 < rightbreak1)      //���������С���������ʱ�Ÿı�
			ccd1_center = (leftbreak1 + rightbreak1)/2.0;
		else
			ccd1_center = 64;
		}
}
      /*  if(((ccd_center1[5]-ccd_center1[0])/5-(ccd1_center-ccd_center1[5])>20)||((ccd_center1[5]-ccd_center1[0])/5-(ccd1_center-ccd_center1[5])<-20))
           ccd1_center=ccd_center1[5];
        else 
        {
          ccd_center1[0]=ccd_center1[1];
          ccd_center1[1]=ccd_center1[2];
          ccd_center1[2]=ccd_center1[3];
          ccd_center1[3]=ccd_center1[4];
          ccd_center1[4]=ccd_center1[5];
          ccd_center1[5]=ccd1_center;
        }
	*/
/*
	//////////////////////////////////////////////////////////////////////////
    ////////////////////////////////CCD2//////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
	
	//------------------��൥��ѭ��---------------------//
	if(leftlose2 == 0 && rightlose2 == 1)
	{
		ccd2_center = (leftbreak2 + leftbreak2+97) / 2;
    }
	
	//------------------�Ҳ൥��ѭ��---------------------//
	
	else if(leftlose2 == 1 && rightlose2 == 0)
	{				    
		ccd2_center = (rightbreak2 + rightbreak2-97) / 2;
    }
	
	//-------------------˫��ѭ��------------------------//
	
	else if(leftlose2 == 0 && rightlose2 == 0)
	{
        if(leftbreak2 < rightbreak2)      //���������С���������ʱ�Ÿı����Ƕ�
    	    ccd2_center = (leftbreak2 + rightbreak2)/2.0;
	}
       }

*/
/*********************************************************************************/
void CarDircAdjust(void)
{
          direction_err=64-ccd1_center;
          nDirControlOutNew = nDirControlOutOld;
          nDirControlOutNew = direction_err*DIRC_P;
  
}




void DirectionControlOutput(void) 
{ 
    int16 Value = 0;
    Value = nDirControlOutNew - nDirControlOutOld;
    nDirControlOut = Value * nDirectionControlPeriod /2 + nDirControlOutOld;  
                          
}

