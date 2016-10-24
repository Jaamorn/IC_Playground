#include "include.h"
#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "MK60_adc.h"



float OutData[4] = { 0 };                                    //SCIÊ¾²¨Æ÷²ÎÊý
float Gyro_Now,angle_offset_vertical;  //ÍÓÂÝÒÇ×ª»¯ºóµÄ½ÇËÙ¶È£¬×ª»¯ºóµÄ¼ÓËÙ¶È½Ç¶È
float g_fCarAngle,g_fGyroscopeAngleIntegral; //ÈÚºÏºóµÄ½Ç¶È
float Speed_Old = 0, Speed_New = 0, Speed_Keep = 0;
//int8 count_flag = 0;
int16 nSpeedControlOut = 0, nAngleControlOut = 0, nDirControlOut = 0;
float nSpeedControlOut1 = 0,De = 0;         //¼ì²âÊÇ·ñ·Ö20·Ý
int16 temp_Left = 0, temp_Right = 0;
float Speed_L = 0,Speed_R = 0,speed_Start = 0,Speed_L_Last = 0,Speed_R_Last = 0;  //×óÓÒÂÖËÙ¶È £¬×îÖÕËÙ¶È

volatile int16     MMA7361 ,ENC03,real_angle;                             //¼ÓËÙ¶È¼ÆAD ,ÍÓÂÝÒÇAD£¬Ä£¿éÊä³öµÄ½Ç¶È
int16 nLeftVal = 0, nRightVal = 0;
int8 SpeedStepCount = 0;
float WantSpeed = 0.0;
uint8 Iflag = 0;


float I_SPEED = 0.0;



int16 nDirControlOutNew = 0,nDirControlOutOld = 0;
uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];							//¶¨Òå´æ´¢½ÓÊÕCCDÍ¼ÏñµÄÊý×é
uint8 left_side;
uint8 right_side;
int8 dismid = 0;
uint8 time = 6;                             //ÉèÖÃÆØ¹âÊ±¼ä 


int8 count_1ms = 0;
int8 nSpeedControlPeriod = 0;
int8 nSpeedControlCount = 0;
//int8 SpeedStartCount = 0;

int8 nDirectionControlPeriod = 0;
int8 nDirectionControlCount = 0;

float nP = 0, nI = 0;
float nSpeed = 0, nSpeedChange = 0;


//-----------------------ccd1----------------------//

uint8 breakpoint1;     //Ìø±ä·§Öµ

uint8 leftbreak1=0;      //×ó²àÌø±äµã
uint8 rightbreak1=127;     //ÓÒ²àÌø±äµã

uint8 leftlose1;       //×ó²àºÚÏß¶ªÊ§±êÖ¾Î»
uint8 rightlose1;      //ÓÒ²àºÚÏß¶ªÊ§±êÖ¾Î»
int yuzhi;
int Ryuzhi;
//-----------------------ccd2----------------------//

uint8 breakpoint2;		//Ìø±ä·§Öµ

uint8 leftbreak2;      //×ó²àÌø±äµã
uint8 rightbreak2;     //ÓÒ²àÌø±äµã

uint8 leftlose2;       //×ó²àºÚÏß¶ªÊ§±êÖ¾Î»
uint8 rightlose2;      //ÓÒ²àºÚÏß¶ªÊ§±êÖ¾Î»
//-----------------------ccd3----------------------//
uint8 m1,n1;
uint8 n2=0,m2=0;
uint8 breakpoint3;		//Ìø±ä·§Öµ
//extern STEERPID steerpid;
//float ccd_center1[6];   //ccd1µÄÊµ¼ÊÖÐÐÄ 
float ccd1_center;
float ccd2_center;   //ccd2µÄÊµ¼ÊÖÐÐÄ

float direction_err=0;



/*********************************************************************************/

//**************************************************************************
/*
*  ¹¦ÄÜËµÃ÷£ºAD²É¼¯
*  ²ÎÊýËµÃ÷£º ÎÞ
*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
*/
//**************************************************************************
void Rd_Ad_Value(void)
{

    MMA7361 = adc_once(ZOUT, ADC_12bit);   //Z
    ENC03= adc_once(Gyro1,ADC_12bit);    // gyro1

    //ÓÉÓÚÊ¹ÓÃÈí¼þÂË²¨£¬Òò´Ë²»ÔÙÓÃ Ó²¼þÈÚºÏ½Ç¶È
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
*  ¹¦ÄÜËµÃ÷£ºSCIÊ¾²¨Æ÷CRCÐ£Ñé
ÄÚ²¿µ÷ÓÃº¯Êý
*  ²ÎÊýËµÃ÷£º ÎÞ
*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
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
*  ¹¦ÄÜËµÃ÷£ºSCIÊ¾²¨Æ÷·¢ËÍº¯Êý

*  ²ÎÊýËµÃ÷£º
OutData[]  ÐèÒª·¢ËÍµÄÊýÖµ¸³Óè¸ÃÊý×é
*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
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
//   KalmanÂË²¨
//**************************************************************************

float angle, angle_dot;         //Íâ²¿ÐèÒªÒýÓÃµÄ±äÁ¿
//-------------------------------------------------------
// 0.00015     //0.0001
const float Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.005;
//0.0001         //0.00015        //1.2
//×¢Òâ£ºdtµÄÈ¡ÖµÎªkalmanÂË²¨Æ÷²ÉÑùÊ±¼ä;         //0.8
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
//   Çå»ª½Ç¶ÈÂË²¨·½°¸
//*************************************************************************
/*
*  ¹¦ÄÜËµÃ÷£ºÇå»ª½Ç¶ÈÂË²¨
*  ²ÎÊýËµÃ÷£ºG_angle                       ¼ÓËÙ¶È¼Æ½Ç¶È0-90ÄÚ
*            Gyro                         ÍÓÂÝÒÇ½ÇËÙ¶È×ª»¨ºóµÄÊýÖµ
*            GRAVITY_ADJUST_TIME_CONSTANT  Ê±¼äÐ£ÕýÏµÊý
DT                             ¶¨Ê±Æ÷Ê±¼ä µ¥Î»s
*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
* ±¸×¢£º²Î¿¼Çå»ªÔ´Âë
*/
//
//*************************************************************************
void QingHua_AngleCalaulate(float G_angle,float Gyro)
{
    float fDeltaValue;

    g_fCarAngle = g_fGyroscopeAngleIntegral;   //×îÖÕÈÚºÏ½Ç¶È
    fDeltaValue = (G_angle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;  //Ê±¼äÏµÊý½ÃÕý
    g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;                //ÈÚºÏ½Ç¶È
}

//**************************************************************************
/*
*  ¹¦ÄÜËµÃ÷£ºÖ±Á¢½Ç¶È¼ÆËã
*  ²ÎÊýËµÃ÷£º

*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
* ±¸×¢£º²Î¿¼Çå»ªÔ´Âë
*/
//**************************************************************************
void AD_Calculate(void)
{
    

    Rd_Ad_Value();                          //²É¼¯ AD
    
    Gyro_Now = (GYRO_VAL - ENC03)* Gyro_ratio;                            //ÍÓÂÝÒÇ²É¼¯µ½µÄ½ÇËÙ¶È¹éÒ»»¯
    angle_offset_vertical = (MMA7361_vertical - MMA7361) * MMA7361_ratio;  //½«¼ÓËÙ¶È¼Æ²É¼¯µ½µÄ½Ç¶È¹éÒ»»¯£¬³ËÉÏ0.375ÊÇÎªÁË¹éÒ»»¯µ½0~90¡ã
    if(angle_offset_vertical > 90)angle_offset_vertical = 90;               //·ÀÖ¹¼ÓËÙ¶È½Ç¶ÈÒç³ö
    if(angle_offset_vertical < -90)angle_offset_vertical = -90;

    //¼ÆËãÈÚºÏºóµÄ½Ç¶È
    QingHua_AngleCalaulate(angle_offset_vertical,Gyro_Now);                 //Çå»ª½Ç¶ÈÂË²¨·½°¸
    

    /*****************************´®¿Ú¿´²¨ÐÎ£¨Ñ¡ÔñÊ¹ÓÃ£©****************************/
#if 0                           //ºêÌõ¼þ±àÒë Ñ¡ÔñÊÇ·ñÊ¹ÓÃ ÐéÄâÊ¾²¨Æ÷
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
*  ¹¦ÄÜËµÃ÷£ºÖ±Á¢ËÙ¶È¼ÆËã
*  ²ÎÊýËµÃ÷£ºangle                ÈÚºÏºó×îÖÕ½Ç¶È
*            angle_dot            ÍÓÂÝÒÇ½ÇËÙ¶È
*
*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
* ±¸×¢£º²Î¿¼Çå»ªÔ´Âë
*/
/******************************************************************************/
void Speed_Calculate(float angle,float angle_dot)
{
    /***********************************ËÙ¶È¼ÆËã************************************/
    speed_Start = angle * P_ANGLE  + angle_dot * D_ANGLE ;  //Ö±Á¢Ê±ËùÒªµÄËÙ¶È

    //P_ANGLE  P_GYRO  ºê¶¨Òå Ö±Á¢ËùÐèÒªµÄPD²ÎÊý

    Speed_L = speed_Start;//×óÂÖ×ÜËÙ¶È
    Speed_R = speed_Start;//ÓÒÂÖ×ÜËÙ¶È
    /***********************½«×î´óËÙ¶ÈÏÞÖÆÔÚ985¸öPWMÄÚ******************************/
    if(Speed_L > MOTOR_MAX_Z)  Speed_L = MOTOR_MAX_Z;
    if(Speed_L < MOTOR_MIN_F) Speed_L = MOTOR_MIN_F;
    if(Speed_R > MOTOR_MAX_Z)  Speed_R = MOTOR_MAX_Z;
    if(Speed_R < MOTOR_MIN_F) Speed_R = MOTOR_MIN_F;
    
    nAngleControlOut = Speed_L = Speed_R;
    
}
    /***************ÒòÎªÇý¶¯²¿·Ö¼ÓÁË·´ÏàÆ÷£¬ËùÒÔÐè¶ÔËÙ¶È½øÐÐÒ»¸ö×îÖÕµÄ´¦Àí***************
        Speed_L_Last = Speed_L;  //ÕýÏà
    else
        Speed_L_Last = - Speed_L;

    if(Speed_R > 0)     
        Speed_R_Last = Speed_R;  //ÕýÏà
    else
        Speed_R_Last = - Speed_R;

   **********ÓÃËùµÃµ½µÄ¶ÔÓ¦½Ç¶ÈµÄËÙ¶È½øÐÐPWM¿ØÖÆ********************/
    /*
    if(Speed_L >= 0)    //angle´óÓÚ0£¬ÏòÇ°£¬Ð¡ÓÚ0£¬Ïòºó
    {
        ftm_pwm_duty(FTM0,FTM_CH4,0);
        ftm_pwm_duty(FTM0,FTM_CH6,(uint32)(Speed_L - MOTOR_DEAD_VAL_L));    //¼ÓÈëËÀÇøµçÑ¹
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH6,0);
        ftm_pwm_duty(FTM0,FTM_CH4,(uint32)(-Speed_L - MOTOR_DEAD_VAL_L));    //¼ÓÈëËÀÇøµçÑ¹
    }

    if(Speed_R >= 0)    //angle´óÓÚ0£¬ÏòÇ°£¬Ð¡ÓÚ0£¬Ïòºó
    {
        ftm_pwm_duty(FTM0,FTM_CH3,0);
        ftm_pwm_duty(FTM0,FTM_CH5,(uint32)(Speed_R - MOTOR_DEAD_VAL_R));    //¼ÓÈëËÀÇøµçÑ¹
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH5,0);
        ftm_pwm_duty(FTM0,FTM_CH3,(uint32)(-Speed_R - MOTOR_DEAD_VAL_R));   //¼ÓÈëËÀÇøµçÑ¹
    }
}
*/
/***********************************************************************/
	/***********************************************************************/
	/*
	*  ¹¦ÄÜËµÃ÷£ºËÙ¶È¿ØÖÆ
	*  ²ÎÊýËµÃ÷£º
	*
	*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
	*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
	* ±¸×¢£º
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
		  if(Speed_Set-WantSpeed<1) 	 //»ºÂýÆð²½   
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
		  
		 if (Iflag == 0 && nSpeed>=0.9*Speed_Set)									//»ý·Ö¸ôÀë
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


void SpeedControlOutput(void)       //½«ËÙ¶È¾ù·Ö20·Ý
{
     
     
     nSpeedControlOut1 = Speed_New - Speed_Old;
     nSpeedControlOut = nSpeedControlOut1*nSpeedControlPeriod/20 + Speed_Old;
     De = nSpeedControlOut1*nSpeedControlPeriod/20;
     //Cal_Left_Speed = Cal_Right_Speed = nSpeedControlOut;
}


/***********************************************************************/
/*
*  ¹¦ÄÜËµÃ÷£ºÕý½»½âÂë
*  ²ÎÊýËµÃ÷£º
*
*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
* ±¸×¢£º
*/
/******************************************************************************/


void SpeedGet(void)
{
    int16 temp_L = 0, temp_R = 0;
    
    temp_Left = ftm_quad_get(FTM1);          //»ñÈ¡FTM Õý½»½âÂë µÄÂö³åÊý(¸ºÊý±íÊ¾·´·½Ïò)zuo
    temp_Left = -temp_Left;
    temp_Left+= temp_L;
      
    ftm_quad_clean(FTM1);
/*
    if(temp_Left<=0)
    {
        printf("\n×óÕý×ª£º%d",-temp_Left);
    }
    else
    {
        printf("\n×ó·´×ª£º%d",temp_Left);
    }
*/
    
    temp_R = ftm_quad_get(FTM2);          //»ñÈ¡FTM Õý½»½âÂë µÄÂö³åÊý(¸ºÊý±íÊ¾·´·½Ïò)you
    temp_Right+= temp_R;
    ftm_quad_clean(FTM2);
    //count_flag++;
/*
    if(temp_Right>=0)
    {
        printf("\nÓÒÕý×ª£º%d",temp_Right);
    }
    else
    {
        printf("\nÓÒ·´×ª£º%d",-temp_Right);
    }
*/
}

/***********************************************************************/
/*
*  ¹¦ÄÜËµÃ÷£ºµç»úÊä³ö
*  ²ÎÊýËµÃ÷£º
*
*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
* ±¸×¢£º
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
*  ¹¦ÄÜËµÃ÷£º·½Ïò
*  ²ÎÊýËµÃ÷£º
*
*  º¯Êý·µ»Ø£ºÎÞ·ûºÅ½á¹ûÖµ
*  ÐÞ¸ÄÊ±¼ä£º2013-2-10
* ±¸×¢£º

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
 *  @brief      ¼ÆËã²î·Ö¾ø¶ÔÖµ
 *  @since      v5.0
 *  @note       É½Íâ²î·Ö·¨²¹³äËµÃ÷£º²î·Ö×î´óÖµmaxval ºÍ ²î·ÖÆ½¾ùÖµavgval ÕâÁ½¸ö
                ²ÎÊýÊÇÎªÁË±ãÓÚ¶¨ÒåÈ·¶¨ãÐÖµ¶ø¼ÓÈëµÄ£¬¿ÉÉ¾³ý¡£²î·ÖÆ½¾ùÖµ£¬Ò»°ã·µ»Ø½á¹û
                ¶¼·Ç³£Ð¡£¬Òò´Ë¶¥²ãÓÃ²»ÉÏ£¬½¨ÒéÉ¾µô£¨´Ë´¦±£ÁôÊÇÎªÁË¸ø´ó¼ÒÑéÖ¤£©

void abs_diff(uint8 *dst,uint8 *src,uint16 len,uint8 * maxval,uint8 * avgval)
{
    int8 tmp;
    uint8 max = 0;
    uint32 sum = 0;
    uint16 lentmp = len;
    while(--lentmp)                 //½öÑ­»· len-1 ´Î
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
    *dst = 0;               // ×îºóÒ»¸ö µãÅäÖÃÎª 0
    *maxval = max;           // ·µ»Ø×î´ó¾ø¶Ô²îÖµ
    *avgval = (uint8)(sum/(len-1));  //Ç° len -1 ¸öÊýµÄÆ½¾ùÖµ
}
 */
/*!
 *  @brief      ¼òµ¥µÄÒ»¸ö¶þÖµ»¯ Ëã·¨£¨²»ÎÈ¶¨,½ö²âÊÔ£©
 *  @since      v5.0

// diff_threshold ²î·ÖãÐÖµ ,²»Í¬µÄ½Ç¶È£¬²»Í¬µÄ»·¾³¶øÓÐËù²»Í¬
//¿É¸ù¾Ý maxdif ×î´ó²î·ÖÖµÀ´ÅäÖÃ£¬»òÕßÖ±½Ó¹Ì¶¨ãÐÖµ
//#define diff_threshold    ((maxdif> 12) ? ((maxdif*80)/100) :10)     // ²î·ÖãÐÖµ
//#define diff_threshold    10
//#define safe_isolation    3
void bin(uint8 *bin,uint8 * img,uint8 * difimg,uint16 len,uint8 maxdif)
{
    uint16 tmplen = len;
    uint8  thldnum = 0;        //ãÐÖµ´ÎÊý
    uint8  thresholdimg;
    uint8  tmpnum;

    memset(bin,0xFF,len);  //È«²¿µ±×÷

    while(tmplen--)
    {
        if((tmplen == 0)|| (tmplen > len))
        {
            return;
        }

        if(difimg[tmplen] > diff_threshold)                  //ÕÒµ½ ²î·ÖãÐÖµ
        {
            thldnum++;

            //Ñ°ÕÒ×î´ó²î·ÖãÐÖµ
            while(tmplen--)
            {
                if((tmplen == 0)|| (tmplen > len))
                {
                    return;
                }

                if(difimg[tmplen] < difimg[tmplen+1] )    //tmplen+1 Îª×î´óãÐÖµ
                {
                     break;
                }
            }

            //tmplen + 1 ÊÇ ²î·Ö×î´óÖµ £¬ÇÐ»»µ½ ÑÕÉ«É¨Ãè
            if((img[tmplen] <= img[tmplen+1]) ||(img[tmplen+1] <= img[tmplen+2]) )  // Ç°Ãæ ºÚÉ« £¬ºóÃæ °×É«
            {
                //Ñ¡Ôñ ²î·ÖÖµ×î´óÖµµÄÇ°Ò»¸ö ×÷Îª ãÐÖµ
                thresholdimg = (img[tmplen+1] + img[tmplen+2])/2;

                //É¨ÃèÏÂÒ»¸ö ¸ßÓÚ ´ËãÐÖµ (±È´Ëµã¸ü°×)
                while(img[tmplen] <= thresholdimg)
                {
                    bin[tmplen] = 0;                //ºÚÉ«
                    tmplen--;
                    if(tmplen == 0)      //½áÎ²ÁË ,Ö±½ÓÍË³ö
                    {
                        if(img[tmplen] <= thresholdimg)
                        {
                             bin[tmplen] = 0;                //ºÚÉ«
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

                //µÈ´ý²î·ÖÖµ½µµÍ
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
                //Ç°°× ºó ºÚ
                if(thldnum == 1)
                {
                    //ºóÃæµÄ ÄÚÈÝ¶¼ÊÇ ºÚÉ«µÄ
                    tmpnum  = tmplen + 1;
                    while(tmpnum < len)
                    {
                        bin[tmpnum] = 0;                //ºÚÉ«
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
    uint8 cmdf[2] = {CMD_CCD, ~CMD_CCD};    //¿ªÍ·ÃüÁî
    uint8 cmdr[2] = {~CMD_CCD, CMD_CCD};    //½áÎ²ÃüÁî

    uart_putbuff(UART3, cmdf, sizeof(cmdf));    //ÏÈ·¢ËÍÃüÁî

    uart_putbuff(UART3, ccdaddr, ccdsize); //ÔÙ·¢ËÍÍ¼Ïñ

    uart_putbuff(UART3, cmdr, sizeof(cmdr));    //ÔÙ·¢ËÍÃüÁî
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
*	@note		É½Íâ ÏßÐÔCCD ²âÊÔÊµÑé
                              ÐÞ¸Ä PIT0 µÄ¶¨Ê±Ê±¼ä¼´¿ÉÐÞ¸ÄÆØ¹âÊ±¼ä

void  CCD_Get(void)
{
      //uint8 time =6;                             //ÉèÖÃÆØ¹âÊ±¼ä        
/* Site_t site1={0,0};                         //ÏÔÊ¾Í¼Ïñ×óÉÏ½ÇÎ»ÖÃ
  Site_t site1b={0,0+30};                     //ÏÔÊ¾Í¼Ïñ×óÉÏ½ÇÎ»ÖÃ
  Site_t site1max={TSL1401_SIZE+20,0};        //ÏÔÊ¾×î´ó²î·ÖÖµÎ»ÖÃ

  Site_t site2={0,70};                        //ÏÔÊ¾Í¼Ïñ×óÉÏ½ÇÎ»ÖÃ
  Site_t site2b={0,70+30};                    //ÏÔÊ¾Í¼Ïñ×óÉÏ½ÇÎ»ÖÃ

  Site_t site3={0,140};                        //ÏÔÊ¾Í¼Ïñ×óÉÏ½ÇÎ»ÖÃ
  Site_t site3b={0,140+30};                    //ÏÔÊ¾Í¼Ïñ×óÉÏ½ÇÎ»ÖÃ


  Size_t imgsize={TSL1401_SIZE,1};            //Í¼Ïñ´óÐ¡
  Size_t size={TSL1401_SIZE,30};              //ÏÔÊ¾ÇøÓò´óÐ¡
  
  uint8  max[TSL1401_SIZE];
  uint8  avg[TSL1401_SIZE];
*/ 


  //LCD_init();                                 //³õÊ¼»¯
  //uart_init (UART3,115200);

  //³õÊ¼»¯ ÏßÐÔCCD
  //tsl1401_set_addrs(TSL1401_MAX,(uint8 *)&CCD_BUFF[0]);
  //tsl1401_init(time);                         //³õÊ¼»¯ ÏßÐÔCCD £¬ÅäÖÃ ÖÐ¶ÏÊ±¼äÎª time
  //tsl1401_led_en(TSL1401_MAX);                //¿ªÆô²¹¹âµÆ
  //tsl1401_led_dis(TSL1401_MAX);             //¹Ø±Õ²¹¹âµÆ

    /*

     ///²É¼¯ ÏßÐÔCCD Í¼Ïñ
  
      //tsl1401_restet();
      tsl1401_get_img();

//LCD ÏÔÊ¾Í¼Ïñ
      LCD_Img_gray_Z(site1,size,(uint8 *)&CCD_BUFF[0],imgsize);
      LCD_Img_gray_Z(site2,size,(uint8 *)&CCD_BUFF[1],imgsize);
      LCD_Img_gray_Z(site3,size,(uint8 *)&CCD_BUFF[2],imgsize);

      //LCD ÏÔÊ¾²¨ÐÎÍ¼
      LCD_wave_display(site1,size,(uint8 *)&CCD_BUFF[0],BIN_MAX,BLUE );
      LCD_wave_display(site2,size,(uint8 *)&CCD_BUFF[1],BIN_MAX,BLUE );
      LCD_wave_display(site3,size,(uint8 *)&CCD_BUFF[2],BIN_MAX,BLUE );
    

      //ÏÞÖÆ×î´óÖµ
      maxvar((uint8 *)&CCD_BUFF[0],TSL1401_SIZE,BIN_MAX);
      //maxvar((uint8 *)&CCD_BUFF[1],TSL1401_SIZE,BIN_MAX);
      //maxvar((uint8 *)&CCD_BUFF[2],TSL1401_SIZE,BIN_MAX);

      //Çó²¨ÐÎ²î·Ö
      abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+0],(uint8 *)&CCD_BUFF[0],TSL1401_SIZE,&max[0],&avg[0]);
      //abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+1],(uint8 *)&CCD_BUFF[1],TSL1401_SIZE,&max[1],&avg[1]);
      //abs_diff((uint8 *)&CCD_BUFF[TSL1401_MAX+2],(uint8 *)&CCD_BUFF[2],TSL1401_SIZE,&max[2],&avg[2]);
  */
      /*LCD ÏÔÊ¾²î·ÖÍ¼
      LCD_wave_display(site1,size,(uint8 *)&CCD_BUFF[TSL1401_MAX+0],max[0],GREEN);
      LCD_wave_display(site2,size,(uint8 *)&CCD_BUFF[TSL1401_MAX+1],max[1],GREEN);
      LCD_wave_display(site3,size,(uint8 *)&CCD_BUFF[TSL1401_MAX+2],max[2],GREEN);
   

      //¸ù¾Ý²î·Ö²¨ÐÎ¶þÖµ»¯Í¼Ïñ
      bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],(uint8 *)&CCD_BUFF[0],(uint8 *)&CCD_BUFF[TSL1401_MAX+0],TSL1401_SIZE,max[0]);
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+1],(uint8 *)&CCD_BUFF[1],(uint8 *)&CCD_BUFF[TSL1401_MAX+1],TSL1401_SIZE,max[1]);
    //bin((uint8 *)&CCD_BUFF[2*TSL1401_MAX+2],(uint8 *)&CCD_BUFF[2],(uint8 *)&CCD_BUFF[TSL1401_MAX+2],TSL1401_SIZE,max[2]);
    

      /*
    //LCD ÏÔÊ¾¶þÖµ»¯Í¼
      LCD_Img_gray_Z(site1b,size,(uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],imgsize);
      LCD_Img_gray_Z(site2b,size,(uint8 *)&CCD_BUFF[2*TSL1401_MAX+1],imgsize);
      LCD_Img_gray_Z(site3b,size,(uint8 *)&CCD_BUFF[2*TSL1401_MAX+2],imgsize);


      //ÏÔÊ¾×î´ó²î·ÖÖµ
      LCD_num_BC(site1max, max[0],3,FCOLOUR,BCOLOUR);

              
      //vcan_sendccd((uint8 *)&CCD_BUFF[2*TSL1401_MAX+0],TSL1401_SIZE);       //ÕâÀï½ö·¢ËÍÒ»¸öCCDÍ¼ÏñÊý¾Ýµ½ÉÏÎ»»ú£¬Òò´ËÉÏÎ»»úÐèÒªÑ¡ÔñÒ»¸öCCD   

              }
 */


void ccd1_deal(void)
{
  
        //leftbreak1=0;      //×ó²àÌø±äµã 
        //rightbreak1=127;     //ÓÒ²àÌø±äµã
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
	
	//-------------------------Çó³ö3¸ö×î´óÖµ--------------------------//
	
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
	
	//-------------------------Çó³ö3¸ö×îÐ¡Öµ--------------------------//
	
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
		
	//-------------------------×ó²àÌø±äµã¼ì²â-------------------------//
	
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
	
	//-------------------------ÓÒ²àÌø±äµã¼ì²â-------------------------//
	
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
        if(rightbreak1==127)          //ÓÒ²àºÚÏß¶ªÊ§£¬ÓÒ²àµ¥ÏßÑ­¼£
	{
		rightlose1 = 1;   //ÖÃÎ»ÓÒ²àºÚÏß¶ªÊ§±êÖ¾Î»
	}
      //else if(steerpid.pwm>70) rightlose1 = 1;
      if(leftbreak1==0)           //×ó²àºÚÏß¶ªÊ§£¬×ó²àµ¥ÏßÑ­¼£
      {
        leftlose1 = 1;     //ÖÃÎ»×ó²àºÚÏß¶ªÊ§±êÖ¾Î»
      }
      //else if(steerpid.pwm<-70) leftlose1 = 1;
    
}

/*********************************************************************************/

void direction_deal(void)
{  
  ////////////..........ÕÏ°­.............///////////////
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

	//------------------×ó²àµ¥ÏßÑ­¼£---------------------//
    
	
	if(leftlose1 == 0 && rightlose1 == 1)
	{
		ccd1_center = (leftbreak1 + leftbreak1+CENTERADJ_1) / 2;
    }
	
	//------------------ÓÒ²àµ¥ÏßÑ­¼£---------------------//
	
	else if(leftlose1 == 1 && rightlose1 == 0)
	{
		ccd1_center = (rightbreak1 + rightbreak1-CENTERADJ_1) / 2;
  }
	
	//-------------------Ë«ÏßÑ­¼£------------------------//
	
	else if(leftlose1 == 0 && rightlose1 == 0)
	{
		if(leftbreak1 < rightbreak1)      //µ±×óÌø±äµãÐ¡ÓÚÓÒÌø±äµãÊ±²Å¸Ä±ä
			ccd1_center = (leftbreak1 + rightbreak1)/2.0;
		else
			ccd1_center = 64;
	}
		//-------------------Ë«ÏßÑ­¼£------------------------//
	
	else if(leftlose1 == 1 && rightlose1 == 1)
	{
		if(leftbreak1 < rightbreak1)      //µ±×óÌø±äµãÐ¡ÓÚÓÒÌø±äµãÊ±²Å¸Ä±ä¶
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
	
	//------------------×ó²àµ¥ÏßÑ­¼£---------------------//
	if(leftlose2 == 0 && rightlose2 == 1)
	{
		ccd2_center = (leftbreak2 + leftbreak2+97) / 2;
    }
	
	//------------------ÓÒ²àµ¥ÏßÑ­¼£---------------------//
	
	else if(leftlose2 == 1 && rightlose2 == 0)
	{				    
		ccd2_center = (rightbreak2 + rightbreak2-97) / 2;
    }
	
	//-------------------Ë«ÏßÑ­¼£------------------------//
	
	else if(leftlose2 == 0 && rightlose2 == 0)
	{
        if(leftbreak2 < rightbreak2)      //µ±×óÌø±äµãÐ¡ÓÚÓÒÌø±äµãÊ±²Å¸Ä±ä¶æ»ú½Ç¶È
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

