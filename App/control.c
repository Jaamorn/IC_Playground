#include "include.h"


float OutData[4] = { 0 };                                    //SCI示波器参数

uint8 CCD_BUFF[TSL1401_MAX*3][TSL1401_SIZE];
int sum;
int v;
int c=0;
int podao=0;
int pd=0;
int change1=0;
int lv,rv;
int L1_flag=0;
int R1_flag=0;
int L1_Lflag=0;
int R1_Rflag=0;
int s=23;
int CS=0;//入弯时刻初始化为0，每个控制周期累加1。
int Cp=0; //出弯时刻初始化为0，每个控制周期累加1。
int CK1=3;////减速控制比例系数
int CK2=2;//为弯道加速系数
int zhidaomax=120;//135
int zhidaomin=115;//110
int wandaomax=115;//100
int wandaomin=105;//90
int L2_flag=0;
int L2_Lflag=0;
int R2_flag=0;
int R2_Rflag=0;
int speedflag=0;
int a,f;
int shizhi=0;
int CCD1_Offset = 90;

int  Max_Value;
int TC_flag=0;
int Stopflag=0;

float ratio;
float P=0;//2.99
float D=0;//2.

int L1count,R1count;

int b=0,bpre=0;  
int qipao=0;
int ZA_flag=0;
int ZAline=0;
int ZA=0;
uint16  looptemp;
uint16  looptemp2;
int8 temp;
unsigned char temp4;
unsigned char  temp5;
int  looptemp3;
unsigned char   looptemp4;


int breakpoint1;     //跳变阀值

int breakpoint3;


int error1=0,errorpre1=0,errorppre1=0;

int error=0,errorpre=0,FWerror=0, FWerrorpre=0;

int zhongxian1=64,zhongxianpre1=64,zhongxianppre1=64,zhongxian2=64;
int wide=45;
int  zhongxian=64,zhongxianpre=64;
int m=0;
int sd=95;
int miszhongxian1=64,miszhongxian2=64;
int miserror1=0,miserror2=0;
int tempp;
int16 DPWMpre,DPWM;
uint8 ccd1_data[128];
uint8 ccd1_datagy[128];
uint8 ccd2_data[128];
uint8 ccd2_datagy[128];
int   DZerror[5];

int   Ryuzhi;
int8  B1; 
int8  B3; 

//-----------------------ccd2----------------------//

/*
*  归一化
*/
//**************************************************************************
void ccd2_deall(uint8*ccd2_array)
{
  L1count=0;
  R1count=0;
  unsigned char k,i,j,l,p;
  Ryuzhi=35;
  
  for(k=0; k<128; k++)
  {
    
    ccd2_data[k]=ccd2_array[k];    
    ccd2_datagy[k]=ccd2_array[k];
    
  }   
  
  
  
  if(zhongxianpre1<0)
  {
    temp4=0;
  }
  else if(zhongxianpre1>117)
  {temp4=117;
  }
  
  else
  {
    temp4=(unsigned char) zhongxianpre1;
  }
  
  if(temp4!=0)
  {
    for(i=0; i<5; i++)
      
    {
      for(j=i+1; j<zhongxianpre1; j++)
      {
        if(ccd2_data[j] > ccd2_data[i])
        {
          temp = ccd2_data[i];
          ccd2_data[i] = ccd2_data[j];
          ccd2_data[j] =temp;
          
        }
      }
    }
    
    
    
    
    //-------------------------求出中线往左最小值--------------------------//
    
    for(i=5; i<10; i++)
    {
      for(j=i+1; j<zhongxianpre1; j++)
      {
        if(ccd2_data[j] < ccd2_data[i])
        {
          temp = ccd2_data[i] ;
          ccd2_data[i] = ccd2_data[j];
          ccd2_data[j] = temp;
          
        }
      }
    }
  }
  
  
  for(i=temp4; i<temp4+5; i++)
    
  {
    for(j=i+1; j<128; j++)
    {
      if(ccd2_data[j] > ccd2_data[i])
      {
        temp = ccd2_data[i];
        ccd2_data[i] = ccd2_data[j];
        ccd2_data[j] =temp;
        
      }
    }
  }
  
  
  
  //-------------------------求出中线往右最小值--------------------------//
  
  for(i=temp4+5; i<temp4+10; i++)
  {
    for(j=i+1; j<128; j++)
    {
      if(ccd2_data[j] < ccd2_data[i])
      {
        temp = ccd2_data[i] ;
        ccd2_data[i] = ccd2_data[j];
        ccd2_data[j] = temp;
        
      }
    }
  }
  
  
  //-----------------------归一化处理-------------------------//
  
  
  if(ccd2_data[0]>ccd2_data[temp4])        
  {
    Max_Value=ccd2_data[0];
  }
  else
  {
    Max_Value=ccd2_data[temp4];
  }
  
  if(Max_Value>CCD1_Offset) 
  {
    ratio=127.0/(Max_Value-CCD1_Offset);
  } 
  else 
  {
    ratio=0;
  } 
  for(i=0;i<128;i++) 
  {
    if(ccd2_array[i]>CCD1_Offset) 
    {
      tempp=ccd2_array[i]-CCD1_Offset; 
    } 
    else
    {
      tempp=0; 
    }
    ccd2_array[i]=(int)(tempp*ratio);
    
    
  }
  
  if(temp4!=0)
  {
    for(i=0;i<10;i++) 
    {
      if(ccd2_data[i]>CCD1_Offset) 
      {
        tempp=ccd2_data[i]-CCD1_Offset; 
      } 
      else
      {
        tempp=0; 
      }
      ccd2_data[i]=(int)(tempp*ratio);
      
      
    }
  }
  
  
  for(i=temp4;i<temp4+10;i++) 
  {
    
    if(ccd2_data[i]>CCD1_Offset) 
    {
      tempp=ccd2_data[i]-CCD1_Offset; 
    } 
    else
    {
      tempp=0; 
    }
    ccd2_data[i]=(int)(tempp*ratio);
    
    
    
  }
  
  
  
  
  /*****************************************Runout*******************************/
  
  m=0;
  
  
  for(i=0;i<127;i++)
  { 
    
    if (ccd2_datagy[i]<140)
      
    {  m++;
    
    
    }
  }
  
  
  if (m>120)
  {
    
    Stopflag=1;
    
  }
  
  
  
  else
  {
    Stopflag=0;
    
  }
  
  /*------------------------------归一化处理end------------------------*/		
  
  
  
  
  
  breakpoint1 =(char)(((float)Ryuzhi/100)*(ccd2_data[1]+ccd2_data[3]+ccd2_data[7]+ccd2_data[9])/4.0);
  breakpoint3 =(char)(((float)Ryuzhi/100)*(ccd2_data[temp4+1]+ccd2_data[temp4+3]+ccd2_data[temp4+7]+ccd2_data[temp4+9])/4.0);
  
  /**********************************中线限幅****************************/
  
  if(zhongxianpre1>115)
  {
    zhongxianpre1=115;
  }
  if(zhongxianpre1<0)
  {
    zhongxianpre1=0;
  }
  
  /******************找右线********************************/
  for (j=zhongxianpre1;j<115;j++)
  {
    
    if(ccd2_array[j]-ccd2_array[j+1]>=breakpoint3 )
    {
      if(ccd2_array[j]-ccd2_array[j+2]>=breakpoint3)
      {
        
        if(ccd2_array[j]-ccd2_array[j+3]>=breakpoint3)
        {
          
          
          R1_flag=j;
          R1_Rflag=1;
          break;
          
        }
        
      }
      
      
    }
    else
    {
      R1_Rflag=0;  
      R1_flag=0;//wenti
      
      
    }
  }
  
  /************************中线限幅***************************/
  
  if(zhongxianpre1<15)
  {
    zhongxianpre1=15;
  }
  if(zhongxianpre1>127)
  {
    zhongxianpre1=127;
  }
  
  
  /******************找左线***************************/
  
  
  
  for (i=zhongxianpre1;i>15;i--)
  {
    
    
    if(ccd2_array[i]-ccd2_array[i-1]>=breakpoint1)
    {
      if(ccd2_array[i]-ccd2_array[i-2]>=breakpoint1)
      { 
        if(ccd2_array[i]-ccd2_array[i-3]>=breakpoint1)
          
        {
          
          L1_flag=i;
          L1_Lflag=1;
          
          break;
        }          
      } 
      
    }
    
    
    else
    {
      L1_Lflag=0;
      L1_flag=0;
      
    }
    
    
  }
  
  
  
  
  
  
  
  
  
  
  /******************找左线二**************************/
  
  
  
  for (l=115;l>15;l--)
  {
    
    
    if(ccd2_array[l]-ccd2_array[l-1]>=breakpoint1)
    {
      if(ccd2_array[l]-ccd2_array[l-2]>=breakpoint1)
      { 
        if(ccd2_array[l]-ccd2_array[l-3]>=breakpoint1)
          
        {
          
          L2_flag=l;
          L2_Lflag=1;
          
          break;
        }          
      }
    }
    
    
    else
    {
      L2_Lflag=0;
      L2_flag=0;
      
    }
    
    
  }
  /*************************找右线二******************/
  
  
  
  for (p=15;p<115;p++)
  {
    
    if(ccd2_array[p]-ccd2_array[p+1]>=breakpoint3 )
    {
      if(ccd2_array[p]-ccd2_array[p+2]>=breakpoint3)
        
      { 
        if(ccd2_array[p]-ccd2_array[p+3]>=breakpoint3)
          
        { 
          
          R2_flag=p;
          R2_Rflag=1;
          break;
          
          
        }
      }
      
    }
    else
    {
      R2_Rflag=0;  
      R2_flag=0;
      
    }
  }
  
  
  
  
  
  /***************************计算赛道宽度***************************/
  if (L1_Lflag==1&&R1_Rflag==1)
  {
    
    wide=R1_flag-L1_flag;
    
  }
  else if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==1&&R2_Rflag==1&&R2_flag>L2_flag)
  {
    wide=R2_flag-L2_flag;
  }
  else 
  {
    
    wide=0;
  }
  
  
  /**********************起跑线/ZA**********************************/ 
  
  
  
  if(R1_Rflag==1&&L1_Lflag==1&&L2_Lflag==1&&R2_Rflag==1&&R2_flag<L2_flag&&L2_flag<90&&R2_flag>38)
  {
    
    for (j=R1_flag;j<100;j++)
    {
      //elta_line = ccd1_array[j]-ccd1_array[j];
      if(ccd2_array[j]-ccd2_array[j-1]>=breakpoint1 &&ccd2_array[j]-ccd2_array[j-2]>=breakpoint1&&ccd2_array[j]-ccd2_array[j-3]>=breakpoint1)
      {
        L1count++;
      }        
      if(ccd2_array[j-1]-ccd2_array[j]>=breakpoint3&&ccd2_array[j-2]-ccd2_array[j]>=breakpoint3&&ccd2_array[j-3]-ccd2_array[j]>=breakpoint3 )                            
      {
        R1count++;
      }                           
    }              
    
    for (i=L1_flag;i>35;i--)
    {
      if(ccd2_array[i]-ccd2_array[i-1]>=breakpoint1&&ccd2_array[i]-ccd2_array[i-2]>=breakpoint1&&ccd2_array[i]-ccd2_array[i-3]>=breakpoint1)
      {
        L1count++;
      }  
      if(ccd2_array[i-1]-ccd2_array[i]>=breakpoint3&&ccd2_array[i-2]-ccd2_array[i]>=breakpoint3&&ccd2_array[i-3]-ccd2_array[i]<=breakpoint3)
      {                     
        R1count++;                                                         
      }
    }
    
    
    
    
    if(R1count>=2&&L1count>=2&&qipao==0)
    {  
      
      qipao=1;
    }
    
    
    if(R1count>=2&&L1count>=2&&qipao==2)
    {  
      
      
      
      //  DELAY_MS(150);
      ZA_flag=1;     
    }
    
    if(R1count>=2&&L1count>=2&&qipao==3&&wide<30)
    {  
      
      
      
      //  DELAY_MS(150);
      TC_flag=1;     
    }
    
    
    
    
  }
  
  
  /*------------------------------处理线------------------------*/	
  
  
  
  zhongxianpre1=zhongxian1;
  
  
  
  
  
  
  
  
  
  LCD_Show_Number(6,4,DPWM);
  
  
  /****************wide***********************/
  //ZAline=（R2_flag+L2_flag)/2;
  
  
  /*********************计算miserror1*************************/
  
  
  
  
  
  if (L1_Lflag==0&&R1_Rflag==1)
  {
    
    
    miszhongxian1=R1_flag-s;
    
    
    
  }
  
  
  else  if (L1_Lflag==1&&R1_Rflag==0)
    
  {   
    
    miszhongxian1=L1_flag+s;
    
    
    
    
  }
  else
  {
    miszhongxian1=64;
    
  }
  
  miserror1=64-miszhongxian1;
  
  
  
  
  
  /*********************计算miserror2*************************/
  
  
  
  if (L2_Lflag==0&&R2_Rflag==1)
  {
    
    
    miszhongxian2=R2_flag-s;
    
    
    
  }
  
  
  else   if (L2_Lflag==1&&R2_Rflag==0)
  {   
    
    miszhongxian2=L2_flag+s;
    
    
    
    
  }
  else
    
    
  {
    
    miszhongxian2=64;
  }
  
  
  miserror2=64-miszhongxian2;
  
  /***************************障碍物****************/
  
  
  
  
  
  /************************坡道***********************/
  
  
  
  /*************处理找线一正常左右丢线*************************/
  
  if (L1_Lflag==0&&R1_Rflag==1&&miserror1>0)
  {
    zhongxian1=R1_flag-s;
    
    
    f=1;
  }
  if (L1_Lflag==1&&R1_Rflag==0&&miserror1<0)
  {   
    zhongxian1=L1_flag+s;//30
    
    
    
    f=1;
  }
  
  /**********************处理找线一异常左右丢线*****************/
  
  
  
  
  if (L1_Lflag==0&&R1_Rflag==1&&miserror1<0)
  {
    zhongxian1=R1_flag-s;//25
    
    
    f=-1;
    
  }
  
  if (L1_Lflag==1&&R1_Rflag==0&&miserror1>0)
  {zhongxian1=L1_flag+s;//30
  
  
  
  f=-1;
  }
  
  
  
  
  
  /************************第二次找线单线正常处理************************/
  
  
  
  
  
  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==1&&R2_Rflag==0&&miserror2<0)
  {zhongxian1=L2_flag+s;//25
  
  
  f=1;
  }
  
  
  
  
  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==0&&R2_Rflag==1&&miserror2>0)
  {
    zhongxian1=R2_flag-s;//25
    
    f=1;
  }
  
  
  
  /************************第二次找线单线异常处理************************/
  
  
  
  
  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==1&&R2_Rflag==0&&miserror2>0)
  {
    zhongxian1=L2_flag+s;//25
    
    f=-1;
  }
  
  
  
  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==0&&R2_Rflag==1&&miserror2<0)
  {
    zhongxian1=R2_flag-s;//25
    
    f=-1;
  }
  
  
  
  
  
  
  
  
  
  
  
  /***************************障碍物起跑防干扰*********************/
  /*
  if (L1_Lflag==1&&R1_Rflag==1&&L2_Lflag==1&&R2_Rflag==1&&wide<=35&&wide>0&&bpre<12&&bpre>-12)
  {
  
  zhongxian1=zhongxianpre1;//25
  
  
  f=1;
  
}
  
  
  
  
  
  
  
  /********************非十字完全丢线*****************/
  
  
  
  
  
  
  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==0&&R2_Rflag==0&&m>5)
  {
    
    zhongxian1=zhongxianpre1;//25
    
    
    f=1;
    
    
  }
  
  
  
  
  
  /*********************十字*****************************/
  
  
  
  
  
  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==0&&R2_Rflag==0&&m<=5)
    
    
  {       zhongxian1=64;//25
  
  f=1;
  }
  
  
  
  
  
  
  
  /*********************处理找线一正常同时找到左右线***************/
  
  
  
  
  
  //if(L1_Lflag==1&&R1_Rflag==1&&wide>45&&wide<65)
  
  if(L1_Lflag==1&&R1_Rflag==1&&ZA_flag==0&&wide>40)
  {
    
    
    zhongxian1=(R1_flag+L1_flag)/2; 
    
    
    f=1;
  } 
  
  
  
  if (ZA_flag==1)
  {
    
    ZAline=(R2_flag+L2_flag)/2;
    if(ZAline>=64)
    {
      zhongxian1=R2_flag-15;//右障碍
    }
    if(ZAline<64)
    {
      zhongxian1=L2_flag+15;//右障碍
    }
    
    f=1; 
    
    
  }
  
  
  
  
  /***************************突变*************************************/
  
  
  if (change1==1)
    
    
  {       zhongxian1=zhongxianpre1;//25
  
  f=1;
  }
  
  
  
  /****************************找线处理结束*******************************/
  
  
  
  
  
  
  
  
  error1=64-zhongxian1;
  
  
  
  
  
  
}





void shuchu()   
{
  error1=abs(error1);
  
  
  
  
  error=64-zhongxian;
  
  
  b=f*error;
  
  
  
  
  
  
  
  
  
  
  DZerror[4]=DZerror[3];
  DZerror[3]=DZerror[2];
  DZerror[2]=DZerror[1];
  
  DZerror[1]=error;
  
  
  
  
  
  
  
  sd=95;
  
  /*******************输出速度（差速）*************************/
  
  
  
  /***************************Runout*******************/  
  
  
  
  if  (Stopflag==1)
    
  {
    lv=0;
    rv=0;
    
  }
  
  if(TC_flag==1)
    
  {
    lv=0;
    rv=0;
    
  }
  
  /***********************************分段P.D****************************/
  if(error1<=5)
  {
    
    
    P=1;
    D=0;
    
    
    
  }
  else 
  {
    
    
    
    
    
    P=15;//2.6//3//3 2  104
    D=1.5; //2.5//1.8
    
    
    
    
    
    
    
    
    
    
    
    
  }
  
  FWerror=(6*DZerror[1]+2*DZerror[2]+DZerror[3]+DZerror[4])/10;
  
  
  
  
  
  
  /*
  LCD_Show_Number(6,2,pd);
  LCD_Show_Number(6,3,podao);
  
  LCD_Show_Number(6,4,qipao);
  LCD_Show_Number(6,5,ZA_flag);
  
  LCD_Show_Number(6,6,TC_flag);
  
  */
  
  DPWM=3160-P*f*FWerror-D*f*(DZerror[1]-DZerror[2]);
  
  
  
  /*****************舵机左右限幅****************/
  
  
  if(DPWM>3900)
  {
    DPWM=3900;//右边
  }
  
  if(DPWM<2400)
  {
    DPWM=2400;//左边
  }
  /*********************************/
  
  
  
  ftm_pwm_duty(FTM1, FTM_CH0, DPWM);
  
  
  
  FWerrorpre=FWerror;
  
  errorpre=error;
  bpre=b;
  
  
  
  c=error-FWerrorpre;
  c=abs(c);
  
  
  if (c>=25)
    
    
  {
    
    
    change1=1;
    
    
    
  }
  else
  {
    
    
    
    change1=0;
    
    
    
    
  }
  
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
    uart_putchar(UART4,databuf[i]);
}

void SEND(float a,float b,float c,float d)
{       
  OutData[0] = a;                   
  OutData[1] = b;                    
  OutData[2] = c;                
  OutData[3] = d;
  OutPut_Data();
}







void vcan_sendccd(uint8 *ccdaddr, uint32 ccdsize)
{
#define CMD_CCD     2
  uint8 cmdf[2] = {CMD_CCD, ~CMD_CCD};    //开头命令
  uint8 cmdr[2] = {~CMD_CCD, CMD_CCD};    //结尾命令
  
  uart_putbuff(UART4, cmdf, sizeof(cmdf));    //先发送命令
  
  uart_putbuff(UART4, ccdaddr, ccdsize); //再发送图像
  
  uart_putbuff(UART4, cmdr, sizeof(cmdr));    //再发送命令
}





/*!
*	@since		v5.0
*	@note		山外 线性CCD 测试实验
修改 PIT0 的定时时间即可修改曝光时间
*/
void  CCD_Get(void)
{
  
  tsl1401_get_img();
  
}

