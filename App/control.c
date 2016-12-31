#include "include.h"


float OutData[4] = { 0 };                                    //SCIʾ��������

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
int CS=0;//����ʱ�̳�ʼ��Ϊ0��ÿ�����������ۼ�1��
int Cp=0; //����ʱ�̳�ʼ��Ϊ0��ÿ�����������ۼ�1��
int CK1=3;////���ٿ��Ʊ���ϵ��
int CK2=2;//Ϊ�������ϵ��
int zhidaomax=120;//135
int zhidaomin=115;//110
int wandaomax=115;//100
int wandaomin=105;//90
int L2_flag=0;
int L2_Lflag=0;
int R2_flag=0;
int R2_Rflag=0;
int Left_mark;
int Right_mark;
int speedflag=0;
int a,f;
int shizhi=0;
int CCD1_Offset = 80;

int  Max_Value;
int startline_time=0;
int Stopflag=0;
int cont_flag=0;
int startline_flag=0;
extern int timecount;

float ratio;
float P=0;//2.99
float D=0;//2.

int L1count,R1count;

int b=0,bpre=0;
int timedelay;
int qipao=0;
int ZA_flag=0;
int ZAline=0;
int ZA=0;
uint8 acc_flag=0;
uint16  looptemp;
uint16  looptemp2;
int8 temp;
unsigned char temp4;
unsigned char  temp5;
int  looptemp3;
unsigned char   looptemp4;
extern int speedset;

int breakpoint1;     //���䷧ֵ

int breakpoint3;


int error1=0,errorpre1=0,errorppre1=0;

int error=0,errorpre=0,FWerror=0, FWerrorpre=0;

int zhongxian1=64,zhongxianpre1=64,zhongxianppre1=64,zhongxian2=64;
int wide=45;
int  zhongxian=64,zhongxianpre=64;
int m=0;//�ڵ�
int n[128];//�ڰ׶�ֵ��
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
*  ��һ��
*/
//**************************************************************************
void ccd2_deall(uint8*ccd1_array)
{
  L1count=0;
  R1count=0;
  unsigned char k,i,j,l,p;
  Ryuzhi=25;

  for(k=0; k<128; k++)
  {

    ccd2_data[k]=ccd1_array[k];
    ccd2_datagy[k]=ccd1_array[k];

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




    //-------------------------�������������Сֵ--------------------------//

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



  //-------------------------�������������Сֵ--------------------------//

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


  //-----------------------��һ������-------------------------//


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
    if(ccd1_array[i]>CCD1_Offset)
    {
      tempp=ccd1_array[i]-CCD1_Offset;
    }
    else
    {
      tempp=0;
    }
    ccd1_array[i]=(int)(tempp*ratio);


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

    if (ccd1_array[i]<50)

    {
      m++;
      n[i]=0;
    }
    else
    {
      n[i]=1;
    }
  }


  if (m>124)
  {

    Stopflag=1;

  }



  else
  {
    Stopflag=0;

  }

  /*------------------------------��һ������end------------------------*/





  breakpoint1 =(char)(((float)Ryuzhi/100)*(ccd2_data[1]+ccd2_data[3]+ccd2_data[7]+ccd2_data[9])/4.0);
  breakpoint3 =(char)(((float)Ryuzhi/100)*(ccd2_data[temp4+1]+ccd2_data[temp4+3]+ccd2_data[temp4+7]+ccd2_data[temp4+9])/4.0);

  /**********************************�����޷�****************************/

  if(zhongxianpre1>115)
  {
    zhongxianpre1=115;
  }
  if(zhongxianpre1<0)
  {
    zhongxianpre1=0;
  }

  /******************������********************************/
  for (j=zhongxianpre1;j<115;j++)
  {

    if(ccd1_array[j]-ccd1_array[j+1]>=breakpoint3 )
    {
      if(ccd1_array[j]-ccd1_array[j+2]>=breakpoint3)
      {

        if(ccd1_array[j]-ccd1_array[j+3]>=breakpoint3)
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

  /************************�����޷�***************************/

  if(zhongxianpre1<15)
  {
    zhongxianpre1=15;
  }
  if(zhongxianpre1>127)
  {
    zhongxianpre1=127;
  }


  /******************������***************************/



  for (i=zhongxianpre1;i>15;i--)
  {


    if(ccd1_array[i]-ccd1_array[i-1]>=breakpoint1)
    {
      if(ccd1_array[i]-ccd1_array[i-2]>=breakpoint1)
      {
        if(ccd1_array[i]-ccd1_array[i-3]>=breakpoint1)

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










  /******************�����߶�**************************/



  for (l=115;l>15;l--)
  {


    if(ccd1_array[l]-ccd1_array[l-1]>=breakpoint1)
    {
      if(ccd1_array[l]-ccd1_array[l-2]>=breakpoint1)
      {
        Left_mark=l;
        if(ccd1_array[l]-ccd1_array[l-3]>=breakpoint1)

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
  /*************************�����߶�******************/



  for (p=15;p<115;p++)
  {

    if(ccd1_array[p]-ccd1_array[p+1]>=breakpoint3 )
    {
      if(ccd1_array[p]-ccd1_array[p+2]>=breakpoint3)

      {
        Right_mark=p;
        if(ccd1_array[p]-ccd1_array[p+3]>=breakpoint3)

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





  /***************************�����������***************************/
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


  /**********************������/ZA**********************************/

  cont_flag=0;
  for(p=20;p<107;p++)
  {
    if(n[p]==1 && n[p+1]==1 && n[p+2]==1 && n[p+3]==1 && n[p+4]==1)// && n[p+4]==1 && n[p+5]==1 && n[p+6]==1)
    {
      cont_flag=1;
      break;
    }
  }






  if(R1_Rflag==1&&L1_Lflag==1&&L2_Lflag==1&&R2_Rflag==1&&R2_flag<L2_flag&&L2_flag<90&&R2_flag>38)
  {

    for (j=R1_flag;j<100;j++)
    {
      //elta_line = ccd1_array[j]-ccd1_array[j];
      if(ccd1_array[j]-ccd1_array[j-1]>=breakpoint1 &&ccd1_array[j]-ccd1_array[j-2]>=breakpoint1&&ccd1_array[j]-ccd1_array[j-3]>=breakpoint1)
      {
        L1count++;
      }
      if(ccd1_array[j-1]-ccd1_array[j]>=breakpoint3&&ccd1_array[j-2]-ccd1_array[j]>=breakpoint3&&ccd1_array[j-3]-ccd1_array[j]>=breakpoint3 )
      {
        R1count++;
      }
    }

    for (i=L1_flag;i>35;i--)
    {
      if(ccd1_array[i]-ccd1_array[i-1]>=breakpoint1&&ccd1_array[i]-ccd1_array[i-2]>=breakpoint1&&ccd1_array[i]-ccd1_array[i-3]>=breakpoint1)
      {
        L1count++;
      }
      if(ccd1_array[i-1]-ccd1_array[i]>=breakpoint3&&ccd1_array[i-2]-ccd1_array[i]>=breakpoint3&&ccd1_array[i-3]-ccd1_array[i]<=breakpoint3)
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
      //TC_flag=1;
    }




  }


  /*------------------------------������------------------------*/



  zhongxianpre1=zhongxian1;











  /****************wide***********************/
  //ZAline=��R2_flag+L2_flag)/2;


  /*********************����miserror1*************************/





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
    miszhongxian1=64;//64

  }

  miserror1=64-miszhongxian1;





  /*********************����miserror2*************************/



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

  /***************************�ϰ���****************/





  /************************�µ�***********************/



  /*************��������һ�������Ҷ���*************************/

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

  /**********************��������һ�쳣���Ҷ���*****************/




  if (L1_Lflag==0&&R1_Rflag==1&&miserror1<0)
  {
    zhongxian1=R1_flag-s;//25


    f=-1;

  }

  if (L1_Lflag==1&&R1_Rflag==0&&miserror1>0)
  {zhongxian1=L1_flag+s;//30



  f=-1;
  }





  /************************�ڶ������ߵ�����������************************/





  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==1&&R2_Rflag==0&&miserror2<0)
  {zhongxian1=L2_flag+s;//25


  f=1;
  }




  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==0&&R2_Rflag==1&&miserror2>0)
  {
    zhongxian1=R2_flag-s;//25

    f=1;
  }



  /************************�ڶ������ߵ����쳣����************************/




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











  /***************************�ϰ������ܷ�����*********************/
  /*
  if (L1_Lflag==1&&R1_Rflag==1&&L2_Lflag==1&&R2_Rflag==1&&wide<=35&&wide>0&&bpre<12&&bpre>-12)
  {

  zhongxian1=zhongxianpre1;//25


  f=1;

}







  /********************��ʮ����ȫ����*****************/






  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==0&&R2_Rflag==0&&m>5)
  {

    zhongxian1=zhongxianpre1;//25


    f=1;


  }





  /*********************ʮ��*****************************/





  if (L1_Lflag==0&&R1_Rflag==0&&L2_Lflag==0&&R2_Rflag==0&&m<=5)


  {       zhongxian1=64;//25

  f=1;
  }







  /*********************��������һ����ͬʱ�ҵ�������***************/





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
      zhongxian1=R2_flag-15;//���ϰ�
    }
    if(ZAline<64)
    {
      zhongxian1=L2_flag+15;//���ϰ�
    }

    f=1;


  }




  /***************************ͻ��*************************************/


  if (change1==1)


  {       zhongxian1=zhongxianpre1;//25

  f=1;
  }



  /****************************���ߴ������*******************************/



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

  LCD_Show_Number (1,4,error);





  sd=95;


  /***************************Runout*******************/



  if  (Stopflag==1)

  {
    speedset=0;
  }

  if(timecount>=100 && cont_flag==0  && abs(DZerror[3])<=7)
  {
    startline_flag=1;
  }

  if (startline_flag == 1)
  {
      startline_time++;
      if(startline_time>=100)
      {
       speedset=0;
      }
  }

  /***********************************�ֶ�P.D****************************/
  if(error1<=5)
  {
    P=8;
    D=0;
//    acc_flag=1;
  }
  else
  {
//    acc_flag=0;
  }
//  else
//  {
//
//
//
//
//
//    P=10;//15
//    D=7; //5
//
//
//    }
  P=18;
  D=20;

//  FWerror=(8*DZerror[1]+3*DZerror[2]+2*DZerror[3]+DZerror[4])/14;
  FWerror=error;





  /*
  LCD_Show_Number(6,2,pd);
  LCD_Show_Number(6,3,podao);

  LCD_Show_Number(6,4,qipao);
  LCD_Show_Number(6,5,ZA_flag);

  LCD_Show_Number(6,6,TC_flag);

  */

  DPWM=3290-P*FWerror-D*(DZerror[1]-DZerror[2]);



  /*****************��������޷�****************/


  if(DPWM>3880)
  {
    DPWM=3880;//�ұ�
  }

  if(DPWM<2650)
  {
    DPWM=2650;//���
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
  uint8 cmdf[2] = {CMD_CCD, ~CMD_CCD};    //��ͷ����
  uint8 cmdr[2] = {~CMD_CCD, CMD_CCD};    //��β����

  uart_putbuff(UART4, cmdf, sizeof(cmdf));    //�ȷ�������

  uart_putbuff(UART4, ccdaddr, ccdsize); //�ٷ���ͼ��

  uart_putbuff(UART4, cmdr, sizeof(cmdr));    //�ٷ�������
}





/*!
*	@since		v5.0
*	@note		ɽ�� ����CCD ����ʵ��
�޸� PIT0 �Ķ�ʱʱ�伴���޸��ع�ʱ��
*/
void  CCD_Get(void)
{

  tsl1401_get_img();

}
