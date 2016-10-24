/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       VCAN_TSL1401.c
 * @brief      ����CCD������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2014-01-04
 */

#include "common.h"
#include "math.h"
#include <stdarg.h>
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "MK60_adc.h"
#include "MK60_pit.h"
#include "VCAN_TSL1401.h"


static volatile tsl1401_status_e  tsl1401_flag = tsl1401_noint;
static volatile uint32            tsl1401_time;


 void tsl1401_restet();
static void tsl1401_delay(volatile uint32 time );
static void tsl1401_gather(void);
uint8 ccd1_array[138] = {0};  //ccd1 128�����ص�ѹ
uint8 ccd2_array[138] = {0};  //ccd2 128�����ص�ѹ
/*//-----------------------ccd1----------------------//

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
*/



//extern  void PIT1_IRQHandler(void);


//����CCD�ܽ�
//                          CCD1            CCD2        CCD3

#if 1             //����������ѡ��ͬ���ź�����
ADCn_Ch_e   tsl1401_ch[] = {ADC0_SE9,     ADC0_DP0,    ADC1_DP1};       //CCD���õ� ADCͨ��(ԭʼ�ź�)
#else
ADCn_Ch_e   tsl1401_ch[] = {ADC0_SE8,     ADC0_DM0,    ADC1_DM1};       //CCD���õ� ADCͨ��(�Ŵ��ź�)
#endif
PTXn_e      tsl1401_si[] = {PTE6 ,        PTE8 ,       PTE10};          //CCD���õ� SI�ܽ�
PTXn_e      tsl1401_clk[] = {PTE7 ,        PTE9 ,       PTE12};         //CCD���õ� SI�ܽ�
PTXn_e      tsl1401_led[] = {PTA8,        PTA9 ,       PTD15};         //CCD ���� ���õ� LED_EN �ܽ�

//CCD �ɼ������Ĵ洢�ռ�ָ������
uint8 *tsl1401_addr[TSL1401_MAX] = {0};                                 //CCD �ɼ�ͼ��ĵ�ַ����




/*********************************************************************************/

void ccd_init(void)
{
       
	   adc_init(tsl1401_ch[0]); //��ʼ�� CCD ���õ� ADCͨ��
	   
			  //��ʼ�� CCD �Ŀ��ƹܽ� CLK �� SI
			  gpio_init (tsl1401_clk[0], GPO, 0);
			  gpio_init (tsl1401_si[0] , GPO, 0);
		  
	   
		  
		  tsl1401_restet(); 					  //  ������һ֡�����ݣ��ҵģ�
       

}



/*********************************************************************************/

void ccd1_sample(void)
{
	uint8 i;
   
    //tsl1401_delay(TSL1401_TIME);
    tsl1401_si_out(1);              //SI  = 1
    //tsl1401_delay(TSL1401_TIME);
    tsl1401_clk_out(1); 
	tsl1401_si_out(0); 
	/*si1 = 1;
	clk1 = 1;
	si1 = 0;*/
	
	for(i=0; i<128; i++)
	{	
		tsl1401_clk_out(0); //clk1 = 0;
		ccd2_delay();
		
		ccd1_array[i] = ad_ave(tsl1401_ch[0], ADC_8bit,6);
		tsl1401_clk_out(1); //clk1 = 1;
		ccd2_delay();
	}
	
	tsl1401_clk_out(0); //clk1 = 0;
     /*for(i=0;i<128;i++)
        {
              if(i>120) ccd1_array[i+7]=ccd1_array[127];
              if(ccd1_array[i]>(ccd1_array[i+1]+3)&&ccd1_array[i+7]>(ccd1_array[i+1]+3))
                ccd1_array[i+1]=(ccd1_array[i]+ccd1_array[i+7])/2;
              else if(ccd1_array[i]<(ccd1_array[i+1]-3)&&ccd1_array[i+7]<(ccd1_array[i+1]-3))
                ccd1_array[i+1]=(ccd1_array[i]+ccd1_array[i+7])/2;
        }*/

}

/*********************************************************************************/

void ccd_delay(void)
{
	uint8 i;
	for(i=5; i>0; i--)
		asm("nop");
}


void ccd2_delay(void)
{
	uint8 i;
	for(i=5; i>0; i--)
		asm("nop");
}

//ȫ��CCD��SI�ܽ��������
void tsl1401_si_out(uint8 data)
{
    uint8   i = TSL1401_MAX;

    ASSERT(data < 2);

    while(i--)
    {
        gpio_set(tsl1401_si[i], data);
    }
}

//ȫ��CCD��CLK�ܽ��������
void tsl1401_clk_out(uint8 data)
{
    uint8   i = TSL1401_MAX;

    ASSERT(data < 2);

    while(i--)
    {
        gpio_set(tsl1401_clk[i], data);
    }
}

//tsl1401,����CCD��ʼ����timeΪ�ع�ʱ�䣬��λ�� TSL1401_INT_TIME ������һ�£���ʼ����������tsl1401_set_addrs �����ô洢��ַ�����������ô洢ͼ���ַ��
void tsl1401_init(uint32 time)
{
    uint8 i = TSL1401_MAX;

    while(i)
    {
        i--;
        adc_init(tsl1401_ch[i]); //��ʼ�� CCD ���õ� ADCͨ��

        //��ʼ�� CCD �Ŀ��ƹܽ� CLK �� SI
        gpio_init (tsl1401_clk[i], GPO, 0);
        gpio_init (tsl1401_si[i] , GPO, 0);
    }

    tsl1401_time = time;
    TSL1401_INT_TIME(tsl1401_time);         //  �����ж�ʱ��

    tsl1401_restet();                       //  ������һ֡�����ݣ��ҵģ�
}

//���òɼ�ͼ��ĵ�ַ
//num Ϊ TSL1401_MAX ʱ������ȫ����ַ����������ָ���ĵ�ַ
void tsl1401_set_addrs(TSL1401_e num, uint8 *addr, ... )
{
    ASSERT(num <= TSL1401_MAX);

    va_list ap;                                 //����ջָ��ap
    uint8_t *value;
    uint8_t n = 0;

    if(num < TSL1401_MAX)
    {
        tsl1401_addr[num] = addr;
    }
    else if(num == TSL1401_MAX)
    {
        va_start(ap, addr);                         //��ȡ�ɱ�����б�ĵ�һ�������ĵ�ַ
        value = addr;
        while(num--)
        {
            tsl1401_addr[n++] = value;
            value = va_arg(ap, uint8_t * );             //��ȡ�ɱ�����ĵ�ǰ����������ָ�����Ͳ���ָ��ָ����һ����
        }

        va_end(ap);                                 //���va_list�ɱ�����б�
    }
}


//num Ϊ TSL1401_MAX ʱ������ȫ��LED����������Ӧ�� LED
void tsl1401_led_en(TSL1401_e num)
{
    ASSERT(num <= TSL1401_MAX);

    if(num < TSL1401_MAX)
    {
        gpio_init(tsl1401_led[num], GPO, 0);
    }
    else if(num == TSL1401_MAX)
    {

        while(num--)
        {
            gpio_init(tsl1401_led[num], GPO, 0);
        }
    }
}

//num Ϊ TSL1401_MAX ʱ������ȫ��LED����������Ӧ�� LED
void tsl1401_led_dis(TSL1401_e num)
{
    ASSERT(num <= TSL1401_MAX);

    if(num < TSL1401_MAX)
    {
        gpio_init(tsl1401_led[num], GPI, 1);
    }
    else if(num == TSL1401_MAX)
    {

        while(num--)
        {
            gpio_init(tsl1401_led[num], GPI, 1);
        }
    }
}

//tsl1401,����CCD�ɼ�ͼ��
void tsl1401_get_img(void)
{
    //tsl1401_restet();
    tsl1401_flag = tsl1401_start;           //�����ɼ�
    while(tsl1401_flag != tsl1401_finish)      //�ȴ��ɼ����
    {
        tsl1401_gather();
        tsl1401_flag = tsl1401_finish;
    }
    
}

//��ȡ tsl1401,����CCD �ع�ʱ��
uint32 tsl1401_get_time(void)
{
    return tsl1401_time;
}

//���� tsl1401,����CCD �ع�ʱ��
void tsl1401_set_time(uint32 time)
{
    tsl1401_time = time;
    TSL1401_INT_TIME(tsl1401_time);         //  �����ж�ʱ��
}

//���붨ʱ�ж���Ա��ڶ�ʱ��λ�ع�ʱ��
void tsl1401_time_isr()
{
    //ֻ�ж��Ƿ�ʼ�ɼ����Ƿ�ɼ����
    if(tsl1401_flag == tsl1401_start)
    {
        tsl1401_gather();
        tsl1401_flag = tsl1401_finish;
    }
    else
    {
        tsl1401_restet();                   // ��λ
    }
}

void tsl1401_delay( uint32 time )
{
    volatile uint32 i = time;

    while(i--);
}


//�ɼ�ͼ��
void tsl1401_gather(void)
{
#define TSL1401_TIME   10   //��ʱʱ��

    uint8_t n = TSL1401_SIZE, k = 0;
    uint8_t i;

    tsl1401_clk_out(0);             //CLK = 0
    tsl1401_delay(TSL1401_TIME);
    tsl1401_si_out(1);              //SI  = 1
    tsl1401_delay(TSL1401_TIME);
    tsl1401_clk_out(1);             //CLK = 1
    tsl1401_delay(TSL1401_TIME);

    tsl1401_si_out(0);              //SI  = 0
    tsl1401_delay(TSL1401_TIME);

    while(n--)
    {

        tsl1401_clk_out(0);        //CLK = 0

        i = TSL1401_MAX;
        while(i--)
        {
            (tsl1401_addr[i])[k] = (uint8_t)adc_once(tsl1401_ch[i], ADC_8bit);
            //*img++ =  adc_once(ADC1_AD8, ADC_8bit);
        }

        tsl1401_clk_out(1);        //CLK = 1
        tsl1401_delay(TSL1401_TIME);
        k++;
    }
#undef TSL1401_TIME
}

//���ڵ����ع�ʱ��
void tsl1401_restet()
{
#define TSL1401_RETIME   1

    uint8 n = TSL1401_SIZE;

    tsl1401_clk_out(0);             //CLK = 0
    tsl1401_delay(TSL1401_RETIME);
    tsl1401_si_out(1);              //SI  = 1
    tsl1401_delay(TSL1401_RETIME);
    tsl1401_clk_out(1);             //CLK = 1
    tsl1401_delay(TSL1401_RETIME);

    tsl1401_si_out(0);              //SI  = 0
    tsl1401_delay(TSL1401_RETIME);

    while(n--)
    {
        tsl1401_clk_out(0);             //CLK = 0
        tsl1401_delay(TSL1401_RETIME);
        tsl1401_clk_out(1);             //CLK = 1
        tsl1401_delay(TSL1401_RETIME);
    }
#undef TSL1401_RETIME
}



