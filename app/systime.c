/****************************************Copyright (c)************************************************
**                                      [����ة������]
**--------------File Info-----------------------------------------------------------------------------
** File name         : systime.c
** Last modified Date: 2020-10-15        
** Last Version      :		   
** Descriptions      : ʹ�õ�SDK�汾-SDK_16.0
**						
**----------------------------------------------------------------------------------------------------
** Created by        : [zhengyx]
** Created date      : 2020-10-15
** Version           : 1.0
** Descriptions      : ϵͳ��ʱ����ϵͳʱ���ļ�
**---------------------------------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pca10040.h"
#include "nrf_delay.h"
//��ʱ����Ҫ���õ�ͷ�ļ�
#include "systime.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"



volatile uint32_t sys_time = 0;		//ϵͳʱ���ۼ�ֵ
volatile uint32_t AsynFinished = 0;	//ͬ����ɱ�־
volatile uint16_t syncAddTime = 0;	//ϵͳʱ��

systime_t systimedata;

//����Timer0����������ʵ������������ʵ����ID��ӦTimer��ID����NRF_DRV_TIMER_INSTANCE(0)��ӦTimer0
const nrfx_timer_t TIMER_SYST = NRFX_TIMER_INSTANCE(2);
//����RTC0����������ʵ����0��ӦRTC0
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);


const unsigned char NonLeapMonth[13]={0,31,28,31,30,31,30,31,31,30,31,30,31};	  //ÿ�µ�����
const unsigned char LeapMonth[13]={0,31,29,31,30,31,30,31,31,30,31,30,31};	  //ÿ�µ�����

extern uint8_t Ble_RecSync;

/************************************************************************************************* 
* ��������	: �����ж�        
*
* ����			: year ���                                             
*                               
* ����			: ���귵��1����֮����0��                                       
*        			                                   
* ��ע			:                                                
*                                                                        
**************************************************************************************************/ 
static unsigned char IsLeap(unsigned short year)
{
	if(((year%100==0)&&(year%400==0))||
	   ((year%100!=0) && (year%4==0))) 
		return 1;
	else
		return 0;
}

static int ReturnWeekDay( unsigned int year, unsigned int month, unsigned int day )  
{
    int week = 0;  
    unsigned int y = 0, c = 0, m = 0, d = 0;  
  
    if ( month == 1 || month == 2 )  
    {  
        c = ( year - 1 ) / 100;  
        y = ( year - 1 ) % 100;  
        m = month + 12;  
        d = day;  
    }  
    else  
    {  
        c = year / 100;  
        y = year % 100;  
        m = month;  
        d = day;  
    }  
      
    week = y + y / 4 + c / 4 - 2 * c + 26 * ( m + 1 ) / 10 + d - 1;    //���չ�ʽ  
    week = week >= 0 ? ( week % 7 ) : ( week % 7 + 7 );    //iWeekΪ��ʱȡģ  
    if ( week == 0 )    //�����ղ���Ϊһ�ܵĵ�һ��  
    {  
        week = 7;  
    }  
  
    return week;  
}

static void systimedata_Init(void)
{
	systimedata.year = 2020;
	systimedata.month = 10;
	systimedata.day = 0;
	systimedata.hour = 0;
	systimedata.minute = 0;
	systimedata.second = 0;
	
	systimedata.week = ReturnWeekDay(systimedata.year,systimedata.month,systimedata.day);
}

static void Calendar_Timer(void)
{
	systimedata.week = ReturnWeekDay(systimedata.year,systimedata.month,systimedata.day);
	
    systimedata.second++;
	
    if (!(systimedata.second == 0x3C))
    {
        return;
    }
    systimedata.second = 0;
    systimedata.minute++;
    if (!(systimedata.minute == 0x3C))
    {
        return;
    }
    systimedata.minute=0;
    systimedata.hour++;
    if (!(systimedata.hour == 0x18))
    {
        return;
    }
    systimedata.hour = 0;
    systimedata.day++;

    if (IsLeap(systimedata.year))
    {
        if (systimedata.day <= LeapMonth[systimedata.month])
        {
            return;
        }
    }
    else
    {
        if (systimedata.day <= NonLeapMonth[systimedata.month])
        {
            return;
        }
    }
    systimedata.day=1;
    systimedata.month++;
    if (!(systimedata.month == 0x0D))
    {
        return;
    }

    systimedata.month=1;
    systimedata.year++;
}



//Timer�¼��ص�����
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        //��Ϊ�������õ���ʹ��CCͨ��1�������¼��ص��������ж�NRF_TIMER_EVENT_COMPARE1�¼�
			case NRF_TIMER_EVENT_COMPARE2:
            //��תָʾ��D1״̬
            nrf_gpio_pin_toggle(LED_4);
			sys_time++;
			if(sys_time%1000 == 0)
			{
				nrf_gpio_pin_toggle(LED_3);
				Calendar_Timer();
			}
            break;

        default:
            //Do nothing.
            break;
    }
}

//��ʱ����ʼ��
void timer1_init(void)
{
	  uint32_t err_code = NRF_SUCCESS;
	  //��ʱʱ��1ms
	  uint32_t time_ms = 1; 
	  //���涨ʱʱ���Ӧ��Ticks
    uint32_t time_ticks;
	
	  //���嶨ʱ�����ýṹ�壬��ʹ��Ĭ�����ò�����ʼ���ṹ��
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;

	
	  //��ʼ����ʱ������ʼ��ʱ��ע��timer_led_event_handler�¼��ص�����
    err_code = nrfx_timer_init(&TIMER_SYST, &timer_cfg, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);
	  
	  //��ʱʱ��(��λms)ת��Ϊticks
	  time_ticks = nrfx_timer_ms_to_ticks(&TIMER_SYST, time_ms);
    //���ö�ʱ������/�Ƚ�ͨ������ͨ���ıȽ�ֵ��ʹ��ͨ���ıȽ��ж�
    nrfx_timer_extended_compare(
         &TIMER_SYST, NRF_TIMER_CC_CHANNEL2, time_ticks, NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, true);
}


//RTC�¼��ص�����
static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    //�жϲ������¼��Ƿ���TICK�¼������������õ�TICK�¼�ÿ125ms����һ��
	if (int_type == NRFX_RTC_INT_TICK)
    {
        //��תָʾ��D1��״̬ 
		nrf_gpio_pin_toggle(LED_4);

		if(syncAddTime!=0)
		{
			sys_time=sys_time+syncAddTime+2;
			syncAddTime = 0;
		}
		else
		{
			sys_time+=2;
		}
		
		if(sys_time%1000 == 0)	//����
		{
			//δ��ʱǰ��ʱ1s���������ʱָ��
//			if(!AsynFinished)
//				Ble_RecSync = 1;	
			
			nrf_gpio_pin_toggle(LED_3);
			Calendar_Timer();
		}
    }
}
//����RTC
static void rtc_config(void)
{
    uint32_t err_code;
    
	//����RTC��ʼ�����ýṹ�壬��ʹ��Ĭ�ϲ�����ʼ��
    nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
	//��д��Ƶϵ������Ƶϵ������Ϊ4095ʱ�ĵ���Ƶ�� = 32768/(4095+1) = 8Hz����ÿ125ms COUNTER����������һ�Ρ�
	//TICK�¼�����COUNTER����������ʱ����������TICK�¼�ÿ125ms����һ��
    config.prescaler = 63;	//32768/(31+1) = 1024Hz 32768/(63+1) = 512Hz 1.95ms����һ���¼�
	//��ʼ��RTC��������ʵ����������ע���¼����
    err_code = nrfx_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //ʹ��TICK�¼�
    nrfx_rtc_tick_enable(&rtc,true);
	//����RTC
    nrfx_rtc_enable(&rtc);
}


/**********************************
*������zhenegyx 2020.10.15
*���ܣ�ϵͳʱ���ʼ��
*��������
*���أ���
***********************************/
void systime_init(void)
{
	timer1_init();
		//������ʱ��
	nrfx_timer_enable(&TIMER_SYST);

	systimedata_Init();
}

void sysRtc_init(void)
{
	rtc_config();
	
	systimedata_Init();

}

