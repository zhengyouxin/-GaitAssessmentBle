/****************************************Copyright (c)************************************************
**                                      [深圳丞辉威世]
**--------------File Info-----------------------------------------------------------------------------
** File name         : systime.c
** Last modified Date: 2020-10-15        
** Last Version      :		   
** Descriptions      : 使用的SDK版本-SDK_16.0
**						
**----------------------------------------------------------------------------------------------------
** Created by        : [zhengyx]
** Created date      : 2020-10-15
** Version           : 1.0
** Descriptions      : 系统定时器，系统时间文件
**---------------------------------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pca10040.h"
#include "nrf_delay.h"
//定时器需要引用的头文件
#include "systime.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"



volatile uint32_t sys_time = 0;		//系统时间累加值
volatile uint32_t AsynFinished = 0;	//同步完成标志
volatile uint16_t syncAddTime = 0;	//系统时差

systime_t systimedata;

//定义Timer0的驱动程序实例。驱动程序实例的ID对应Timer的ID，如NRF_DRV_TIMER_INSTANCE(0)对应Timer0
const nrfx_timer_t TIMER_SYST = NRFX_TIMER_INSTANCE(2);
//定义RTC0的驱动程序实例，0对应RTC0
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);


const unsigned char NonLeapMonth[13]={0,31,28,31,30,31,30,31,31,30,31,30,31};	  //每月的天数
const unsigned char LeapMonth[13]={0,31,29,31,30,31,30,31,31,30,31,30,31};	  //每月的天数

extern uint8_t Ble_RecSync;

/************************************************************************************************* 
* 函数功能	: 闰年判断        
*
* 参数			: year 年份                                             
*                               
* 返回			: 闰年返回1；反之返回0；                                       
*        			                                   
* 备注			:                                                
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
      
    week = y + y / 4 + c / 4 - 2 * c + 26 * ( m + 1 ) / 10 + d - 1;    //蔡勒公式  
    week = week >= 0 ? ( week % 7 ) : ( week % 7 + 7 );    //iWeek为负时取模  
    if ( week == 0 )    //星期日不作为一周的第一天  
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



//Timer事件回调函数
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        //因为我们配置的是使用CC通道1，所以事件回调函数中判断NRF_TIMER_EVENT_COMPARE1事件
			case NRF_TIMER_EVENT_COMPARE2:
            //翻转指示灯D1状态
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

//定时器初始化
void timer1_init(void)
{
	  uint32_t err_code = NRF_SUCCESS;
	  //定时时间1ms
	  uint32_t time_ms = 1; 
	  //保存定时时间对应的Ticks
    uint32_t time_ticks;
	
	  //定义定时器配置结构体，并使用默认配置参数初始化结构体
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;

	
	  //初始化定时器，初始化时会注册timer_led_event_handler事件回调函数
    err_code = nrfx_timer_init(&TIMER_SYST, &timer_cfg, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);
	  
	  //定时时间(单位ms)转换为ticks
	  time_ticks = nrfx_timer_ms_to_ticks(&TIMER_SYST, time_ms);
    //设置定时器捕获/比较通道及该通道的比较值，使能通道的比较中断
    nrfx_timer_extended_compare(
         &TIMER_SYST, NRF_TIMER_CC_CHANNEL2, time_ticks, NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, true);
}


//RTC事件回调函数
static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    //判断产生的事件是否是TICK事件，本例中设置的TICK事件每125ms产生一次
	if (int_type == NRFX_RTC_INT_TICK)
    {
        //翻转指示灯D1的状态 
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
		
		if(sys_time%1000 == 0)	//测试
		{
			//未对时前定时1s发送请求对时指令
//			if(!AsynFinished)
//				Ble_RecSync = 1;	
			
			nrf_gpio_pin_toggle(LED_3);
			Calendar_Timer();
		}
    }
}
//配置RTC
static void rtc_config(void)
{
    uint32_t err_code;
    
	//定义RTC初始化配置结构体，并使用默认参数初始化
    nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
	//重写分频系数，分频系数设置为4095时的递增频率 = 32768/(4095+1) = 8Hz，即每125ms COUNTER计数器递增一次。
	//TICK事件是在COUNTER计数器递增时发生，所以TICK事件每125ms产生一次
    config.prescaler = 63;	//32768/(31+1) = 1024Hz 32768/(63+1) = 512Hz 1.95ms产生一次事件
	//初始化RTC驱动程序实例的驱动，注册事件句柄
    err_code = nrfx_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //使能TICK事件
    nrfx_rtc_tick_enable(&rtc,true);
	//启动RTC
    nrfx_rtc_enable(&rtc);
}


/**********************************
*创建：zhenegyx 2020.10.15
*功能：系统时间初始化
*参数：无
*返回：无
***********************************/
void systime_init(void)
{
	timer1_init();
		//启动定时器
	nrfx_timer_enable(&TIMER_SYST);

	systimedata_Init();
}

void sysRtc_init(void)
{
	rtc_config();
	
	systimedata_Init();

}

