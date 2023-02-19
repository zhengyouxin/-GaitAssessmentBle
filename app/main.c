/****************************************Copyright (c)************************************************
**                                      [艾克姆科技]
**                                        IIKMSIK 
**                            官方店铺：https://acmemcu.taobao.com
**                            官方论坛：http://www.e930bbs.com
**                                   
**--------------File Info-----------------------------------------------------------------------------
** File name         : main.c
** Last modified Date: 2019-12-30        
** Last Version      :		   
** Descriptions      : 使用的SDK版本-SDK_16.0
**						
**----------------------------------------------------------------------------------------------------
** Created by        : [艾克姆]
** Created date      : 2018-12-24
** Version           : 1.0
** Descriptions      : 串口透传长包传输（最大传输长度244个字节），本例中使用的设备名称是中文设备名称：艾克姆串口透传
**                   ：增加了对通知是否使能的判断
**                   ：为防止在未连接、通知未使能的情况下串口数据干扰程序，只有通知使能后才初始化串口接收串口数据，通知关闭后，关闭串口
**---------------------------------------------------------------------------------------------------*/
//引用的C库头文件
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//Log需要引用的头文件
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//APP定时器需要引用的头文件
#include "app_timer.h"

#include "bsp_btn_ble.h"
//广播需要引用的头文件
#include "ble_advdata.h"
#include "ble_advertising.h"
//电源管理需要引用的头文件
#include "nrf_pwr_mgmt.h"
//SoftDevice handler configuration需要引用的头文件
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
//排序写入模块需要引用的头文件
#include "nrf_ble_qwr.h"
//GATT需要引用的头文件
#include "nrf_ble_gatt.h"
//连接参数协商需要引用的头文件
#include "ble_conn_params.h"
//串口透传需要引用的头文件
#include "my_ble_uarts.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "app_uart.h"
#include "systime.h"
#include "utility.h"
#include "nrf_drv_clock.h"
#include "ble_gap.h"

#include "time_sync.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"

#define SOFT_VERSION					"V1.2.0"
#define DEVICE_NAME                     "GAIT_DEVICE_CHWS"                 // 设备名称字符串 
#define UARTS_SERVICE_UUID_TYPE         BLE_UUID_TYPE_VENDOR_BEGIN         // 串口透传服务UUID类型：厂商自定义UUID
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)   // 最小连接间隔 (0.1 秒) 20毫秒 50
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)   // 最大连接间隔 (0.2 秒) 30毫秒 80
#define SLAVE_LATENCY                   0                                  // 从机延迟 
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)    // 监督超时(4 秒) 
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)              // 定义首次调用sd_ble_gap_conn_param_update()函数更新连接参数延迟时间（5秒）
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)             // 定义每次调用sd_ble_gap_conn_param_update()函数更新连接参数的间隔时间（30秒）
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                  // 定义放弃连接参数协商前尝试连接参数协商的最大次数（3次）

#define APP_ADV_INTERVAL                320                                // 广播间隔 (200ms)，单位0.625 ms 
#define APP_ADV_DURATION                0                                  // 广播持续时间，单位：10ms。设置为0表示不超时 

#define APP_BLE_OBSERVER_PRIO           3               //应用程序BLE事件监视者优先级，应用程序不能修改该数值
#define APP_BLE_CONN_CFG_TAG            1               //SoftDevice BLE配置标志

#define UART_TX_BUF_SIZE 256                            //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 256                            //串口接收缓存大小（字节数）

//用于stack dump的错误代码，可以用于栈回退时确定堆栈位置
#define DEAD_BEEF                       0xDEADBEEF     
               
BLE_UARTS_DEF(m_uarts, NRF_SDH_BLE_TOTAL_LINK_COUNT);    //定义名称为m_uarts的串口透传服务实例
NRF_BLE_GATT_DEF(m_gatt);                                //定义名称为m_gatt的GATT模块实例
NRF_BLE_QWR_DEF(m_qwr);                                  //定义一个名称为m_qwr的排队写入实例
BLE_ADVERTISING_DEF(m_advertising);                      //定义名称为m_advertising的广播模块实例

#define WAVETEST 0

//该变量用于保存连接句柄，初始值设置为无连接
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; 
//发送的最大数据长度
static uint16_t   m_ble_uarts_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            
static bool uart_enabled = false;
//定义串口透传服务UUID列表
static ble_uuid_t m_adv_uuids[]          =                                          
{
    {BLE_UUID_UARTS_SERVICE, UARTS_SERVICE_UUID_TYPE}
};

uint8_t Ble_SyncTime=0;
uint8_t Ble_RecSync=0;
uint8_t Ble_RecConfig=0;

//设备名称数组  中文名称：艾克姆串口透传
const char device_name[21] = {0xE8,0x89,0xBE,0xE5,0x85,0x8B,0xE5,0xA7,0x86,0xE4,0xB8,0xB2,0xE5,0x8F,0xA3,0xE9,0x80,0x8F,0xE4,0xBC,0xA0};

//IMU设备命令
uint8_t GotoConfig[5]={0xFA,0xFF,0x30,0x00,0xD1};
uint8_t GotoMesure[5]={0xFA,0xFF,0x10,0x00,0xF1};
uint8_t BleAnsSync[5]={0xFB,0xFF,0x00,0x00,0xA5};
uint8_t BleSyncOK[7]={0xFD,0xFF,0x00,0x00,0x00,0x00,0x01};

//接收APP指令，串口透传
uint8_t startSYNC[6]={0xFA,0xFF,0xA3,0x01,0x5A,0x02};
uint8_t stopSYNC[6]={0xFA,0xFF,0xA4,0x01,0xAA,0xB1};
//下发APP/蓝牙模块指令，串口透传
uint8_t bleConnected[6]={0xFA,0xFF,0xA5,0x01,0x01,0x59};
uint8_t bleDisconnected[6]={0xFA,0xFF,0xA5,0x01,0x02,0x58};

//uart rec buf lenth
uint8_t data_array[BLE_UARTS_MAX_DATA_LEN];
uint8_t uRecLen = 0;

//蓝牙连接标志
#define CONNECTED 		1
#define DISCONNECTED	2
uint8_t bleConState = 0;

#define IMU_D_SEND_LEN_1 	31
#define IMU_D_SEND_LEN_2 	62
#define IMU_D_SEND_LEN 		93	//2-IMU 31*3 //105 1-IMU 35*3
#define CMD_SEND_LEN 	6	
#define STA_SEND_LEN 	7	

uint8_t GetIMUDat[IMU_D_SEND_LEN] = {0};
uint8_t GetCMDDat[CMD_SEND_LEN] = {0};
uint8_t GetSTADat[STA_SEND_LEN] = {0};

uint8_t IMUDatSendFlag = 0;
uint8_t CMDDatSendFlag = 0;
uint8_t STADatSendFlag = 0;

uint8_t IMUSeqSave[256*10]={0};
uint16_t SeqErro = 0;
uint16_t seqCnt=0;

extern uint32_t sys_time;
extern uint32_t AsynFinished;
extern uint16_t syncAddTime;

extern systime_t systimedata;
extern volatile uint32_t SYNCStartFlag;
extern volatile uint8_t  slaveSYNCFlag;

//GAP参数初始化，该函数配置需要的GAP参数，包括设备名称，外观特征、首选连接参数
static void gap_params_init(void)
{
    ret_code_t              err_code;
	  //定义连接参数结构体变量
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    //设置GAP的安全模式
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    //设置GAP设备名称，使用英文设备名称
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)DEVICE_NAME,
                                              strlen(DEVICE_NAME));
	
	  //设置GAP设备名称，这里使用了中文设备名称
//    err_code = sd_ble_gap_device_name_set(&sec_mode,
//                                          (const uint8_t *)device_name,
//                                          sizeof(device_name));
																					
    //检查函数返回的错误代码
		APP_ERROR_CHECK(err_code);
																				
    //设置首选连接参数，设置前先清零gap_conn_params
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;//最小连接间隔
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;//最小连接间隔
    gap_conn_params.slave_latency     = SLAVE_LATENCY;    //从机延迟
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT; //监督超时
    //调用协议栈API sd_ble_gap_ppcp_set配置GAP参数
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
																					
}
//GATT事件处理函数，该函数中处理MTU交换事件
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    //如果是MTU交换事件
	  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        //设置串口透传服务的有效数据长度（MTU-opcode-handle）
			  m_ble_uarts_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_uarts_max_data_len, m_ble_uarts_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}
//初始化GATT程序模块
static void gatt_init(void)
{
    //初始化GATT程序模块
	  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	  //检查函数返回的错误代码
    APP_ERROR_CHECK(err_code);
	  //设置ATT MTU的大小,这里设置的值为247
	  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

//排队写入事件处理函数，用于处理排队写入模块的错误
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    //检查错误代码
	  APP_ERROR_HANDLER(nrf_error);
}
//串口事件回调函数，串口初始化时注册，该函数中判断事件类型并进行处理
//当接收的数据长度达到设定的最大值或者接收到换行符后，则认为一包数据接收完成，之后将接收的数据发送给主机
void uart_event_handle(app_uart_evt_t * p_event)
{
//    uint32_t err_code;
//	uint16_t length;
	
    //判断事件类型
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY://串口接收事件 MCU下发IMU数据，通过蓝牙透传给APP
            UNUSED_VARIABLE(app_uart_get(&data_array[uRecLen]));
            uRecLen++;

			if(data_array[0] != 0xfa || uRecLen>IMU_D_SEND_LEN)
			{
				uRecLen = 0;
				memset(data_array,0,IMU_D_SEND_LEN);
			}
            //接收串口数据
			if(uRecLen == CMD_SEND_LEN && data_array[1] == 0xff && (data_array[2]==0xA1||data_array[2]==0xA7))
			{
				memset(GetCMDDat,0,CMD_SEND_LEN);
				memcpy(GetCMDDat,data_array,CMD_SEND_LEN);

                uRecLen = 0;
				CMDDatSendFlag = 1;
//				memset(data_array,0,CMD_SEND_LEN);
			}
			else if(uRecLen == STA_SEND_LEN && data_array[1] == 0xff && data_array[2]==0xA8)
			{
				memset(GetSTADat,0,STA_SEND_LEN);
				memcpy(GetSTADat,data_array,STA_SEND_LEN);

                uRecLen = 0;
				STADatSendFlag = 1;
			}
			else if(uRecLen == IMU_D_SEND_LEN && data_array[1] == 0xff)
			{
				
//				if(data_array[30] == CheckSum(&data_array[2],28) && data_array[61] == CheckSum(&data_array[33],28) &&
//					 data_array[92] == CheckSum(&data_array[64],28))
				{

					memset(GetIMUDat,0,IMU_D_SEND_LEN);
					memcpy(GetIMUDat,data_array,IMU_D_SEND_LEN);
					
					IMUDatSendFlag = 1;
					uRecLen = 0;
				}
				
			}
		#if 0
			else if(uRecLen == IMU_D_SEND_LEN_1 && (data_array[30] != CheckSum(&data_array[2],28)))
			{
//				memset(&data_array[0],0,IMU_D_SEND_LEN_1);
				uRecLen = 0;
			}
			else if(uRecLen == IMU_D_SEND_LEN_2 && (data_array[61] != CheckSum(&data_array[33],28)))
			{
//				memset(&data_array[IMU_D_SEND_LEN_1],0,IMU_D_SEND_LEN_1);
				uRecLen = 0;//IMU_D_SEND_LEN_1;
			}
			else if(uRecLen == IMU_D_SEND_LEN && data_array[0] == 0xfa && data_array[1] == 0xff)
			{
				//check sum
//				if(data_array[30] == CheckSum(&data_array[2],28) && data_array[61] == CheckSum(&data_array[33],28) &&
//					 data_array[92] == CheckSum(&data_array[64],28))
				if(data_array[92] != CheckSum(&data_array[64],28))
				{
//					memset(&data_array[IMU_D_SEND_LEN_2],0,IMU_D_SEND_LEN_1);
					uRecLen = 0;//IMU_D_SEND_LEN_2;
				}
				else
				{
					memset(GetIMUDat,0,IMU_D_SEND_LEN);
					memcpy(GetIMUDat,data_array,IMU_D_SEND_LEN);
//					memset(data_array,0,IMU_D_SEND_LEN);

					IMUDatSendFlag = 1;
					uRecLen = 0;
				}
			}
		#endif
#if 0
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                //串口接收的数据使用notify发送给BLE主机
                do
                {
                    length = (uint16_t)index;
                    err_code = ble_uarts_data_send(&m_uarts, data_array, &length, m_conn_handle);
                    if ((err_code != NRF_ERROR_INVALID_STATE) &&
                        (err_code != NRF_ERROR_RESOURCES) &&
                        (err_code != NRF_ERROR_NOT_FOUND))
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_RESOURCES);
#endif
            break;
        //通讯错误事件，进入错误处理
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        //FIFO错误事件，进入错误处理
        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
//串口配置
void uart_config(void)
{
	uint32_t err_code;
	
	//定义串口通讯参数配置结构体并初始化
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//定义uart接收引脚
    TX_PIN_NUMBER,//定义uart发送引脚
    RTS_PIN_NUMBER,//定义uart RTS引脚，流控关闭后虽然定义了RTS和CTS引脚，但是驱动程序会忽略，不会配置这两个引脚，两个引脚仍可作为IO使用
    CTS_PIN_NUMBER,//定义uart CTS引脚
    APP_UART_FLOW_CONTROL_DISABLED,//关闭uart硬件流控
    false,//禁止奇偶检验
    NRF_UART_BAUDRATE_115200//uart波特率设置为115200bps
  };
  //初始化串口，注册串口事件回调函数
  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_event_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

  APP_ERROR_CHECK(err_code);
	
}
static void uart_reconfig(void)
{
	if(uart_enabled == false)//初始化串口
	{
		uart_config();
		uart_enabled = true;
	}
	else
	{
		app_uart_close();//反初始化串口
		uart_enabled = false;
	}
}
//串口透传事件回调函数，串口透出服务初始化时注册
static void uarts_data_handler(ble_uarts_evt_t * p_evt)
{
//	uint32_t getAsynTime=0;
	  //通知使能后才初始化串口
	  if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
		{
			uart_reconfig();
		}
		//通知关闭后，关闭串口
		else if(p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
		{
		  uart_reconfig();
		}
	  //判断事件类型:接收到新数据事件
    if ((p_evt->type == BLE_UARTS_EVT_RX_DATA) && (uart_enabled == true))
    {
        uint32_t err_code;
        //串口打印出蓝牙接收的数据
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
				
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
		
		if(p_evt->params.rx_data.p_data[0]==0xFA&&p_evt->params.rx_data.p_data[2]==0xA3&&p_evt->params.rx_data.p_data[4]==0x5A)
		{
			Ble_RecSync = 1;	//APP下发同步指令
			memset(data_array,0,IMU_D_SEND_LEN);
			uRecLen = 0;
			
			NRF_LOG_INFO("Receive Ble SYNC Start Command.");
		}
		else if(p_evt->params.rx_data.p_data[0]==0xFA&&p_evt->params.rx_data.p_data[2]==0xA4&&p_evt->params.rx_data.p_data[4]==0xAA)
		{
			Ble_RecConfig = 1;	//APP下发停止采集
			memset(data_array,0,IMU_D_SEND_LEN);
			uRecLen = 0;
			
			NRF_LOG_INFO("Receive Ble SYNC Stop Command.");
		}
#if 0	////APP下发时间差
		else if(p_evt->params.rx_data.p_data[0]==0xA5 && p_evt->params.rx_data.p_data[3]==0xA5)
		{
			Ble_SyncTime = 1;	//APP下发时间差
			syncAddTime  = (uint16_t)(p_evt->params.rx_data.p_data[1] << 8);
			syncAddTime |= p_evt->params.rx_data.p_data[2];
		}
		else if(p_evt->params.rx_data.p_data[0]==0xFC)
		{
			//接收到主机同步指令
			getAsynTime |= (uint32_t)(p_evt->params.rx_data.p_data[2] << 24);
			getAsynTime |= (uint32_t)(p_evt->params.rx_data.p_data[3] << 16);
			getAsynTime |= (uint32_t)(p_evt->params.rx_data.p_data[4] << 8);
			getAsynTime |= p_evt->params.rx_data.p_data[5];

			if(getAsynTime > sys_time - 40)//判断时差值
			{
				sys_time = getAsynTime + 40;
				if(AsynFinished)
					AsynFinished = 0;//未完成对时
			}
			else
			{
				AsynFinished = 1;	//完成时间同步标志
			}
			getAsynTime = 0;
		}
#endif
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }
		//判断事件类型:发送就绪事件，该事件在后面的试验会用到，当前我们在该事件中翻转指示灯D4的状态，指示该事件的产生
#ifdef IKM_TEST
    if (p_evt->type == BLE_UARTS_EVT_TX_RDY)
    {
			nrf_gpio_pin_toggle(LED_4);
	}
#endif
}
void uart_sendCommand(uint8_t *buf,uint8_t len)
{
	uint32_t i;
	uint32_t err_code;
	
	for (i = 0; i < len; i++)
    {
        do
        {
            err_code = app_uart_put(buf[i]);
			
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                APP_ERROR_CHECK(err_code);
            }
			
        }while (err_code == NRF_ERROR_BUSY);
    }
}

//服务初始化，包含初始化排队写入模块和初始化应用程序使用的服务
static void services_init(void)
{
    ret_code_t         err_code;
	  //定义串口透传初始化结构体
	  ble_uarts_init_t     uarts_init;
	  //定义排队写入初始化结构体变量
    nrf_ble_qwr_init_t qwr_init = {0};

    //排队写入事件处理函数
    qwr_init.error_handler = nrf_qwr_error_handler;
    //初始化排队写入模块
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
		//检查函数返回值
    APP_ERROR_CHECK(err_code);
    
		
		/*------------------以下代码初始化串口透传服务-------------*/
		//清零串口透传服务初始化结构体
		memset(&uarts_init, 0, sizeof(uarts_init));
		//设置串口透传事件回调函数
    uarts_init.data_handler = uarts_data_handler;
    //初始化串口透传服务
    err_code = ble_uarts_init(&m_uarts, &uarts_init);
    APP_ERROR_CHECK(err_code);
		/*------------------初始化串口透传服务-END-----------------*/
}

//连接参数协商模块事件处理函数
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;
    //判断事件类型，根据事件类型执行动作
	  //连接参数协商失败，断开当前连接
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
		//连接参数协商成功
		if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
       //功能代码;
    }
}

//连接参数协商模块错误处理事件，参数nrf_error包含了错误代码，通过nrf_error可以分析错误信息
static void conn_params_error_handler(uint32_t nrf_error)
{
    //检查错误代码
	  APP_ERROR_HANDLER(nrf_error);
}


//连接参数协商模块初始化
static void conn_params_init(void)
{
    ret_code_t             err_code;
	  //定义连接参数协商模块初始化结构体
    ble_conn_params_init_t cp_init;
    //配置之前先清零
    memset(&cp_init, 0, sizeof(cp_init));
    //设置为NULL，从主机获取连接参数
    cp_init.p_conn_params                  = NULL;
	  //连接或启动通知到首次发起连接参数更新请求之间的时间设置为5秒
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	  //每次调用sd_ble_gap_conn_param_update()函数发起连接参数更新请求的之间的间隔时间设置为：30秒
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	  //放弃连接参数协商前尝试连接参数协商的最大次数设置为：3次
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	  //连接参数更新从连接事件开始计时
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	  //连接参数更新失败不断开连接
    cp_init.disconnect_on_fail             = false;
	  //注册连接参数更新事件句柄
    cp_init.evt_handler                    = on_conn_params_evt;
	  //注册连接参数更新错误事件句柄
    cp_init.error_handler                  = conn_params_error_handler;
    //调用库函数（以连接参数更新初始化结构体为输入参数）初始化连接参数协商模块
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

//广播事件处理函数
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;
    //判断广播事件类型
    switch (ble_adv_evt)
    {
        //快速广播启动事件：快速广播启动后会产生该事件
			  case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
			      //设置广播指示灯为正在广播（D1指示灯闪烁）
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        //广播IDLE事件：广播超时后会产生该事件
        case BLE_ADV_EVT_IDLE:
					  //设置广播指示灯为广播停止（D1指示灯熄灭）
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}
//广播初始化
static void advertising_init(void)
{
    ret_code_t             err_code;
	  //定义广播初始化配置结构体变量
    ble_advertising_init_t init;
    //配置之前先清零
    memset(&init, 0, sizeof(init));
    //设备名称类型：全称
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	  //是否包含外观：包含
    init.advdata.include_appearance      = false;
	  //Flag:一般可发现模式，不支持BR/EDR
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	  //UUID放到扫描响应里面
	  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
	
    //设置广播模式为快速广播
    init.config.ble_adv_fast_enabled  = true;
	  //设置广播间隔和广播持续时间
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    //广播事件回调函数
    init.evt_handler = on_adv_evt;
    //初始化广播
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);
    //设置广播配置标记。APP_BLE_CONN_CFG_TAG是用于跟踪广播配置的标记，这是为未来预留的一个参数，在将来的SoftDevice版本中，
		//可以使用sd_ble_gap_adv_set_configure()配置新的广播配置
		//当前SoftDevice版本（S132 V7.0.1版本）支持的最大广播集数量为1，因此APP_BLE_CONN_CFG_TAG只能写1。
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

//BLE事件处理函数
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
    //判断BLE事件类型，根据事件类型执行相应操作
    switch (p_ble_evt->header.evt_id)
    {
        //断开连接事件
			  case BLE_GAP_EVT_DISCONNECTED:

				m_conn_handle = BLE_CONN_HANDLE_INVALID;
			  
				bleConState = DISCONNECTED;
			    //打印提示信息
			    NRF_LOG_INFO("Disconnected.");
//			    uart_reconfig();	//断开时不改变uart状态。
            break;
				
        //连接事件
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
			//设置指示灯状态为连接状态，即指示灯D1常亮
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
			bleConState = CONNECTED;
			
			//保存连接句柄
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			//将连接句柄分配给排队写入实例，分配后排队写入实例和该连接关联，这样，当有多个连接的时候，通过关联不同的排队写入实例，很方便单独处理各个连接
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            break;
				
        //PHY更新事件
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
						//响应PHY更新规程
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
				//安全参数请求事件
				case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            //不支持配对
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
				 
				//系统属性访问正在等待中
				case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            //系统属性没有存储，更新系统属性
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
        //GATT客户端超时事件
        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Client Timeout.");
				    //断开当前连接
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
				
        //GATT服务器超时事件
        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Server Timeout.");
				    //断开当前连接
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

//初始化BLE协议栈
static void ble_stack_init(void)
{
    ret_code_t err_code;
    //请求使能SoftDevice，该函数中会根据sdk_config.h文件中低频时钟的设置来配置低频时钟
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    
    //定义保存应用程序RAM起始地址的变量
    uint32_t ram_start = 0;
	  //使用sdk_config.h文件的默认参数配置协议栈，获取应用程序RAM起始地址，保存到变量ram_start
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    //使能BLE协议栈
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    //注册BLE事件回调函数
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
//初始化电源管理模块
static void power_management_init(void)
{
    ret_code_t err_code;
	  //初始化电源管理
    err_code = nrf_pwr_mgmt_init();
	  //检查函数返回的错误代码
    APP_ERROR_CHECK(err_code);
}

//初始化指示灯
//static void leds_init(void)
//{
//    ret_code_t err_code;
//    //初始化BSP指示灯
//    err_code = bsp_init(BSP_INIT_LEDS, NULL);
//    APP_ERROR_CHECK(err_code);

//}
//初始化APP定时器模块
static void timers_init(void)
{
    //初始化APP定时器模块
    ret_code_t err_code = app_timer_init();
	  //检查返回值
    APP_ERROR_CHECK(err_code);

    //加入创建用户定时任务的代码，创建用户定时任务。 

}
static void log_init(void)
{
    //初始化log程序模块
	  ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //设置log输出终端（根据sdk_config.h中的配置设置输出终端为UART或者RTT）
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//空闲状态处理函数。如果没有挂起的日志操作，则睡眠直到下一个事件发生后唤醒系统
static void idle_state_handle(void)
{
    //处理挂起的log
	  if (NRF_LOG_PROCESS() == false)
    {
        //运行电源管理，该函数需要放到主循环里面执行
			  nrf_pwr_mgmt_run();
    }
}
//启动广播，该函数所用的模式必须和广播初始化中设置的广播模式一样
static void advertising_start(void)
{
   //使用广播初始化中设置的广播模式启动广播
	 ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	 //检查函数返回的错误代码
   APP_ERROR_CHECK(err_code);
}

/**********************************
*创建：zhenegyx 2020.10.16
*功能：蓝牙串口发送数据
*参数：发送的数组地址，长度
*返回：无
***********************************/
void ble_uart_send_data(uint8_t *sendBuf,uint16_t len)
{
	uint32_t err_code;
	uint16_t length;
	
    do
    {
        length = len;
        err_code = ble_uarts_data_send(&m_uarts, sendBuf, &length, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);

}

/**********************************
*创建：zhenegyx 2020.10.16
*功能：蓝牙串口发送实时时间数据
*参数：无
*返回：无
***********************************/
void ble_send_synctime(void)
{
	uint32_t err_code;
	uint16_t length;

	uint8_t timedata[7];

	timedata[0]=0xFB;
	timedata[1]=0xFF;

	timedata[2]=sys_time>>24;
	timedata[3]=sys_time>>16;
	timedata[4]=sys_time>>8;
	timedata[5]=sys_time;
	
	timedata[6]=CheckSum(&timedata[2],4);
	
	//串口接收的数据使用notify发送给BLE主机
    do
    {
        length = (uint16_t)7;
        err_code = ble_uarts_data_send(&m_uarts, timedata, &length, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);
}

void ble_send_test(void)
{
	uint32_t err_code;
	uint16_t length;

	uint8_t timedata[140];

	for(uint8_t i=0;i<140;i++)
	{
		timedata[i]=i;
	}
	
	//串口接收的数据使用notify发送给BLE主机
    do
    {
        length = (uint16_t)140;
        err_code = ble_uarts_data_send(&m_uarts, timedata, &length, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);
}

/**********************************
*创建：zhenegyx 2021.07.09
*功能：同步指令处理：开始数据采集
*参数：无
*返回：无
***********************************/
static void ble_rec_start_no_sync(void)
{
	uint32_t err_code;

//	if(SYNCStartFlag >= 5)
//		return;
	
	SYNCStartFlag = 0;
	
	SYNC_PIN_RESET();		//interrupt to MCU
	NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_RESET\r\n");
	nrf_delay_ms(10);
	SYNC_PIN_SET();
	NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_SET\r\n");
	
	NRF_LOG_INFO("Starting data transmission!\r\n");
}

/**********************************
*创建：zhenegyx 2020.12.10
*功能：同步指令处理：开始同步时钟和数据采集
*参数：无
*返回：无
***********************************/
static void ble_rec_start_sync(void)
{
	uint32_t err_code;

	if(SYNCStartFlag >= 5)
		return;
	
	SYNCStartFlag = 0;
	
	err_code = ts_tx_start(200);	//开始同步多设备时钟
	APP_ERROR_CHECK(err_code);
	SYNC_PIN_SET();		//interrupt to MCU
	NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_SET\r\n");
	
	NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
}

/**********************************
*创建：zhenegyx 2020.12.10
*功能：同步指令处理：停止同步和数据采集
*参数：无
*返回：无
***********************************/
static void ble_rec_stop_sync(void)
{
	uint32_t err_code;
	if(SYNCStartFlag)
	{
		err_code = ts_tx_stop();
		APP_ERROR_CHECK(err_code);
		SYNCStartFlag = 0;
	}

	SYNC_PIN_RESET();		//interrupt to MCU
	NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_RESET\r\n");
	nrf_delay_ms(10);
	SYNC_PIN_SET();
	NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_SET\r\n");
	NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");

	if(slaveSYNCFlag == 1)
	{
		slaveSYNCFlag = 0;
	}
	


}


/**********************************
*创建：zhenegyx 2020.11.23
*功能：同步指令处理：开始同步多设备时钟
*参数：无
*返回：无
***********************************/
static void ble_rec_sync_handle(void)
{
	uint32_t err_code;
	static bool m_send_sync_pkt = false;

	if (m_send_sync_pkt)
	{
		m_send_sync_pkt = false;
		SYNCStartFlag = 0;
		
//		bsp_board_leds_off();
		
		err_code = ts_tx_stop();
		APP_ERROR_CHECK(err_code);
		SYNC_PIN_RESET();		//interrupt to MCU
		NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_RESET\r\n");
		nrf_delay_ms(10);
		SYNC_PIN_SET();
		NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_SET\r\n");
		NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");
	}
	else
	{
		m_send_sync_pkt = true;
		SYNCStartFlag = 0;
		
//		bsp_board_leds_on();
		
		err_code = ts_tx_start(200);	//开始同步多设备时钟
		APP_ERROR_CHECK(err_code);
		SYNC_PIN_SET();		//interrupt to MCU
		NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_SET\r\n");
		
		NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
	}
}


void ble_ondisconnect()
{
	if(AsynFinished)
	{
		sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	}
}

void buttons_handle()
{
	uint32_t err_code;
	static bool m_send_sync_pkt = false;
	
	if(nrf_gpio_pin_read(BUTTON_1) == 0)
	{
		//点亮LED指示灯D1
		nrf_gpio_pin_clear(LED_2);
		//等待按键释放
		while(nrf_gpio_pin_read(BUTTON_1) == 0);
		//熄灭LED指示灯D1
		nrf_gpio_pin_set(LED_2);
		//按下按键通过蓝牙发送一次时间
		//Ble_RecSync = 1;

		//按下按键开始同步
        if (m_send_sync_pkt)
        {
            m_send_sync_pkt = false;
            
//            bsp_board_leds_off();
            
            err_code = ts_tx_stop();
            APP_ERROR_CHECK(err_code);
            
            NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");
        }
        else
        {
            m_send_sync_pkt = true;
            
//            bsp_board_leds_on();
            
            err_code = ts_tx_start(200);
            APP_ERROR_CHECK(err_code);
            
            NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
        }
	}
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_KEY_0:
        case BSP_EVENT_KEY_1:
        case BSP_EVENT_KEY_2:
        case BSP_EVENT_KEY_3:
            {
                static bool m_send_sync_pkt = false;
                
                if (m_send_sync_pkt)
                {
                    m_send_sync_pkt = false;
					SYNCStartFlag = 0;
                    
                    bsp_board_leds_off();
                    
                    err_code = ts_tx_stop();
                    APP_ERROR_CHECK(err_code);
                    SYNC_PIN_SET();		//interrupt to MCU
					NRF_LOG_INFO("bsp_event_handler SYNC_PIN_SET\r\n");
					
                    NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");
                }
                else
                {
                    m_send_sync_pkt = true;
 					SYNCStartFlag = 0;
                   
                    bsp_board_leds_on();
					
                    err_code = ts_tx_start(200);
                    APP_ERROR_CHECK(err_code);
                    SYNC_PIN_SET();		//interrupt to MCU
					NRF_LOG_INFO("bsp_event_handler SYNC_PIN_SET\r\n");
					
                    NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
                }
            }
            break;

        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing buttons and leds. */
//static void buttons_leds_init(void)
//{
//    ret_code_t err_code;
//    bsp_event_t startup_event;

//    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
//    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);
//}

//sync timer 配置
static void sync_timer_init(void)
{
	
	uint32_t	   err_code;
	uint8_t 	   rf_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x19};
	ts_params_t    ts_params;

	// Debug pin: 
	// nRF52-DK (PCA10040) Toggle P0.24 from sync timer to allow pin measurement
	// nRF52840-DK (PCA10056) Toggle P1.14 from sync timer to allow pin measurement
#if defined(BOARD_PCA10040)
	nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
	nrf_gpiote_task_enable(3);
#elif defined(BOARD_PCA10056)
	nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
	nrf_gpiote_task_enable(3);
#else
#warning Debug pin not set
#endif

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI_CHANNEL0, 
        (uint32_t) nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4),
        nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);
	
    ts_params.high_freq_timer[0] = NRF_TIMER3;
    ts_params.high_freq_timer[1] = NRF_TIMER2;
    ts_params.rtc             = NRF_RTC1;
    ts_params.egu             = NRF_EGU3;
    ts_params.egu_irq_type    = SWI3_EGU3_IRQn;
    ts_params.ppi_chg         = 0;
    ts_params.ppi_chns[0]     = 1;
    ts_params.ppi_chns[1]     = 2;
    ts_params.ppi_chns[2]     = 3;
    ts_params.ppi_chns[3]     = 4;
    ts_params.rf_chn          = 125; /* For testing purposes */
    memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address));
    
    err_code = ts_init(&ts_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = ts_enable();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("Started listening for beacons.\r\n");
    NRF_LOG_INFO("Press Button 1 to start sending sync beacons\r\n");
}

/**********************************
*创建：zhenegyx 2020.10.16
*功能：蓝牙接收同步处理
*参数：无
*返回：无
***********************************/
static void ble_sync_data_process(void)
{
//	static uint8_t syncFinish=0;
//	static uint16_t sendCnt=0;
	
	if(IMUDatSendFlag)
	{
		ble_uart_send_data(GetIMUDat,sizeof(GetIMUDat));
		IMUDatSendFlag = 0;
#if 0	//test uart4
		sendCnt ++;
		if(sendCnt%2)
			uart_sendCommand(startSYNC,sizeof(startSYNC));
		else
			uart_sendCommand(stopSYNC,sizeof(stopSYNC));
#endif
	}
	else if(CMDDatSendFlag)
	{
		
		ble_uart_send_data(GetCMDDat,sizeof(GetCMDDat));
		CMDDatSendFlag = 0;
	}
	else if(STADatSendFlag)
	{
		ble_uart_send_data(GetSTADat,sizeof(GetSTADat));
		STADatSendFlag = 0;
		
	}
	
	if(Ble_RecSync)
	{
//		ble_rec_sync_handle();
		ble_rec_start_no_sync();//ble_rec_start_sync();
				
		Ble_RecSync = 0;

	}
	else if(Ble_RecConfig)
	{
//		ble_uart_send_data(GotoConfig,sizeof(GotoConfig));
//		uart_sendCommand(GotoConfig,sizeof(GotoConfig));
		
		ble_rec_stop_sync();
		Ble_RecConfig = 0;
	}

	if(bleConState == CONNECTED)
	{
//		uart_sendCommand(bleConnected,sizeof(bleConnected));
		bleConState = 0;
	}
	else if(bleConState == DISCONNECTED)
	{
//		uart_sendCommand(bleDisconnected,sizeof(bleDisconnected));
		bleConState = 0;
	}
	
#if 0
	else if(Ble_SyncTime)
	{
		
		ble_uart_send_data(BleAnsSync,sizeof(BleAnsSync));
		Ble_SyncTime = 0;
		
	}
	

	if(AsynFinished==1 && syncFinish==0)
	{
		ble_uart_send_data(BleSyncOK,sizeof(BleSyncOK));
		syncFinish = 1;
	}
#endif

	if(SYNCStartFlag >= 1 && SYNCStartFlag<5)
	{
		SYNC_PIN_RESET();//nrf_gpio_pin_clear(25);//interrupt to MCU//master
		NRF_LOG_INFO("ble_rec_sync SYNC_PIN_RESET\r\n");
//		nrf_delay_ms(5);
		
	}
	else if(SYNCStartFlag >= 5)
	{
		SYNC_PIN_SET();
		NRF_LOG_INFO("ble_rec_sync SYNC_PIN_SET\r\n");
	}


}

//主函数
int main(void)
{
	//初始化log程序模块
	log_init();
	//初始化串口
//	uart_config();
	//初始化APP定时器
	timers_init();

//	buttons_leds_init();

	
	//出使唤按键和指示灯
//	leds_init();
	//按键初始化
//	bsp_board_init(BSP_INIT_BUTTONS);

	//初始化电源管理
	power_management_init();

//	systime_init();
//	sysRtc_init();
#if 1
	//初始化协议栈
	ble_stack_init();
	//配置GAP参数
	gap_params_init();
	//初始化GATT
	gatt_init();
	//初始化服务
	services_init();
	//初始化广播
	advertising_init();	
	//连接参数协商初始化
	conn_params_init();

	sync_timer_init();
	
	SYNC_OUTPUT_SETUP();
	SYNC_PIN_SET();

	NRF_LOG_INFO("BLE Template example started.");  
	//启动广播
	advertising_start();
#endif

	
#if 0
	if(uart_enabled == 0)
	{
		uart_reconfig();
		uart_sendCommand(GotoConfig,sizeof(GotoConfig));
		delay_ms(5);
		uart_reconfig();
	}
#endif
  //主循环
	while(true)
	{
		//处理挂起的LOG和运行电源管理
		idle_state_handle();
//		buttons_handle();
		ble_sync_data_process();
//		ble_ondisconnect();
	}
}



