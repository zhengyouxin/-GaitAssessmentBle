/****************************************Copyright (c)************************************************
**                                      [����ķ�Ƽ�]
**                                        IIKMSIK 
**                            �ٷ����̣�https://acmemcu.taobao.com
**                            �ٷ���̳��http://www.e930bbs.com
**                                   
**--------------File Info-----------------------------------------------------------------------------
** File name         : main.c
** Last modified Date: 2019-12-30        
** Last Version      :		   
** Descriptions      : ʹ�õ�SDK�汾-SDK_16.0
**						
**----------------------------------------------------------------------------------------------------
** Created by        : [����ķ]
** Created date      : 2018-12-24
** Version           : 1.0
** Descriptions      : ����͸���������䣨����䳤��244���ֽڣ���������ʹ�õ��豸�����������豸���ƣ�����ķ����͸��
**                   �������˶�֪ͨ�Ƿ�ʹ�ܵ��ж�
**                   ��Ϊ��ֹ��δ���ӡ�֪ͨδʹ�ܵ�����´������ݸ��ų���ֻ��֪ͨʹ�ܺ�ų�ʼ�����ڽ��մ������ݣ�֪ͨ�رպ󣬹رմ���
**---------------------------------------------------------------------------------------------------*/
//���õ�C��ͷ�ļ�
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//Log��Ҫ���õ�ͷ�ļ�
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//APP��ʱ����Ҫ���õ�ͷ�ļ�
#include "app_timer.h"

#include "bsp_btn_ble.h"
//�㲥��Ҫ���õ�ͷ�ļ�
#include "ble_advdata.h"
#include "ble_advertising.h"
//��Դ������Ҫ���õ�ͷ�ļ�
#include "nrf_pwr_mgmt.h"
//SoftDevice handler configuration��Ҫ���õ�ͷ�ļ�
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
//����д��ģ����Ҫ���õ�ͷ�ļ�
#include "nrf_ble_qwr.h"
//GATT��Ҫ���õ�ͷ�ļ�
#include "nrf_ble_gatt.h"
//���Ӳ���Э����Ҫ���õ�ͷ�ļ�
#include "ble_conn_params.h"
//����͸����Ҫ���õ�ͷ�ļ�
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
#define DEVICE_NAME                     "GAIT_DEVICE_CHWS"                 // �豸�����ַ��� 
#define UARTS_SERVICE_UUID_TYPE         BLE_UUID_TYPE_VENDOR_BEGIN         // ����͸������UUID���ͣ������Զ���UUID
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)   // ��С���Ӽ�� (0.1 ��) 20���� 50
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)   // ������Ӽ�� (0.2 ��) 30���� 80
#define SLAVE_LATENCY                   0                                  // �ӻ��ӳ� 
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)    // �ල��ʱ(4 ��) 
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)              // �����״ε���sd_ble_gap_conn_param_update()�����������Ӳ����ӳ�ʱ�䣨5�룩
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)             // ����ÿ�ε���sd_ble_gap_conn_param_update()�����������Ӳ����ļ��ʱ�䣨30�룩
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                  // ����������Ӳ���Э��ǰ�������Ӳ���Э�̵���������3�Σ�

#define APP_ADV_INTERVAL                320                                // �㲥��� (200ms)����λ0.625 ms 
#define APP_ADV_DURATION                0                                  // �㲥����ʱ�䣬��λ��10ms������Ϊ0��ʾ����ʱ 

#define APP_BLE_OBSERVER_PRIO           3               //Ӧ�ó���BLE�¼����������ȼ���Ӧ�ó������޸ĸ���ֵ
#define APP_BLE_CONN_CFG_TAG            1               //SoftDevice BLE���ñ�־

#define UART_TX_BUF_SIZE 256                            //���ڷ��ͻ����С���ֽ�����
#define UART_RX_BUF_SIZE 256                            //���ڽ��ջ����С���ֽ�����

//����stack dump�Ĵ�����룬��������ջ����ʱȷ����ջλ��
#define DEAD_BEEF                       0xDEADBEEF     
               
BLE_UARTS_DEF(m_uarts, NRF_SDH_BLE_TOTAL_LINK_COUNT);    //��������Ϊm_uarts�Ĵ���͸������ʵ��
NRF_BLE_GATT_DEF(m_gatt);                                //��������Ϊm_gatt��GATTģ��ʵ��
NRF_BLE_QWR_DEF(m_qwr);                                  //����һ������Ϊm_qwr���Ŷ�д��ʵ��
BLE_ADVERTISING_DEF(m_advertising);                      //��������Ϊm_advertising�Ĺ㲥ģ��ʵ��

#define WAVETEST 0

//�ñ������ڱ������Ӿ������ʼֵ����Ϊ������
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; 
//���͵�������ݳ���
static uint16_t   m_ble_uarts_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            
static bool uart_enabled = false;
//���崮��͸������UUID�б�
static ble_uuid_t m_adv_uuids[]          =                                          
{
    {BLE_UUID_UARTS_SERVICE, UARTS_SERVICE_UUID_TYPE}
};

uint8_t Ble_SyncTime=0;
uint8_t Ble_RecSync=0;
uint8_t Ble_RecConfig=0;

//�豸��������  �������ƣ�����ķ����͸��
const char device_name[21] = {0xE8,0x89,0xBE,0xE5,0x85,0x8B,0xE5,0xA7,0x86,0xE4,0xB8,0xB2,0xE5,0x8F,0xA3,0xE9,0x80,0x8F,0xE4,0xBC,0xA0};

//IMU�豸����
uint8_t GotoConfig[5]={0xFA,0xFF,0x30,0x00,0xD1};
uint8_t GotoMesure[5]={0xFA,0xFF,0x10,0x00,0xF1};
uint8_t BleAnsSync[5]={0xFB,0xFF,0x00,0x00,0xA5};
uint8_t BleSyncOK[7]={0xFD,0xFF,0x00,0x00,0x00,0x00,0x01};

//����APPָ�����͸��
uint8_t startSYNC[6]={0xFA,0xFF,0xA3,0x01,0x5A,0x02};
uint8_t stopSYNC[6]={0xFA,0xFF,0xA4,0x01,0xAA,0xB1};
//�·�APP/����ģ��ָ�����͸��
uint8_t bleConnected[6]={0xFA,0xFF,0xA5,0x01,0x01,0x59};
uint8_t bleDisconnected[6]={0xFA,0xFF,0xA5,0x01,0x02,0x58};

//uart rec buf lenth
uint8_t data_array[BLE_UARTS_MAX_DATA_LEN];
uint8_t uRecLen = 0;

//�������ӱ�־
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

//GAP������ʼ�����ú���������Ҫ��GAP�����������豸���ƣ������������ѡ���Ӳ���
static void gap_params_init(void)
{
    ret_code_t              err_code;
	  //�������Ӳ����ṹ�����
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    //����GAP�İ�ȫģʽ
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    //����GAP�豸���ƣ�ʹ��Ӣ���豸����
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)DEVICE_NAME,
                                              strlen(DEVICE_NAME));
	
	  //����GAP�豸���ƣ�����ʹ���������豸����
//    err_code = sd_ble_gap_device_name_set(&sec_mode,
//                                          (const uint8_t *)device_name,
//                                          sizeof(device_name));
																					
    //��麯�����صĴ������
		APP_ERROR_CHECK(err_code);
																				
    //������ѡ���Ӳ���������ǰ������gap_conn_params
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;//��С���Ӽ��
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;//��С���Ӽ��
    gap_conn_params.slave_latency     = SLAVE_LATENCY;    //�ӻ��ӳ�
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT; //�ල��ʱ
    //����Э��ջAPI sd_ble_gap_ppcp_set����GAP����
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
																					
}
//GATT�¼����������ú����д���MTU�����¼�
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    //�����MTU�����¼�
	  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        //���ô���͸���������Ч���ݳ��ȣ�MTU-opcode-handle��
			  m_ble_uarts_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_uarts_max_data_len, m_ble_uarts_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}
//��ʼ��GATT����ģ��
static void gatt_init(void)
{
    //��ʼ��GATT����ģ��
	  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	  //��麯�����صĴ������
    APP_ERROR_CHECK(err_code);
	  //����ATT MTU�Ĵ�С,�������õ�ֵΪ247
	  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

//�Ŷ�д���¼������������ڴ����Ŷ�д��ģ��Ĵ���
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    //���������
	  APP_ERROR_HANDLER(nrf_error);
}
//�����¼��ص����������ڳ�ʼ��ʱע�ᣬ�ú������ж��¼����Ͳ����д���
//�����յ����ݳ��ȴﵽ�趨�����ֵ���߽��յ����з�������Ϊһ�����ݽ�����ɣ�֮�󽫽��յ����ݷ��͸�����
void uart_event_handle(app_uart_evt_t * p_event)
{
//    uint32_t err_code;
//	uint16_t length;
	
    //�ж��¼�����
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY://���ڽ����¼� MCU�·�IMU���ݣ�ͨ������͸����APP
            UNUSED_VARIABLE(app_uart_get(&data_array[uRecLen]));
            uRecLen++;

			if(data_array[0] != 0xfa || uRecLen>IMU_D_SEND_LEN)
			{
				uRecLen = 0;
				memset(data_array,0,IMU_D_SEND_LEN);
			}
            //���մ�������
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
                //���ڽ��յ�����ʹ��notify���͸�BLE����
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
        //ͨѶ�����¼������������
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        //FIFO�����¼������������
        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
//��������
void uart_config(void)
{
	uint32_t err_code;
	
	//���崮��ͨѶ�������ýṹ�岢��ʼ��
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//����uart��������
    TX_PIN_NUMBER,//����uart��������
    RTS_PIN_NUMBER,//����uart RTS���ţ����عرպ���Ȼ������RTS��CTS���ţ����������������ԣ������������������ţ����������Կ���ΪIOʹ��
    CTS_PIN_NUMBER,//����uart CTS����
    APP_UART_FLOW_CONTROL_DISABLED,//�ر�uartӲ������
    false,//��ֹ��ż����
    NRF_UART_BAUDRATE_115200//uart����������Ϊ115200bps
  };
  //��ʼ�����ڣ�ע�ᴮ���¼��ص�����
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
	if(uart_enabled == false)//��ʼ������
	{
		uart_config();
		uart_enabled = true;
	}
	else
	{
		app_uart_close();//����ʼ������
		uart_enabled = false;
	}
}
//����͸���¼��ص�����������͸�������ʼ��ʱע��
static void uarts_data_handler(ble_uarts_evt_t * p_evt)
{
//	uint32_t getAsynTime=0;
	  //֪ͨʹ�ܺ�ų�ʼ������
	  if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
		{
			uart_reconfig();
		}
		//֪ͨ�رպ󣬹رմ���
		else if(p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
		{
		  uart_reconfig();
		}
	  //�ж��¼�����:���յ��������¼�
    if ((p_evt->type == BLE_UARTS_EVT_RX_DATA) && (uart_enabled == true))
    {
        uint32_t err_code;
        //���ڴ�ӡ���������յ�����
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
			Ble_RecSync = 1;	//APP�·�ͬ��ָ��
			memset(data_array,0,IMU_D_SEND_LEN);
			uRecLen = 0;
			
			NRF_LOG_INFO("Receive Ble SYNC Start Command.");
		}
		else if(p_evt->params.rx_data.p_data[0]==0xFA&&p_evt->params.rx_data.p_data[2]==0xA4&&p_evt->params.rx_data.p_data[4]==0xAA)
		{
			Ble_RecConfig = 1;	//APP�·�ֹͣ�ɼ�
			memset(data_array,0,IMU_D_SEND_LEN);
			uRecLen = 0;
			
			NRF_LOG_INFO("Receive Ble SYNC Stop Command.");
		}
#if 0	////APP�·�ʱ���
		else if(p_evt->params.rx_data.p_data[0]==0xA5 && p_evt->params.rx_data.p_data[3]==0xA5)
		{
			Ble_SyncTime = 1;	//APP�·�ʱ���
			syncAddTime  = (uint16_t)(p_evt->params.rx_data.p_data[1] << 8);
			syncAddTime |= p_evt->params.rx_data.p_data[2];
		}
		else if(p_evt->params.rx_data.p_data[0]==0xFC)
		{
			//���յ�����ͬ��ָ��
			getAsynTime |= (uint32_t)(p_evt->params.rx_data.p_data[2] << 24);
			getAsynTime |= (uint32_t)(p_evt->params.rx_data.p_data[3] << 16);
			getAsynTime |= (uint32_t)(p_evt->params.rx_data.p_data[4] << 8);
			getAsynTime |= p_evt->params.rx_data.p_data[5];

			if(getAsynTime > sys_time - 40)//�ж�ʱ��ֵ
			{
				sys_time = getAsynTime + 40;
				if(AsynFinished)
					AsynFinished = 0;//δ��ɶ�ʱ
			}
			else
			{
				AsynFinished = 1;	//���ʱ��ͬ����־
			}
			getAsynTime = 0;
		}
#endif
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }
		//�ж��¼�����:���;����¼������¼��ں����������õ�����ǰ�����ڸ��¼��з�תָʾ��D4��״̬��ָʾ���¼��Ĳ���
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

//�����ʼ����������ʼ���Ŷ�д��ģ��ͳ�ʼ��Ӧ�ó���ʹ�õķ���
static void services_init(void)
{
    ret_code_t         err_code;
	  //���崮��͸����ʼ���ṹ��
	  ble_uarts_init_t     uarts_init;
	  //�����Ŷ�д���ʼ���ṹ�����
    nrf_ble_qwr_init_t qwr_init = {0};

    //�Ŷ�д���¼�������
    qwr_init.error_handler = nrf_qwr_error_handler;
    //��ʼ���Ŷ�д��ģ��
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
		//��麯������ֵ
    APP_ERROR_CHECK(err_code);
    
		
		/*------------------���´����ʼ������͸������-------------*/
		//���㴮��͸�������ʼ���ṹ��
		memset(&uarts_init, 0, sizeof(uarts_init));
		//���ô���͸���¼��ص�����
    uarts_init.data_handler = uarts_data_handler;
    //��ʼ������͸������
    err_code = ble_uarts_init(&m_uarts, &uarts_init);
    APP_ERROR_CHECK(err_code);
		/*------------------��ʼ������͸������-END-----------------*/
}

//���Ӳ���Э��ģ���¼�������
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;
    //�ж��¼����ͣ������¼�����ִ�ж���
	  //���Ӳ���Э��ʧ�ܣ��Ͽ���ǰ����
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
		//���Ӳ���Э�̳ɹ�
		if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
       //���ܴ���;
    }
}

//���Ӳ���Э��ģ��������¼�������nrf_error�����˴�����룬ͨ��nrf_error���Է���������Ϣ
static void conn_params_error_handler(uint32_t nrf_error)
{
    //���������
	  APP_ERROR_HANDLER(nrf_error);
}


//���Ӳ���Э��ģ���ʼ��
static void conn_params_init(void)
{
    ret_code_t             err_code;
	  //�������Ӳ���Э��ģ���ʼ���ṹ��
    ble_conn_params_init_t cp_init;
    //����֮ǰ������
    memset(&cp_init, 0, sizeof(cp_init));
    //����ΪNULL����������ȡ���Ӳ���
    cp_init.p_conn_params                  = NULL;
	  //���ӻ�����֪ͨ���״η������Ӳ�����������֮���ʱ������Ϊ5��
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	  //ÿ�ε���sd_ble_gap_conn_param_update()�����������Ӳ������������֮��ļ��ʱ������Ϊ��30��
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	  //�������Ӳ���Э��ǰ�������Ӳ���Э�̵�����������Ϊ��3��
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	  //���Ӳ������´������¼���ʼ��ʱ
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	  //���Ӳ�������ʧ�ܲ��Ͽ�����
    cp_init.disconnect_on_fail             = false;
	  //ע�����Ӳ��������¼����
    cp_init.evt_handler                    = on_conn_params_evt;
	  //ע�����Ӳ������´����¼����
    cp_init.error_handler                  = conn_params_error_handler;
    //���ÿ⺯���������Ӳ������³�ʼ���ṹ��Ϊ�����������ʼ�����Ӳ���Э��ģ��
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

//�㲥�¼�������
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;
    //�жϹ㲥�¼�����
    switch (ble_adv_evt)
    {
        //���ٹ㲥�����¼������ٹ㲥�������������¼�
			  case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
			      //���ù㲥ָʾ��Ϊ���ڹ㲥��D1ָʾ����˸��
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        //�㲥IDLE�¼����㲥��ʱ���������¼�
        case BLE_ADV_EVT_IDLE:
					  //���ù㲥ָʾ��Ϊ�㲥ֹͣ��D1ָʾ��Ϩ��
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}
//�㲥��ʼ��
static void advertising_init(void)
{
    ret_code_t             err_code;
	  //����㲥��ʼ�����ýṹ�����
    ble_advertising_init_t init;
    //����֮ǰ������
    memset(&init, 0, sizeof(init));
    //�豸�������ͣ�ȫ��
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	  //�Ƿ������ۣ�����
    init.advdata.include_appearance      = false;
	  //Flag:һ��ɷ���ģʽ����֧��BR/EDR
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	  //UUID�ŵ�ɨ����Ӧ����
	  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
	
    //���ù㲥ģʽΪ���ٹ㲥
    init.config.ble_adv_fast_enabled  = true;
	  //���ù㲥����͹㲥����ʱ��
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    //�㲥�¼��ص�����
    init.evt_handler = on_adv_evt;
    //��ʼ���㲥
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);
    //���ù㲥���ñ�ǡ�APP_BLE_CONN_CFG_TAG�����ڸ��ٹ㲥���õı�ǣ�����Ϊδ��Ԥ����һ���������ڽ�����SoftDevice�汾�У�
		//����ʹ��sd_ble_gap_adv_set_configure()�����µĹ㲥����
		//��ǰSoftDevice�汾��S132 V7.0.1�汾��֧�ֵ����㲥������Ϊ1�����APP_BLE_CONN_CFG_TAGֻ��д1��
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

//BLE�¼�������
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
    //�ж�BLE�¼����ͣ������¼�����ִ����Ӧ����
    switch (p_ble_evt->header.evt_id)
    {
        //�Ͽ������¼�
			  case BLE_GAP_EVT_DISCONNECTED:

				m_conn_handle = BLE_CONN_HANDLE_INVALID;
			  
				bleConState = DISCONNECTED;
			    //��ӡ��ʾ��Ϣ
			    NRF_LOG_INFO("Disconnected.");
//			    uart_reconfig();	//�Ͽ�ʱ���ı�uart״̬��
            break;
				
        //�����¼�
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
			//����ָʾ��״̬Ϊ����״̬����ָʾ��D1����
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
			bleConState = CONNECTED;
			
			//�������Ӿ��
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			//�����Ӿ��������Ŷ�д��ʵ����������Ŷ�д��ʵ���͸����ӹ��������������ж�����ӵ�ʱ��ͨ��������ͬ���Ŷ�д��ʵ�����ܷ��㵥�������������
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            break;
				
        //PHY�����¼�
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
						//��ӦPHY���¹��
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
				//��ȫ���������¼�
				case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            //��֧�����
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
				 
				//ϵͳ���Է������ڵȴ���
				case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            //ϵͳ����û�д洢������ϵͳ����
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
        //GATT�ͻ��˳�ʱ�¼�
        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Client Timeout.");
				    //�Ͽ���ǰ����
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
				
        //GATT��������ʱ�¼�
        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Server Timeout.");
				    //�Ͽ���ǰ����
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

//��ʼ��BLEЭ��ջ
static void ble_stack_init(void)
{
    ret_code_t err_code;
    //����ʹ��SoftDevice���ú����л����sdk_config.h�ļ��е�Ƶʱ�ӵ����������õ�Ƶʱ��
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    
    //���屣��Ӧ�ó���RAM��ʼ��ַ�ı���
    uint32_t ram_start = 0;
	  //ʹ��sdk_config.h�ļ���Ĭ�ϲ�������Э��ջ����ȡӦ�ó���RAM��ʼ��ַ�����浽����ram_start
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    //ʹ��BLEЭ��ջ
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    //ע��BLE�¼��ص�����
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
//��ʼ����Դ����ģ��
static void power_management_init(void)
{
    ret_code_t err_code;
	  //��ʼ����Դ����
    err_code = nrf_pwr_mgmt_init();
	  //��麯�����صĴ������
    APP_ERROR_CHECK(err_code);
}

//��ʼ��ָʾ��
//static void leds_init(void)
//{
//    ret_code_t err_code;
//    //��ʼ��BSPָʾ��
//    err_code = bsp_init(BSP_INIT_LEDS, NULL);
//    APP_ERROR_CHECK(err_code);

//}
//��ʼ��APP��ʱ��ģ��
static void timers_init(void)
{
    //��ʼ��APP��ʱ��ģ��
    ret_code_t err_code = app_timer_init();
	  //��鷵��ֵ
    APP_ERROR_CHECK(err_code);

    //���봴���û���ʱ����Ĵ��룬�����û���ʱ���� 

}
static void log_init(void)
{
    //��ʼ��log����ģ��
	  ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //����log����նˣ�����sdk_config.h�е�������������ն�ΪUART����RTT��
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//����״̬�����������û�й������־��������˯��ֱ����һ���¼���������ϵͳ
static void idle_state_handle(void)
{
    //��������log
	  if (NRF_LOG_PROCESS() == false)
    {
        //���е�Դ�����ú�����Ҫ�ŵ���ѭ������ִ��
			  nrf_pwr_mgmt_run();
    }
}
//�����㲥���ú������õ�ģʽ����͹㲥��ʼ�������õĹ㲥ģʽһ��
static void advertising_start(void)
{
   //ʹ�ù㲥��ʼ�������õĹ㲥ģʽ�����㲥
	 ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	 //��麯�����صĴ������
   APP_ERROR_CHECK(err_code);
}

/**********************************
*������zhenegyx 2020.10.16
*���ܣ��������ڷ�������
*���������͵������ַ������
*���أ���
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
*������zhenegyx 2020.10.16
*���ܣ��������ڷ���ʵʱʱ������
*��������
*���أ���
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
	
	//���ڽ��յ�����ʹ��notify���͸�BLE����
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
	
	//���ڽ��յ�����ʹ��notify���͸�BLE����
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
*������zhenegyx 2021.07.09
*���ܣ�ͬ��ָ�����ʼ���ݲɼ�
*��������
*���أ���
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
*������zhenegyx 2020.12.10
*���ܣ�ͬ��ָ�����ʼͬ��ʱ�Ӻ����ݲɼ�
*��������
*���أ���
***********************************/
static void ble_rec_start_sync(void)
{
	uint32_t err_code;

	if(SYNCStartFlag >= 5)
		return;
	
	SYNCStartFlag = 0;
	
	err_code = ts_tx_start(200);	//��ʼͬ�����豸ʱ��
	APP_ERROR_CHECK(err_code);
	SYNC_PIN_SET();		//interrupt to MCU
	NRF_LOG_INFO("ble_rec_sync_handle SYNC_PIN_SET\r\n");
	
	NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
}

/**********************************
*������zhenegyx 2020.12.10
*���ܣ�ͬ��ָ���ֹͣͬ�������ݲɼ�
*��������
*���أ���
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
*������zhenegyx 2020.11.23
*���ܣ�ͬ��ָ�����ʼͬ�����豸ʱ��
*��������
*���أ���
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
		
		err_code = ts_tx_start(200);	//��ʼͬ�����豸ʱ��
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
		//����LEDָʾ��D1
		nrf_gpio_pin_clear(LED_2);
		//�ȴ������ͷ�
		while(nrf_gpio_pin_read(BUTTON_1) == 0);
		//Ϩ��LEDָʾ��D1
		nrf_gpio_pin_set(LED_2);
		//���°���ͨ����������һ��ʱ��
		//Ble_RecSync = 1;

		//���°�����ʼͬ��
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

//sync timer ����
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
*������zhenegyx 2020.10.16
*���ܣ���������ͬ������
*��������
*���أ���
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

//������
int main(void)
{
	//��ʼ��log����ģ��
	log_init();
	//��ʼ������
//	uart_config();
	//��ʼ��APP��ʱ��
	timers_init();

//	buttons_leds_init();

	
	//��ʹ��������ָʾ��
//	leds_init();
	//������ʼ��
//	bsp_board_init(BSP_INIT_BUTTONS);

	//��ʼ����Դ����
	power_management_init();

//	systime_init();
//	sysRtc_init();
#if 1
	//��ʼ��Э��ջ
	ble_stack_init();
	//����GAP����
	gap_params_init();
	//��ʼ��GATT
	gatt_init();
	//��ʼ������
	services_init();
	//��ʼ���㲥
	advertising_init();	
	//���Ӳ���Э�̳�ʼ��
	conn_params_init();

	sync_timer_init();
	
	SYNC_OUTPUT_SETUP();
	SYNC_PIN_SET();

	NRF_LOG_INFO("BLE Template example started.");  
	//�����㲥
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
  //��ѭ��
	while(true)
	{
		//��������LOG�����е�Դ����
		idle_state_handle();
//		buttons_handle();
		ble_sync_data_process();
//		ble_ondisconnect();
	}
}



