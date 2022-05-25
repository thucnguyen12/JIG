/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_storage_if.h"
#include "tusb.h"
#include "app_debug.h"
#include "fatfs.h"
#include "lwrb.h"
#include <stdbool.h>
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "esp_loader.h"
#include "example_common.h"
#include "md5_hash.h"
#include "app_btn.h"
#include "utilities.h"
#include "usart.h"
#include "stm32_port.h"
#include "app_ethernet.h"
#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip.h"
#include "stdio.h"
#include "app_http.h"
#include "lwip/dns.h"
#include "mqtt_client.h"
#include "min.h"
#include "min_id.h"
#include "ringBuffer.h"
#include "adc.h"
#include "sntp.h"
#include "rtc.h"
#include "time.h"
#include "app_cli.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/***********    BUTTON DEFINE     ******************/
#define BIT_EVENT_GROUP_KEY_0_PRESSED        (1 << 0)
//#define BIT_EVENT_GROUP_KEY_1_PRESSED        (1 << 1)

#define HW_BTN_CONFIG                                       \
{                                                           \
    /* PinName       Last state   Idle level*/              \
	{0,				1,				1						},}
//   {1, 1, 1}, HIEN TAI CO 1 NUT NHAN
/***************************************************/

//*****************   CDC TUSB DEFINE *********************************//

#define CDC_STACK_SZIE      configMINIMAL_STACK_SIZE
#define USB_CDC_TX_RING_BUFFER_SIZE		1024

#define TIMEOUT 8000

//********************************time typedef*********************//
typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} date_time_t;
static const uint8_t day_in_month[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
#define FIRSTYEAR 2000 // start year
#define FIRSTDAY 6     // 0 = Sunday
//8*************************************************
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum
{
	NOT_CONNECTED,
	CONNECT,
	ETHERNET_CONNECTED
} ETHERNET_STATE;
typedef union
{
	struct
	{
		uint32_t rs485: 1;
		uint32_t rs232: 1;
		uint32_t test_wd_ok: 1;
		uint32_t sim_ok :1;
		uint32_t temper_ok: 1;
//		uint32_t vin_ok: 1;
		uint32_t vbat_ok: 1;
		uint32_t v1v8_ok: 1;
		uint32_t v3v3_ok: 1;
		uint32_t v5v_ok: 1;
		uint32_t vgsm4v2_ok: 1;
		uint32_t charge_ok: 1;
		uint32_t alarm_ok:1;
		uint32_t fault_ok:1;
		uint32_t relay0_ok: 1;
		uint32_t relay1_ok: 1;
		uint32_t buttonTest_ok: 1;
		uint32_t sosButton_ok: 1;
		uint32_t vsys_ok:1;
		uint32_t reserved : 13;
	} result;
	uint32_t value;
}__attribute__((packed)) func_test_t;
typedef union
{

	struct

	{

		uint16_t eth : 1;

		uint16_t wifi : 1;

		uint16_t gsm : 1;

		uint16_t server : 1;

		uint16_t input0_pass : 1;

		uint16_t input1_pass : 1;

		uint16_t input2_pass : 1;

		uint16_t input3_pass : 1;

		uint16_t button_pass : 1;

		uint16_t main_power_pass : 1;

		uint16_t backup_power_pass : 1;

		uint16_t reserve : 5;

	} name;

	uint16_t value;

} __attribute__((packed)) jig_peripheral_t;
typedef struct

{

	char gsm_imei[16];

	char sim_imei[16];

	uint8_t mac[6];

	uint16_t gsm_voltage;

	jig_peripheral_t peripheral;

    uint8_t temperature;

    uint8_t device_type;  // 0 = "B01", 1 = "B02"

    uint8_t fw_version[3];  // Major.minor.build

    uint8_t hw_version[3];  // Major.minor.build

    func_test_t test_result;

    uint32_t timestamp;
} __attribute__((packed)) jig_value_t;

typedef struct
{
	uint16_t vin_max;
	uint16_t vin_min;
	uint16_t vbat_max;
	uint16_t vbat_min;
	uint16_t v1v8_max;
	uint16_t v1v8_min;
	uint16_t v3v3_max;
	uint16_t v3v3_min;
	uint16_t v5v_max;
	uint16_t v5v_min;
	uint16_t v4v2_max;
	uint16_t v4v2_min;
	uint16_t vsys_max;
	uint16_t vsys_min;
} __attribute__((packed)) test_info;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static EventBits_t uxBits;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//***************** WATCHDOG TIMER VAR*****************************//
	EventGroupHandle_t m_wdg_event_group = NULL;
#define defaultTaskB 	(1 << 0)
#define cdcTaskB 		(1 << 1)
#define usbTaskB 		(1 << 2)
#define flashTaskB 		(1 << 3)
#define netTaskB 		(1 << 4)

//****************************************************************//
/**************        BUTTON VAR               ***********/
	static app_btn_hw_config_t m_button_cfg[] = HW_BTN_CONFIG;
	static EventGroupHandle_t m_button_event_group = NULL;
/**********************************************************/

/********************   FLASH DISK VARIABLE        **********************************/
	BYTE gFSWork[_MAX_SS];
	UINT br, bw;  // File read/write count
	UINT fbr, fbw;  // File read/write count
	FRESULT flash_res;
	bool m_disk_is_mounted = false;
/*************************************************************************************/

/**************     CDC TASK VAR         *********************************************/
	StackType_t  cdc_stack[CDC_STACK_SZIE];
	StaticTask_t cdc_taskdef;
/*************************************************************************************/
//**************************    tiny USB TASK VAR     ***************************************//
	static TaskHandle_t m_USB_handle = NULL;
	bool tusb_init_flag = false;

//*******************************************************************************************//

/*********************      flash task var             ***********************/
	static TaskHandle_t m_task_connect_handle = NULL;

	static example_binaries_t m_binary;
	static char m_file_address[128];
	volatile uint32_t led_busy_toggle = 0;

	static const char *info_file = "info.txt";
	static const char *bootloader_file = "bootloader.bin";
	static const char *firmware_file = "fire_safe_normal.bin";
	static const char *ota_file = "ota_data_initial.bin";
	static const char *partition_file = "partition-table.bin";

	static const char *chip_des[] = {"ESP8266", "ESP32", "ESP32S2", "ESP32C3", "ESP32S3", "ESP32C2", "ESP32H2", "UNKNOWN"};
/*******************************************************************/

/*********************       NET VAR    **********************************/
	struct netif g_netif;

	static TaskHandle_t m_task_handle_protocol = NULL; //NET APP TASK
	osThreadId DHCP_id;
	SemaphoreHandle_t hHttpStart;
	xQueueHandle httpQueue;
/*****************************************************************/

//************************** MD5 CRC CONFIG VAR ************************//
	static uint8_t m_buffer[4096];
	static MD5Context_t md5_context;
	static esp_loader_config_t m_loader_cfg =
	{
	    .baud_rate = 115200,
	    .reset_trigger_pin = ESP_EN_Pin,
	    .gpio0_trigger_pin = ESP_IO0_Pin,
	    .buffer = m_buffer,
	    .buffer_size = 4096,
	    .sync_timeout = 100,
	    .trials = 10,
	    .md5_context = &md5_context
	};
//*********************************************************************//

//***************************  TESTING TASK VAR   *********************************//
	static TaskHandle_t mTest_Handle_t = NULL;
	char gsm_imei[16];
	char sim_imei[16];
	uint8_t MAC[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t lastMAC[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static jig_value_t *rx_value;
	static jig_value_t *to_send_value;
	jig_peripheral_t jig_result;
	bool get_jig_info = false;
	static func_test_t test_res;
	bool allPassed = false;
	static const char *test_info_file = "test_info.txt";
	char json_send_to_sever[1024];
	test_info voltage_info;
	uint8_t V_info_buff[512];
	uint8_t relay_toggle_0;
	uint8_t relay_toggle_1;
	bool ready_send_to_sever = false;
	// adc read var
	uint16_t ADCScan[6];
	uint8_t idle_detect;
//*****************************************************************************//

//************************* TIME PROTOCOL PRO AND VAR***********************************//
	void lwip_sntp_recv_cb (uint32_t time);
//****************************************************************************
//********************** user name and pwd of wifi **************************//
	static char UserName[32];
	static char PassWord[32];
	//*********************************************************************//

//********************************* MIN PROTOCOL VAR**********************//
	lwrb_t m_ringbuffer_host_rx;
	uint8_t m_rs232_host_buffer[512];
	static uint8_t m_min_rx_buffer[256];
	static min_context_t m_min_context;
	static min_frame_cfg_t m_min_setting = MIN_DEFAULT_CONFIG();
//*******************************************************************//
	// ******************** RING BUFFER********************************//
	uint8_t rs485ringBuffer [6144];
	lwrb_t m_ringbuffer_rs485_rx;
//******************************************************************//
//************************** RTC VAR*******************************//

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_TimeTypeDef sTimeToSend = {0};
  RTC_DateTypeDef sDateToSend = {0};
//  RTC_HandleTypeDef hrtc1;
  static date_time_t date_time;
//********************************************************************//


// ********************* APP CLI VARIABLE****************************//
  uint32_t cdc_tx(const void *buffer, uint32_t size);
  int32_t USB_puts(char *msg);
  static app_cli_cb_t m_tcp_cli =
  {
      .puts = cdc_tx,
      .printf = USB_puts,
      .terminate = NULL
  };
//  static char m_cli_rx_buffer[128];
//  lwrb_t m_ringbuffer_cli_rx;
  void fakeMac (char *MACstring);
  void getTimeNow (void);
  static bool m_cli_started = false;
// ******************************************************************//
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/************      BUTTON FP          ***************/
void button_initialize(uint32_t button_num);

uint32_t btn_read(uint32_t pin);

void on_btn_pressed(int number, int event, void * pData);

void on_btn_release(int number, int event, void * pData);

void on_btn_hold(int number, int event, void * pData);

static void on_btn_hold_so_long(int index, int event, void * pData);
/********************************************************/

/***************  ETHERNET PFP      ****************************/
void Netif_Config (bool restart);
void net_task(void *argument);

/**************************************************************/

/*****************        TASK PFP                *************/
void cdc_task(void* params);

void usb_task (void* params);

void flash_task(void *argument);

void testing_task (void *arg);

/**************************************************************/
//*******************test prototype**********************//

int16_t json_build(jig_value_t *value, func_test_t * test, char *json_str);

void Get_test_result (jig_value_t * value, jig_peripheral_t* peripheral);

//void make_string_from_mac(char* str);

void send_test_command(min_msg_t * test_command);

bool RS232_tx (void *ctx, uint8_t byte);

void Get_sim_imei(jig_value_t * value, char *sim_imei);

void Get_gsm_imei(jig_value_t * value, char *gsm_imei);

void Get_MAC(jig_value_t * value, uint8_t *MAC);
//*******************************************//

// ********************* RTC PFP****************//
void reInitRTC ( RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate);

static void initialize_stnp(void);

static uint32_t convert_date_time_to_second(date_time_t *t);
//*************************************************//

//*******************fatfs var********************//
FILINFO fno;
FRESULT fre;  // result

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
//  MX_LWIP_Init();

  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
/********  DISCONNECT USB TO MAKE RENUM EVENT  *********/
  MX_FATFS_Init();

  vTaskDelay(500);		// time for usb renum

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*            RECONNECT USB         */
  //************************* INIT FUNCTIONS **************************//
  MX_USB_DEVICE_Init();

  m_button_event_group = xEventGroupCreate(); //>>>>>>> CREATE BUTTON EVENT VAR
         ////init lwip
  m_wdg_event_group = xEventGroupCreate();

//*************************** INIT BUTTON APP**********************//
  app_btn_config_t btn_conf;
  btn_conf.config = m_button_cfg;
  btn_conf.btn_count = 1;
  btn_conf.get_tick_cb = xTaskGetTickCount;
  btn_conf.btn_initialize = button_initialize;
  btn_conf.btn_read = btn_read;
  btn_conf.scan_interval_ms = 50;
  app_btn_initialize(&btn_conf);
//    app_btn_initialize(&btn_conf);
  app_btn_register_callback(APP_BTN_EVT_HOLD, on_btn_hold, NULL);
  app_btn_register_callback(APP_BTN_EVT_HOLD_SO_LONG, on_btn_hold_so_long, NULL);
  app_btn_register_callback(APP_BTN_EVT_PRESSED, on_btn_pressed, NULL);
  app_btn_register_callback(APP_BTN_EVT_RELEASED, on_btn_release, NULL);
//************************* INIT BUTTON END **********************//

//****************** MOUNT FLASH DISK *****************************//
  flash_res = f_mount(&USERFatFS, USERPath, 1);
  if (flash_res != FR_OK)
  {
	DEBUG_WARN("Mount flash fail\r\n");
	flash_res = f_mkfs(USERPath, FM_ANY, 0, gFSWork, sizeof gFSWork);
	flash_res = f_mount(&USERFatFS, USERPath, 1);
	if (flash_res == FR_OK)
	{
		m_disk_is_mounted = true;
		DEBUG_INFO ("format disk and mount again\r\n");
	}
	else
	{
		DEBUG_ERROR("Mount flash error\r\n");
	}
  }
  else
  {
	m_disk_is_mounted = true;
	DEBUG_INFO ("Mount flash ok\r\n");
  }
  TCHAR label[32];
  f_getlabel(USERPath, label, 0);
  DEBUG_INFO("Label %s\r\n", label);
  if (strcmp(label, "BSAFE JIG"))
  {
	DEBUG_INFO("Set label\r\n");
	f_setlabel("BSAFE JIG");
  }
/************************************************************/
//*******************start adc ************************/
	HAL_ADC_Start_DMA (&hadc1, (uint32_t*)ADCScan, 6);
	vTaskDelay(500);
//******************************************//
  // Create flashtask
	 httpQueue = xQueueCreate (1, sizeof(uint32_t));
  if (m_USB_handle == NULL)
	{
	  xTaskCreate(usb_task, "usb_task", 1024, NULL, 4, &m_USB_handle);// pio =1
	}

  if (m_task_handle_protocol == NULL)
  {
  	  xTaskCreate(net_task, "net_task", 1024, NULL, 0, &m_task_handle_protocol);
  }
  if (mTest_Handle_t == NULL)
  {
	  xTaskCreate (testing_task, "testing_task", 1024, NULL, 5, &mTest_Handle_t);
  }

#if LWIP_DHCP

//	  /* Start DHCPClient */

	  osThreadDef(DHCP, DHCP_Thread, 2, 0, configMINIMAL_STACK_SIZE * 2);
	  DHCP_id = osThreadCreate (osThread(DHCP), &g_netif);
#endif

	  if (m_task_connect_handle == NULL)
	  {
		  xTaskCreate(flash_task, "flash_task", 1024, NULL, 3, &m_task_connect_handle);// pio =1
	  }
//	  lwrb_init (&m_ringbuffer_cli_rx, &m_cli_rx_buffer, sizeof (m_cli_rx_buffer));
  /* Infinite loop */
	  HAL_RTC_GetTime (&hrtc, &sTimeToSend, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate (&hrtc, &sDateToSend, RTC_FORMAT_BIN);
	  DEBUG_INFO ("GET TIME: %d: %d: %d \r\n", (uint8_t)(sTimeToSend.Hours), (uint8_t)(sTimeToSend.Minutes),(uint8_t)(sTimeToSend.Seconds));
	  DEBUG_INFO ("GET date: %d: %d: %d \r\n", (uint8_t)(sDateToSend.Date), (uint8_t)(sDateToSend.Month),(uint8_t)(sDateToSend.Year));
  for(;;)
  {
//	HAL_GPIO_TogglePin (LED_DONE_GPIO_Port, LED_DONE_Pin);
	  app_btn_scan(NULL);
	  if (led_busy_toggle == 0)
	  {
		  HAL_GPIO_WritePin (LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	  }

	  if (m_cli_started == false)
	  {
		  m_cli_started = true;
		  app_cli_start(&m_tcp_cli);
		  DEBUG_WARN ("APP CLI STARTED \r\n");
	  }

//	  while (lwrb_get_full(&m_ringbuffer_cli_rx))
//	  {
////		  DEBUG_VERBOSE ("NEED BREAK POINT\r\n");
//		  uint8_t ch = lwrb_read (&m_ringbuffer_cli_rx, &ch,1);
////		  app_cli_poll(ch);
//	  }

	  xEventGroupSetBits(m_wdg_event_group, defaultTaskB);
	  osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/****************      CDC FUNCTION TASK          *************/

//   app cli  tasking//
void tusb_read_callback (void)
{
	while (1)
{
	uint8_t usb_ch;
	uint32_t count = tud_cdc_read(&usb_ch, 1);
	if (count)
	{
//				DEBUG_WARN ("THERE IS DATA FROM USB \r\n");
//		lwrb_write (&m_ringbuffer_cli_rx, &usb_ch, 1);
		app_cli_poll(usb_ch);
	}
	else
	{
		break;
	}
}
}

static bool m_cdc_debug_register = false;
static lwrb_t m_ringbuffer_usb_cdc_tx;
static uint8_t m_lwrb_tx_raw_buffer[USB_CDC_TX_RING_BUFFER_SIZE];
uint32_t cdc_tx(const void *buffer, uint32_t size)
{
	lwrb_write(&m_ringbuffer_usb_cdc_tx, buffer, size);
	return size;
}
void usb_task (void* params)
{
	DEBUG_INFO ("USB TASK\r\n");
	tusb_init_flag = tusb_init ();
	// Create CDC task
	(void) xTaskCreateStatic(cdc_task, "cdc", CDC_STACK_SZIE, NULL, 1, cdc_stack, &cdc_taskdef);// pio =2

	while (1)
	{
		tud_task();
	}
}
void cdc_task(void* params)
{
	DEBUG_INFO("ENTER CDC TASK\r\n");
	lwrb_init(&m_ringbuffer_usb_cdc_tx, m_lwrb_tx_raw_buffer, USB_CDC_TX_RING_BUFFER_SIZE);
	for (;;)
	{
//	    // connected() check for DTR bit
//	    // Most but not all terminal client set this when making connection
//		tusb_read_callback();
		if (tud_cdc_connected())
		{
			if (m_cdc_debug_register == false)
			{
				m_cdc_debug_register = true;
//				app_debug_register_callback_print(cdc_tx);
			}
			if (tud_cdc_available())
			{
				tusb_read_callback();
			}
		}
		else
		{
			if (m_cdc_debug_register)
			{
				m_cdc_debug_register = false;
				app_debug_unregister_callback_print(cdc_tx);
				// Flush all cdc tx buffer
				char tmp[1];
				while (lwrb_read(&m_ringbuffer_usb_cdc_tx, tmp, 1))
				{

				}
			}
		}

		char buffer[ (TUD_OPT_HIGH_SPEED ? 512 : 64)];
		uint32_t size;
		while (1)
		{
			uint32_t avai = tud_cdc_write_available();
			if (avai >= sizeof(buffer))
			{
				avai = sizeof(buffer);
			}
			size = lwrb_read(&m_ringbuffer_usb_cdc_tx, buffer, avai);
			if (size)
			{
				tud_cdc_write(buffer, size);
				tud_cdc_write_flush();
			}
			else
			{
				break;
			}
		}
		xEventGroupSetBits(m_wdg_event_group, cdcTaskB);
//		DEBUG_INFO ("CDC TASK \r\n");
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
/**********************************************************************/

//************************** FLASH MAIN BOARD TASK*******************************************//

void flash_task(void *argument)
{
	DEBUG_INFO("ENTER flash TASK\r\n");
	int32_t file_size = 0;
	 GPIO_InitTypeDef GPIO_InitStruct1 = {0};
	 while (!tusb_init_flag)
	 {
		 vTaskDelay(100);
	 }
	 while (!m_ip_assigned)
	 {
		 vTaskDelay(100);
	 }
	 DEBUG_INFO ("SEM TOOK \r\n");
	if (m_disk_is_mounted)
	{
//		vTaskDelay (200);
		file_size = fatfs_read_file(info_file, (uint8_t*)m_file_address, sizeof(m_file_address) - 1);
		if (file_size > 0)
		{
			DEBUG_INFO ("READ THE INFO FILE \r\n");
			/*
			{
				"boot":4096,
				"firmware":65536,
				"ota_data":2555904,
				"partition":32768
			}
			*/
			char *ptr = strstr(m_file_address, "\"boot\":");
			if (ptr)
			{
				DEBUG_INFO ("FOUND THE BOOT STRING\r\n");
				ptr += strlen("\"boot\":");
				m_binary.boot.addr = utilities_get_number_from_string(0, ptr);
			}

			ptr = strstr(m_file_address, "\"firmware\":");
			if (ptr)
			{
				ptr += strlen("\"firmware\":");
				m_binary.firm.addr = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr(m_file_address, "\"ota_data\":");
			if (ptr)
			{
				ptr += strlen("\"ota_data\":");
				m_binary.ota.addr = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr(m_file_address, "\"partition\":");
			if (ptr)
			{
				ptr += strlen("\"partition\":");
				m_binary.part.addr = utilities_get_number_from_string(0, ptr);
			}
		}

		file_size = fatfs_get_file_size(bootloader_file);
		if (file_size > -1)
		{
			m_binary.boot.size = file_size;
			m_binary.boot.file_name = bootloader_file;
		}

		file_size = fatfs_get_file_size(firmware_file);
		if (file_size > -1)
		{
			m_binary.firm.size = file_size;
			m_binary.firm.file_name = firmware_file;
		}

		file_size = fatfs_get_file_size(ota_file);
		if (file_size > -1)
		{
			m_binary.ota.size = file_size;
			m_binary.ota.file_name = ota_file;
		}

		file_size = fatfs_get_file_size(partition_file);
		if (file_size > -1)
		{
			m_binary.part.size = file_size;
			m_binary.part.file_name = partition_file;
		}

		DEBUG_INFO("Bootloader offset 0x%08X, firmware 0x%08X, ota  0x%08X, partition table 0x%08X\r\n", m_binary.boot.addr, m_binary.firm.addr, m_binary.ota.addr, m_binary.part.addr);
		DEBUG_INFO("Bootloader %u bytes, firmware %u bytes, ota %u bytes, partition table %u bytes\r\n", m_binary.boot.size, m_binary.firm.size, m_binary.ota.size, m_binary.part.size);
	}
	m_loader_cfg.gpio0_trigger_port = (uint32_t)ESP_IO0_GPIO_Port;
	m_loader_cfg.reset_trigger_port = (uint32_t)ESP_EN_GPIO_Port;
	m_loader_cfg.uart_addr = (uint32_t)USART2;
	esp_loader_error_t err;

	// Clear led busy & success, set led error
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_DONE_GPIO_Port, LED_DONE_Pin, GPIO_PIN_SET);

	for (;;)
	{
		if (0)
		{
		DEBUG_INFO("ENTER flash LOOP\r\n");
		if (led_busy_toggle > 10)
		{
			led_busy_toggle = 1;
		}
		xEventGroupWaitBits(m_button_event_group,
								BIT_EVENT_GROUP_KEY_0_PRESSED,
								pdTRUE,
								pdFALSE,
								portMAX_DELAY);
		//   THRERE NO KEY NOW
		DEBUG_INFO("KEY IS PRESSED\r\n");
		USART2->CR1 |= (uint32_t)(1<<2);
		DEBUG_INFO ("RECIEVE ENABLE \r\n");
		GPIO_InitStruct1.Pin = ESP_EN_Pin|ESP_IO0_Pin;
		GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct1.Pull = GPIO_NOPULL;
		GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct1);
		DEBUG_INFO ("REINT IO0 \r\n");
		HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_DONE_GPIO_Port, LED_DONE_Pin, GPIO_PIN_SET);
		uint32_t now = xTaskGetTickCount();
		uint32_t retry = 4;
		while (m_binary.part.size > 0
				&& m_binary.firm.size > 0
				&& m_binary.ota.size >0
				&& m_binary.part.size > 0
				)
		{
			if (retry == 0)
			{
				break;
			}
			retry--;
			loader_port_change_baudrate(&m_loader_cfg, 115200);
			DEBUG_INFO("Connecting to target remain %u times\r\n", retry);
			led_busy_toggle = 1000000;
			err = esp_loader_connect(&m_loader_cfg);
			if (err != ESP_LOADER_SUCCESS)
			{
				DEBUG_ERROR("Connect to target failed %d\r\n", err);
				HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_RESET);
				continue;
			}
			else
			{
				DEBUG_INFO("Connected to target %s\r\n", chip_des[m_loader_cfg.target]);
			}
			DEBUG_INFO("Change baudrate\r\n");
			err = esp_loader_change_baudrate(&m_loader_cfg, 115200);
			if (err == ESP_LOADER_ERROR_UNSUPPORTED_FUNC)
			{
				DEBUG_ERROR("ESP8266 does not support change baudrate command\r\n");
			}
			else if (err != ESP_LOADER_SUCCESS)
			{
				DEBUG_ERROR("Unable to change baud rate on target\r\n");
			}
			else
			{
				err = loader_port_change_baudrate(&m_loader_cfg, 115200);
				if (err != ESP_LOADER_SUCCESS)
				{
					DEBUG_ERROR("Unable to change baud rate\r\n");
				}
				else
				{
					DEBUG_INFO("Port[%u] : Baudrate changed\r\n");
				}
			}

			DEBUG_INFO("Flash bootloader\r\n");
			if (flash_binary_stm32(&m_loader_cfg, &m_binary.boot) != ESP_LOADER_SUCCESS)
			{
				DEBUG_INFO("FLASH BOOTLOADER FAIL \r\n");
				HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
				vTaskDelay(4000);
				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
				xEventGroupClearBits(m_button_event_group,
									BIT_EVENT_GROUP_KEY_0_PRESSED);
				break;
			}

			DEBUG_INFO("Flash firm\r\n");
			if (flash_binary_stm32(&m_loader_cfg, &m_binary.firm) != ESP_LOADER_SUCCESS)
			{
				DEBUG_INFO("FLASH firmware FAIL \r\n");
				HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
				vTaskDelay(4000);
				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
				xEventGroupClearBits(m_button_event_group,
									BIT_EVENT_GROUP_KEY_0_PRESSED);
				break;
			}
#warning "Chua nap thanh cong file ota"
//			DEBUG_INFO("Flash ota\r\n");
//			if (flash_binary_stm32(&m_loader_cfg, &m_binary.ota) != ESP_LOADER_SUCCESS)
//			{
//				DEBUG_INFO("FLASH OTA FAIL \r\n");
//				HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
//				vTaskDelay(4000);
//				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
//				xEventGroupClearBits(m_button_event_group,
//									BIT_EVENT_GROUP_KEY_0_PRESSED);
//				break;
//			}

			DEBUG_INFO("Flash partition table\r\n");
			if (flash_binary_stm32(&m_loader_cfg, &m_binary.part) != ESP_LOADER_SUCCESS)
			{
				DEBUG_INFO("FLASH partition FAIL \r\n");
				HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
				vTaskDelay(4000);
				HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
				xEventGroupClearBits(m_button_event_group,
									BIT_EVENT_GROUP_KEY_0_PRESSED);

				break;
			}
			retry = 0;
			xEventGroupClearBits(m_button_event_group,
								BIT_EVENT_GROUP_KEY_0_PRESSED);
			DEBUG_INFO ("Total flash write time %us\r\n", (xTaskGetTickCount() - now)/1000);
			HAL_GPIO_WritePin (LED_ERROR_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOF, LED_DONE_Pin, GPIO_PIN_RESET);
			if (led_busy_toggle > 10)
			{
				led_busy_toggle = 1;
			}

			loader_port_change_baudrate (&m_loader_cfg, 115200);
			// loader_port_reset_target(&m_loader_cfg);
			// Led success on, led busy off
			HAL_GPIO_WritePin(LED_DONE_GPIO_Port, LED_DONE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);

			//// io0 reint
			GPIO_InitStruct1.Pin = ESP_IO0_Pin;
			GPIO_InitStruct1.Mode = GPIO_MODE_ANALOG;
			GPIO_InitStruct1.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(ESP_IO0_GPIO_Port, &GPIO_InitStruct1);
			HAL_GPIO_WritePin((GPIO_TypeDef*)m_loader_cfg.gpio0_trigger_port, m_loader_cfg.gpio0_trigger_pin, GPIO_PIN_SET);
			/// end reint
			DEBUG_INFO ("SET GPIO0 ANALOG MODE \r\n");
			USART2->CR1 &= (uint32_t)(~(1<<2));
			DEBUG_INFO ("RE IS DISABLE \r\n");
			break;
		}
		vTaskDelay(45000);
		DEBUG_INFO ("WAIT 45S DONE \r\n");
		}
		else
		{
//			DEBUG_INFO ("IN FLASH TASK \r\n");
			xEventGroupSetBits(m_wdg_event_group, flashTaskB);
			vTaskDelay(10);
		}
	}
}

//*******************************************************************************************//

//***************************** NET CONFIG FUNCTION *************************//
void Netif_Config (bool restart)
{
	ip4_addr_t ipaddr;
	ip4_addr_t netmask;
	ip4_addr_t gw;
	  /* IP addresses initialization with DHCP (IPv4) */
	  ipaddr.addr = 0;
	  netmask.addr = 0;
	  gw.addr = 0;
	  if (restart)
	  {
			netif_remove (&g_netif);
			DEBUG_INFO ("NET IF REMOVE \r\n");
	  /* Start DHCP negotiation for a network interface (IPv4) */
	  }
	  /* add the network interface (IPv4/IPv6) with RTOS */
	  netif_add(&g_netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);//&tcpip_input =>null

	  /* Registers the default network interface */
	  netif_set_default(&g_netif);
	  netif_set_link_callback(&g_netif, ethernet_link_status_updated);
	  if (netif_is_link_up(&g_netif))
	  {
	    /* When the netif is fully configured this function must be called */
	    netif_set_up(&g_netif);
	  }
	  else
	  {
	    /* When the netif link is down this function must be called */
	    netif_set_down(&g_netif);
	  }
	  app_ethernet_notification(&g_netif);
	  /* Set the link callback function, this function is called on change of link status*/


	  DEBUG_INFO ("SET LINK CALLBACK \r\n");

}
//****************************************************************************//

//*********************    NET APP TASK       **********************************//
void net_task(void *argument)
{
	DEBUG_INFO("ENTER THE HTTP AND MQTT TASK\r\n");
//	xSemaphoreTake(hHttpStart, portMAX_DELAY);
//	DEBUG_INFO ("PASS SEM TAKE \r\n");
//	m_http_test_started = true;
//	while (!m_ip_assigned)
//		 {
//			 vTaskDelay(10);
//		 }
//	  initialize_stnp();
//	  osDelay (1);
//	DEBUG_WARN ("GOT IP START TO SEND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
#if 1
	// http://httpbin.org/get

	jig_value_t* rev_jig_value;
	app_http_config_t http_cfg;
	uint16_t len;
	ETHERNET_STATE e_state = NOT_CONNECTED;
	char file_name [64];
	for (;;)
	{
		switch (e_state)
		{
		case NOT_CONNECTED:
			if (m_ip_assigned) e_state = CONNECT;
			xEventGroupSetBits(m_wdg_event_group, netTaskB);
			break;
		case CONNECT:
			initialize_stnp();
			if (check_file ("offline_test") == 1)
			{
				fre = create_a_dir ("offline_test");
			}
			//scan file sequence
			{
				DIR dir;
				UINT i;
				char path[20];
				sprintf (path, "%s","offline_test");
				fre = f_opendir(&dir, path);
				if (fre == FR_OK)
				{
					DEBUG_INFO ("OPEN DIR OK\r\n");
//					for (;;)
//					{
//						fre = f_readdir(&dir, &fno);                   /* Read a directory item */
//						if (fre != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
//						if (fno.fattrib & AM_DIR)     /* It is a directory */
//						{
//							if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
////							i = strlen(path);
////							sprintf(&path[i], "/%s", fno.fname);
//////							fre = scan_files(path);                     /* Enter the directory */
//////							if (fre != FR_OK) break;
////							path[i] = 0;
//						}
//						else
//						{                                       /* It is a file. */
//						   if (!check_file (fno.fname))
//						   {
//							   len = fatfs_read_file (fno.fname, (uint8_t *)json_send_to_sever, 1024);
//								if (len)
//								{
//									DEBUG_INFO ("json from file: %s\r\n", json_send_to_sever);
//									sprintf(http_cfg.url, "%s", "dev-api.basato.vn");
//									http_cfg.port = 80;
//									sprintf(http_cfg.file, "%s", "/fact/api/firesafe/test-result");
//									http_cfg.on_event_cb = (void*)0;
//									http_cfg.method = APP_HTTP_POST;
//									trans_content_to_body ((uint8_t *) json_send_to_sever, len);
//									DEBUG_INFO ("SEND");
//									app_http_start(&http_cfg,  (int)len);
//									delete_a_file (fno.fname);
//									DEBUG_INFO ("SEND OFFLINE FILE\r\n");
//								}
//						   }
//						}
//					}
					f_closedir(&dir);
				}
			}

//			if(!check_file ("offline_file"))
//			{
//				len = fatfs_read_file ("offline_file", (uint8_t *)json_send_to_sever, 1024);
//				if (len)
//				{
//					DEBUG_INFO ("json from file: %s\r\n", json_send_to_sever);
//					sprintf(http_cfg.url, "%s", "dev-api.basato.vn");
//					http_cfg.port = 80;
//					sprintf(http_cfg.file, "%s", "/fact/api/firesafe/test-result");
//					http_cfg.on_event_cb = (void*)0;
//					http_cfg.method = APP_HTTP_POST;
//					trans_content_to_body ((uint8_t *) json_send_to_sever, len);
//					DEBUG_INFO ("SEND");
//					app_http_start(&http_cfg,  (int)len);
//					delete_a_file ("offline_file");
//					DEBUG_INFO ("SEND OFFLINE FILE\r\n");
//				}
//			}
			e_state = ETHERNET_CONNECTED;
			break;
		case ETHERNET_CONNECTED:
			if (!eth_is_cable_connected (&g_netif) )
			{
				e_state = NOT_CONNECTED;
				m_ip_assigned = false;
			}
			break;
		default:
			break;
		}
		if (xQueueReceive(httpQueue, &rev_jig_value, 10) == (BaseType_t)1)
		{
			len = json_build (rev_jig_value, &(rev_jig_value->test_result), json_send_to_sever);
			vPortFree(rev_jig_value);
			if (eth_is_cable_connected (&g_netif) )
			{
				sprintf(http_cfg.url, "%s", "dev-api.basato.vn");
				http_cfg.port = 80;
				sprintf(http_cfg.file, "%s", "/fact/api/firesafe/test-result");
				http_cfg.on_event_cb = (void*)0;
				http_cfg.method = APP_HTTP_POST;
				trans_content_to_body ((uint8_t *) json_send_to_sever, len);
				app_http_start(&http_cfg,  (int)len);
			}
			else
			{
				if (check_file ("offline_test") == 1)
				{
					create_a_dir ("offline_test");
				}
				{
				DIR dir;
				UINT i;
				fre = f_opendir(&dir, "offline_test");
				if (fre == FR_OK)
				{
					RTC_DateTypeDef sDateWriteFile = {0};
					HAL_RTC_GetDate (&hrtc, &sDateWriteFile, RTC_FORMAT_BIN);
					sprintf (file_name, "offline_test/test%d-%d-%d", sDateWriteFile.Date, sDateWriteFile.Month, sDateWriteFile.Year);
					uint32_t byte_write = fatfs_write_to_a_file (file_name, json_send_to_sever, (uint32_t)len);
					if (byte_write == len)
					{
						DEBUG_INFO ("WRITE FILE OK \r\n");
					}
					else
					{
						DEBUG_ERROR ("WRITE FILE ERROR \r\n");
					}

					f_closedir(&dir);
				}
				}
//				uint32_t byte_write = fatfs_write_to_a_file ("offline_file", json_send_to_sever, (uint32_t)len);
//				if (byte_write == len)
//				{
//					DEBUG_INFO ("WRITE FILE OK \r\n");
//				}
//				else
//				{
//					DEBUG_ERROR ("WRITE FILE ERROR \r\n");
//				}
			}
			xEventGroupSetBits(m_wdg_event_group, netTaskB);
		}
		else
		{
			xEventGroupSetBits(m_wdg_event_group, netTaskB);
		}
		osDelay (10);
//		if (eth_is_cable_connected (&g_netif) )
//		{
////			DEBUG_INFO ("NOW ONLINE \r\n");
//			if(!check_file ("offline_file"))
//			{
//				len = fatfs_read_file ("offline_file", json_send_to_sever, 1024);
//				sprintf(http_cfg.url, "%s", "dev-api.basato.vn");
//				http_cfg.port = 80;
//				sprintf(http_cfg.file, "%s", "/fact/api/firesafe/test-result");
//				http_cfg.on_event_cb = (void*)0;
//				http_cfg.method = APP_HTTP_POST;
//				trans_content_to_body ((uint8_t *) json_send_to_sever, len);
//				DEBUG_INFO ("SEND");
//				app_http_start(&http_cfg,  (int)len);
//				delete_a_file ("offline_file");
//				DEBUG_INFO ("SEND OFFLINE FILE\r\n");
//			}
//		}
////		else
////		{
//////			DEBUG_INFO ("NOW OFFLINE \r\n");
////		}
//			if (xQueueReceive(httpQueue, &rev_jig_value, 10) == (BaseType_t)1)
//			{
//			DEBUG_WARN ("GOT THE QUEUE \r\n");
//			len = json_build (rev_jig_value, &(rev_jig_value->test_result), json_send_to_sever);
//			vPortFree(rev_jig_value);
//			DEBUG_INFO ("%s\r\n", json_send_to_sever);
//			if (!eth_is_cable_connected (&g_netif) )
//			{
//
//			}
//
//			m_http_test_started = true;
//			//app_http_config_t http_cfg;
//			sprintf(http_cfg.url, "%s", "dev-api.basato.vn");
//			http_cfg.port = 80;
//			sprintf(http_cfg.file, "%s", "/fact/api/firesafe/test-result");
//			http_cfg.on_event_cb = (void*)0;
//			http_cfg.method = APP_HTTP_POST;
//			trans_content_to_body ((uint8_t *) json_send_to_sever, len);
//			DEBUG_INFO ("SEND");
//			app_http_start(&http_cfg,  (int)len);
//			}
//			else
//			{
//
//				xEventGroupSetBits(m_wdg_event_group, netTaskB);
//			}
//			osDelay(10);
	}
#else
    mqtt_client_cfg_t mqtt_cfg =
    {
        .periodic_sub_req_s = 120,            // second
        .broker_addr = "broker.hivemq.com",
        .port = 1883,
        .password = NULL,
        .client_id = "test_lwip_porting",
    };
    mqtt_client_initialize(&mqtt_cfg);
	for(;;)
	{
//		xSemaphoreTake(hHttpStart, portMAX_DELAY);
		mqtt_client_polling_task(NULL);
		osDelay(1000);

	}
#endif
//	for(;;)
//	{
//
//		osDelay(1000);
//
//	}
}


//*********************** TESING TASK *************************//


void send_test_command(min_msg_t * test_command)
{
	min_send_frame (&m_min_context, test_command);
}
void Get_MAC(jig_value_t * value, uint8_t *MAC)
{
	for (uint8_t i = 0 ; i < 6; i++)
	{
		MAC[i] = value ->mac[i];
	}
}


void min_rx_callback (void *min_context , min_msg_t *frame)
{
	switch (frame ->id)
	{
	case MIN_ID_RS232_ENTER_TEST_MODE:
		rx_value = (jig_value_t *)frame->payload;
		Get_MAC(rx_value, (uint8_t *)MAC);
		DEBUG_INFO ("MAC: %02x: %02x: %02x: %02x: %02x: %02x\r\n", MAC[0], MAC[1], MAC[2],MAC[3],MAC[4],MAC[5]);
		get_jig_info = true;
		test_res.result.rs232 = 1;
		break;
	case MIN_ID_RS232_ESP32_RESET:
		test_res.result.test_wd_ok = 1;
		test_res.result.rs232 = 1;
		break;
	default:
		test_res.result.rs232 = 0;
		break;
	}
}
bool RS232_tx (void *ctx, uint8_t byte)
{
	(void)ctx;
	putChar (USART3, byte);
	return true;
}

void volTest(void)
{
	DEBUG_VERBOSE ("VOL TESTING ENTER \r\n");
	uint8_t res_cnt;
	uint16_t VolRes[6];
	for (uint8_t i = 0; i < 6; i ++)
	{
		VolRes [i] = (ADCScan[i]*3300/4095);
		VolRes [i] = VolRes[i] *2;
	}
	DEBUG_VERBOSE ("V4V2 : %d mV\r\n", VolRes [0]);
	DEBUG_VERBOSE ("VBAT : %d mV\r\n", VolRes [1]);
	DEBUG_VERBOSE ("V5v : %d mV\r\n", VolRes [2]);
	DEBUG_VERBOSE ("V3v3 : %d mV\r\n", VolRes [3]);
	DEBUG_VERBOSE ("V1v8 : %d mV\r\n", VolRes [4]);
	DEBUG_VERBOSE ("Vsys : %d mV\r\n", VolRes [5]);
	res_cnt = 0;
	if (voltage_info.v4v2_min <= VolRes[0] && VolRes[0] <= voltage_info.v4v2_max)
	{
		test_res.result.vgsm4v2_ok = 1;
		res_cnt++;
	}
	else
	{
		test_res.result.vgsm4v2_ok = 0;
	}
	if (voltage_info.vbat_min <= VolRes[1]&& VolRes[1] <= voltage_info.vbat_max)
	{
		test_res.result.vbat_ok = 1;
		res_cnt++;
	}
	else
	{
		test_res.result.vbat_ok = 0;
		res_cnt++;
	}
	if (voltage_info.v5v_min <= VolRes[2] && VolRes[2] <= voltage_info.v5v_max)
	{
		test_res.result.v5v_ok = 1;
		res_cnt++;
	}
	else
	{
		test_res.result.v5v_ok = 0;
	}
	if (voltage_info.v3v3_min <= VolRes[3] && VolRes[3] <= voltage_info.v3v3_max)
	{
		test_res.result.v3v3_ok = 1;
		res_cnt++;
	}
	else
	{
		test_res.result.v3v3_ok = 0;
	}
	if (voltage_info.v1v8_min <= VolRes[4] && VolRes[4] <= voltage_info.v1v8_max)
	{
		test_res.result.v1v8_ok = 1;
		res_cnt++;
	}
	else
	{
		test_res.result.v1v8_ok = 0;
	}
	if (voltage_info.vsys_min <= VolRes[4] && VolRes[4] <= voltage_info.vsys_max)
	{
		test_res.result.vsys_ok = 1;
		res_cnt++;
	}
	else
	{
		test_res.result.vsys_ok = 0;
	}
	if (res_cnt == 6)
	{
//		return true;
		DEBUG_VERBOSE ("VOLTAGE OK\r\n");
	}
	else
	{
		DEBUG_VERBOSE ("VOLTAGE FAIL \r\n");
	}
//	return false;
}

bool PassTest (jig_value_t * value)
{
	if (strlen (value->gsm_imei) >= 15)
	{
		test_res.result.sim_ok = 1;
		DEBUG_VERBOSE ("SIM OK \r\n");
	}
	else
	{
		test_res.result.sim_ok = 0;
		DEBUG_VERBOSE ("SIM NOT OK \r\n");
	}
	if (25 <= value->temperature && value->temperature <=50)
	{
		test_res.result.temper_ok = 1;
		DEBUG_VERBOSE ("TEMPER IS OK \r\n");
	}
	else
	{
		test_res.result.temper_ok = 0;
		DEBUG_VERBOSE ("TEMPER IS not OK \r\n");
	}
	if (test_res.result.rs232
		&& test_res.result.rs485
		&& test_res.result.relay0_ok
		&& test_res.result.relay1_ok
		&& test_res.result.v1v8_ok
		&& test_res.result.v3v3_ok
		&& test_res.result.v5v_ok
		&& test_res.result.vbat_ok
		&& test_res.result.vsys_ok
		&& test_res.result.sim_ok
		&& test_res.result.test_wd_ok
		&& test_res.result.temper_ok
		)
	{
		allPassed = true;
		DEBUG_VERBOSE ("ALL TEST RESULT PASS \r\n");
		return true;
	}
	else
	{
		allPassed = false;
		return false;
	}
	return 0;
}

void testing_task (void *arg)
{
	//init min protocol
	lwrb_init (&m_ringbuffer_rs485_rx, &rs485ringBuffer, sizeof (rs485ringBuffer));
	lwrb_init (&m_ringbuffer_host_rx, &m_rs232_host_buffer, sizeof (m_rs232_host_buffer));
	m_min_setting.get_ms = sys_get_ms;
	m_min_setting.last_rx_time = 0x00;
	m_min_setting.rx_callback = min_rx_callback;
	m_min_setting.timeout_not_seen_rx = 5000;
	m_min_setting.tx_byte = RS232_tx;
	m_min_setting.use_timeout_method = 1;

	m_min_context.callback = &m_min_setting;
	m_min_context.rx_frame_payload_buf = m_min_rx_buffer;
	min_init_context(&m_min_context);
	uint32_t last_tick = 0;
	uint32_t last_tick_time_out = 0;
	uint32_t last_vol_tick = 0;
//	uint32_t json_len;
	const min_msg_t test_cmd =
	{
			.id = MIN_ID_RS232_ENTER_TEST_MODE,
			.len = 0,
			.payload = NULL
	};
	const min_msg_t reset_cmd_wd =
	{
			.id = MIN_ID_TEST_WATCHDOG,
			.len = 0,
			.payload = NULL
	};
	if (m_disk_is_mounted)
	{
		DEBUG_VERBOSE ("READ VOLTAGE CONFIG FILE\r\n");
		uint32_t file_size = fatfs_read_file(test_info_file, (uint8_t*)V_info_buff, sizeof(V_info_buff) - 1);
		/*
		 * {vbatmax:4.3,
		 * 	vbatmin:4.5,
		 *
		 *
		 * }
		 * */
		DEBUG_VERBOSE ("READ %d byte size\r\n", file_size);
		DEBUG_VERBOSE ("%s", V_info_buff);
		if (file_size > 0)
		{
			char *ptr = strstr((char*)V_info_buff, "\"vbat_max\":");
			if (ptr)
			{
				ptr+=strlen("\"vbat_max\":");
				voltage_info.vbat_max = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff,"\"vbat_min\":");
			if (ptr)
			{
				ptr+=strlen("\"vbat_min\":");
				voltage_info.vbat_min = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v1v8_max\":");
			if (ptr)
			{
				ptr+=strlen("\"v1v8_max\":");
				voltage_info.v1v8_max = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v1v8_min\":");
			if (ptr)
			{
				ptr+=strlen("\"v1v8_min\":");
				voltage_info.v1v8_min = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v3v3_max\":");
			if (ptr)
			{
				ptr+=strlen("\"v3v3_max\":");
				voltage_info.v3v3_max = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v3v3_min\":");
			if (ptr)
			{
				ptr+=strlen("\"v3v3_min\":");
				voltage_info.v3v3_min = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v5v_max\":");
			if (ptr)
			{
				ptr+=strlen("\"v5v_max\":");
				voltage_info.v5v_max = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v5v_min\":");
			if (ptr)
			{
				ptr+=strlen("\"v5v_min\":");
				voltage_info.v5v_min = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v4v2_max\":");
			if (ptr)
			{
				ptr+=strlen("\"v4v2_max\":");
				voltage_info.v4v2_max = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v4v2_min\":");
			if (ptr)
			{
				ptr+=strlen("\"v4v2_min\":");
				voltage_info.v4v2_min = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v4v2_max\":");
			if (ptr)
			{
				ptr+=strlen("\"vsys_max\":");
				voltage_info.vsys_max = utilities_get_number_from_string (0, ptr);
			}
			ptr = strstr((char*)V_info_buff, "\"v4v2_min\":");
			if (ptr)
			{
				ptr+=strlen("\"vsys_min\":");
				voltage_info.vsys_min = utilities_get_number_from_string (0, ptr);
			}
#warning "viet them ham tim ten va mat khau wifi"
			DEBUG_INFO ("%d < vbat < %d (mV) \r\n", voltage_info.vbat_min, voltage_info.vbat_max);
			DEBUG_INFO ("%d < v5v < %d (mV) \r\n", voltage_info.v5v_min, voltage_info.v5v_max);
			DEBUG_INFO ("%d < v1v8 < %d (mV) \r\n", voltage_info.v1v8_min, voltage_info.v1v8_max);
			DEBUG_INFO ("%d < v3v3 < %d (mV) \r\n", voltage_info.v3v3_min, voltage_info.v3v3_max);
			DEBUG_INFO ("%d < v4v2 < %d (mV) \r\n", voltage_info.v4v2_min, voltage_info.v4v2_max);
			DEBUG_INFO ("%d < vsys < %d (mV) \r\n", voltage_info.vsys_min, voltage_info.vsys_max);
		}
	}
	bool sent_to_sever =false;
	send_test_command (&test_cmd);
	for (;;)
	{
//		DEBUG_INFO ("ENTER TESTING LOOP \r\n");
		uint32_t now = HAL_GetTick();
		uint8_t ch;
		if (lwrb_read(&m_ringbuffer_host_rx, &ch, 1))
		{
			min_rx_feed(&m_min_context, &ch, 1);
		}
		else
		{

			min_timeout_poll(&m_min_context);
		}
		if(1)//HAL_GPIO_ReadPin(MODE1_GPIO_Port, MODE1_Pin) && HAL_GPIO_ReadPin(MODE2_GPIO_Port, MODE2_Pin))//xet trang thai bit gat
		{
			if(idle_detect)
			{
				idle_detect --;
				if (idle_detect == 0)
				{
//					DEBUG_INFO ("RS485 SAY:\r\n %s", rs485ringBuffer);
					char * testptr = strstr((char*)rs485ringBuffer, "IN TEST MODE");
					if (testptr)
					{
						test_res.result.rs485 = true;
					}
				}
			}

			if (get_jig_info)
			{
					DEBUG_INFO ("GET EOF START CHECK MAC \r\n");
					if (strcmp((char*)MAC, (char*)lastMAC) != 0 )
					{
						ready_send_to_sever = false;
						sent_to_sever =false;
						test_res.result.rs485 = 0;
						relay_toggle_0 = 0;
						relay_toggle_1 = 0;
						allPassed = 0;
						test_res.result.relay0_ok = 0;
						test_res.result.relay1_ok = 0;
						DEBUG_INFO ("MAC CHANGED: %02x: %02x: %02x: %02x: %02x: %02x\r\n", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4],MAC[5]);
						to_send_value = rx_value;
						if (((to_send_value ->peripheral.value) == 0xFFFF) && (test_res.result.test_wd_ok != 1))
						{
							send_test_command ((min_msg_t *)&reset_cmd_wd);
						}
					}
					else
					{

						DEBUG_WARN ("MAC STILL THE SAME, UPDATE \r\n");
						if (allPassed)
						{
							break;
						}
						if (rx_value ->temperature)
						{
							to_send_value ->temperature = rx_value ->temperature;
						}
						if (rx_value->peripheral.name.eth != 1)
						{
							to_send_value->peripheral.name.eth = rx_value ->peripheral.name.eth;
						}
						if (rx_value ->peripheral.name.wifi  != 1)
						{
							to_send_value ->peripheral.name.wifi  = rx_value ->peripheral.name.wifi ;
						}
						if (rx_value ->peripheral.name.gsm  != 1)
						{
							to_send_value ->peripheral.name.gsm  = rx_value ->peripheral.name.gsm ;
						}
						if (rx_value ->peripheral.name.server  != 1)
						{
							to_send_value ->peripheral.name.server  = rx_value ->peripheral.name.server ;
						}
						if (rx_value ->peripheral.name.input0_pass  != 1)
						{
							to_send_value ->peripheral.name.input0_pass  = rx_value ->peripheral.name.input0_pass ;
						}
						if (rx_value ->peripheral.name.input1_pass  != 1)
						{
							to_send_value ->peripheral.name.input1_pass  = rx_value ->peripheral.name.input1_pass ;
						}
						if (rx_value ->peripheral.name.input2_pass  != 1)
						{
							to_send_value ->peripheral.name.input2_pass  = rx_value ->peripheral.name.input2_pass ;
						}
						if (rx_value ->peripheral.name.input3_pass  != 1)
						{
							to_send_value ->peripheral.name.input3_pass  = rx_value ->peripheral.name.input3_pass ;
						}
						if (rx_value ->peripheral.name.button_pass   != 1)
						{
							to_send_value ->peripheral.name.button_pass   = rx_value ->peripheral.name.button_pass  ;
						}
						if (rx_value ->peripheral.name.main_power_pass   != 1)
						{
							to_send_value ->peripheral.name.main_power_pass   = rx_value ->peripheral.name.main_power_pass  ;
						}
						if (rx_value ->peripheral.name.backup_power_pass   != 1)
						{
							to_send_value ->peripheral.name.backup_power_pass   = rx_value ->peripheral.name.backup_power_pass  ;
						}

	//					json_build (to_send_value,  &test_res, json_send_to_sever);
	//					DEBUG_INFO ("BUILT A JSON: %s", json_send_to_sever);
						if (((to_send_value ->peripheral.value) == 0xFFFF) && (test_res.result.test_wd_ok != 1))
						{
							send_test_command ((min_msg_t *)&reset_cmd_wd);
						}
						if (PassTest(to_send_value))
						{
//							json_len = json_build (to_send_value, &test_res, json_send_to_sever);
							ready_send_to_sever = true;
							sent_to_sever = false;
							DEBUG_INFO ("NOW PASSED SEND AGAIN \r\n");
						}
					}
					for (uint8_t i = 0; i < 6; i++)
					{
						lastMAC[i] = MAC[i];
					}
				if (PassTest(to_send_value))
				{
					DEBUG_INFO ("PASSED ALL TESTS \r\n");
//					json_len = json_build (to_send_value, &test_res, json_send_to_sever);
					ready_send_to_sever = true;
				}
				else
				{
					if ((now - last_tick_time_out) > TIMEOUT)
					{
//						send_json_to_sever();// ban bang queue
//						json_len = json_build (to_send_value, &test_res, json_send_to_sever);
//						DEBUG_INFO ("JSON STRING: %s\r\n", json_send_to_sever);
						DEBUG_INFO ("TIME OUT PREPARE TO SEND \r\n");
						ready_send_to_sever = true;
						last_tick_time_out = now;

					}
				}
				get_jig_info = false;
			}
			// THEM TRUONG HOP MACH MAIN KHONG TRUYEN TIN QUA RS232

			if (ready_send_to_sever && (sent_to_sever == false))
			{
				date_time_t date_time_buff;
				sent_to_sever = true;
				jig_value_t* buff_jig_var;
				buff_jig_var = (jig_value_t *) pvPortMalloc(sizeof (jig_value_t));

				HAL_RTC_GetTime (&hrtc, &sTimeToSend, RTC_FORMAT_BIN);
				HAL_RTC_GetDate (&hrtc, &sDateToSend, RTC_FORMAT_BIN);
				date_time_buff.day = sDateToSend.Date;
				date_time_buff.month = sDateToSend.Month;
				date_time_buff.year = sDateToSend.Year;
				date_time_buff.hour = sTimeToSend.Hours - 7;
				date_time_buff.minute = sTimeToSend.Minutes;
				date_time_buff.second = sTimeToSend.Seconds;
				buff_jig_var->timestamp = convert_date_time_to_second (&date_time_buff);
				buff_jig_var->timestamp += 946684800; // add time from 1970 to 2000
				DEBUG_INFO ("GET TIME: %d: %d: %d \r\n", (uint8_t)(sTimeToSend.Hours), (uint8_t)(sTimeToSend.Minutes),(uint8_t)(sTimeToSend.Seconds));
				DEBUG_INFO ("GET date: %d: %d: %d \r\n", (uint8_t)(sDateToSend.Date), (uint8_t)(sDateToSend.Month),(uint8_t)(sDateToSend.Year));
				DEBUG_WARN ("CALCULATED TIME : %u\r\n", buff_jig_var->timestamp);

				buff_jig_var->device_type = to_send_value->device_type;
				memcpy (buff_jig_var->fw_version, to_send_value->fw_version, 3);
				memcpy (buff_jig_var->hw_version, to_send_value->hw_version, 3);
				memcpy (buff_jig_var->gsm_imei, to_send_value->gsm_imei, 16);
				memcpy (buff_jig_var->mac, to_send_value->mac, 6);
				memcpy (buff_jig_var->sim_imei, to_send_value->sim_imei, 16);
				buff_jig_var->peripheral.value = to_send_value->peripheral.value;
				buff_jig_var->test_result.value = test_res.value;
				xQueueSend (httpQueue, &buff_jig_var, 0);
				DEBUG_WARN ("SEND queue \r\n");
			}
			if ((now - last_tick) > 500 )
			{
//				DEBUG_INFO ("SEND TEST CMD \r\n");
				send_test_command (&test_cmd);
				last_tick = now;
			}
			if ((now - last_vol_tick) > 1500)
			{
				volTest ();
				last_vol_tick = now;
			}
		}
			uxBits = xEventGroupWaitBits(m_wdg_event_group,
				defaultTaskB | cdcTaskB | usbTaskB | flashTaskB | netTaskB,
										pdTRUE,
										pdTRUE,
										10);

		 if ((uxBits & (defaultTaskB | cdcTaskB | usbTaskB | flashTaskB | netTaskB)) == (defaultTaskB | cdcTaskB | usbTaskB | flashTaskB | netTaskB))
		 {
			 HAL_IWDG_Refresh(&hiwdg);
		 }


	}
}
//***************************************************************//

/**************    BUTTON APP FUNCTION         **************/
void button_initialize(uint32_t button_num)
{

}

uint32_t btn_read(uint32_t pin)
{
    if (pin == 0)
    {
        return HAL_GPIO_ReadPin(BT_IN_GPIO_Port, BT_IN_Pin);
    }
    return 1;
//    else if (pin == 1)
//    {
//	return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
//    }
//    return HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
}

void on_btn_pressed(int number, int event, void * pData)
{
    DEBUG_INFO("On button %d pressed\r\n", number);
    if (number == 0)
    {
        xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_KEY_0_PRESSED);
    }
//    else if (number == 1)
//    {
//        xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_KEY_1_PRESSED);
//    }
//    else
//    {
//        //xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_BUTTON_1_PRESSED);
//    }
}

void on_btn_release(int number, int event, void * pData)
{
    DEBUG_VERBOSE("On button %d release\r\n", number);
    if (number == 0)
    {
        xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_KEY_0_PRESSED);
    }
//    else if (number == 1)
//    {
//        xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_KEY_1_PRESSED);
//    }
//    else if (number == 2)
//    {
//      xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_BUTTON_2_PRESSED);
//    }
}

void on_btn_hold(int number, int event, void * pData)
{
    DEBUG_INFO("On button %d pair hold, enter pair mode\r\n", number);
}


static void on_btn_hold_so_long(int index, int event, void * pData)
{
    DEBUG_INFO("Button hold so long\r\n");
}
/*******************************************************************/


// *********************** interrupt callback ************************//
void rs232_rx_callback (void)
{
	uint8_t ch = getChar (USART3);
	lwrb_write (&m_ringbuffer_host_rx, &ch, 1);
}

void rs485_rxcallback (void)
{
	idle_detect = 5;
	uint8_t ch =getChar(UART5);
	lwrb_write (&m_ringbuffer_rs485_rx, &ch, 1);
//	DEBUG_ISR ("%c",ch);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  if (GPIO_Pin == RELAY_NO_Pin)
  {
	  relay_toggle_0 ++;
	  if (relay_toggle_0 > 5)
	  {
		  test_res.result.relay0_ok = 1;
		  relay_toggle_0 = 0;
	  }
  }
  else if (GPIO_Pin == RELAY_NC_Pin)
  {
	  relay_toggle_0 ++;
	  if (relay_toggle_1 > 5)
	  {
		  relay_toggle_1 = 0;
		  test_res.result.relay1_ok = 1;
	  }
  }
}
//********************************************************************//

// **************************** TIME PROTOCOL ***************************//
static void convert_second_to_date_time(uint32_t sec, date_time_t *t, uint8_t Calyear)
{
    uint16_t day;
    uint8_t year;
    uint16_t days_of_year;
    uint8_t leap400;
    uint8_t month;

    t->second = sec % 60;
    sec /= 60;
    t->minute = sec % 60;
    sec /= 60;
    t->hour = sec % 24;

    if (Calyear == 0)
        return;

    day = (uint16_t)(sec / 24);

    year = FIRSTYEAR % 100;                   // 0..99
    leap400 = 4 - ((FIRSTYEAR - 1) / 100 & 3); // 4, 3, 2, 1

    for (;;)
    {
        days_of_year = 365;
        if ((year & 3) == 0)
        {
            days_of_year = 366; // leap year
            if (year == 0 || year == 100 || year == 200)
            { // 100 year exception
                if (--leap400)
                { // 400 year exception
                    days_of_year = 365;
                }
            }
        }
        if (day < days_of_year)
        {
            break;
        }
        day -= days_of_year;
        year++; // 00..136 / 99..235
    }
    t->year = year + FIRSTYEAR / 100 * 100 - 2000; // + century
    if (days_of_year & 1 && day > 58)
    {          // no leap year and after 28.2.
        day++; // skip 29.2.
    }

    for (month = 1; day >= day_in_month[month - 1]; month++)
    {
        day -= day_in_month[month - 1];
    }

    t->month = month; // 1..12
    t->day = day + 1; // 1..31
}

static uint32_t convert_date_time_to_second(date_time_t *t)
{
    uint8_t i;
    uint32_t result = 0;
    uint16_t idx, year;

    year = t->year + 2000;

    /* Calculate days of years before */
    result = (uint32_t)year * 365;
    if (t->year >= 1)
    {
        result += (year + 3) / 4;
        result -= (year - 1) / 100;
        result += (year - 1) / 400;
    }

    /* Start with 2000 a.d. */
    result -= 730485UL;

    /* Make month an array index */
    idx = t->month - 1;

    /* Loop thru each month, adding the days */
    for (i = 0; i < idx; i++)
    {
        result += day_in_month[i];
    }

    /* Leap year? adjust February */
    if (year % 400 == 0 || (year % 4 == 0 && year % 100 != 0))
    {
        ;
    }
    else
    {
        if (t->month > 2)
        {
            result--;
        }
    }

    /* Add remaining days */
    result += t->day;

    /* Convert to seconds, add all the other stuff */
//    if (year < 2000)
//    {
//	    result = (result - 1) * 86400L + (uint32_t)t->hour * 3600 +
//	             (uint32_t)t->minute * 60 + t->second;
//	}
//	else
//	{
//		result = (result-2) * 86400L + (uint32_t)t->hour * 3600 +
//             (uint32_t)t->minute * 60 + t->second;
//	}
    result = (result - 1) * 86400L + (uint32_t)t->hour * 3600 +
    	             (uint32_t)t->minute * 60 + t->second;
    return result;
}

void lwip_sntp_recv_cb (uint32_t time)
{

	if (time == 0)
	{
		DEBUG_INFO ("NTP ERROR \r\n");
	}
	else
	{
//		reInitRTC()
		DEBUG_INFO (" GOT TIME : it's been %u second from 1970\r\n", time);

		time_t rawtime = time;
		struct tm ts;

		// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
		ts = *localtime(&rawtime);

		DEBUG_INFO("Time now: %02d:%02d:%02d  %02d-%02d-%d\r\n", ts.tm_hour, ts.tm_min, ts.tm_sec, ts.tm_mday, ts.tm_mon + 1, ts.tm_year + 1900);

		date_time.year = (ts.tm_year + 1900) % 2000; //year - 1900
		date_time.month = ts.tm_mon + 1;             // month, where 0 = jan
		date_time.day = ts.tm_mday;                  // day of the month
		date_time.hour = ts.tm_hour;
		date_time.minute = ts.tm_min;
		date_time.second = ts.tm_sec;

		uint32_t time_buff = convert_date_time_to_second (&date_time) + 25200;// time for gmt +7

		convert_second_to_date_time (time_buff, &date_time, 1);

		DEBUG_INFO ("TIME NOW IS: %d:%d:%d %d-%d-%d\r\n", date_time.hour, date_time.minute, date_time.second, date_time.day,date_time.month, (date_time.year + 2000));
		sTime.Hours = date_time.hour;
		sTime.Minutes = date_time.minute;
		sTime.Seconds = date_time.second;
		sDate.Year = date_time.year;
		sDate.Month = date_time.month;
		sDate.Date = date_time.day;
		reInitRTC (sTime,sDate);
		HAL_RTC_GetTime (&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate (&hrtc, &sDate, RTC_FORMAT_BIN);
		DEBUG_INFO ("GET TIME: %d: %d: %d \r\n", (uint8_t)(sTime.Hours), (uint8_t)(sTime.Minutes),(uint8_t)(sTime.Seconds));
		DEBUG_INFO ("GET date: %d: %d: %d \r\n", (uint8_t)(sDate.Date), (uint8_t)(sDate.Month),(uint8_t)(sDate.Year));
//		uint32_t timenew = convert_date_time_to_second (&date_time);
//		DEBUG_INFO ("TIME CALCULATE: %u", timenew);
	}
}

static void initialize_stnp(void)
{
    static bool sntp_start = false;
    if (sntp_start == false)
    {
        sntp_start = true;

        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
        sntp_init();
        DEBUG_INFO("Initialize stnp\r\n");
    }
}

//************************************************************************//
// ************************* TIME RTC*********************************//
void reInitRTC ( RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate)
{
	  /** Initialize RTC Only
	  */
	  /* USER CODE BEGIN Check_RTC_BKUP */

	  /* USER CODE END Check_RTC_BKUP */

	  /** Initialize RTC and set the Time and Date
	  */
//	  sTime.Hours = 0x0;
//	  sTime.Minutes = 0x0;
//	  sTime.Seconds = 0x0;
//	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  DEBUG_ERROR ("CAN'T SET TIME \r\n");
	    Error_Handler();
	  }
//	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//	  sDate.Month = RTC_MONTH_JANUARY;
//	  sDate.Date = 0x12;
//	  sDate.Year = 0x0;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  DEBUG_ERROR ("CAN'T SET date \r\n");
	    Error_Handler();
	  }
}
//*****************************************************************//
//*********************** json encode*********************************//
int16_t json_build(jig_value_t *value, func_test_t * test, char *json_str)
{
	char mac_str[18];
	int16_t index = 0;
//	sprintf (json_str_buff, "{\r\n\"timestamp\": %d:%d:%d,\r\n", 	hour, minute, sencond);
//	strcat (json_str, json_strbuff);
//	sprintf (json_str_buff, "\"day test\": %d:%d:%d,\r\n", 			day, month, year);
//	strcat (json_str, json_strbuff);
	index += sprintf (json_str+index, "[{\"timestamp\":%lu,", 				value ->timestamp);
	index += sprintf (json_str+index, "\"DeviceType\":\"%s\",", 			value ->device_type ? "B02":"B01");
	index += sprintf (json_str+index, "\"FirmwareVersion\":\"%d.%d.%d\",",	value ->fw_version[0],value ->fw_version[1],value ->fw_version[2] );
	index += sprintf (json_str+index, "\"HardwardVersion\":\"%d.%d.%d\",",	value ->hw_version[0],value ->hw_version[1],value ->hw_version[2] );
	index += sprintf (json_str+index, "\"GsmIMEI\":\"%s\",", 				value ->gsm_imei);
	sprintf (mac_str, "%02x:%02x:%02x:%02x:%02x:%02x",	value->mac[0],
														value->mac[1],
														value->mac[2],
														value->mac[3],
														value->mac[4],
														value->mac[5]);
//	make_string_from_mac (mac_str);
	index += sprintf (json_str+index, "\"MacJIG\":\"%s\",", 			mac_str);
	index += sprintf (json_str+index, "\"ErrorResults\":{");
	index += sprintf (json_str+index, "\"sim\":%s,", 					test->result.sim_ok ? "true" : "false");
	index += sprintf (json_str+index, "\"vGsm4V2\":%s,", 				test ->result.vgsm4v2_ok ? "true" : "false");
	index += sprintf (json_str+index, "\"eth\":%s,", 					value ->peripheral.name.eth ? "true" : "false");
	index += sprintf (json_str+index, "\"wifi\":%s,", 					value ->peripheral.name.wifi ? "true" : "false");
	index += sprintf (json_str+index, "\"server\":%s,", 				value ->peripheral.name.server ? "true" : "false");
//	sprintf (json_str_buff, "\"GSM status\":%d,\r\n", 				value ->peripheral.name.gsm);

	index += sprintf (json_str+index, "\"mainPower\":%s,", 				value ->peripheral.name.main_power_pass ? "true" : "false");
	index += sprintf (json_str+index, "\"backupPower\":%s,", 			value ->peripheral.name.backup_power_pass? "true" : "false");
	index += sprintf (json_str+index, "\"buttonTest\":%s,", 			value ->peripheral.name.button_pass? "true" : "false");
	index += sprintf (json_str+index, "\"input1\":%s,",					value->peripheral.name.input0_pass ? "true" : "false");
	index += sprintf (json_str+index, "\"input2\":%s,",					value->peripheral.name.input1_pass ? "true" : "false");
	index += sprintf (json_str+index, "\"input3\":%s,",					value->peripheral.name.input2_pass ? "true" : "false");
	index += sprintf (json_str+index, "\"input4\":%s,",					value->peripheral.name.input3_pass ? "true" : "false");
	index += sprintf (json_str+index, "\"temperature\":%s,", 			test->result.temper_ok? "true" : "false");
	index += sprintf (json_str+index, "\"charge\":%s,", 				test->result.charge_ok? "true" : "false");
	index += sprintf (json_str+index, "\"alarmIn\":%s,", 				test->result.alarm_ok? "true" : "false");
	index += sprintf (json_str+index, "\"faultIn\":%s,", 				test->result.fault_ok? "true" : "false");
	index += sprintf (json_str+index, "\"sosButton\":%s,", 				test->result.sosButton_ok? "true" : "false");
	index += sprintf (json_str+index, "\"relay0\":%s,", 				test->result.relay0_ok? "true" : "false");
	index += sprintf (json_str+index, "\"relay1\":%s,", 				test->result.relay1_ok? "true" : "false");
	index += sprintf (json_str+index, "\"watchdog\":%s,", 				test->result.test_wd_ok? "true" : "false");
	index += sprintf (json_str+index, "\"v4v2\":%s,", 					test->result.vgsm4v2_ok? "true" : "false");
	index += sprintf (json_str+index, "\"vbat\":%s,", 					test->result.vbat_ok? "true" : "false");
	index += sprintf (json_str+index, "\"v1v8\":%s,", 					test->result.v1v8_ok? "true" : "false");
	index += sprintf (json_str+index, "\"v3v3\":%s,", 					test->result.v3v3_ok? "true" : "false");
	index += sprintf (json_str+index, "\"v5v\":%s,", 					test->result.v5v_ok? "true" : "false");
	index += sprintf (json_str+index, "\"vsys\":%s,", 					test->result.vsys_ok? "true" : "false");
	index += sprintf (json_str+index, "\"allPassed\":%s}}]", 			allPassed? "true" : "false");
	return index;
}
//********************************************************************//
void fakeMac (char *MACstring)
{
	DEBUG_WARN ("ENTER FAKE MAC FUNC\r\n");
	for (uint8_t i = 0; i < 6; i++)
	{
		MAC [i] = (uint8_t) MACstring [i];
	}
	to_send_value ->device_type = 1;
	to_send_value ->fw_version[0] = 0;
	to_send_value ->fw_version [1] = 0;
	to_send_value ->fw_version [2] = 1;
	to_send_value ->hw_version [0] = 0;
	to_send_value ->hw_version [1] = 0;
	to_send_value ->hw_version [2] = 0;
	to_send_value ->gsm_imei[16] = 1;
	to_send_value ->sim_imei[16] = 1;
	to_send_value ->temperature = 12;
	to_send_value ->test_result.value = 23142412;
	to_send_value ->peripheral.value = 1238;

// co the fake them cac chi so khac de test.
	get_jig_info = true;
}
void getTimeNow (void)
{
	  RTC_TimeTypeDef sTime1 = {0};
	  RTC_DateTypeDef sDate1= {0};
	  HAL_RTC_GetTime (&hrtc, &sTime1, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate (&hrtc, &sDate1, RTC_FORMAT_BIN);
	  DEBUG_INFO ("GET TIME: %d: %d: %d \r\n", (uint8_t)(sTime1.Hours), (uint8_t)(sTime1.Minutes),(uint8_t)(sTime1.Seconds));
	  DEBUG_INFO ("GET date: %d: %d: %d \r\n", (uint8_t)(sDate1.Date), (uint8_t)(sDate1.Month),(uint8_t)(sDate1.Year));

}
int32_t USB_puts(char *msg)
{
    uint32_t len = strlen(msg);

	cdc_tx((uint8_t*)msg, len);
    return len;
}

//FRESULT scan_files (char* pat)
//{
//    DIR dir;
//    UINT i;
//
//    char path[20];
//    sprintf (path, "%s",pat);
//
//    fresult = f_opendir(&dir, path);                       /* Open the directory */
//    if (fresult == FR_OK)
//    {
//        for (;;)
//        {
//            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
//            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
//            if (fno.fattrib & AM_DIR)     /* It is a directory */
//            {
//            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
////            	sprintf (buffer, "Dir: %s\r\n", fno.fname);
////            	send_uart(buffer);
//                i = strlen(path);
//                sprintf(&path[i], "/%s", fno.fname);
//                fresult = scan_files(path);                     /* Enter the directory */
//                if (fresult != FR_OK) break;
//                path[i] = 0;
//            }
//            else
//            {                                       /* It is a file. */
//               sprintf(buffer,"File: %s/%s\n", path, fno.fname);
//
////               send_uart(buffer);
//            }
//        }
//        f_closedir(&dir);
//    }
//    return fresult;
//}

/* USER CODE END Application */

