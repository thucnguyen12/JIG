/******************************************************************************
 * @file:    app_cli.c
 * @brief:
 * @version: V0.0.0
 * @date:    2019/11/12
 * @author:
 * @email:
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". VINSMART
 * JSC MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. VINSMART JSC
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 *
 * VINSMART JSC SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 *
 * (C)Copyright VINSMART JSC 2019 All rights reserved
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "app_cli.h"
#include "app_shell.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_spi_flash.h"
#include "app_drv_spi.h"
#include "utilities.h"
//#include "esp_log.h"
//#include "esp_system.h"
//#include "app_flash.h"
//#include "app_ota.h"
//#include "app_mqtt.h"
//#include "DataDefine.h"
//#include "app_aes.h"
//#include "base64.h"
//#include "app_audio.h"
#include "main.h"
#include "app_debug.h"
#include "diskio.h"
//static const char *TAG = "cli";
extern app_flash_drv_t m_spi_flash;
extern void fakeMac (char *MACstring);
extern void getTimeNow (void);
static app_cli_cb_t *m_cb;
//static int32_t cli_get_memory(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_ota_update(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_get_server_state(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_change_mqtt_info(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_get_mqtt_info(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_set_master(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_set_master(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_set_aes_key(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_factory_reset(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_disable_console(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_whitelist_server(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_get_firmware_version(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_enable_encryption(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_change_console_username_password(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_show_interface_availble(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_show_imei(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_change_device_volume(p_shell_context_t context, int32_t argc, char **argv);

//static const shell_command_context_t cli_command_table[] =
//{
////    {"mem", "\tmem: Get free memory size\r\n", cli_get_memory, 0},
////    {"reset", "\treset: Reboot system\r\n", cli_reset, -1},
////    {"server", "\tserver: Set server info, following format URI username password\r\n", cli_change_mqtt_info, -1},
////    {"ota", "\tota: Update firmware, format http://asdasd/bin\r\n", cli_ota_update, 1},
////    {"status", "\tstatus: Get server connection information\r\n", cli_get_server_state, 0},
////    {"getKey", "\tgetKey: Get server key\r\n", cli_get_mqtt_info, 0},
////    {"master", "\tmaster: Set master\r\n", cli_set_master, 1},
////    {"disableConsole", "\tdisableConsole: Disable console\r\n", cli_disable_console, 0},
////    {"setEnckey", "\tsetEnckey: Set encryption key\r\n", cli_set_aes_key, 1},
////    {"encEnable", "\tencEnable: Enable/disable mqtt enc mode\r\n", cli_enable_encryption, 1},
////    {"version", "\tversion: Get firmware version\r\n", cli_get_firmware_version, 0},
////    {"factory", "\tfactory: Factory reset\r\n", cli_factory_reset, 0},
////    {"whitelist", "\twhitelist: set-get whitelist servers addr\r\n", cli_whitelist_server, -1},
////    {"changeConsolePass", "\tchangeConsolePass: Change console username and password\r\n", cli_change_console_username_password, 2},
////    {"imei", "\timei: Get device imei\r\n", cli_show_imei, 0},
////    {"interface", "\tinterface: Show list of interfaces availble on device\r\n", cli_show_interface_availble, 0},
////    {"setVolume", "\tsetVolume: Change device volume and send to server\r\n", cli_change_device_volume, 1}
//};
static int32_t fakeMAC (p_shell_context_t context, int32_t argc, char **argv);
static int32_t resetChip (p_shell_context_t context, int32_t argc, char **argv);
static int32_t getTime (p_shell_context_t context, int32_t argc, char **argv);
static int32_t readPage (p_shell_context_t context, int32_t argc, char **argv);
static const shell_command_context_t cli_command_table[] =
{
		 {"fakeMAC", "\tfakeMAC: create a fake MAC add to test\r\n", fakeMAC, 1},
		 {"resetChip", "\tresetChip: Call reset function \r\n", resetChip, 0},
		 {"getTime", "\tgetTime: get time now \r\n", resetChip, 0},
		 {"readPage", "\treadPage x: read page number x in flash", readPage, 1}
};
static shell_context_struct m_user_context;
static app_cli_cb_t *m_cb;

void app_cli_poll(uint8_t ch)
{
    app_shell_task(ch);
}

void app_cli_start (app_cli_cb_t *callback)
{
    m_cb = callback;
    app_shell_set_context(&m_user_context);
    app_shell_init(&m_user_context,
                   m_cb->puts,
                   m_cb->printf,
                   "",
                   true);

    /* Register CLI commands */
    for (int i = 0; i < sizeof(cli_command_table) / sizeof(shell_command_context_t); i++)
    {
        app_shell_register_cmd(&cli_command_table[i]);
    }

    /* Run CLI task */
    app_shell_task(APP_SHELL_INVALID_CHAR);
}

static int32_t fakeMAC (p_shell_context_t context, int32_t argc, char **argv)
{
	DEBUG_INFO ("ENTER THE FAKE MAC FUNC\r\n");
	if (strlen (argv[1]) == 6)
	{
		fakeMac (argv[1]);
	}
	return 0;
}

static int32_t resetChip (p_shell_context_t context, int32_t argc, char **argv)
{
	NVIC_SystemReset ();
	 m_cb->printf("system reset done\r\n");
	return 0;
}

static int32_t getTime (p_shell_context_t context, int32_t argc, char **argv)
{
	void getTimeNow (void);
}
static int32_t readPage (p_shell_context_t context, int32_t argc, char **argv)
{
	uint8_t* data_read = (uint8_t *)pvPortMalloc(256* sizeof (uint8_t));
	if (argv[1])
	{
		uint16_t pageRead = utilities_get_number_from_string (0, argv[1]);
		DEBUG_INFO ("page read:%d\r\n", pageRead);
		app_spi_flash_read_bytes (&m_spi_flash, (pageRead * 256), data_read, 256);
		DEBUG_INFO ("data: %s\r\n", data_read);
	}
	vPortFree (data_read);
	return 0;
}
/* Reset System */
//static int32_t cli_get_memory(p_shell_context_t context, int32_t argc, char **argv)
//{
//    char tmp[256];
////    sprintf(tmp, "Free memory size %u-%u\r\n", xPortGetFreeHeapSize(), esp_get_free_internal_heap_size());
//    // char tmp[512+128];
//    // vTaskGetRunTimeStats(tmp);
//    // ESP_LOGI(TAG, tmp);
//    // ESP_LOGI(TAG, "\r\n");
//    m_cb->printf(tmp);
//    return 0;
//}
//
//static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf("System will be reset, close console\r\nBye bye\r\n");
//    vTaskDelay(1000);
//    m_cb->terminate();
////    esp_restart();
//    return 0;
//}
//
//// extern void app_ota_start(char *url);
//static int32_t cli_ota_update(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (strstr(argv[1], "http://") || strstr(argv[1], "https://"))
//    {
//        ESP_LOGI(TAG, "OTA url %s\r\n", argv[1]);
//        // Create ota task
//        static char *url = NULL;
//        if (!url)
//        {
//            static app_ota_info_t info;
//            url = malloc(strlen(argv[1]) + 128);
//            info.url = url;
//            info.type = APP_OTA_DEVICE_ESP32;
//            sprintf(url, "%s", argv[1]);
//            m_cb->printf("Update firmware on url : ");
//            m_cb->printf(url);
//            m_cb->printf("\r\n");
//            vTaskDelay(50/portTICK_PERIOD_MS);
//            xTaskCreate(app_ota_download_task, "ota_task", 8192, (void*)&info, 5, NULL);
//            // Ko dc free contro URL
//        }
//    }
//    else
//    {
//        m_cb->printf("URL prefix must be \"http(s)://\"");
//    }
//
//    return 0;
//}
//
//// static int32_t cli_get_current_time(p_shell_context_t context, int32_t argc, char **argv)
//// {
////     // app_sntp_debug_timenow();
////     return 0;
//// }
//
//// static int32_t cli_get_config(p_shell_context_t context, int32_t argc, char **argv)
//// {
////     if (strstr(argv[1], "dump"))
////     {
////         //		internal_flash_cfg_t *cfg = internal_flash_get_config();
////         //		(void)cfg;
////         //		ESP_LOGI(TAG, "\t\tConfig addr %s:%d\r\n\tPing nterval %ums", cfg->host_addr, cfg->port, cfg->ping_cycle);
////     }
////     return 0;
//// }
//
//static int32_t cli_change_mqtt_info(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (argc == 4)      // url, username, pass
//    {
//        ESP_LOGI(TAG, "MQTT url %s, username %s, password %s", argv[1], argv[2], argv[3]);
//        char *reply = malloc(strlen(argv[1]) + strlen(argv[2]) + strlen(argv[3]) + 256);
//        if (reply)
//        {
//            if (strstr(argv[1], "mqtt://") == 0 && strstr(argv[1], "mqtts://") == 0)
//            {
//                sprintf(reply, "%s", "Invalid URL\r\n");
//                m_cb->printf(reply);
//            }
//            else
//            {
//                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_URL, argv[1]);
//                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_USERNAME, argv[2]);
//                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_PASSWORD, argv[3]);
//                sprintf(reply, "Set mqtt URL %s, username %s, password %s. Device will be reboot within 5s", argv[1], argv[2], argv[3]);
//                m_cb->printf(reply);
//                vTaskDelay(4000);
//                m_cb->printf("\r\nREBOOT now\r\n");
//                vTaskDelay(1000);
//                m_cb->terminate();
//                esp_restart();
//            }
//            free(reply);
//        }
//    }
//    else if (argc == 2)
//    {
//        char *reply = malloc(strlen(argv[1]) + 128);
//        if (reply)
//        {
//            if (strstr(argv[1], "mqtt://") == 0 && strstr(argv[1], "mqtts://") == 0)
//            {
//                sprintf(reply, "%s", "Invalid URL\r\n");
//                m_cb->printf(reply);
//            }
//            else
//            {
//                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_URL, argv[1]);
//                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_USERNAME, "");
//                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_PASSWORD, "");
//                sprintf(reply, "Set mqtt URL %s, username NULL, password NULL. Device will be within 5s", argv[1]);
//                m_cb->printf(reply);
//                free(reply);
//                vTaskDelay(4000);
//                m_cb->printf("\r\nREBOOT now\r\n");
//                vTaskDelay(1000);
//                m_cb->terminate();
//                esp_restart();
//            }
//        }
//    }
//    else
//    {
//        m_cb->printf("Invalid mqtt arguments");
//    }
//    return 0;
//}
//
//static int32_t cli_get_server_state(p_shell_context_t context, int32_t argc, char **argv)
//{
//    static const char *server_state[] =
//    {
//        "Server state =>> Disconnected\r\n",
//        "Server state =>> Connecting\r\n",
//        "Server state =>> Connected\r\n"
//    };
//    m_cb->printf(server_state[app_mqtt_get_state()]);
//    m_cb->printf("Domain name = ");
//    m_cb->printf(app_flash_get_mqtt_server_url());
//    m_cb->printf("\r\n");
//    return 0;
//}
//
//static int32_t cli_set_master(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (strlen(argv[1]) == 15)
//    {
//        memcpy(app_flash_get_master(APP_FLASH_MASTER_HUYEN1), argv[1], 15);
//        app_flash_node_nvs_write_string(APP_FLASH_MASTER_H1_KEY, app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
//        m_cb->printf("Set master ");
//        m_cb->printf(argv[1]);
//        m_cb->printf("\r\nSystem will be reboot after 1s\r\n");
//        vTaskDelay(1000/portTICK_PERIOD_MS);
//        m_cb->terminate();
//        esp_restart();
//    }
//    return 0;
//}
//
//static int32_t cli_factory_reset(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf("Factory reset\r\n");
//    m_cb->printf("\r\nSystem will be reboot after 1s\r\n");
//    app_flash_tcp_console_disable();
//    vTaskDelay(1000/portTICK_PERIOD_MS);
//    m_cb->terminate();
//    esp_restart();
//    return 0;
//}
//
//static int32_t cli_disable_console(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf("Disable console reset\r\n");
//    m_cb->printf("\r\nSystem will be reboot after 1s\r\n");
//    app_flash_tcp_console_disable();
//    vTaskDelay(1000/portTICK_PERIOD_MS);
//    esp_restart();
//    return 0;
//}
//
//static void dump_whitelist_server(void)
//{
//    char *server0;
//    char *server1;
//    app_flash_get_whitelist_server(&server0, &server1);
//    ESP_LOGI(TAG, "Server %s, %s", server0, server1);
//    m_cb->printf("=>> Current whitelist servers : ");
//    m_cb->printf(server0);
//    m_cb->printf(", ");
//    m_cb->printf(server1);
//    m_cb->printf("\r\n");
//}
//
//static int32_t cli_whitelist_server(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (argc == 2 && (strcmp(argv[1], "get") == 0))
//    {
//        dump_whitelist_server();
//    }
//    else if ((argc == 4 || argc == 3) && (strcmp(argv[1], "set") == 0))
//    {
//        ESP_LOGI(TAG, "Set whitelist servers %s-%s\r\n", argv[2], argv[3]);
//        bool do_reset = false;
//        if (argc == 4)
//        {
//            do_reset = app_flash_set_whitelist_server(argv[2], argv[3]);
//        }
//        else
//        {
//            char *server0;
//            char *server1;
//            app_flash_get_whitelist_server(&server0, &server1);
//            do_reset = app_flash_set_whitelist_server(argv[2], server1);
//        }
//        dump_whitelist_server();
//
//        if (do_reset)
//        {
//            m_cb->printf("Server changed, device will be reset in 3s\r\n");
//            vTaskDelay(3000);
//            m_cb->terminate();
//            esp_restart();
//        }
//    }
//    else
//    {
//        m_cb->printf("Invalid \"whitelist\" command, use \"whitelist get\" or \"whitelist set 123.123.123.123 456.456.567.234\"\r\n");
//    }
//    return 0;
//}
//
//static int32_t cli_set_aes_key(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (strlen(argv[1]) == 16 && strcmp(argv[1], (char*)app_flash_get_aes_key()))
//    {
//        m_cb->printf("Set new encryption key ");
//        app_flash_set_aes_key((uint8_t*)argv[1]);
//        m_cb->printf((char*)app_flash_get_aes_key());
//        m_cb->printf("\r\nr\n");
//    }
//    return 0;
//}
//
//static int32_t cli_get_firmware_version(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf(__FIRMWARE_VERSION__);
//    m_cb->printf("\r\n\r\n\r\n");
//    return 0;
//}
//
//static int32_t cli_get_mqtt_info(p_shell_context_t context, int32_t argc, char **argv)
//{
//    char *uri = app_flash_get_mqtt_server_url();
//    char *username = app_flash_get_mqtt_server_username();
//    char *password = app_flash_get_mqtt_server_password();
//
//    // ESP_LOGI(TAG, "payload len %u byte\r\n", len);
//    char *aes_output = calloc(strlen(uri) + 17, sizeof(char));
//    if (aes_output)
//    {
//        uint32_t size = app_aes_ecb_encrypt((uint8_t*)uri, app_flash_get_aes_key(), (uint8_t*)aes_output, strlen(uri));
//        // ESP_LOGI(TAG, "AES len %u byte\r\n", size);
//        uint8_t *base64_output;
//        base64_output = calloc(4*size/3+12, sizeof(char));        //base64 len = 4*raw_len/3
//        if (base64_output)
//        {
//            size = b64_encode((const char*)aes_output, size, (char*)base64_output);
//            ESP_LOGI(TAG, "MQTT broker %.*s", size, base64_output);
//            m_cb->printf("Broker : ");
//            m_cb->printf((char*)base64_output);
//            m_cb->printf("\r\n");
//        }
//        free (aes_output);
//        if (base64_output)
//        {
//            free(base64_output);
//        }
//    }
//
//    aes_output = calloc(strlen(username) + 17, sizeof(char));
//    if (aes_output)
//    {
//        uint32_t size = app_aes_ecb_encrypt((uint8_t*)username, app_flash_get_aes_key(), (uint8_t*)aes_output, strlen(username));
//        // ESP_LOGI(TAG, "AES len %u byte\r\n", size);
//        uint8_t *base64_output;
//        base64_output = calloc(4*size/3+12, sizeof(char));        //base64 len = 4*raw_len/3
//        if (base64_output)
//        {
//            size = b64_encode((const char*)aes_output, size, (char*)base64_output);
//            ESP_LOGI(TAG, "MQTT username %.*s", size, base64_output);
//            m_cb->printf("MQTT username : ");
//            m_cb->printf((char*)base64_output);
//            m_cb->printf("\r\n");
//        }
//        free (aes_output);
//        if (base64_output)
//        {
//            free(base64_output);
//        }
//    }
//
//    aes_output = calloc(strlen(password) + 17, sizeof(char));
//    if (aes_output)
//    {
//        uint32_t size = app_aes_ecb_encrypt((uint8_t*)password, app_flash_get_aes_key(), (uint8_t*)aes_output, strlen(password));
//        // ESP_LOGI(TAG, "AES len %u byte\r\n", size);
//        uint8_t *base64_output;
//        base64_output = calloc(4*size/3+12, sizeof(char));        //base64 len = 4*raw_len/3
//        if (base64_output)
//        {
//            size = b64_encode((const char*)aes_output, size, (char*)base64_output);
//            m_cb->printf("MQTT password : ");
//            m_cb->printf((char*)base64_output);
//            m_cb->printf("\r\n");
//            ESP_LOGI(TAG, "MQTT password %.*s", size, base64_output);
//        }
//        free (aes_output);
//        if (base64_output)
//        {
//            free(base64_output);
//        }
//    }
//
//    // master
//    for (uint8_t master_index = MASTER_TINH1; master_index < MASTER_TOTAL; master_index++)
//    {
//        char *master = app_flash_get_master(master_index);
//
//        aes_output = calloc(strlen(master) + 17, sizeof(char));
//        if (aes_output)
//        {
//            uint32_t size = app_aes_ecb_encrypt((uint8_t*)password, app_flash_get_aes_key(), (uint8_t*)aes_output, strlen(master));
//            // ESP_LOGI(TAG, "AES len %u byte\r\n", size);
//            uint8_t *base64_output;
//            base64_output = calloc(4*size/3+12, sizeof(char));        //base64 len = 4*raw_len/3
//            if (base64_output)
//            {
//                size = b64_encode((const char*)aes_output, size, (char*)base64_output);
//                m_cb->printf("MQTT master : ");
//                m_cb->printf((char*)base64_output);
//                m_cb->printf("\r\n");
//                ESP_LOGI(TAG, "MQTT master %.*s", size, base64_output);
//            }
//            free (aes_output);
//            if (base64_output)
//            {
//                free(base64_output);
//            }
//        }
//    }
//
//    // Console username and password
//    char *console_username;
//    char *console_password;
//    app_flash_get_console_info(&console_username, &console_password);
//    aes_output = calloc(strlen(console_username) + 17, sizeof(char));
//    if (aes_output)
//    {
//        uint32_t size = app_aes_ecb_encrypt((uint8_t*)console_username, app_flash_get_aes_key(), (uint8_t*)aes_output, strlen(console_username));
//        // ESP_LOGI(TAG, "AES len %u byte\r\n", size);
//        uint8_t *base64_output;
//        base64_output = calloc(4*size/3+12, sizeof(char));        //base64 len = 4*raw_len/3
//        if (base64_output)
//        {
//            size = b64_encode((const char*)aes_output, size, (char*)base64_output);
//            m_cb->printf("Console username : ");
//            m_cb->printf((char*)base64_output);
//            m_cb->printf("\r\n");
//        }
//        free (aes_output);
//        if (base64_output)
//        {
//            free(base64_output);
//        }
//    }
//
//    aes_output = calloc(strlen(console_password) + 17, sizeof(char));
//    if (aes_output)
//    {
//        uint32_t size = app_aes_ecb_encrypt((uint8_t*)console_password, app_flash_get_aes_key(), (uint8_t*)aes_output, strlen(console_password));
//        // ESP_LOGI(TAG, "AES len %u byte\r\n", size);
//        uint8_t *base64_output;
//        base64_output = calloc(4*size/3+12, sizeof(char));        //base64 len = 4*raw_len/3
//        if (base64_output)
//        {
//            size = b64_encode((const char*)aes_output, size, (char*)base64_output);
//            m_cb->printf("Console password : ");
//            m_cb->printf((char*)base64_output);
//            m_cb->printf("\r\n");
//        }
//        free (aes_output);
//        if (base64_output)
//        {
//            free(base64_output);
//        }
//    }
//
//    return 0;
//}
//
//static int32_t cli_enable_encryption(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (strstr(argv[1], "enable"))
//    {
//        m_cb->printf("Enable mqtt encryption\r\n");
//        app_flash_encrypt_payload_enable();
//    }
//    else if (strstr(argv[1], "disable"))
//    {
//        m_cb->printf("Disable mqtt encryption\r\n");
//        app_flash_encrypt_payload_disable();
//    }
//    return 0;
//}
//
//static int32_t cli_change_console_username_password(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (strlen(argv[1]) && strlen(argv[2]) && app_flash_set_console_info(argv[1], argv[2]))
//    {
//        m_cb->printf("Change console username to:\t");
//        m_cb->printf(argv[1]);
//        m_cb->printf("\r\n");
//        m_cb->printf("Change console password to:\t");
//        m_cb->printf(argv[2]);
//        m_cb->printf("\r\nClose session\r\n");
//        vTaskDelay(1000);
//        m_cb->terminate();
//        vTaskDelay(1000);
//        esp_restart();
//    }
//    else
//    {
//        m_cb->printf("Username and password is invalid\r\n");
//    }
//
//    return 0;
//}
//
//static int32_t cli_show_imei(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf("IMEI = ");
//    m_cb->printf(app_flash_get_imei());
//    m_cb->printf("\r\n");
//    return 0;
//}
//
//static int32_t cli_show_interface_availble(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf("Supported interfaces: MQTT(S), HTTP(S), Console\r\n");
//    return 0;
//}
//
//static int32_t cli_change_device_volume(p_shell_context_t context, int32_t argc, char **argv)
//{
//    char tmp[96];
//    int volume = atoi(argv[1]);
//    if (volume > 100)
//    {
//        volume = 100;
//    }
//    else if (volume < 0)
//    {
//        volume = 0;
//    }
//
//    sprintf(tmp, "Set device volume: %u\r\nSend to server after 1 minus\r\n", volume);
//    m_cb->printf(tmp);
//
//    app_flash_set_volume(volume); /* 0 - 100 */
//
//    // Write to NVS
//    app_flash_write_u8(APP_FLASH_VOLUME_KEY, volume);
//
//    // Set volume cho audio codec
//    app_audio_change_codec_vol(app_flash_get_volume());
//    return 0;
//}
