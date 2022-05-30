#include "app_http.h"
#include "app_debug.h"

#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/apps/http_client.h"
#include "lwip/dns.h"
#include "lwip/debug.h"
#include "lwip/mem.h"
//#include "lwip/altcp_tls.h"
#include "lwip/init.h"
#include "string.h"
#include "fatfs.h"
#include "semphr.h"

#define HTTP_DOWNLOAD_BUFFER_SIZE 1024

static app_http_config_t m_http_cfg;
static uint32_t m_total_bytes_recv = 0;
static uint32_t m_content_length = 0;
static char m_http_cmd_buffer[256];

static httpc_connection_t m_conn_settings_try;
static err_t httpc_file_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static httpc_state_t *m_http_connection_state;

static uint8_t post_body[1024];
static uint16_t body_len;
static char name[64];
static uint8_t *data_carry;
app_http_config_t *app_http_get_config(void)
{
    return &m_http_cfg;
}
//extern bool send_offline_file;
extern SemaphoreHandle_t sent_an_offline_file;
void app_http_cleanup(void)
{
    m_total_bytes_recv = 0;
    m_content_length = 0;
    memset(&m_http_cmd_buffer, 0, sizeof(m_http_cmd_buffer));
    memset(&m_http_cfg, 0, sizeof(m_http_cfg));
}

static uint32_t m_begin = 0;
bool m_http_free = true;
bool app_http_is_idle(void)
{
    return m_http_free;
}
/**
 * @brief Result transfer done callback
 */
static void httpc_result_callback(void *arg, httpc_result_t httpc_result, u32_t rx_content_len, u32_t srv_res, err_t err)
{
    DEBUG_INFO("result: %d, content len: %d, status code: %d, mem %u\r\n", httpc_result, rx_content_len, srv_res, xPortGetFreeHeapSize());
//    audio_queue_t audio_queue;
//    audio_queue.id = AUDIO_ID_STREAM_STOP;
//    audio_queue.need_free = 0;
//    audio_queue.data = NULL;
//    audio_queue.size = 0;
//    sys_send_audio_queue(audio_queue, 2000);
    vPortFree (data_carry);
    switch (httpc_result)
    {
        case HTTPC_RESULT_OK: /** File successfully received */
        {
            DEBUG_INFO("HTTPC_RESULT_OK, speed %uKB/s\r\n", rx_content_len/(sys_get_ms()-m_begin));
            xSemaphoreGive (sent_an_offline_file);
//            send_offline_file = true;

//            bool status = false;

//            if (m_content_length && (m_content_length == m_total_bytes_recv))
//            {
//                status = true;
//            }

//            if (status)
//            {
//                if (m_http_cfg.on_event_cb)
//                    m_http_cfg.on_event_cb(APP_HTTP_EVENT_FINISH_SUCCESS, &m_total_bytes_recv);
//            }
//            else
//            {
//                if (m_http_cfg.on_event_cb)
//                        m_http_cfg.on_event_cb(APP_HTTP_EVENT_FINISH_FAILED, &m_total_bytes_recv);
//            }
        }
            break;

        case HTTPC_RESULT_ERR_UNKNOWN:     /** Unknown error */
                                           //break;
        case HTTPC_RESULT_ERR_CONNECT:     /** Connection to server failed */
                                           //break;
        case HTTPC_RESULT_ERR_HOSTNAME:    /** Failed to resolve server hostname */
                                           //break;
        case HTTPC_RESULT_ERR_CLOSED:      /** Connection unexpectedly closed by remote server */
                                           //break;
        case HTTPC_RESULT_ERR_TIMEOUT:     /** Connection timed out (server didn't respond in time) */
                                           //break;
        case HTTPC_RESULT_ERR_SVR_RESP:    /** Server responded with an error code */
                                           //break;
        case HTTPC_RESULT_ERR_MEM:         /** Local memory error */
                                           //break;
        case HTTPC_RESULT_LOCAL_ABORT:     /** Local abort */
                                           //break;
        case HTTPC_RESULT_ERR_CONTENT_LEN: /** Content length mismatch */
            DEBUG_ERROR("Error content length\r\n");
//            if (m_http_cfg.on_event_cb)
//                m_http_cfg.on_event_cb(APP_HTTP_EVENT_FINISH_FAILED, &m_total_bytes_recv);
            break;

        default:
            DEBUG_INFO("httpc_result_callback error %d\r\n", err);
            break;
    }
    
    m_http_free = true;
}
void trans_content_to_body (uint8_t * databuff, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
	{
		post_body [i] = databuff [i];
	}
	body_len = len;
}
/**
 * @brief Result transfer done callback
 */
static err_t http_post_make_body(httpc_state_t *connection, void *arg, uint8_t **buffer, uint16_t *len)
{
    DEBUG_INFO("HTTP post body\r\n");
//    static bool tested = false;
//    if (tested == false)
//    {
//        tested = true;

        *buffer = (uint8_t*)post_body;
        *len = body_len;
//    }
//    else
//    {
//        *len = 0;
//    }
    return ERR_OK;
}

void trans_file_name_to_make_body (const char * file)
{
	memcpy (name, file, 64);
	data_carry = (uint8_t *) pvPortMalloc(512* sizeof (uint8_t));
}

static err_t http_post_make_body_from_file(httpc_state_t *connection, void *arg, uint8_t **buffer, uint16_t *len)
{
		DEBUG_INFO("HTTP post body\r\n");
    	static uint32_t block = 0;
    	static uint32_t last_pos = 0;
    	uint32_t byte_read;
    	static char file_name [64];
    	memcpy (file_name, name, 64);
    	uint32_t size_of_file = fatfs_get_file_size (file_name);
//    	uint8_t *data = (uint8_t *)data_carry;
//    	static bool started = false;
//    	const char * start = "[";
//    	const char * end = "]";
//    	if (last_pos == size_of_file)
//    	{
//    		*len = 0;
//    		last_pos = 0;
//    		goto end;
//    	}
    	if(size_of_file)
    	{

//			if (started == false)
//			{
//				started = true;
//				*buffer =  (uint8_t *)start;
//				*len = 1;
//				goto end;
//			}
//			if (last_pos == size_of_file)
//			{
//				DEBUG_INFO ("NOW REACH THE END FILE\r\n");
//				*buffer = (uint8_t *)end;
//				*len = 1;
//				goto end;
//			}
    		memset (data_carry, '\0', 512);
			byte_read = fatfs_read_file_at_pos (file_name, data_carry, (uint32_t)512, last_pos);
//			DEBUG_INFO ("DATA READ: %s \r\n", data);
			if (byte_read)
			{


//				memcpy(data_carry, data, byte_read);
				DEBUG_INFO ("DATA CARRY NOW: %s\r\n", data_carry);
//				data_carry += byte_read;
				if (block*512 < size_of_file)
				{
					block ++;
					*len = 512;
				}
				else
				{
					*len = size_of_file - (512 * block) + 2;
					block = 0;
					DEBUG_INFO("All data was send to server\r\n");
				}
				*buffer = (uint8_t *)data_carry;
				last_pos += byte_read;
			}
			else
			{
				*len = 0;
				last_pos = 0;
				block = 0;
			}
			DEBUG_INFO ("LAST POS NOW IS %u\r\n",last_pos);
    	}

    	end:
    return ERR_OK;
}
/**
 * @brief Header received done callback
 */
err_t httpc_headers_done_callback(httpc_state_t *connection, void *arg, struct pbuf *hdr, u16_t hdr_len, u32_t content_len)
{
    DEBUG_INFO("httpc_headers_callback, content length %d\r\n", content_len);
    DEBUG_INFO("%.*s\r\n", hdr_len, (char*)hdr->payload);
    
    m_content_length = content_len;
    if (content_len == 0xFFFFFFFF)
    {
        DEBUG_INFO("Invalid content length\r\n");
        if (m_http_cfg.on_event_cb)
            m_http_cfg.on_event_cb(APP_HTTP_EVENT_FINISH_FAILED, &m_content_length);
    }
    else
    {
        if (m_http_cfg.on_event_cb)
            m_http_cfg.on_event_cb(APP_HTTP_EVENT_CONNTECTED, &m_content_length);
    }

    return ERR_OK;
}

/** 
 * @brief Handle data connection incoming data
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param pointer to incoming pbuf
 * @param state of incoming process
 */
uint32_t last_tick = 0;
static err_t httpc_file_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    // DEBUG_INFO("lwip http event cb\r\n");
    if (p)
    {
        struct pbuf *q;
        for (q = p; q; q = q->next)
        {
//#if TEST_VS1003 == 0
//            uint8_t *payload = (uint8_t *)q->payload;
//            uint32_t write_size = q->len;
//            
            m_total_bytes_recv += q->len;

//            app_http_data_t data;
//            data.length = write_size;
//            data.data = payload;
//            if (m_http_cfg.on_event_cb)
//                m_http_cfg.on_event_cb(APP_HTTP_EVENT_DATA, &data);
//#if 0
//            audio_queue_t audio_queue;
//            audio_queue.id = AUDIO_ID_INPUT_STREAM_DATA;
//            audio_queue.need_free = 1;
//            audio_queue.data = (uint8_t*)pvPortMalloc(q->len);
//            audio_queue.size = q->len;
//            if (audio_queue.data)
//            {
//                memcpy(audio_queue.data, q->payload, q->len);
//                if (!sys_send_audio_queue(audio_queue, 2000))
//                {
//                    vPortFree(audio_queue.data);
//                    DEBUG_ERROR("HTTP to audio : Send queue error\r\n");
//                }   
//            }
//            else
//            {
//                DEBUG_ERROR("HTTP to audio : no memory\r\n");
//            }
//#else
//            if (!sys_send_audio_raw(data.data, data.length, 5000))
//            {
//                DEBUG_ERROR("HTTP to audio : no memory\r\n");
//            }
//#endif
//#endif /* TEST_VS1003 */
//            static uint32_t dbg_count = 0;
////            if (dbg_count++ == 50)
//            {
//                dbg_count = 0;
//                DEBUG_INFO("RX %u bytes, total %u KB\r\n", q->len, m_total_bytes_recv/1024);
//            }
        }

        // MUST used 2 commands!
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
    }
    else
    {
        DEBUG_WARN("tcp_close\r\n");
        tcp_close(tpcb);
//        audio_queue_t audio_queue;
//        audio_queue.id = AUDIO_ID_STREAM_STOP;
//        audio_queue.need_free = 0;
//        audio_queue.data = NULL;
//        audio_queue.size = 0;
//        sys_send_audio_queue(audio_queue, 2000);
        return ERR_ABRT;
    }

    //DEBUG_INFO("Done\r\n");
    return ERR_OK;
}

bool app_http_start(app_http_config_t *config, int pos_len)
{
    // ASSERT(config);

    app_http_cleanup();
    sprintf(m_http_cfg.url, "%s", config->url);
    m_http_cfg.port = config->port;
    sprintf(m_http_cfg.file, "%s", config->file);
    m_http_cfg.on_event_cb = (void*)0;


    /* Init Http connection params */
    m_conn_settings_try.use_proxy = 0;
    m_conn_settings_try.headers_done_fn = httpc_headers_done_callback;
    m_conn_settings_try.result_fn = httpc_result_callback;
    
    DEBUG_INFO("HTTP url %s%s, port %d\r\n", m_http_cfg.url, m_http_cfg.file, m_http_cfg.port);
    err_t error;
    if (config->method == APP_HTTP_GET)
    {
        m_conn_settings_try.method = HTTP_METHOD_GET;
        error = httpc_get_file_dns((const char*)m_http_cfg.url, 
                                        m_http_cfg.port, 
                                        m_http_cfg.file, 
                                        &m_conn_settings_try, 
                                        httpc_file_recv_callback , 
                                        NULL, 
                                        &m_http_connection_state);
    }
    else        // post
    {
        m_conn_settings_try.method = HTTP_METHOD_POST;
        DEBUG_INFO ("POST LEN TRAN IS :%d\r\n",pos_len);
        error = httpc_post_file_dns((const char*)m_http_cfg.url, 
                                        m_http_cfg.port, 
                                        m_http_cfg.file, 
                                        &m_conn_settings_try, 
                                        httpc_file_recv_callback, 
                                        NULL, 
                                        &m_http_connection_state,
										pos_len);
    }
    
    m_conn_settings_try.headers_done_fn = httpc_headers_done_callback;
    m_conn_settings_try.result_fn = httpc_result_callback;
    if ((config->transfile) == TRANS_STRING)
    {
    	m_conn_settings_try.on_post_body_cb = http_post_make_body;
    }
    else
    {
    	DEBUG_INFO ("SET NEW CALLBACK \r\n");
    	m_conn_settings_try.on_post_body_cb = http_post_make_body_from_file;
    }
    if (error != ERR_OK)
    {
        DEBUG_INFO("Cannot connect HTTP server, error %d\r\n", error);
        return false;
    }
    
    m_begin = sys_get_ms();
    if (m_http_cfg.on_event_cb)
        m_http_cfg.on_event_cb(APP_HTTP_EVENT_START, (void*)0);
    m_http_free = false;
    return true;
}

