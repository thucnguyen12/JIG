 /** 
  *
  *  Portions COPYRIGHT 2016 STMicroelectronics
  *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
  *
  ******************************************************************************
  * @file    ssl_client.c 
  * @author  MCD Application Team
  * @brief   SSL client application 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 


#if 0
int main(void)
{

    DEBUG_INFO("MBEDTLS_BIGNUM_C and/or MBEDTLS_ENTROPY_C and/or "
           "MBEDTLS_SSL_TLS_C and/or MBEDTLS_SSL_CLI_C and/or "
           "MBEDTLS_NET_C and/or MBEDTLS_RSA_C and/or "
           "MBEDTLS_CTR_DRBG_C and/or MBEDTLS_X509_CRT_PARSE_C "
           "not defined.\n");

    return(0);
}
#else

#include "mbedtls/net_sockets.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
//#include "mbedtls/certs.h"
#include "mbedtls/memory_buffer_alloc.h"

#include "main.h"
#include "cmsis_os.h"
#include "app_debug.h"
#include <string.h>


#define GW_ADDRESS      "192.168.1.1"
#define NETMASK_ADDRESS "255.255.255.0"
//#define SERVER_PORT "80"
//#define SERVER_NAME "radiotech.vn"

#define SERVER_PORT "443"
//#define SERVER_NAME "www.google.com"
//#define SERVER_NAME "dns.basato.vn"
#define SERVER_NAME "developer.mbed.org"

#define GET_REQUEST "GET / HTTP/1.0\r\n\r\n"
//
//const char user_cert[] =
//	"-----BEGIN CERTIFICATE-----\r\n"\
//	"MIIDMTCCAhmgAwIBAgIJQgAAABBiTPH0MA0GCSqGSIb3DQEBCwUAMFQxGTAXBgNV\r\n"\
//	"BAoMEEFPIEthc3BlcnNreSBMYWIxNzA1BgNVBAMMLkthc3BlcnNreSBBbnRpLVZp\r\n"\
//	"cnVzIFBlcnNvbmFsIFJvb3QgQ2VydGlmaWNhdGUwHhcNMjExMDA2MDE1MDQ0WhcN\r\n"\
//	"MjIxMDA1MDE1MDQ0WjAXMRUwEwYDVQQDEwxyYWRpb3RlY2gudm4wggEiMA0GCSqG\r\n"\
//	"SIb3DQEBAQUAA4IBDwAwggEKAoIBAQDCSM3I13zMXkR0THpeUphw7JSF9u0TJmS2\r\n"\
//	"0BxDbvxhaEjD84NjN0QaZjqyiPi5Tv5QPbWSIgSLFDVCUDBhF8iGhxeS1oA9CQvY\r\n"\
//	"7LpgTYy/l8hAK2+7CnDh1kx+rB7cGnVMrnhnBiAWi/4kh5J5rbrLalj3/BBlK3lL\r\n"\
//	"pwMKIEkpgvt72RDcqvh9vkr7klQUeB9QO2qOCNwIljv1xLz2HmfPG4Wog3o6WMgr\r\n"\
//	"fr0K7cey2mxg061+e07kMr89+7UISP4dskbG9IZkdVZqMBoJboYYsS9K5yZN9+4r\r\n"\
//	"lzL1yuBxFh5l2PCAwkNDaT1OhrQsnqRvgWwwJyWDOR8AA3a9G9ejAgMBAAGjQzBB\r\n"\
//	"MBMGA1UdJQQMMAoGCCsGAQUFBwMBMAsGA1UdDwQEAwIFoDAdBgNVHREEFjAUhwQb\r\n"\
//	"R+LAggxyYWRpb3RlY2gudm4wDQYJKoZIhvcNAQELBQADggEBAAkziqdea24oSgT5\r\n"\
//	"WTgJz3/DLV4caLMfZ2sza2tF72fTgaSJCU8W6/b34sqBQYcDgPNLPZzZTpEsjEAq\r\n"\
//	"pgQsNmHk9IX+aPaYjkIKJdLks4l+Nb7MFSlmPzfURGqqsr0ph+wXLJpQsFLPf7x/\r\n"\
//	"HKrIcnQFP1f+6MSTcZ3qD14NCkQiN8VmXl5+4OjqfWLBW4uhIc1JU/kgBNni7yV/\r\n"\
//	"KYD9A30jhk6BQfwr+xIpHEW/0/DOqYKP/RDj5KowdhrpNZcasY3xchRrCqeIB4J7\r\n"\
//	"HvEC46IO0wFDsmwrT2cva4xGW9PX7swql3UFhnkJxP9atnCkEo5rk/U9Z25fuiJY\r\n"\
//	"L7a4yVs=\r\n"\
//	"-----END CERTIFICATE-----\r\n";
//#if 0
//const char mbedtls_google_root_certificate[] = "-----BEGIN CERTIFICATE-----\r\n"
//    "MIIDujCCAqKgAwIBAgILBAAAAAABD4Ym5g0wDQYJKoZIhvcNAQEFBQAwTDEgMB4G\r\n"
//    "A1UECxMXR2xvYmFsU2lnbiBSb290IENBIC0gUjIxEzARBgNVBAoTCkdsb2JhbFNp\r\n"
//    "Z24xEzARBgNVBAMTCkdsb2JhbFNpZ24wHhcNMDYxMjE1MDgwMDAwWhcNMjExMjE1\r\n"
//    "MDgwMDAwWjBMMSAwHgYDVQQLExdHbG9iYWxTaWduIFJvb3QgQ0EgLSBSMjETMBEG\r\n"
//    "A1UEChMKR2xvYmFsU2lnbjETMBEGA1UEAxMKR2xvYmFsU2lnbjCCASIwDQYJKoZI\r\n"
//    "hvcNAQEBBQADggEPADCCAQoCggEBAKbPJA6+Lm8omUVCxKs+IVSbC9N/hHD6ErPL\r\n"
//    "v4dfxn+G07IwXNb9rfF73OX4YJYJkhD10FPe+3t+c4isUoh7SqbKSaZeqKeMWhG8\r\n"
//    "eoLrvozps6yWJQeXSpkqBy+0Hne/ig+1AnwblrjFuTosvNYSuetZfeLQBoZfXklq\r\n"
//    "tTleiDTsvHgMCJiEbKjNS7SgfQx5TfC4LcshytVsW33hoCmEofnTlEnLJGKRILzd\r\n"
//    "C9XZzPnqJworc5HGnRusyMvo4KD0L5CLTfuwNhv2GXqF4G3yYROIXJ/gkwpRl4pa\r\n"
//    "zq+r1feqCapgvdzZX99yqWATXgAByUr6P6TqBwMhAo6CygPCm48CAwEAAaOBnDCB\r\n"
//    "mTAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUm+IH\r\n"
//    "V2ccHsBqBt5ZtJot39wZhi4wNgYDVR0fBC8wLTAroCmgJ4YlaHR0cDovL2NybC5n\r\n"
//    "bG9iYWxzaWduLm5ldC9yb290LXIyLmNybDAfBgNVHSMEGDAWgBSb4gdXZxwewGoG\r\n"
//    "3lm0mi3f3BmGLjANBgkqhkiG9w0BAQUFAAOCAQEAmYFThxxol4aR7OBKuEQLq4Gs\r\n"
//    "J0/WwbgcQ3izDJr86iw8bmEbTUsp9Z8FHSbBuOmDAGJFtqkIk7mpM0sYmsL4h4hO\r\n"
//    "291xNBrBVNpGP+DTKqttVCL1OmLNIG+6KYnX3ZHu01yiPqFbQfXf5WRDLenVOavS\r\n"
//    "ot+3i9DAgBkcRcAtjOj4LaR0VknFBbVPFd5uRHg5h6h+u/N5GJG79G+dwfCMNYxd\r\n"
//    "AfvDbbnvRG15RjF+Cv6pgsH/76tuIMRQyV+dTZsXjAzlAcmgQWpzU/qlULRuJQ/7\r\n"
//    "TBj0/VLZjmmx6BEP3ojY+x1J96relc8geMJgEtslQIxq/H5COEBkEveegeGTLg==\r\n"
//    "-----END CERTIFICATE-----\r\n";
//#else
//
//const char mbedtls_google_root_certificate[] = "-----BEGIN CERTIFICATE-----\r\n"
//"MIIEijCCA3KgAwIBAgIRAKHT0VJ6AwApElT6dyEGqWYwDQYJKoZIhvcNAQELBQAw\r\n"
//"RjELMAkGA1UEBhMCVVMxIjAgBgNVBAoTGUdvb2dsZSBUcnVzdCBTZXJ2aWNlcyBM\r\n"
//"TEMxEzARBgNVBAMTCkdUUyBDQSAxQzMwHhcNMjIwNDE4MDk0NzM2WhcNMjIwNzEx\r\n"
//"MDk0NzM1WjAZMRcwFQYDVQQDEw53d3cuZ29vZ2xlLmNvbTBZMBMGByqGSM49AgEG\r\n"
//"CCqGSM49AwEHA0IABEqVwVujYbkMQasddAJm62PWFmAaO0e7TBTAbRQPgeuxEcd6\r\n"
//"dqwdfXyHONQiDPS3O15Jz89YWdYSdSnkJ6pxS1ujggJpMIICZTAOBgNVHQ8BAf8E\r\n"
//"BAMCB4AwEwYDVR0lBAwwCgYIKwYBBQUHAwEwDAYDVR0TAQH/BAIwADAdBgNVHQ4E\r\n"
//"FgQUrqXrpDrss/VYXkvak4/i6uNe7zwwHwYDVR0jBBgwFoAUinR/r4XN7pXNPZzQ\r\n"
//"4kYU83E1HScwagYIKwYBBQUHAQEEXjBcMCcGCCsGAQUFBzABhhtodHRwOi8vb2Nz\r\n"
//"cC5wa2kuZ29vZy9ndHMxYzMwMQYIKwYBBQUHMAKGJWh0dHA6Ly9wa2kuZ29vZy9y\r\n"
//"ZXBvL2NlcnRzL2d0czFjMy5kZXIwGQYDVR0RBBIwEIIOd3d3Lmdvb2dsZS5jb20w\r\n"
//"IQYDVR0gBBowGDAIBgZngQwBAgEwDAYKKwYBBAHWeQIFAzA8BgNVHR8ENTAzMDGg\r\n"
//"L6AthitodHRwOi8vY3Jscy5wa2kuZ29vZy9ndHMxYzMvUXFGeGJpOU00OGMuY3Js\r\n"
//"MIIBBgYKKwYBBAHWeQIEAgSB9wSB9ADyAHcARqVV63X6kSAwtaKJafTzfREsQXS+\r\n"
//"/Um4havy/HD+bUcAAAGAPEj6YQAABAMASDBGAiEA7SmtGTgeNtZFs6Vjy0BENToo\r\n"
//"MvLOx1NX8paYGwHzH9sCIQCDCLwPSbL4TAhX4Q98j/9Mgtfu3gognXDGI5yU8SCU\r\n"
//"1AB3AFGjsPX9AXmcVm24N3iPDKR6zBsny/eeiEKaDf7UiwXlAAABgDxI+qwAAAQD\r\n"
//"AEgwRgIhAPrK6DXSDxgTkfW5OhrrX7lCUZqCGIpmWg4Vhjc1qsvaAiEA1kHlOf/X\r\n"
//"C0oH3/F1R8vO/UFYizPVyA7a1SVhIIKC4GAwDQYJKoZIhvcNAQELBQADggEBAArJ\r\n"
//"0YCodFNys5W9iPqNTlQIC7E07x3vU85NLmaZ4M0BddA17TgXJ1R0CwbwuTbPxAsM\r\n"
//"b8wgQn4ZQ/mY7SoEpWjn8lBWszb1vGFkfWKhyW1Ce3BwKbdaTpGwcM4zpdW2IFzG\r\n"
//"tinyfKFgJqWqUKdaEwarNWB+QfhUk/LEXe1LlQyBi4WTOIBinQkr750jB3tRvS+G\r\n"
//"HvjMnKsshrCAvyY7qnzFzkCB+XxjPY91OPHRS7y0RctEMD9vV+78Dji2HKn7Fh/C\r\n"
//"Bl1P80HvmVWW9v39r4Hd9iOvvLsy3Q1UuYcGNT/u3AFO9Fl/ETSKyk4vuvZct+Uo\r\n"
//"wBmB6AAXkEnxae08SH0=\r\n"
//"-----END CERTIFICATE-----\r\n";
//#endif

const char SSL_CA_PEM[] =
/* GlobalSign Root certificate */
"-----BEGIN CERTIFICATE-----\n"
"MIIEaTCCA1GgAwIBAgILBAAAAAABRE7wQkcwDQYJKoZIhvcNAQELBQAwVzELMAkG\n"
"A1UEBhMCQkUxGTAXBgNVBAoTEEdsb2JhbFNpZ24gbnYtc2ExEDAOBgNVBAsTB1Jv\n"
"b3QgQ0ExGzAZBgNVBAMTEkdsb2JhbFNpZ24gUm9vdCBDQTAeFw0xNDAyMjAxMDAw\n"
"MDBaFw0yNDAyMjAxMDAwMDBaMGYxCzAJBgNVBAYTAkJFMRkwFwYDVQQKExBHbG9i\n"
"YWxTaWduIG52LXNhMTwwOgYDVQQDEzNHbG9iYWxTaWduIE9yZ2FuaXphdGlvbiBW\n"
"YWxpZGF0aW9uIENBIC0gU0hBMjU2IC0gRzIwggEiMA0GCSqGSIb3DQEBAQUAA4IB\n"
"DwAwggEKAoIBAQDHDmw/I5N/zHClnSDDDlM/fsBOwphJykfVI+8DNIV0yKMCLkZc\n"
"C33JiJ1Pi/D4nGyMVTXbv/Kz6vvjVudKRtkTIso21ZvBqOOWQ5PyDLzm+ebomchj\n"
"SHh/VzZpGhkdWtHUfcKc1H/hgBKueuqI6lfYygoKOhJJomIZeg0k9zfrtHOSewUj\n"
"mxK1zusp36QUArkBpdSmnENkiN74fv7j9R7l/tyjqORmMdlMJekYuYlZCa7pnRxt\n"
"Nw9KHjUgKOKv1CGLAcRFrW4rY6uSa2EKTSDtc7p8zv4WtdufgPDWi2zZCHlKT3hl\n"
"2pK8vjX5s8T5J4BO/5ZS5gIg4Qdz6V0rvbLxAgMBAAGjggElMIIBITAOBgNVHQ8B\n"
"Af8EBAMCAQYwEgYDVR0TAQH/BAgwBgEB/wIBADAdBgNVHQ4EFgQUlt5h8b0cFilT\n"
"HMDMfTuDAEDmGnwwRwYDVR0gBEAwPjA8BgRVHSAAMDQwMgYIKwYBBQUHAgEWJmh0\n"
"dHBzOi8vd3d3Lmdsb2JhbHNpZ24uY29tL3JlcG9zaXRvcnkvMDMGA1UdHwQsMCow\n"
"KKAmoCSGImh0dHA6Ly9jcmwuZ2xvYmFsc2lnbi5uZXQvcm9vdC5jcmwwPQYIKwYB\n"
"BQUHAQEEMTAvMC0GCCsGAQUFBzABhiFodHRwOi8vb2NzcC5nbG9iYWxzaWduLmNv\n"
"bS9yb290cjEwHwYDVR0jBBgwFoAUYHtmGkUNl8qJUC99BM00qP/8/UswDQYJKoZI\n"
"hvcNAQELBQADggEBAEYq7l69rgFgNzERhnF0tkZJyBAW/i9iIxerH4f4gu3K3w4s\n"
"32R1juUYcqeMOovJrKV3UPfvnqTgoI8UV6MqX+x+bRDmuo2wCId2Dkyy2VG7EQLy\n"
"XN0cvfNVlg/UBsD84iOKJHDTu/B5GqdhcIOKrwbFINihY9Bsrk8y1658GEV1BSl3\n"
"30JAZGSGvip2CTFvHST0mdCF/vIhCPnG9vHQWe3WVjwIKANnuvD58ZAWR65n5ryA\n"
"SOlCdjSXVWkkDoPWoC209fN5ikkodBpBocLTJIg1MGCUF7ThBCIxPTsvFwayuJ2G\n"
"K1pp74P1S8SqtCr4fKGxhZSM9AyHDPSsQPhZSZg=\n"
"-----END CERTIFICATE-----\n";

//const size_t mbedtls_google_root_certificate_len = sizeof(mbedtls_google_root_certificate);


#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#include <stdlib.h>
#define mbedtls_time       time 
#define mbedtls_time_t     time_t
#define mbedtls_fprintf    fprintf
#define mbedtls_printf     DEBUG_INFO
#endif


#if !defined(MBEDTLS_BIGNUM_C) || !defined(MBEDTLS_ENTROPY_C) ||  \
    !defined(MBEDTLS_SSL_TLS_C) || !defined(MBEDTLS_SSL_CLI_C) || \
    !defined(MBEDTLS_NET_C) || !defined(MBEDTLS_RSA_C) ||         \
    !defined(MBEDTLS_CERTS_C) || !defined(MBEDTLS_PEM_PARSE_C) || \
    !defined(MBEDTLS_CTR_DRBG_C) || !defined(MBEDTLS_X509_CRT_PARSE_C)
#endif

static mbedtls_net_context server_fd;
static uint32_t flags;
static uint8_t buf[1024];
static const uint8_t *pers = (uint8_t *)("ssl_client");
static uint8_t vrfy_buf[512];

static int ret;

mbedtls_entropy_context entropy;
mbedtls_ctr_drbg_context ctr_drbg;
mbedtls_ssl_context ssl;
mbedtls_ssl_config conf;
mbedtls_x509_crt cacert;

/* use static allocation to keep the heap size as low as possible */
#ifdef MBEDTLS_MEMORY_BUFFER_ALLOC_C
uint8_t memory_buf[MAX_MEM_SIZE];
#endif



static void f_dbg(void* ctx, int level, const char* file, int line, const char* msg)
{
	char *file_ext = strstr(file, "/");
	char *last_file = file;
	if (file_ext)
	{
		file_ext++;
	}
	while (file_ext)
	{
		last_file = file_ext;
		file_ext = strstr(file_ext, "/");
		if (file_ext)
		{
			file_ext++;
		}
	}
	DEBUG_RAW("%s-:%u %s\r\n",/*level,*/ last_file, line, msg);
}

void* rtosMalloc(size_t a, size_t b)
{
	return pvPortMalloc(a*b);
}

void SSL_Client(void const *argument)
{
  int len;
	DEBUG_INFO("Taskl SSL_Client\r\n");
	mbedtls_platform_set_calloc_free(rtosMalloc, vPortFree);
  /*
   * 0. Initialize the RNG and the session data
   */
#ifdef MBEDTLS_MEMORY_BUFFER_ALLOC_C
  mbedtls_memory_buffer_alloc_init(memory_buf, sizeof(memory_buf));
#endif
  mbedtls_net_init(NULL);
  mbedtls_ssl_init(&ssl);
  mbedtls_ssl_config_init(&conf);
  mbedtls_x509_crt_init(&cacert);
  mbedtls_ctr_drbg_init(&ctr_drbg);

  DEBUG_INFO("Seeding the random number generator...\r\n");
  //DEBUG_INFO("let go init entropy\r\n");
  mbedtls_entropy_init(&entropy);
  len = strlen((char *)pers);
  if ((ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                             (const unsigned char *) pers, len)) != 0)
  {
    DEBUG_ERROR("mbedtls_ctr_drbg_seed returned %d\r\n", ret);
    goto exit;
  }

  DEBUG_INFO("ok\r\n");

//  /*
//   * 0. Initialize certificates
//   */
  DEBUG_INFO("Loading the CA root certificate ...\r\n");

  ret = mbedtls_x509_crt_parse(&cacert, (const unsigned char *) SSL_CA_PEM,
                        sizeof(SSL_CA_PEM));
//  ret = mbedtls_x509_crt_parse(&cacert, (const unsigned char*) mbedtls_google_root_certificate, mbedtls_google_root_certificate_len);

  if (ret < 0)
  {
    DEBUG_INFO(" failed\r\n  !  mbedtls_x509_crt_parse returned -0x%x\r\n", -ret);
    goto exit;
  }

  DEBUG_INFO("ok (%d skipped)\r\n", ret);

  /*
   * 1. Start the connection
   */
  DEBUG_INFO("Connecting to %s:%s\r\n", SERVER_NAME, SERVER_PORT);
  
  if ((ret = mbedtls_net_connect(&server_fd, SERVER_NAME,
                                       SERVER_PORT, MBEDTLS_NET_PROTO_TCP)) != 0)
  {
    DEBUG_INFO("Failed\r\n  ! mbedtls_net_connect returned %d\n\n", ret);
    goto exit;
  }

  DEBUG_INFO("Connected\r\n");

  /*
   * 2. Setup stuff
   */
  DEBUG_INFO("Setting up the SSL/TLS structure...\r\n");
  
  if ((ret = mbedtls_ssl_config_defaults(&conf,
                  MBEDTLS_SSL_IS_CLIENT,
                  MBEDTLS_SSL_TRANSPORT_STREAM,
                  MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
  {
    DEBUG_INFO(" failed\r\n  ! mbedtls_ssl_config_defaults returned %d\r\n", ret);
    goto exit;
  }

  DEBUG_INFO("mbedtls_ssl_config_defaults finish\r\n");

  /* OPTIONAL is not optimal for security,
   * but makes interop easier in this simplified example */
	mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_OPTIONAL);
	mbedtls_ssl_conf_ca_chain(&conf, &cacert, NULL);
	mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);
	mbedtls_ssl_conf_dbg(&conf, f_dbg, NULL);
	mbedtls_debug_set_threshold(3);

	if ((ret = mbedtls_ssl_setup(&ssl, &conf)) != 0)
	{
		DEBUG_INFO(" failed\r\n  ! mbedtls_ssl_setup returned %d\r\n", ret);
		goto exit;
	}

	if ((ret = mbedtls_ssl_set_hostname(&ssl, SERVER_NAME)) != 0)
	{
		DEBUG_INFO(" failed\r\n  ! mbedtls_ssl_set_hostname returned %d\r\n", ret);
		goto exit;
	}

	mbedtls_ssl_set_bio(&ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

  /*
   * 4. Handshake
   */
  DEBUG_INFO("Performing the SSL/TLS handshake...\r\n");

  while ((ret = mbedtls_ssl_handshake(&ssl)) != 0)
  {
    if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
    {
      DEBUG_ERROR("mbedtls_ssl_handshake returned -0x%x\n\n", -ret);
      goto exit;
    }
  }

  DEBUG_INFO("Handshake done\r\n");

  /*
   * 5. Verify the server certificate
   */
  DEBUG_INFO("  . Verifying peer X.509 certificate...\r\n");

  if ((flags = mbedtls_ssl_get_verify_result(&ssl)) != 0)
  {
  
    DEBUG_INFO(" failed\n");
    mbedtls_x509_crt_verify_info((char *)vrfy_buf, sizeof(vrfy_buf), "  ! ", flags);

    DEBUG_INFO("%s\n", vrfy_buf);
  }
  else
  {
    DEBUG_INFO("ok\n");
  }
  
  /*
   * 6. Write the GET request
   */
  
  DEBUG_INFO("  > Write to server:\r\n");
  
  sprintf((char *) buf, GET_REQUEST);
  len = strlen((char *) buf);
  
  while ((ret = mbedtls_ssl_write(&ssl, buf, len)) <= 0)
  {
    if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
    {
      DEBUG_INFO(" failed\n  ! mbedtls_ssl_write returned %d\r\n", ret);
      goto exit;
    }
  }

  len = ret;
  DEBUG_INFO(" %d bytes written\n\n%s", len, (char *) buf);

  /*
   * 7. Read the HTTP response
   */
   DEBUG_INFO("  < Read from server:");

  do
  {
    len = sizeof(buf) - 1;
    memset(buf, 0, sizeof(buf));
    ret = mbedtls_ssl_read(&ssl, buf, len);

    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE)
    {
      continue;
    }
    
    if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
    {
      break;
    }

    if (ret < 0)
    {
      DEBUG_INFO("failed\n  ! mbedtls_ssl_read returned %d\n\n", ret);
      break;
    }

    if (ret == 0)
    {
      DEBUG_INFO("\n\nEOF\n\n");
      break;
    }

    len = ret;
    DEBUG_INFO(" %d bytes read\n\n%s", len, (char *) buf);
  }
  while(1);

  mbedtls_ssl_close_notify(&ssl);

exit:
  mbedtls_net_free(&server_fd);

  mbedtls_x509_crt_free(&cacert);
  mbedtls_ssl_free(&ssl);
  mbedtls_ssl_config_free(&conf);
  mbedtls_ctr_drbg_free(&ctr_drbg);
  mbedtls_entropy_free(&entropy);
  
  if ((ret < 0) && (ret != MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY))
  {
			DEBUG_INFO("MBEDTLS fail\r\n");
  }
  else
  {
			DEBUG_INFO("MBEDTLS success\r\n");
  }
}

#endif /* MBEDTLS_BIGNUM_C && MBEDTLS_ENTROPY_C && MBEDTLS_SSL_TLS_C &&
          MBEDTLS_SSL_CLI_C && MBEDTLS_NET_C && MBEDTLS_RSA_C &&
          MBEDTLS_CERTS_C && MBEDTLS_PEM_PARSE_C && MBEDTLS_CTR_DRBG_C &&
          MBEDTLS_X509_CRT_PARSE_C */
