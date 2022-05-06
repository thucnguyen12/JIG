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
int main( void )
{

    DEBUG_INFO("MBEDTLS_BIGNUM_C and/or MBEDTLS_ENTROPY_C and/or "
           "MBEDTLS_SSL_TLS_C and/or MBEDTLS_SSL_CLI_C and/or "
           "MBEDTLS_NET_C and/or MBEDTLS_RSA_C and/or "
           "MBEDTLS_CTR_DRBG_C and/or MBEDTLS_X509_CRT_PARSE_C "
           "not defined.\n");

    return( 0 );
}
#else

#include "mbedtls/net_sockets.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"
#include "mbedtls/memory_buffer_alloc.h"

#include "main.h"
#include "cmsis_os.h"

#include <string.h>


#define GW_ADDRESS      "192.168.1.1"
#define NETMASK_ADDRESS "255.255.255.0"
#define SERVER_PORT "80"
#define SERVER_NAME "radiotech.vn"
#define GET_REQUEST "GET / HTTP/1.0\r\n\r\n"


const char user_cert[] =
	"-----BEGIN CERTIFICATE-----\r\n"\
	"MIIDMTCCAhmgAwIBAgIJQgAAABBiTPH0MA0GCSqGSIb3DQEBCwUAMFQxGTAXBgNV\r\n"\
	"BAoMEEFPIEthc3BlcnNreSBMYWIxNzA1BgNVBAMMLkthc3BlcnNreSBBbnRpLVZp\r\n"\
	"cnVzIFBlcnNvbmFsIFJvb3QgQ2VydGlmaWNhdGUwHhcNMjExMDA2MDE1MDQ0WhcN\r\n"\
	"MjIxMDA1MDE1MDQ0WjAXMRUwEwYDVQQDEwxyYWRpb3RlY2gudm4wggEiMA0GCSqG\r\n"\
	"SIb3DQEBAQUAA4IBDwAwggEKAoIBAQDCSM3I13zMXkR0THpeUphw7JSF9u0TJmS2\r\n"\
	"0BxDbvxhaEjD84NjN0QaZjqyiPi5Tv5QPbWSIgSLFDVCUDBhF8iGhxeS1oA9CQvY\r\n"\
	"7LpgTYy/l8hAK2+7CnDh1kx+rB7cGnVMrnhnBiAWi/4kh5J5rbrLalj3/BBlK3lL\r\n"\
	"pwMKIEkpgvt72RDcqvh9vkr7klQUeB9QO2qOCNwIljv1xLz2HmfPG4Wog3o6WMgr\r\n"\
	"fr0K7cey2mxg061+e07kMr89+7UISP4dskbG9IZkdVZqMBoJboYYsS9K5yZN9+4r\r\n"\
	"lzL1yuBxFh5l2PCAwkNDaT1OhrQsnqRvgWwwJyWDOR8AA3a9G9ejAgMBAAGjQzBB\r\n"\
	"MBMGA1UdJQQMMAoGCCsGAQUFBwMBMAsGA1UdDwQEAwIFoDAdBgNVHREEFjAUhwQb\r\n"\
	"R+LAggxyYWRpb3RlY2gudm4wDQYJKoZIhvcNAQELBQADggEBAAkziqdea24oSgT5\r\n"\
	"WTgJz3/DLV4caLMfZ2sza2tF72fTgaSJCU8W6/b34sqBQYcDgPNLPZzZTpEsjEAq\r\n"\
	"pgQsNmHk9IX+aPaYjkIKJdLks4l+Nb7MFSlmPzfURGqqsr0ph+wXLJpQsFLPf7x/\r\n"\
	"HKrIcnQFP1f+6MSTcZ3qD14NCkQiN8VmXl5+4OjqfWLBW4uhIc1JU/kgBNni7yV/\r\n"\
	"KYD9A30jhk6BQfwr+xIpHEW/0/DOqYKP/RDj5KowdhrpNZcasY3xchRrCqeIB4J7\r\n"\
	"HvEC46IO0wFDsmwrT2cva4xGW9PX7swql3UFhnkJxP9atnCkEo5rk/U9Z25fuiJY\r\n"\
	"L7a4yVs=\r\n"\
	"-----END CERTIFICATE-----\r\n";


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

static void f_dbg(void* ctx, int level, const char* file, int line, const char* msg )
{
	DEBUG_RAW("LEVEL:%d-FILE:%s-LINE:%d-MESSAGE:%s\r\n",level, file, line, msg);
}

void SSL_Client(void const *argument)
{
  int len;
	DEBUG_INFO("Taskl SSL_Client\r\n");
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

  DEBUG_INFO( "\n  . Seeding the random number generator..." );
  //DEBUG_INFO("let go init entropy\r\n");
  mbedtls_entropy_init( &entropy );
  len = strlen((char *)pers);
  if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                             (const unsigned char *) pers, len ) ) != 0 )
  {
    DEBUG_INFO( " failed\n  ! mbedtls_ctr_drbg_seed returned %d\n", ret );
    goto exit;
  }

  DEBUG_INFO( " ok\n" );

  /*
   * 0. Initialize certificates
   */
  DEBUG_INFO( "  . Loading the CA root certificate ..." );

  ret = mbedtls_x509_crt_parse( &cacert, (const unsigned char *) user_cert,
                        sizeof(user_cert) );
  if( ret < 0 )
  {
    DEBUG_INFO( " failed\n  !  mbedtls_x509_crt_parse returned -0x%x\n\n", -ret );
    goto exit;
  }

  DEBUG_INFO( " ok (%d skipped)\n", ret );

  /*
   * 1. Start the connection
   */
  DEBUG_INFO( "  . Connecting to tcp/%s/%s...", SERVER_NAME, SERVER_PORT );
  
  if( ( ret = mbedtls_net_connect( &server_fd, SERVER_NAME,
                                       SERVER_PORT, MBEDTLS_NET_PROTO_TCP ) ) != 0 )
  {
    DEBUG_INFO( " failed\n  ! mbedtls_net_connect returned %d\n\n", ret );
    goto exit;
  }

  DEBUG_INFO( " ok\n" );

  /*
   * 2. Setup stuff
   */
  DEBUG_INFO( "  . Setting up the SSL/TLS structure..." );
  
  if( ( ret = mbedtls_ssl_config_defaults( &conf,
                  MBEDTLS_SSL_IS_CLIENT,
                  MBEDTLS_SSL_TRANSPORT_STREAM,
                  MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 )
  {
    DEBUG_INFO( " failed\n  ! mbedtls_ssl_config_defaults returned %d\n\n", ret );
    goto exit;
  }

  DEBUG_INFO( " ok\n" );

  /* OPTIONAL is not optimal for security,
   * but makes interop easier in this simplified example */
  mbedtls_ssl_conf_authmode( &conf, MBEDTLS_SSL_VERIFY_OPTIONAL );
  mbedtls_ssl_conf_ca_chain( &conf, &cacert, NULL );
  mbedtls_ssl_conf_rng( &conf, mbedtls_ctr_drbg_random, &ctr_drbg );
	mbedtls_ssl_conf_dbg(&conf, f_dbg, NULL);
	mbedtls_debug_set_threshold(2);

  if( ( ret = mbedtls_ssl_setup( &ssl, &conf ) ) != 0 )
  {
    DEBUG_INFO( " failed\n  ! mbedtls_ssl_setup returned %d\n\n", ret );
    goto exit;
  }

  if( ( ret = mbedtls_ssl_set_hostname( &ssl, "localhost" ) ) != 0 )
  {
    DEBUG_INFO( " failed\n  ! mbedtls_ssl_set_hostname returned %d\n\n", ret );
    goto exit;
  }

  mbedtls_ssl_set_bio( &ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv, NULL );

  /*
   * 4. Handshake
   */
  DEBUG_INFO( "  . Performing the SSL/TLS handshake..." );

  while( ( ret = mbedtls_ssl_handshake( &ssl ) ) != 0 )
  {
    if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE )
    {
      DEBUG_INFO( " failed\n  ! mbedtls_ssl_handshake returned -0x%x\n\n", -ret );
      goto exit;
    }
  }

  DEBUG_INFO( " ok\n" );

  /*
   * 5. Verify the server certificate
   */
  DEBUG_INFO( "  . Verifying peer X.509 certificate..." );

  if( ( flags = mbedtls_ssl_get_verify_result( &ssl ) ) != 0 )
  {
  
    DEBUG_INFO( " failed\n" );
    mbedtls_x509_crt_verify_info( (char *)vrfy_buf, sizeof( vrfy_buf ), "  ! ", flags );

    DEBUG_INFO( "%s\n", vrfy_buf );
  }
  else
  {
    DEBUG_INFO( " ok\n" );
  }
  
  /*
   * 6. Write the GET request
   */
  
  DEBUG_INFO( "  > Write to server:" );
  
  sprintf( (char *) buf, GET_REQUEST );
  len = strlen((char *) buf);
  
  while( ( ret = mbedtls_ssl_write( &ssl, buf, len ) ) <= 0 )
  {
    if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE )
    {
      DEBUG_INFO( " failed\n  ! mbedtls_ssl_write returned %d\n\n", ret );
      goto exit;
    }
  }

  len = ret;
  DEBUG_INFO( " %d bytes written\n\n%s", len, (char *) buf );

  /*
   * 7. Read the HTTP response
   */
   DEBUG_INFO( "  < Read from server:" );

  do
  {
    len = sizeof( buf ) - 1;
    memset( buf, 0, sizeof( buf ) );
    ret = mbedtls_ssl_read( &ssl, buf, len );

    if( ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE )
    {
      continue;
    }
    
    if( ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY )
    {
      break;
    }

    if( ret < 0 )
    {
      DEBUG_INFO( "failed\n  ! mbedtls_ssl_read returned %d\n\n", ret );
      break;
    }

    if( ret == 0 )
    {
      DEBUG_INFO( "\n\nEOF\n\n" );
      break;
    }

    len = ret;
    DEBUG_INFO( " %d bytes read\n\n%s", len, (char *) buf );
  }
  while( 1 );

  mbedtls_ssl_close_notify( &ssl );

exit:
  mbedtls_net_free( &server_fd );

  mbedtls_x509_crt_free( &cacert );
  mbedtls_ssl_free( &ssl );
  mbedtls_ssl_config_free( &conf );
  mbedtls_ctr_drbg_free( &ctr_drbg );
  mbedtls_entropy_free( &entropy );
  
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
