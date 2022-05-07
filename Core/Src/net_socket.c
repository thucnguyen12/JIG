/** 
  *  Portions COPYRIGHT 2016 STMicroelectronics
  *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
  *
  ******************************************************************************
  * @file    net_sockets.c
  * @author  MCD Application Team
  * @brief   TCP/IP or UDP/IP networking functions iplementation based on LwIP API
             see the file "mbedTLS/library/net_socket_template.c" for the standard
			 implmentation
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

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include <string.h>
#include <stdint.h>
#if defined(MBEDTLS_NET_C)

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdlib.h>
#endif



#include "mbedtls/net_sockets.h"

#include "lwip/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
   
#include "netif/ethernet.h"

#include "ethernetif.h"

#include "main.h"

#include "app_debug.h"

extern struct netif netif;
static int net_would_block( const mbedtls_net_context *ctx );
/*
 * Initialize LwIP stack and get a dynamic IP address.
 */
void mbedtls_net_init( mbedtls_net_context *ctx )
{
//  
//  ip4_addr_t addr;
//  ip4_addr_t netmask;
//  ip4_addr_t gw;
//  uint32_t start;
// 
    ctx->fd = -1;
//  
//  tcpip_init(NULL, NULL);

//  /* IP default settings, to be overridden by DHCP */  

//  /*IP4_ADDR(&addr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
//  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
//  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);*/
//  
//  /* add the network interface */    
//  netif_add(&netif, &addr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
//  
//  /* register the default network interface. */
//  netif_set_default(&netif);
// 
//  /*f (netif_is_link_up(&netif))
//  {
//    netif_set_up(&netif);
//  }
//  else
//  {
//    netif_set_down(&netif);
//  }*/
//  netif.ip_addr.addr = 0;
//  dhcp_start(&netif);


//  start = sys_get_ms();
//  
//  while((netif.ip_addr.addr == 0) && (sys_get_ms() - start < 40000))
//  {
//  }
//  
//  if (netif.ip_addr.addr == 0)
//  {
//    DEBUG_INFO(" Failed to get ip address! Please check your network configuration.\n");
//  }
//  else
//  {
//     DEBUG_INFO("\nIpAdress = %lu.%lu.%lu.%lu\n", (netif.ip_addr.addr & 0xff), ((netif.ip_addr.addr >> 8) & 0xff)
//                                        , ((netif.ip_addr.addr >> 16) & 0xff), ((netif.ip_addr.addr >> 24)& 0xff));
//  }
}

/*
 * Initiate a TCP connection with host:port and the given protocol
 */
int mbedtls_net_connect( mbedtls_net_context *ctx, const char *host, const char *port, int proto )
{
  int ret;
  struct addrinfo hints;
  struct addrinfo *list;
  struct addrinfo *current;

  /* Do name resolution with both IPv6 and IPv4 */
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = proto == MBEDTLS_NET_PROTO_UDP ? SOCK_DGRAM : SOCK_STREAM;
  hints.ai_protocol = proto == MBEDTLS_NET_PROTO_UDP ? IPPROTO_UDP : IPPROTO_TCP;

  if(getaddrinfo(host, port, &hints, &list) != 0)
    return MBEDTLS_ERR_NET_UNKNOWN_HOST;

  /* Try the sockaddrs until a connection succeeds */
  ret = MBEDTLS_ERR_NET_UNKNOWN_HOST;
  for( current = list; current != NULL; current = current->ai_next)
  {
    ctx->fd = (int) socket(current->ai_family, current->ai_socktype, current->ai_protocol);
    if(ctx->fd < 0)
    {
    	DEBUG_INFO("MBEDTLS_ERR_NET_SOCKET_FAILED\r\n");
      ret = MBEDTLS_ERR_NET_SOCKET_FAILED;
      continue;
    }

    if(connect(ctx->fd, current->ai_addr, (uint32_t)current->ai_addrlen) == 0)
    {
    	DEBUG_INFO("Connected\r\n");
      ret = 0;
      break;
    }

    close( ctx->fd );
    DEBUG_INFO("MBEDTLS_ERR_NET_CONNECT_FAILED\r\n");
    ret = MBEDTLS_ERR_NET_CONNECT_FAILED;
  }

  freeaddrinfo(list);

  return ret;

}

/*
 * Create a listening socket on bind_ip:port
 */
int mbedtls_net_bind( mbedtls_net_context *ctx, const char *bind_ip, const char *port, int proto )
{
  int ret = 0;
  DEBUG_INFO ("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
  return ret;
}

/*
 * Accept a connection from a remote client
 */
int mbedtls_net_accept( mbedtls_net_context *bind_ctx,
                        mbedtls_net_context *client_ctx,
                        void *client_ip, size_t buf_size, size_t *ip_len )
{
  DEBUG_INFO ("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
  return 0;
}

/*
 * Set the socket blocking or non-blocking
 */
int mbedtls_net_set_block( mbedtls_net_context *ctx )
{
  DEBUG_INFO ("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
  return 0;
}

int mbedtls_net_set_nonblock( mbedtls_net_context *ctx )
{
  DEBUG_INFO ("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
  return 0;
}

/*
 * Portable usleep helper
 */
void mbedtls_net_usleep( unsigned long usec )
{
  DEBUG_INFO ("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
}

/*
 * Read at most 'len' characters
 */
int mbedtls_net_recv( void *ctx, unsigned char *buf, size_t len )
{
  int32_t ret;
  int32_t fd = ((mbedtls_net_context *) ctx)->fd;

  if( fd < 0 )
  {
	  DEBUG_ERROR("[%s] , Invalid context\r\n", __FUNCTION__);
    return MBEDTLS_ERR_NET_INVALID_CONTEXT;
  }
 
  ret = (int32_t) read( fd, buf, len );

  if( ret < 0 )
  {
    if(net_would_block(ctx) != 0)
    {
    	DEBUG_ERROR("[%s] , MBEDTLS_ERR_SSL_WANT_READ\r\n", __FUNCTION__);
      return MBEDTLS_ERR_SSL_WANT_READ;
    }
    
    if(errno == EPIPE || errno == ECONNRESET)
    {
    	DEBUG_ERROR("[%s] , MBEDTLS_ERR_NET_CONN_RESET\r\n", __FUNCTION__);
      return MBEDTLS_ERR_NET_CONN_RESET;
    }

    if(errno == EINTR)
    {
    	DEBUG_ERROR("[%s] , MBEDTLS_ERR_SSL_WANT_READ\r\n", __FUNCTION__);
      return MBEDTLS_ERR_SSL_WANT_READ;
    }
    DEBUG_ERROR("[%s] , MBEDTLS_ERR_NET_RECV_FAILED\r\n", __FUNCTION__);
    return MBEDTLS_ERR_NET_RECV_FAILED;
  }

  return ret;
}

/*
 * Read at most 'len' characters, blocking for at most 'timeout' ms
 */
int mbedtls_net_recv_timeout( void *ctx, unsigned char *buf, size_t len,
                      uint32_t timeout )
{
  DEBUG_INFO ("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
  
  return mbedtls_net_recv( ctx, buf, len );
}


static int net_would_block( const mbedtls_net_context *ctx )
{
  /*
   * Never return 'WOULD BLOCK' on a non-blocking socket
   */
  int val = 0;
  (void)(val);

  if( ( fcntl( ctx->fd, F_GETFL, val) & O_NONBLOCK ) != O_NONBLOCK )
    return( 0 );

  switch( errno )
  {
#if defined EAGAIN
    case EAGAIN:
#endif
#if defined EWOULDBLOCK && EWOULDBLOCK != EAGAIN
    case EWOULDBLOCK:
#endif
    return( 1 );
  }

  return( 0 );
}

/*
 * Write at most 'len' characters
 */
int mbedtls_net_send( void *ctx, const unsigned char *buf, size_t len )
{
  int32_t ret;
  int fd = ((mbedtls_net_context *) ctx)->fd;

  if( fd < 0 )
  {
	  DEBUG_INFO ("%s() NOT MBEDTLS_ERR_NET_INVALID_CONTEXT!!\n", __FUNCTION__);
    return MBEDTLS_ERR_NET_INVALID_CONTEXT;
  }
  
  ret = (int32_t) write(fd, buf, len);

  if( ret < 0 )
  {
    if(net_would_block(ctx) != 0)
    {
    	DEBUG_INFO ("%s() NOT MBEDTLS_ERR_SSL_WANT_WRITE!!\n", __FUNCTION__);
      return MBEDTLS_ERR_SSL_WANT_WRITE;
    }
    
    if(errno == EPIPE || errno == ECONNRESET)
    {
    	DEBUG_INFO ("%s() NOT MBEDTLS_ERR_NET_CONN_RESET!!\n", __FUNCTION__);
      return MBEDTLS_ERR_NET_CONN_RESET;
    }
    
    if(errno == EINTR)
    {
    	DEBUG_INFO ("%s() NOT MBEDTLS_ERR_SSL_WANT_WRITE!!\n", __FUNCTION__);
      return MBEDTLS_ERR_SSL_WANT_WRITE;
    }

    DEBUG_INFO ("%s() NOT MBEDTLS_ERR_NET_SEND_FAILED!!\n", __FUNCTION__);
    return MBEDTLS_ERR_NET_SEND_FAILED;
  }

  return ret;
}

/*
 * Gracefully close the connection
 */
void mbedtls_net_free( mbedtls_net_context *ctx )
{
  if( ctx->fd == -1 )
    return;

  shutdown( ctx->fd, 2 );
  close( ctx->fd );

  ctx->fd = -1;
 }

#endif /* MBEDTLS_NET_C */
