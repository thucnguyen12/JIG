/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Client/Src/app_ethernet.c
  * @author  MCD Application Team
  * @brief   Ethernet specefic module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "main.h"
#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif
#include "app_ethernet.h"
#include "ethernetif.h"
#ifdef USE_LCD
#include "lcd_log.h"
#endif
#include "app_debug.h"
#include "stdbool.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip.h"
#include "stdio.h"
#include "app_http.h"
#include "lwip/dns.h"

extern void Netif_Config (bool restart);
extern ETH_HandleTypeDef heth;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t EthernetLinkTimer;

#if LWIP_DHCP
#define MAX_DHCP_TRIES  100
uint32_t DHCPfineTimer = 0;
uint8_t DHCP_state = DHCP_OFF;
bool m_last_link_up_status = false;
static bool m_ip_assigned = false;
bool m_http_test_started = false;
static bool m_got_ip = false;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
void ethernet_link_status_updated(struct netif *netif)
{
	DEBUG_INFO ("LINK STATUS CALLBACK\r\n");
	static uint32_t m_last_link_status = 0;
	uint32_t link_is_up = !netif_is_link_up(netif);
    if (link_is_up == false && (m_last_link_status != link_is_up))
    {
        DEBUG_INFO("Netif link down\r\n");
    }
    else if (link_is_up == true && (m_last_link_status != link_is_up))
    {
    	DEBUG_INFO("Netif link up\r\n");
    }
    m_last_link_status = link_is_up;
}

void app_ethernet_notification(struct netif *netif)
{
  if (netif_is_up(netif))
  {
    /* Update DHCP state machine */
    DHCP_state = DHCP_START;
    DEBUG_INFO("Start DHCP\r\n");
//    m_last_link_up_status = true;
  }
  else
  {
    /* Update DHCP state machine */
    DHCP_state = DHCP_LINK_DOWN;
    DEBUG_INFO("The network cable is not connected\r\n");
  }
}

static void dns_initialize(void)
{
    ip_addr_t dns_server_0 = IPADDR4_INIT_BYTES(8, 8, 8, 8);
    ip_addr_t dns_server_1 = IPADDR4_INIT_BYTES(1, 1, 1, 1);
    dns_setserver(0, &dns_server_0);
    dns_setserver(1, &dns_server_1);
    dns_init();
}

bool eth_got_ip(void)
{
	return m_got_ip;
}

#if LWIP_DHCP
/**
  * @brief  DHCP Process
  * @param  argument: network interface
  * @retval None
  */
uint8_t iptxt[32];
bool eth_is_cable_connected(struct netif *netif)
{
	uint32_t phyreg = 0;
	uint32_t err = 0;
	bool phy_link_status = false;
	if (HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &phyreg) == HAL_OK)
	{
		phy_link_status = phyreg & PHY_LINKED_STATUS ? 1 : 0;
		if (phy_link_status == false && m_last_link_up_status != phy_link_status)
		{
			m_got_ip = false;
			DEBUG_INFO("Ethernet disconnected\n", err);
			m_last_link_up_status = false;
			return false;
		}
		else if (phy_link_status  && (m_last_link_up_status != phy_link_status))
		{
			DEBUG_INFO("Ethernet connected\n", err);
			m_last_link_up_status = true;
			netif_set_up(netif);
//            if (DHCP_state == DHCP_OFF)
//            {
////            	Netif_Config (false);
//            }
//            DEBUG_INFO("Restart DHCP\n", err);
			DHCP_state = DHCP_START;
		}
		else if (!phy_link_status)
		{
			DEBUG_VERBOSE("PHY 0x%04X\r\n", phyreg);
		}
	}
	return phy_link_status;
}

void DHCP_Thread(void const * argument)
{
//	MX_LWIP_Init();
	//	  init lwip
	tcpip_init( NULL, NULL );
	Netif_Config (false);
	dns_initialize();

  struct netif *netif = (struct netif *) argument;
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  struct dhcp *dhcp;

  DEBUG_INFO("DHCP THREAT START \r\n");

  for (;;)
  {
    switch (DHCP_state)
    {
    case DHCP_START:
      {
    	  m_got_ip = false;
        ip_addr_set_zero_ip4(&netif->ip_addr);
        ip_addr_set_zero_ip4(&netif->netmask);
        ip_addr_set_zero_ip4(&netif->gw);
        if (eth_is_cable_connected(netif))
        {
			DEBUG_INFO ("LOOKING FOR DHCP \r\n");
			err_t err = dhcp_start (netif);
			if (err == ERR_OK)
			{
				DHCP_state = DHCP_WAIT_ADDRESS;
				DEBUG_INFO ("LOOKING FOR DHCP SEVER\r\n");
			}
        }
      }
      break;
    case DHCP_WAIT_ADDRESS:
      {
    	  DEBUG_INFO ("WAITING DHCP SEVER\r\n");
        if (dhcp_supplied_address(netif))
        {
          DHCP_state = DHCP_ADDRESS_ASSIGNED;
          DEBUG_INFO("IP address assigned by a DHCP server %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif)));
//          xSemaphoreGive(hHttpStart);
          m_http_test_started = false;
          m_got_ip = true;
        }
        else
        {
          dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

          /* DHCP timeout */
          if (dhcp->tries > MAX_DHCP_TRIES)
          {
            DHCP_state = DHCP_TIMEOUT;
            dhcp_stop (netif);
            /* Static address used */
            IP_ADDR4(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
            IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
            IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
            netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));

            sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
            DEBUG_ERROR ("DHCP Timeout !! \r\n");
            DEBUG_INFO ("Static IP address: %s\r\n", iptxt);
            m_last_link_up_status = true;
            m_ip_assigned = true;
            vTaskDelay(1000);
            NVIC_SystemReset();
          }
        }
      }
      break;
    case DHCP_ADDRESS_ASSIGNED:
    {

//    	xSemaphoreGive(hHttpStart);
    }
    break;
    case DHCP_TIMEOUT:
    {
//    	xSemaphoreGive(hHttpStart);

    }
    	break;
    case DHCP_LINK_DOWN:
    {
      DHCP_state = DHCP_OFF;
      dhcp_stop (netif);
      m_ip_assigned = false;
      DEBUG_INFO ("DHCPLINK DOWN\r\n");
      m_got_ip = false;
//      Netif_Config (true);
    }
    break;
    default: break;
    }
    eth_is_cable_connected(netif);
    // doi theo PHY
    /* wait 500 ms */
    osDelay(500);
  }
}
#endif  /* LWIP_DHCP */
/**
  * @brief  DHCP periodic check
  * @param  netif
  * @retval None
  */
//void DHCP_Periodic_Handle(struct netif *netif)
//{
//
//  /* Fine DHCP periodic process every 500ms */
//  if (HAL_GetTick() - DHCPfineTimer >= DHCP_FINE_TIMER_MSECS)
//  {
////	  DEBUG_INFO("GET INTO DHCP HANDLE\r\n");
//	  DHCPfineTimer =  HAL_GetTick();
//    /* process DHCP state machine */
//    //DHCP_Process(netif);
//  }
//}

