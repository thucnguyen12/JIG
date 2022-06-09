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
bool m_ip_assigned = false;
bool m_http_test_started = false;
#endif

/* Private function prototypes -----------------------------------------------*/
static void dns_initialize(void);
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
void ethernet_link_status_updated(struct netif *netif)
{
	DEBUG_INFO ("LINK STATUS CALLBACK \r\n");
  if (netif_is_link_up(netif))
  {
#if LWIP_DHCP
    /* Update DHCP state machine */
    DHCP_state = DHCP_START;
#endif /* LWIP_DHCP */
    DEBUG_INFO("DHCP START STATUS UPDATE\r\n");

  }
  else
  {
#if LWIP_DHCP
    /* Update DHCP state machine */
    DHCP_state = DHCP_LINK_DOWN;
    DEBUG_INFO("DHCP LINK DOWN STATUS UPDATE\r\n");
  }
#endif
}

void app_ethernet_notification(struct netif *netif)
{
  if (netif_is_up(netif))
  {
    /* Update DHCP state machine */
    DHCP_state = DHCP_START;
    DEBUG_INFO("Start DHCP\r\n");
  }
  else
  {
    /* Update DHCP state machine */
    DHCP_state = DHCP_LINK_DOWN;
    DEBUG_INFO("The network cable is not connected\r\n");
  }
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
			DEBUG_INFO("Ethernet disconnected\n", err);
			m_last_link_up_status = false;
			return false;
		}
		else if (phy_link_status  && (m_last_link_up_status != phy_link_status))
		{
			DEBUG_INFO("Ethernet connected\n", err);
			m_last_link_up_status = true;
			netif_set_up(netif);
//			  __IO uint32_t tickstart = 0;
//			  uint32_t regvalue = 0;
//
//			  if(netif_is_link_up(netif))
//			  {
//			    /* Restart the auto-negotiation */
//			    if(heth.Init.AutoNegotiation != ETH_AUTONEGOTIATION_DISABLE)
//			    {
//			      /* Enable Auto-Negotiation */
//			      HAL_ETH_WritePHYRegister(&heth, PHY_BCR, PHY_AUTONEGOTIATION);
//
//			      /* Get tick */
//			      tickstart = HAL_GetTick();
//
//			      /* Wait until the auto-negotiation will be completed */
//			      do
//			      {
//			        HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);
//
//			        /* Check for the Timeout ( 1s ) */
//			        if((HAL_GetTick() - tickstart ) > 1000)
//			        {
//			          /* In case of timeout */
//			          goto error;
//			        }
//			      } while (((regvalue & PHY_AUTONEGO_COMPLETE) != PHY_AUTONEGO_COMPLETE));
//
//			      /* Read the result of the auto-negotiation */
//			      HAL_ETH_ReadPHYRegister(&heth, PHY_SR, &regvalue);
//
//			      /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
//			      if((regvalue & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
//			      {
//			        /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
//			        heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
//			      }
//			      else
//			      {
//			        /* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
//			        heth.Init.DuplexMode = ETH_MODE_HALFDUPLEX;
//			      }
//			      /* Configure the MAC with the speed fixed by the auto-negotiation process */
//			      if(regvalue & PHY_SPEED_STATUS)
//			      {
//			        /* Set Ethernet speed to 10M following the auto-negotiation */
//			        heth.Init.Speed = ETH_SPEED_10M;
//			      }
//			      else
//			      {
//			        /* Set Ethernet speed to 100M following the auto-negotiation */
//			        heth.Init.Speed = ETH_SPEED_100M;
//			      }
//			    }
//			    else /* AutoNegotiation Disable */
//			    {
//			    error :
//			      /* Check parameters */
//			      assert_param(IS_ETH_SPEED(heth.Init.Speed));
//			      assert_param(IS_ETH_DUPLEX_MODE(heth.Init.DuplexMode));
//
//			      /* Set MAC Speed and Duplex Mode to PHY */
//			      HAL_ETH_WritePHYRegister(&heth, PHY_BCR, ((uint16_t)(heth.Init.DuplexMode >> 3) |
//			                                                     (uint16_t)(heth.Init.Speed >> 1)));
//			    }
//
//			    /* ETHERNET MAC Re-Configuration */
//			    HAL_ETH_ConfigMAC(&heth, (ETH_MACInitTypeDef *) NULL);
//
//			    /* Restart MAC interface */
//			    HAL_ETH_Start(&heth);
//			  }
//			  else
//			  {
//			    /* Stop MAC interface */
//			    HAL_ETH_Stop(&heth);
//			  }
//
//			  ethernetif_notify_conn_changed(netif);
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
    	  DEBUG_INFO ("WAITING DHCP SEVER \r\n");
        if (dhcp_supplied_address(netif))
        {
        	dhcp->tries++;
          DHCP_state = DHCP_ADDRESS_ASSIGNED;
          DEBUG_WARN ("IP address assigned by a DHCP server %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif)));
//          xSemaphoreGive(hHttpStart);
          m_ip_assigned = true;
          m_http_test_started = false;
        }
        else
        {
          dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);
          dhcp->tries++;
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
            DEBUG_ERROR ("DHCP Timeout !! \n");
            DEBUG_INFO ("Static IP address: %s\n", iptxt);
            m_last_link_up_status = true;
            m_ip_assigned = true;

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
      Netif_Config (true);
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

//**************************  DSN APP ******************************/
static void dns_initialize(void)
{
    ip_addr_t dns_server_0 = IPADDR4_INIT_BYTES(8, 8, 8, 8);
    ip_addr_t dns_server_1 = IPADDR4_INIT_BYTES(1, 1, 1, 1);
    dns_setserver(0, &dns_server_0);
    dns_setserver(1, &dns_server_1);
    dns_init();
}
//******************************************************************//
