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
#define netif_start Netif_Config
extern ETH_HandleTypeDef heth;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t EthernetLinkTimer;

#if LWIP_DHCP
#define MAX_DHCP_TRIES  4
uint32_t DHCPfineTimer = 0;
uint8_t DHCP_state = DHCP_OFF;
bool m_last_link_up_status = false;
static bool m_ip_assigned = false;
bool m_http_test_started = false;
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

//#if LWIP_DHCP
///**
//  * @brief  DHCP Process
//  * @param  argument: network interface
//  * @retval None
//  */
//uint8_t iptxt[32];
//void DHCP_Thread(void const * argument)
//{
//  struct netif *netif = (struct netif *) argument;
//  ip_addr_t ipaddr;
//  ip_addr_t netmask;
//  ip_addr_t gw;
//  struct dhcp *dhcp;
//  uint32_t phyreg = 0;
//  DEBUG_INFO("DHCP THREAT START \r\n");
//  uint32_t err = 0;
//  for (;;)
//  {
////	  DEBUG_INFO ("%d", DHCP_state);
////	  if (!DHCP_state)
////	  {
////		  ethernet_link_status_updated(netif);
////	  }
//	 // DEBUG_INFO("DHCP STATE : %d\r\n", DHCP_state);
//    switch (DHCP_state)
//    {
//    case DHCP_START:
//      {
//        ip_addr_set_zero_ip4(&netif->ip_addr);
//        ip_addr_set_zero_ip4(&netif->netmask);
//        ip_addr_set_zero_ip4(&netif->gw);
//
//        dhcp_start (netif);
//        DHCP_state = DHCP_WAIT_ADDRESS;
//        DEBUG_INFO ("LOOKING FOR DHCP SEVER \r\n");
//      }
//      break;
//    case DHCP_WAIT_ADDRESS:
//      {
//    	  DEBUG_INFO ("WAITING DHCP SEVER \r\n");
//        if (dhcp_supplied_address(netif))
//        {
//          DHCP_state = DHCP_ADDRESS_ASSIGNED;
//          DEBUG_INFO("IP address assigned by a DHCP server %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif)));
//          xSemaphoreGive(hHttpStart);
//          m_http_test_started = false;
//        }
//        else
//        {
//          dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);
//
//          /* DHCP timeout */
//          if (dhcp->tries > MAX_DHCP_TRIES)
//          {
//            DHCP_state = DHCP_TIMEOUT;
//            dhcp_stop (netif);
//            /* Static address used */
//            IP_ADDR4(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
//            IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
//            IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
//            netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
//
//            sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
//            DEBUG_ERROR ("DHCP Timeout !! \n");
//            DEBUG_INFO ("Static IP address: %s\n", iptxt);
//            m_last_link_up_status = true;
//            m_ip_assigned = true;
//
//          }
//        }
//      }
//      break;
//    case DHCP_ADDRESS_ASSIGNED:
//    {
//
////    	xSemaphoreGive(hHttpStart);
//    }
//    break;
//    case DHCP_TIMEOUT:
//    {
////    	xSemaphoreGive(hHttpStart);
//
//    }
//    	break;
//    case DHCP_LINK_DOWN:
//    {
//      DHCP_state = DHCP_OFF;
//      dhcp_stop (netif);
//      m_ip_assigned = false;
//      DEBUG_INFO ("DHCPLINK DOWN\r\n");
//      Netif_Config (true);
//    }
//    break;
//    default: break;
//    }
//    if ((err = HAL_ETH_ReadPHYRegister(&heth, DP83848_PHY_ADDRESS, PHY_BSR, &phyreg)) != HAL_OK)
//    {
//        DEBUG_INFO("HAL_ETH_ReadPHYRegister error %d\n", err);
//    }
//    else
//    {
//        bool phy_link_status = phyreg & PHY_LINKED_STATUS ? 1 : 0;
//        if (phy_link_status == false && m_last_link_up_status != phy_link_status)
//        {
//            DEBUG_INFO("Ethernet disconnected\n", err);
//            m_last_link_up_status = false;
//        }
//        else if (phy_link_status  && (m_last_link_up_status != phy_link_status))
//        {
//            DEBUG_INFO("Ethernet connected\n", err);
//            m_last_link_up_status = true;
//            DHCP_state = DHCP_START;
//        }
//    }
//    /* wait 500 ms */
//    osDelay(500);
//  }
//}
//#endif  /* LWIP_DHCP */
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

