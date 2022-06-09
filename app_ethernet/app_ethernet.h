/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Client/Inc/app_ethernet.h 
  * @author  MCD Application Team
  * @brief   Header for app_ethernet.c module
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_ETHERNET_H
#define __APP_ETHERNET_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lwip/netif.h"
#include "stdbool.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* DHCP process states */
#define DHCP_OFF                   (uint8_t) 0
#define DHCP_START                 (uint8_t) 1
#define DHCP_WAIT_ADDRESS          (uint8_t) 2
#define DHCP_ADDRESS_ASSIGNED      (uint8_t) 3
#define DHCP_TIMEOUT               (uint8_t) 4
#define DHCP_LINK_DOWN             (uint8_t) 5
   
/* Exported macro ------------------------------------------------------------*/

#define DEST_IP_ADDR0   (uint8_t) 52
#define DEST_IP_ADDR1   (uint8_t) 59
#define DEST_IP_ADDR2   (uint8_t) 17
#define DEST_IP_ADDR3   (uint8_t) 149

#define DEST_PORT       ((uint16_t)7U)

/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   ((uint8_t) 192U)
#define IP_ADDR1   ((uint8_t) 168U)
#define IP_ADDR2   ((uint8_t) 1U)
#define IP_ADDR3   ((uint8_t) 95U)

/*NETMASK*/
#define NETMASK_ADDR0   ((uint8_t) 255U)
#define NETMASK_ADDR1   ((uint8_t) 255U)
#define NETMASK_ADDR2   ((uint8_t) 255U)
#define NETMASK_ADDR3   ((uint8_t) 0U)

/*Gateway Address*/
#define GW_ADDR0   ((uint8_t) 192U)
#define GW_ADDR1   ((uint8_t) 168U)
#define GW_ADDR2   ((uint8_t) 0U)
#define GW_ADDR3   ((uint8_t) 1U)
/* Exported functions ------------------------------------------------------- */
void ethernet_link_status_updated(struct netif *netif);
void Ethernet_Link_Periodic_Handle(struct netif *netif);
bool eth_is_cable_connected(struct netif *netif);
#if LWIP_DHCP

void DHCP_Thread(void const * argument);
void app_ethernet_notification(struct netif *netif);
#endif  
extern bool m_http_test_started;
extern bool m_ip_assigned;
#ifdef __cplusplus
}
#endif

#endif /* __APP_ETHERNET_H */
