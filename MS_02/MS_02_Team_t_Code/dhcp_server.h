// dhcp_server.h - Minimal DHCP server for the Pico W's own Wi-Fi network (access point).
//
// Hands out 192.168.4.16 .. 192.168.4.23 to up to 8 devices that join the
// Pico's network, with the Pico (192.168.4.1) as gateway and DNS.
// Call from code that holds the lwIP lock (cyw43_arch_lwip_begin/end).
#ifndef DHCP_SERVER_H
#define DHCP_SERVER_H

#include <stdbool.h>
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

// Start answering DHCP requests on `netif` (the access-point interface).
bool dhcp_server_start(struct netif *netif);

#endif // DHCP_SERVER_H
