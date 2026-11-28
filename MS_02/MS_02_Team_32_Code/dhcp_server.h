// dhcp_server.h - Minimal DHCP server for the Pico W's own Wi-Fi network (access point).
//
// Hands out 192.168.4.16 .. 192.168.4.23 to up to 8 devices that join the
// Pico's network, with the Pico (192.168.4.1) as gateway and DNS.
// Call from code that holds the lwIP lock (cyw43_arch_lwip_begin/end).
#ifndef DHCP_SERVER_H                   // include guard: skip this file if it was already included
#define DHCP_SERVER_H                   // mark this file as included

#include <stdbool.h>                    // bool type
#include "lwip/ip4_addr.h"              // lwIP's IPv4 address type
#include "lwip/netif.h"                 // lwIP's network interface type

// Start answering DHCP requests on `netif` (the access-point interface).
bool dhcp_server_start(struct netif *netif);            // returns false if the UDP port could not be opened

#endif // DHCP_SERVER_H
