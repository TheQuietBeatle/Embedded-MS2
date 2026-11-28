// lwipopts.h - lwIP settings for the web control program (Pico W, no OS).
// Based on the Raspberry Pi pico-examples defaults for
// pico_cyw43_arch_lwip_threadsafe_background.
#ifndef LWIPOPTS_H                      // include guard: skip this file if it was already included
#define LWIPOPTS_H                      // mark this file as included

#define NO_SYS                      1   // no operating system: lwIP runs from callbacks
#define LWIP_SOCKET                 0   // no BSD-style sockets API (it needs an OS)
#define LWIP_NETCONN                0   // no netconn API (it needs an OS too)
#define MEM_LIBC_MALLOC             0   // use lwIP's own heap instead of the C library's malloc
#define MEM_ALIGNMENT               4   // align memory to 4 bytes (32-bit CPU)
#define MEM_SIZE                    16000               // size of lwIP's heap in bytes
#define MEMP_NUM_TCP_SEG            32  // how many TCP segments may be queued at once
#define MEMP_NUM_ARP_QUEUE          10  // how many packets may wait for an ARP answer
#define PBUF_POOL_SIZE              24  // number of receive packet buffers
#define LWIP_ARP                    1   // ARP on: find a device's MAC address from its IP
#define LWIP_ETHERNET               1   // Ethernet-style frames on (Wi-Fi uses them here)
#define LWIP_ICMP                   1   // ICMP on: the Pico answers pings
#define LWIP_RAW                    1   // raw IP API on
#define TCP_MSS                     1460                // biggest TCP payload per packet, in bytes
#define TCP_WND                     (8 * TCP_MSS)       // receive window: data a sender may have in flight
#define TCP_SND_BUF                 (8 * TCP_MSS)       // send buffer size (limits how much is queued at once)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))  // max packet buffers in the send queue, rounded up
#define LWIP_NETIF_STATUS_CALLBACK  1   // allow a callback when the IP address changes
#define LWIP_NETIF_LINK_CALLBACK    1   // allow a callback when the link goes up or down
#define LWIP_NETIF_HOSTNAME         1   // allow the interface to have a hostname
#define LWIP_NETIF_TX_SINGLE_PBUF   1   // send each packet as one contiguous buffer
#define LWIP_CHKSUM_ALGORITHM       3   // which of lwIP's checksum routines to use
#define LWIP_IPV4                   1   // IPv4 on
#define LWIP_TCP                    1   // TCP on (the web server)
#define LWIP_UDP                    1   // UDP on (DHCP and DNS)
#define LWIP_DNS                    1   // DNS client on
#define LWIP_DHCP                   1   // DHCP client on (gets an address when joining a router)
#define DHCP_DOES_ARP_CHECK         0   // don't ARP-probe the offered address (older lwIP name)
#define LWIP_DHCP_DOES_ACD_CHECK    0   // don't run address-conflict detection (faster connect)
#define LWIP_TCP_KEEPALIVE          1   // allow TCP keep-alive packets
#define MEM_STATS                   0   // no heap statistics (saves memory)
#define SYS_STATS                   0   // no system statistics
#define MEMP_STATS                  0   // no memory-pool statistics
#define LINK_STATS                  0   // no link statistics

#endif // LWIPOPTS_H
