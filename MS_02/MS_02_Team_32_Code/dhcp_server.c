// dhcp_server.c - Minimal DHCP server for the Pico W's own Wi-Fi network (access point).
//
// Handles DISCOVER -> OFFER and REQUEST -> ACK (or NAK when a device asks for an
// address from another network). One address per device, kept for 24 h.
#include "dhcp_server.h"                // our own public interface

#include <string.h>                     // memcpy, memset, memcmp
#include "lwip/udp.h"                   // lwIP UDP API (DHCP runs over UDP)
#include "lwip/pbuf.h"                  // lwIP packet buffers

#define DHCP_SERVER_PORT  67            // standard port a DHCP server listens on
#define DHCP_CLIENT_PORT  68            // standard port DHCP clients listen on
#define DHCP_MAX_LEASES   8             // how many devices can get an address
#define DHCP_FIRST_HOST   16        // first address handed out: x.x.x.16
#define DHCP_LEASE_SECS   86400u        // lease time told to clients: 24 hours
#define DHCP_REPLY_LEN    300       // BOOTP minimum message size

// Message layout (RFC 2131)
#define OFF_OP       0                  // message direction: 1 = request, 2 = reply
#define OFF_XID      4                  // transaction id, copied back so the client can match the reply
#define OFF_FLAGS    10                 // client flags (e.g. "please broadcast the reply")
#define OFF_YIADDR   16                 // "your address": the IP being handed out
#define OFF_SIADDR   20                 // server address
#define OFF_CHADDR   28                 // client hardware (MAC) address
#define OFF_MAGIC    236                // magic cookie marking the message as DHCP
#define OFF_OPTIONS  240                // start of the variable-length options

// Option codes and message types
#define OPT_PAD          0              // filler byte with no length field
#define OPT_SUBNET_MASK  1              // network mask
#define OPT_ROUTER       3              // default gateway
#define OPT_DNS          6              // DNS server
#define OPT_REQUESTED_IP 50             // address the client is asking for
#define OPT_LEASE_TIME   51             // how long the address is valid
#define OPT_MSG_TYPE     53             // DHCP message type (one of the values below)
#define OPT_SERVER_ID    54             // which server is answering
#define OPT_END          255            // marks the end of the options
#define DHCPDISCOVER 1                  // client: "is there a DHCP server?"
#define DHCPOFFER    2                  // server: "you can have this address"
#define DHCPREQUEST  3                  // client: "I want this address"
#define DHCPACK      5                  // server: "it's yours"
#define DHCPNAK      6                  // server: "no, you can't have that address"

static const uint8_t MAGIC[4] = {99, 130, 83, 99};      // fixed DHCP magic cookie (RFC 2131)

static struct udp_pcb *dhcp_pcb;        // lwIP's handle for our UDP socket
static struct netif   *ap_netif;        // network interface we serve (the access point)
static uint8_t         lease_mac[DHCP_MAX_LEASES][6];   // MAC address that owns each slot
static bool            lease_used[DHCP_MAX_LEASES];     // true = this slot is taken

static int lease_for(const uint8_t *mac) {              // slot number for this MAC, or -1 if none is free
    for (int i = 0; i < DHCP_MAX_LEASES; i++) {         // first look for a slot this device already has
        if (lease_used[i] && memcmp(lease_mac[i], mac, 6) == 0) return i;  // same MAC: reuse its address
    }                                   // end of search for an existing slot
    for (int i = 0; i < DHCP_MAX_LEASES; i++) {         // new device: look for a free slot
        if (!lease_used[i]) {           // this one is free
            lease_used[i] = true;       // claim it
            memcpy(lease_mac[i], mac, 6);               // remember who owns it
            return i;                   // slot number decides the address
        }                               // end of free-slot check
    }                                   // end of search for a free slot
    return -1;                       // table full
}                                       // end of lease_for

// Returns a pointer to the option's value, or NULL. *len receives its length.
static const uint8_t *find_option(const uint8_t *msg, size_t msg_len, uint8_t code, uint8_t *len) {  // msg = whole DHCP message, code = option wanted
    size_t i = OFF_OPTIONS;             // start at the first option
    while (i < msg_len) {               // stay inside the received message
        uint8_t c = msg[i];             // this option's code
        if (c == OPT_END) break;        // no more options
        if (c == OPT_PAD) { i++; continue; }            // padding is a single byte: skip it
        if (i + 1 >= msg_len) break;    // length byte missing: malformed
        uint8_t l = msg[i + 1];         // length of this option's value
        if (i + 2 + l > msg_len) break;                 // value runs past the message: malformed
        if (c == code) {                // this is the option we want
            *len = l;                   // report its length
            return &msg[i + 2];         // the value starts after code and length
        }                               // end of match check
        i += 2u + l;                    // jump over code, length and value
    }                                   // end of option loop
    return NULL;                        // option not present
}                                       // end of find_option

static uint8_t *put_option(uint8_t *p, uint8_t code, const void *val, uint8_t len) {  // append one option at p, return where the next goes
    *p++ = code;                        // option code first
    *p++ = len;                         // then the value's length
    memcpy(p, val, len);                // then the value itself
    return p + len;                     // new write position
}                                       // end of put_option

static void send_reply(const uint8_t *req, uint8_t type, const ip4_addr_t *yiaddr) {  // build one reply and broadcast it
    uint8_t out[DHCP_REPLY_LEN];        // buffer for the reply
    memset(out, 0, sizeof out);         // start with everything zero

    out[OFF_OP] = 2;                                 // BOOTREPLY
    out[1] = 1;                                      // Ethernet
    out[2] = 6;                                      // MAC length
    memcpy(&out[OFF_XID], &req[OFF_XID], 4);            // same transaction id as the request
    memcpy(&out[OFF_FLAGS], &req[OFF_FLAGS], 2);        // same flags as the request
    if (yiaddr) memcpy(&out[OFF_YIADDR], &yiaddr->addr, 4);             // address handed out (none for a NAK)
    const ip4_addr_t *server = netif_ip4_addr(ap_netif);                // our own IP address
    memcpy(&out[OFF_SIADDR], &server->addr, 4);         // put it in the server field
    memcpy(&out[OFF_CHADDR], &req[OFF_CHADDR], 16);     // copy the client's MAC field back
    memcpy(&out[OFF_MAGIC], MAGIC, 4);  // mark the message as DHCP

    uint8_t *p = &out[OFF_OPTIONS];     // options are written from here on
    p = put_option(p, OPT_MSG_TYPE, &type, 1);          // what kind of reply this is
    p = put_option(p, OPT_SERVER_ID, &server->addr, 4);                 // who is answering
    if (type != DHCPNAK) {              // only OFFER and ACK carry network settings
        uint8_t lease[4] = { DHCP_LEASE_SECS >> 24, (DHCP_LEASE_SECS >> 16) & 0xff,  // lease time, high byte first...
                             (DHCP_LEASE_SECS >> 8) & 0xff, DHCP_LEASE_SECS & 0xff };  // ...down to the low byte (network byte order)
        const ip4_addr_t *mask = netif_ip4_netmask(ap_netif);           // our network mask
        p = put_option(p, OPT_LEASE_TIME, lease, 4);    // how long the address is valid
        p = put_option(p, OPT_SUBNET_MASK, &mask->addr, 4);             // network mask
        p = put_option(p, OPT_ROUTER, &server->addr, 4);                // gateway = the Pico
        p = put_option(p, OPT_DNS, &server->addr, 4);   // DNS = the Pico
    }                                   // end of network settings
    *p = OPT_END;                       // finish the options list

    struct pbuf *q = pbuf_alloc(PBUF_TRANSPORT, sizeof out, PBUF_RAM);  // get an lwIP buffer for the packet
    if (!q) return;                     // out of memory: send nothing
    memcpy(q->payload, out, sizeof out);                // copy the reply into it
    udp_sendto_if(dhcp_pcb, q, IP_ADDR_BROADCAST, DHCP_CLIENT_PORT, ap_netif);  // broadcast to every device on the Pico's network
    pbuf_free(q);                       // give the buffer back
}                                       // end of send_reply

static void on_dhcp(void *arg, struct udp_pcb *pcb, struct pbuf *p,     // lwIP calls this when a packet arrives on port 67
                    const ip_addr_t *addr, u16_t port) {                // addr and port = where it came from
    (void)arg; (void)pcb; (void)addr; (void)port;       // unused parameters: silence compiler warnings
    uint8_t msg[548];                   // copy of the message (at most 548 bytes)
    size_t len = pbuf_copy_partial(p, msg, sizeof msg, 0);              // copy it out of lwIP's buffer
    pbuf_free(p);                       // lwIP's buffer is no longer needed

    if (len < OFF_OPTIONS + 3 || msg[OFF_OP] != 1 || memcmp(&msg[OFF_MAGIC], MAGIC, 4) != 0) {  // too short, not a request, or no magic cookie
        return;                                      // not a DHCP request
    }                                   // end of sanity check
    uint8_t olen = 0;                   // length of the option found
    const uint8_t *type = find_option(msg, len, OPT_MSG_TYPE, &olen);   // find the message type
    if (!type || olen != 1) return;     // missing or malformed: ignore it

    int idx = lease_for(&msg[OFF_CHADDR]);              // address slot for this device's MAC
    if (idx < 0) return;                             // no free address

    ip4_addr_t yi;                      // the address we give this device
    const ip4_addr_t *server = netif_ip4_addr(ap_netif);                // our own IP address
    IP4_ADDR(&yi, ip4_addr1(server), ip4_addr2(server), ip4_addr3(server), DHCP_FIRST_HOST + idx);  // our first 3 numbers, then 16 + slot

    if (*type == DHCPDISCOVER) {        // client is looking for a server
        send_reply(msg, DHCPOFFER, &yi);                // offer it the address
    } else if (*type == DHCPREQUEST) {  // client wants to take an address
        // A device remembering an address from another network asks for it: say no,
        // so it starts over and takes ours.
        const uint8_t *want = find_option(msg, len, OPT_REQUESTED_IP, &olen);  // which address does it want?
        if (want && olen == 4 && memcmp(want, &yi.addr, 4) != 0) {      // not the address we assigned to it
            send_reply(msg, DHCPNAK, NULL);             // refuse
        } else {                        // it wants our address (or didn't say)
            send_reply(msg, DHCPACK, &yi);              // confirm it
        }                               // end of address check
    }                                   // end of message type check
}                                       // end of on_dhcp

bool dhcp_server_start(struct netif *netif) {           // see dhcp_server.h
    ap_netif = netif;                   // remember which interface we serve
    dhcp_pcb = udp_new();               // create a UDP socket
    if (!dhcp_pcb) return false;        // out of memory
    if (udp_bind(dhcp_pcb, IP_ANY_TYPE, DHCP_SERVER_PORT) != ERR_OK) {  // listen on port 67; if that fails...
        udp_remove(dhcp_pcb);           // ...free the socket
        dhcp_pcb = NULL;                // ...forget it
        return false;                   // ...and report failure
    }                                   // end of bind check
    udp_recv(dhcp_pcb, on_dhcp, NULL);  // call on_dhcp for every packet received
    return true;                        // server is running
}                                       // end of dhcp_server_start
