// dhcp_server.c - Minimal DHCP server for the Pico W's own Wi-Fi network (access point).
//
// Handles DISCOVER -> OFFER and REQUEST -> ACK (or NAK when a device asks for an
// address from another network). One address per device, kept for 24 h.
#include "dhcp_server.h"

#include <string.h>
#include "lwip/udp.h"
#include "lwip/pbuf.h"

#define DHCP_SERVER_PORT  67
#define DHCP_CLIENT_PORT  68
#define DHCP_MAX_LEASES   8
#define DHCP_FIRST_HOST   16        // first address handed out: x.x.x.16
#define DHCP_LEASE_SECS   86400u
#define DHCP_REPLY_LEN    300       // BOOTP minimum message size

// Message layout (RFC 2131)
#define OFF_OP       0
#define OFF_XID      4
#define OFF_FLAGS    10
#define OFF_YIADDR   16
#define OFF_SIADDR   20
#define OFF_CHADDR   28
#define OFF_MAGIC    236
#define OFF_OPTIONS  240

// Option codes and message types
#define OPT_PAD          0
#define OPT_SUBNET_MASK  1
#define OPT_ROUTER       3
#define OPT_DNS          6
#define OPT_REQUESTED_IP 50
#define OPT_LEASE_TIME   51
#define OPT_MSG_TYPE     53
#define OPT_SERVER_ID    54
#define OPT_END          255
#define DHCPDISCOVER 1
#define DHCPOFFER    2
#define DHCPREQUEST  3
#define DHCPACK      5
#define DHCPNAK      6

static const uint8_t MAGIC[4] = {99, 130, 83, 99};

static struct udp_pcb *dhcp_pcb;
static struct netif   *ap_netif;
static uint8_t         lease_mac[DHCP_MAX_LEASES][6];
static bool            lease_used[DHCP_MAX_LEASES];

static int lease_for(const uint8_t *mac) {
    for (int i = 0; i < DHCP_MAX_LEASES; i++) {
        if (lease_used[i] && memcmp(lease_mac[i], mac, 6) == 0) return i;
    }
    for (int i = 0; i < DHCP_MAX_LEASES; i++) {
        if (!lease_used[i]) {
            lease_used[i] = true;
            memcpy(lease_mac[i], mac, 6);
            return i;
        }
    }
    return -1;                       // table full
}

// Returns a pointer to the option's value, or NULL. *len receives its length.
static const uint8_t *find_option(const uint8_t *msg, size_t msg_len, uint8_t code, uint8_t *len) {
    size_t i = OFF_OPTIONS;
    while (i < msg_len) {
        uint8_t c = msg[i];
        if (c == OPT_END) break;
        if (c == OPT_PAD) { i++; continue; }
        if (i + 1 >= msg_len) break;
        uint8_t l = msg[i + 1];
        if (i + 2 + l > msg_len) break;
        if (c == code) {
            *len = l;
            return &msg[i + 2];
        }
        i += 2u + l;
    }
    return NULL;
}

static uint8_t *put_option(uint8_t *p, uint8_t code, const void *val, uint8_t len) {
    *p++ = code;
    *p++ = len;
    memcpy(p, val, len);
    return p + len;
}

static void send_reply(const uint8_t *req, uint8_t type, const ip4_addr_t *yiaddr) {
    uint8_t out[DHCP_REPLY_LEN];
    memset(out, 0, sizeof out);

    out[OFF_OP] = 2;                                 // BOOTREPLY
    out[1] = 1;                                      // Ethernet
    out[2] = 6;                                      // MAC length
    memcpy(&out[OFF_XID], &req[OFF_XID], 4);
    memcpy(&out[OFF_FLAGS], &req[OFF_FLAGS], 2);
    if (yiaddr) memcpy(&out[OFF_YIADDR], &yiaddr->addr, 4);
    const ip4_addr_t *server = netif_ip4_addr(ap_netif);
    memcpy(&out[OFF_SIADDR], &server->addr, 4);
    memcpy(&out[OFF_CHADDR], &req[OFF_CHADDR], 16);
    memcpy(&out[OFF_MAGIC], MAGIC, 4);

    uint8_t *p = &out[OFF_OPTIONS];
    p = put_option(p, OPT_MSG_TYPE, &type, 1);
    p = put_option(p, OPT_SERVER_ID, &server->addr, 4);
    if (type != DHCPNAK) {
        uint8_t lease[4] = { DHCP_LEASE_SECS >> 24, (DHCP_LEASE_SECS >> 16) & 0xff,
                             (DHCP_LEASE_SECS >> 8) & 0xff, DHCP_LEASE_SECS & 0xff };
        const ip4_addr_t *mask = netif_ip4_netmask(ap_netif);
        p = put_option(p, OPT_LEASE_TIME, lease, 4);
        p = put_option(p, OPT_SUBNET_MASK, &mask->addr, 4);
        p = put_option(p, OPT_ROUTER, &server->addr, 4);
        p = put_option(p, OPT_DNS, &server->addr, 4);
    }
    *p = OPT_END;

    struct pbuf *q = pbuf_alloc(PBUF_TRANSPORT, sizeof out, PBUF_RAM);
    if (!q) return;
    memcpy(q->payload, out, sizeof out);
    udp_sendto_if(dhcp_pcb, q, IP_ADDR_BROADCAST, DHCP_CLIENT_PORT, ap_netif);
    pbuf_free(q);
}

static void on_dhcp(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)pcb; (void)addr; (void)port;
    uint8_t msg[548];
    size_t len = pbuf_copy_partial(p, msg, sizeof msg, 0);
    pbuf_free(p);

    if (len < OFF_OPTIONS + 3 || msg[OFF_OP] != 1 || memcmp(&msg[OFF_MAGIC], MAGIC, 4) != 0) {
        return;                                      // not a DHCP request
    }
    uint8_t olen = 0;
    const uint8_t *type = find_option(msg, len, OPT_MSG_TYPE, &olen);
    if (!type || olen != 1) return;

    int idx = lease_for(&msg[OFF_CHADDR]);
    if (idx < 0) return;                             // no free address

    ip4_addr_t yi;
    const ip4_addr_t *server = netif_ip4_addr(ap_netif);
    IP4_ADDR(&yi, ip4_addr1(server), ip4_addr2(server), ip4_addr3(server), DHCP_FIRST_HOST + idx);

    if (*type == DHCPDISCOVER) {
        send_reply(msg, DHCPOFFER, &yi);
    } else if (*type == DHCPREQUEST) {
        // A device remembering an address from another network asks for it: say no,
        // so it starts over and takes ours.
        const uint8_t *want = find_option(msg, len, OPT_REQUESTED_IP, &olen);
        if (want && olen == 4 && memcmp(want, &yi.addr, 4) != 0) {
            send_reply(msg, DHCPNAK, NULL);
        } else {
            send_reply(msg, DHCPACK, &yi);
        }
    }
}

bool dhcp_server_start(struct netif *netif) {
    ap_netif = netif;
    dhcp_pcb = udp_new();
    if (!dhcp_pcb) return false;
    if (udp_bind(dhcp_pcb, IP_ANY_TYPE, DHCP_SERVER_PORT) != ERR_OK) {
        udp_remove(dhcp_pcb);
        dhcp_pcb = NULL;
        return false;
    }
    udp_recv(dhcp_pcb, on_dhcp, NULL);
    return true;
}
