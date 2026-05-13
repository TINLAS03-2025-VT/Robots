#ifndef __TRANSPORT_COMMON_H__
#define __TRANSPORT_COMMON_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

#include "board.h"

#ifndef PICO // PICO_W ONLY
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

struct micro_ros_agent_locator
{
    ip_addr_t address;
    int port;
};

struct transport_buffer
{
    uint8_t *buf;
    bool packet_received;
};

#endif

void usleep(uint64_t us);
int clock_gettime(clockid_t unused, struct timespec *tp);

#endif
