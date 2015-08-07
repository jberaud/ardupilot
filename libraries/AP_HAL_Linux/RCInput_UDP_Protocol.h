#ifndef _RCINPUT_UDP_PROTOCOL_H
#define _RCINPUT_UDP_PROTOCOL_H

#include <AP_Common/AP_Common.h>

#define RCINPUT_UDP_NUM_CHANNELS 16
#define RCINPUT_UDP_VERSION 1

struct PACKED rc_udp_packet {
    uint32_t version;
    uint64_t timestamp_us;
    uint16_t pwms[RCINPUT_UDP_NUM_CHANNELS];
};

#endif
