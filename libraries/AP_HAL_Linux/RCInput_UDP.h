
#ifndef _AP_HAL_LINUX_RCINPUT_UDP_H
#define _AP_HAL_LINUX_RCINPUT_UDP_H

#include "RCInput.h"
#include "../AP_HAL/utility/Socket.h"
#include "RCInput_UDP_Protocol.h"

#define RCINPUT_UDP_DEF_PORT 777

class Linux::LinuxRCInput_UDP : public Linux::LinuxRCInput
{
public:
    void init(void*);
    void _timer_tick(void);
private:
    SocketAPM   _socket{true};
    uint16_t     _port;
    struct rc_udp_packet _buf;
    uint64_t _last_buf_ts;
};
#endif // _AP_HAL_LINUX_RCINPUT_UDP_H
