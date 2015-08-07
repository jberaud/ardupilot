#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#include "RCInput_UDP.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

void LinuxRCInput_UDP::init(void *)
{
    _port = RCINPUT_UDP_DEF_PORT;
    if(!_socket.bind("0.0.0.0", _port)) {
        hal.console->printf("failed to bind UDP socket\n");
    }

    _socket.set_blocking(false);

    return;
}

void LinuxRCInput_UDP::_timer_tick(void)
{
    uint16_t pwms[LINUX_RC_INPUT_NUM_CHANNELS];
    uint64_t delay;

    /* Read from udp */
    while (_socket.recv(&_buf, sizeof(_buf), 10) == sizeof(_buf)) {
        if (_buf.version != RCINPUT_UDP_VERSION) {
            hal.console->printf("bad protocol version for UDP RCInput\n");
            return;
        }
        if ((delay = _buf.timestamp_us - _last_buf_ts) > 100000) {
            hal.console->printf("no rc cmds received for %llu\n", delay);
        }
        _last_buf_ts = _buf.timestamp_us;
        memcpy(pwms, _buf.pwms, sizeof(pwms));
        _update_periods(pwms, LINUX_RC_INPUT_NUM_CHANNELS);
    }
}
#endif
