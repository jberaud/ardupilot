// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP

#include "RPM_Bebop.h"
#include <AP_HAL_Linux/RCOutput_Bebop.h>

extern const AP_HAL::HAL& hal;

AP_RPM_Bebop::AP_RPM_Bebop(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State* _state) :
    AP_RPM_Backend(_ap_rpm, instance, _state)
{
    _rcout = (Linux::LinuxRCOutput_Bebop *)hal.rcout;
}

void AP_RPM_Bebop::update(void) {
    BebopBLDC_ObsData data;

    if (_rcout->read_obs_data(data) != 0) {
        hal.console->printf("rpm bebop : couldn't read obs data\n");
        return;
    }

    for (unsigned int i=0; i<BEBOP_BLDC_MOTORS_NUM; i++) {
        state[i].rate_rpm = data.rpm[i];
        state[i].instance = 0;
        state[i].last_reading_ms = data.timestamp_ms;
    }
}
#endif
