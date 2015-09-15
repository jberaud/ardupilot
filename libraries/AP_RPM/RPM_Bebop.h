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

#ifndef __AP_RPM_BEBOP_H__
#define __AP_RPM_BEBOP_H_h

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "RPM_Backend.h"
#include <AP_HAL_Linux/RCOutput_Bebop.h>

class AP_RPM_Bebop : public AP_RPM_Backend
{
public:
    AP_RPM_Bebop(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State* _state);
    // update state
    void update(void);

private:
    Linux::LinuxRCOutput_Bebop *_rcout;
};

#endif
