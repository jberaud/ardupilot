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

#ifndef __HEAT_BEBOP_H__
#define __HEAT_BEBOP_H__

#include "AP_HAL_Linux.h"

class Linux::LinuxHeatPwm : public AP_HAL::Heat {
public:
    LinuxHeatPwm(const char* pwm_sysfs_path, float Kp, float Ki,uint32_t period_ns, float target);
    int set_imu_temp(float current);

private:
    int _duty_fd;
    int _period_fd;
    int _run_fd;
    uint64_t _last_temp_update;
    float _Kp;
    float _Ki;
    uint32_t _period_ns;
    float _sum_error;
    float _target;

    void _set_duty(uint32_t duty);
    void _set_period(uint32_t period);
    void _set_run();
};
#endif
