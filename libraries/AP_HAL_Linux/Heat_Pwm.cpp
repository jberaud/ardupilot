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
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/limits.h>
#include <string.h>
#include <math.h>
#include "Heat_Pwm.h"

extern const AP_HAL::HAL& hal;

#define HEAT_PWM_DUTY "/duty_ns"
#define HEAT_PWM_PERIOD "/period_ns"
#define HEAT_PWM_RUN "/run"

using namespace Linux;

/*
 * Constructor :
 * argument : pwm_sysfs_path is the path to the pwm directory,
 * i.e /sys/class/pwm/pwm_6 on the bebop
 */
LinuxHeatPwm::LinuxHeatPwm(const char* pwm_sysfs_path, float Kp, float Ki, uint32_t period_ns, float target) :
    _run_fd(-1),
    _duty_fd(-1),
    _period_fd(-1),
    _last_temp_update(0),
    _Kp(Kp),
    _Ki(Ki),
    _period_ns(period_ns),
    _target(target)
{
    char duty_path[PATH_MAX];
    char period_path[PATH_MAX];
    char run_path[PATH_MAX];

    strncpy(duty_path, pwm_sysfs_path, sizeof(duty_path));
    strncat(duty_path, HEAT_PWM_DUTY, sizeof(duty_path));
    _duty_fd = open(duty_path, O_RDWR);
    if (_duty_fd == -1) {
        perror("opening duty");
    }

    strncpy(period_path, pwm_sysfs_path, sizeof(period_path));
    strncat(period_path, HEAT_PWM_PERIOD, sizeof(period_path));
    _period_fd = open(period_path, O_RDWR);
    if (_period_fd == -1) {
        perror("opening period");
    }

    strncpy(run_path, pwm_sysfs_path, sizeof(run_path));
    strncat(run_path, HEAT_PWM_RUN, sizeof(run_path));
    _run_fd = open(run_path, O_RDWR);
    if (_run_fd == -1) {
        perror("opening run");
    }

    _set_period(_period_ns);
    _set_duty(0);
    _set_run();
    return;
}

int LinuxHeatPwm::set_imu_temp(float current)
{
    float error, output;

    if (hal.scheduler->micros() - _last_temp_update < 5000) {
        return 0;
    }

    /* minimal PI algo without dt */
    error = _target - current;
    /* Don't accumulate errors if the integrated error is superior
     * to the max duty cycle(pwm_period)
     */
    if ((fabsf(_sum_error) * _Ki < _period_ns)) {
        _sum_error = _sum_error + error;
    }

    output = _Kp*error + _Ki * _sum_error;

    if (output > _period_ns)
        output = _period_ns;
    else if (output < 0)
        output = 0;
    
    _set_duty(output);
    _last_temp_update = hal.scheduler->micros();
    return 0;
}

void LinuxHeatPwm::_set_duty(uint32_t duty)
{
    /* 0x + 8 digits + terminating char*/
    char duty_string[2 + 8 + 1];
    ssize_t ret;

    snprintf(duty_string, sizeof(duty_string), "0x%x", duty);
    ret = write(_duty_fd, duty_string, strlen(duty_string));
    if (ret == -1)
        perror("pwm set_duty");
}

void LinuxHeatPwm::_set_period(uint32_t period)
{
    /* 0x + 8 digits + terminating char*/
    char period_string[2 + 8 + 1];

    snprintf(period_string, sizeof(period_string), "0x%x", period);
    write(_period_fd, period_string, strlen(period_string));
}

void LinuxHeatPwm::_set_run()
{
    const char one = '1';
    if (write(_run_fd, &one, 1) == -1)
        perror("pwm set_run");
}

#endif
