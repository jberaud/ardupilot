/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "PWM_Sysfs.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <AP_Math/AP_Math.h>

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace Linux {

PWM_Sysfs::PWM_Sysfs(char* export_path, char* polarity_path,
                     char* enable_path, char* duty_path,
                     char* period_path, uint8_t channel)
    : _export_path(export_path)
    , _polarity_path(polarity_path)
    , _enable_path(enable_path)
    , _duty_path(duty_path)
    , _period_path(period_path)
{
    /*
     * Not checking for write errors here because if the PWM channel was
     * already exported, the write will return a EBUSY and we can catch up
     * any other error when trying to open the duty_cycle file descriptor.
     */
    if (_export_path != NULL) {
        Util::from(hal.util)->write_file(_export_path, "%u", channel);
    }

    if (_duty_path != NULL) {
        _duty_cycle_fd = ::open(_duty_path, O_RDWR | O_CLOEXEC);
    } else {
        hal.scheduler->panic("LinuxPWM_Sysfs:No duty path specified\n");
    }
    if (_duty_cycle_fd < 0) {
        hal.scheduler->panic("LinuxPWM_Sysfs:Unable to open file %s: %s",
                             _duty_path, strerror(errno));
    }
    free(_duty_path);
}

PWM_Sysfs::~PWM_Sysfs()
{
    if (_duty_cycle_fd >= 0) {
        ::close(_duty_cycle_fd);
    }
    if (_export_path != NULL) {
        free(_export_path);
    }
    if (_polarity_path != NULL) {
        free(_polarity_path);
    }
    if (_enable_path != NULL) {
        free(_enable_path);
    }
    if (_period_path != NULL) {
        free(_period_path);
    }
}

void PWM_Sysfs::enable(bool value)
{
    if ((_enable_path != NULL) &&
         Util::from(hal.util)->write_file(_enable_path, "%u", value) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to %s\n",
                            _enable_path, value ? "enable" : "disable");
    }
}

bool PWM_Sysfs::is_enabled()
{
    unsigned int enabled;

    if ((_enable_path != NULL) &&
         Util::from(hal.util)->read_file(_enable_path, "%u", &enabled) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to get status\n",
                            _enable_path);
    }
    return enabled;
}

void PWM_Sysfs::set_period(uint32_t nsec_period)
{
    if ((_period_path != NULL) &&
         Util::from(hal.util)->write_file(_period_path, "%u", nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to set period\n",
                            _period_path);
    }
}

uint32_t PWM_Sysfs::get_period()
{
    uint32_t nsec_period;

    if ((_period_path != NULL) &&
         Util::from(hal.util)->read_file(_period_path, "%u", &nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to get period\n",
                            _period_path);
        nsec_period = 0;
    }
    return nsec_period;
}

void PWM_Sysfs::set_freq(uint32_t freq)
{
    set_period(hz_to_nsec(freq));
}

uint32_t PWM_Sysfs::get_freq()
{
    return nsec_to_hz(get_period());
}

bool PWM_Sysfs::set_duty_cycle(uint32_t nsec_duty_cycle)
{
    /* Don't log fails since this could spam the console */
    if (dprintf(_duty_cycle_fd, "%u", nsec_duty_cycle) < 0) {
        return false;
    }

    _nsec_duty_cycle_value = nsec_duty_cycle;
    return true;
}

uint32_t PWM_Sysfs::get_duty_cycle()
{
    return _nsec_duty_cycle_value;
}

void PWM_Sysfs::set_polarity(PWM_Sysfs::Polarity polarity)
{
    if ((_polarity_path != NULL) &&
         Util::from(hal.util)->write_file(_polarity_path, "%s",
                                         polarity == NORMAL ?
                                         "normal" : "inversed") < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to set polarity\n",
                            _polarity_path);
    }
}

PWM_Sysfs::Polarity PWM_Sysfs::get_polarity()
{
    char polarity[16];

    if ((_polarity_path != NULL) &&
         Util::from(hal.util)->read_file(_polarity_path, "%s", polarity) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to get polarity\n",
                            _polarity_path);
        return NORMAL;
    }
    return strncmp(polarity, "normal", sizeof(polarity)) ? INVERSE : NORMAL;
}

/* PWM Sysfs api for mainline kernel */
char* PWM_Sysfs_Mainline::_generate_export_path(uint8_t chip)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/export", chip);
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate export path\n");
    }
    return path;
}

char* PWM_Sysfs_Mainline::_generate_polarity_path(uint8_t chip,
                                                  uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/pwm%u/polarity",
                     chip, channel);
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate polarity path\n");
    }
    return path;
}

char* PWM_Sysfs_Mainline::_generate_enable_path(uint8_t chip,
                                                uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/pwm%u/enable",
                     chip, channel);
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate enable path\n");
    }
    return path;
}

char* PWM_Sysfs_Mainline::_generate_duty_path(uint8_t chip,
                                              uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/pwm%u/duty_cycle",
                     chip, channel);
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate duty path\n");
    }
    return path;
}

char* PWM_Sysfs_Mainline::_generate_period_path(uint8_t chip,
                                                uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/pwm%u/period",
                     chip, channel);
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate period path\n");
    }
    return path;
}

PWM_Sysfs_Mainline::PWM_Sysfs_Mainline(uint8_t chip, uint8_t channel) :
    PWM_Sysfs(_generate_export_path(chip),
              _generate_polarity_path(chip, channel),
              _generate_enable_path(chip, channel),
              _generate_duty_path(chip, channel),
              _generate_period_path(chip, channel),
              channel)
{
}

/* PWM Sysfs api for bebop kernel */
char* PWM_Sysfs_Bebop::_generate_export_path()
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/export");
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate export path\n");
    }
    return path;
}

char* PWM_Sysfs_Bebop::_generate_enable_path(uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwm_%u/run",
                     channel);
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate enable path\n");
    }
    return path;
}

char* PWM_Sysfs_Bebop::_generate_duty_path(uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwm_%u/duty_ns",
                     channel);
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate duty path\n");
    }
    return path;
}

char* PWM_Sysfs_Bebop::_generate_period_path(uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwm_%u/period_ns",
                     channel);
    if (r == -1) {
        hal.scheduler->panic("LinuxPWM_Sysfs_Mainline :"
                             "couldn't allocate period path\n");
    }
    return path;
}

PWM_Sysfs_Bebop::PWM_Sysfs_Bebop(uint8_t channel) :
    PWM_Sysfs(_generate_export_path(),
              NULL,
              _generate_enable_path(channel),
              _generate_duty_path(channel),
              _generate_period_path(channel),
              channel)
{
}

}
#endif
