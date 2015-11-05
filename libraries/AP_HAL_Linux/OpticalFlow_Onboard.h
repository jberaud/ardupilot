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
#ifndef __OPTICALFLOW_ONBOARD_H__
#define __OPTICALFLOW_ONBOARD_H__

#include "AP_HAL_Linux.h"
#include "CameraSensor.h"
#include "Flow_PX4.h"
#include "VideoIn.h"
#include <AP_Math/AP_Math.h>
#include <linux/videodev2.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#define OPTFLOW_ONBOARD_ROTATION ROTATION_NONE
#endif

class Linux::OpticalFlow_Onboard : public AP_HAL::OpticalFlow {
public:
    void init(AP_HAL::OpticalFlow::Gyro_Cb);
    class Data_Frame {
    public:
        float pixel_flow_x_integral;
        float pixel_flow_y_integral;
        uint32_t delta_time;
        uint8_t quality;
    };
    bool read(AP_HAL::OpticalFlow::Data_Frame& frame);

private:
    void _run_optflow();
    static void *_read_thread(void *arg);
    void _rotate_back_2d(Vector2f rate, enum Rotation rotation);
    VideoIn* _videoin;
    VideoIn::Frame _last_video_frame;
    CameraSensor* _camerasensor;
    Flow_PX4* _flow;
    pthread_t _thread;
    pthread_mutex_t _mutex;
    bool _initialized = false;
    bool _data_available = false;
    uint32_t _width = 0;
    uint32_t _height = 0;
    uint32_t _format = 0;
    float _pixel_flow_x_integral = 0;
    float _pixel_flow_y_integral = 0;
    uint32_t _integration_timespan = 0;
    uint8_t _surface_quality = 0;
    AP_HAL::OpticalFlow::Gyro_Cb _get_gyro;
};
#endif
