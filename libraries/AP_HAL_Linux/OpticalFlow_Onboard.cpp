/*
    This class has been implemented based on
    yavta -- Yet Another V4L2 Test Application written by:
    Laurent Pinchart <laurent.pinchart@ideasonboard.com>

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
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "OpticalFlow_Onboard.h"
#include "CameraSensor_Mt9v117.h"
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


//#define OPTICALFLOW_ONBOARD_RECORD_VIDEO 1
//#define OPTICALFLOW_ONBOARD_VIDEO_FILE "/data/ftp/internal_000/optflow.nv12"
#define OPTICAL_FLOW_ONBOARD_RTPRIO 13

extern const AP_HAL::HAL& hal;

using namespace Linux;

void OpticalFlow_Onboard::init(AP_HAL::OpticalFlow::Gyro_Cb get_gyro)
{
    uint32_t top, left;
    uint32_t memtype = V4L2_MEMORY_MMAP;
    unsigned int nbufs = 0;
    int ret;
    pthread_attr_t attr;
    struct sched_param param = {
        .sched_priority = OPTICAL_FLOW_ONBOARD_RTPRIO
    };

    if (_initialized) {
        return;
    }

    _get_gyro = get_gyro;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    _videoin = new VideoIn;
    _camerasensor = new CameraSensor_Mt9v117(hal.i2c, 0x5D, MT9V117_QVGA);
    const char* device_path = "/dev/video0";
    memtype = V4L2_MEMORY_MMAP;
    nbufs = 8;
    _width = 320;
    _height = 240;
    _format = V4L2_PIX_FMT_NV12;
#else
    hal.scheduler->panic("OpticalFlow_Onboard: unsupported board\n");
#endif

    if (device_path == NULL ||
        !_videoin->open_device(device_path, memtype)) {
        hal.scheduler->panic("OpticalFlow_Onboard: couldn't open "
                             "video device\n");
        return;
    }
    if (!_videoin->allocate_buffers(nbufs)) {
        hal.scheduler->panic("OpticalFlow_Onboard: couldn't allocate "
                             "video buffers\n");
        return;
    }

    /* we need a square image so set the format and crop
     * to get a centerded square crop of the image captured by the sensor
     */
    if (_width > _height) {
        top = 0;
        left = (_width - _height) / 2;
        _width = _height;
    } else {
        left = 0;
        top = (_height - _width) / 2;
        _height = _width;
    }

    if (!_videoin->set_format(_width, _height, _format)) {
        hal.scheduler->panic("OpticalFlow_Onboard: couldn't set"
                             " video format\n");
        return;
    }

    if (!_videoin->set_crop(top, left, _width, _height)) {
        hal.scheduler->panic("OpticalFlow_Onboard: couldn't set video crop\n");
        return;
    }

    _videoin->prepare_capture();

    /* Use px4 algorithm for optical flow */
    _flow = new Flow_PX4(_width,
                         HAL_FLOW_PX4_MAX_FLOW_PIXEL,
                         HAL_FLOW_PX4_BOTTOM_FLOW_FEATURE_THRESHOLD,
                         HAL_FLOW_PX4_BOTTOM_FLOW_VALUE_THRESHOLD,
                         HAL_FLOW_PX4_BOTTOM_FLOW_HIST_FILTER,
                         HAL_FLOW_PX4_BOTTOM_FLOW_GYRO_COMPENSATION,
                         HAL_FLOW_PX4_GYRO_COMPENSATION_THRESHOLD,
                         HAL_FLOW_PX4_FOCAL_LENGTH_MM);

    /* Create the thread that will be waiting for frames
     * Initialize thread and mutex */
    ret = pthread_mutex_init(&_mutex, NULL);
    if (ret != 0) {
        perror("OpticalFlow_Onboard: failed to init mutex\n");
        return;
    }

    ret = pthread_attr_init(&attr);
    if (ret != 0) {
        perror("OpticalFlow_Onboard: failed to init attr\n");
        return;
    }
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    ret = pthread_create(&_thread, &attr, _read_thread, this);
    if (ret != 0) {
        perror("OpticalFlow_Onboard: failed to create thread\n");
        return;
    }
    _initialized = true;
}

bool OpticalFlow_Onboard::read(AP_HAL::OpticalFlow::Data_Frame& frame)
{
    bool ret;

    pthread_mutex_lock(&_mutex);
    if (!_data_available) {
        ret = false;
        goto end;
    }
    frame.pixel_flow_x_integral = _pixel_flow_x_integral;
    frame.pixel_flow_y_integral = _pixel_flow_y_integral;
    frame.delta_time = _integration_timespan;
    frame.quality = _surface_quality;
    _integration_timespan = 0;
    _pixel_flow_x_integral = 0;
    _pixel_flow_y_integral = 0;
    _data_available = false;
    ret = true;
end:
    pthread_mutex_unlock(&_mutex);
    return ret;
}

void* OpticalFlow_Onboard::_read_thread(void *arg)
{
    OpticalFlow_Onboard* optflow_onboard = (OpticalFlow_Onboard *) arg;

    optflow_onboard->_run_optflow();
    return NULL;
}

void OpticalFlow_Onboard::_run_optflow()
{
    float rate_x, rate_y, rate_z;
    Vector3f gyro_rate;
    Vector2f flow_rate;
    VideoIn::Frame video_frame;
    uint8_t qual;

    while(true) {
        /* wait for next frame to come */
        if (!_videoin->get_frame(video_frame)) {
            hal.scheduler->panic("OpticalFlow_Onboard: couldn't get frame\n");
        }
#ifdef OPTICALFLOW_ONBOARD_RECORD_VIDEO
        int fd = open("OPTICALFLOW_ONBOARD_VIDEO_FILE", O_CREAT | O_WRONLY
                | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP |
                S_IWGRP | S_IROTH | S_IWOTH);
	    if (fd != -1) {
	        write(fd, video_frame.data, video_frame.sizeimage);
	        close(fd);
        }
#endif
        /* if it is at least the second frame we receive
         * since we have to compare 2 frames */
        if (_frame_count == 0) {
            _last_video_frame = video_frame;
            _frame_count++;
            continue;
        }

        _frame_count++;

        /* check that width = height and that they are what we
         * expect (what has been set in the px4 flow constructor)
         */
        if ((video_frame.width != _width) ||
            (video_frame.height != _height)) {
            hal.console->printf("OpticalFlow_Onboard: bad image dimensions\n");
            _videoin->put_frame(video_frame);
            continue;
        }

        /* check that the image format is useable by the optical flow.
         * the frame has to be monochrome or have the luminance in a separate
         * plane (NV12 for instance) and 8 bit pixels
         */
        if (video_frame.pixelformat != _format) {
            hal.console->printf("bad format\n");
            _videoin->put_frame(video_frame);
            continue;
        }

        /* read gyro data from EKF via the opticalflow driver */
        _get_gyro(rate_x, rate_y, rate_z);
        gyro_rate.x = rate_x;
        gyro_rate.y = rate_y;
        gyro_rate.z = rate_z;

        /* rotate it to match the camera direction
         * it wouldn't make much sense for the rotation to be something else
         * than yaw 
         */
        gyro_rate.rotate(OPTFLOW_ONBOARD_ROTATION);

        /* compute gyro data and video frames
         * get flow rate to send it to the opticalflow driver
         */
        qual = _flow->compute_flow((uint8_t*)_last_video_frame.data,
                (uint8_t *)video_frame.data,
                video_frame.timestamp - _last_video_frame.timestamp,
                gyro_rate.x, gyro_rate.y, gyro_rate.z,
                &flow_rate.x, &flow_rate.y);

        /* rotate back the 2d result to get back to the body coordinates
         * only handle yaw direction */
        _rotate_back_2d(flow_rate, OPTFLOW_ONBOARD_ROTATION);

        /* fill data frame for upper layers */
        pthread_mutex_lock(&_mutex);
        _pixel_flow_x_integral += flow_rate.x / HAL_FLOW_PX4_FOCAL_LENGTH_MM;
        _pixel_flow_y_integral += flow_rate.y / HAL_FLOW_PX4_FOCAL_LENGTH_MM;
        _integration_timespan += video_frame.timestamp -
                                 _last_video_frame.timestamp;
        _surface_quality = qual;
        _data_available = true;
        pthread_mutex_unlock(&_mutex);

        /* give the last frame back to the video input driver */
        _videoin->put_frame(_last_video_frame);
        _last_video_frame = video_frame;
    }
}

/*
 * rotate back in a 2d direction. Only deal with yaw since it doesn't make
 * sense to use an optical flow that is rolled or pitched
 */
void OpticalFlow_Onboard::_rotate_back_2d(Vector2f rate, enum Rotation rotation)
{
    float tmp;
    switch (rotation) {
    case ROTATION_NONE:
    case ROTATION_MAX:
        return;
    case ROTATION_YAW_90:
        tmp = rate.x; rate.x = rate.y; rate.y = -tmp;
        return;
    case ROTATION_YAW_180:
        rate.x = -rate.x; rate.y = -rate.y;
        return;
    case ROTATION_YAW_270:
        tmp = rate.x; rate.x = -rate.y; rate.y = tmp;
        return;
    default:
        hal.console->printf("bad rotation for optical flow\n");
        return;
    }
}
#endif
