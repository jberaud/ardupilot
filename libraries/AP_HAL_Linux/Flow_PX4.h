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
#ifndef __FLOW_PX4_H__
#define __FLOW_PX4_H__

#include "AP_HAL_Linux.h"

class Linux::Flow_PX4 {
public:
    Flow_PX4(uint32_t width, float max_flow_pixel,
             float bottom_flow_feature_threshold,
             float bottom_flow_value_threshold,
             float bottom_flow_hist_filter,
             float bottom_flow_gyro_compensation,
             float gyro_compensation_threshold, float focal_length_px);
    uint8_t compute_flow(uint8_t *image1, uint8_t *image2, uint32_t delta_time,
                 float x_rate, float y_rate, float z_rate, float *pixel_flow_x,
                 float *pixel_flow_y);
private:
    uint32_t _width;
    uint32_t _frame_size;
    uint32_t _search_size;
    float _bottom_flow_feature_threshold;
    float _bottom_flow_value_threshold;
    float _bottom_flow_hist_filter;
    float _bottom_flow_gyro_compensation;
    float _gyro_compensation_threshold;
    float _focal_length_px;

};
#endif
