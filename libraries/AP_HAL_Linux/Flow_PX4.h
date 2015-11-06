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
    Flow_PX4(uint32_t width,
             uint32_t max_flow_pixel,
             uint32_t num_blocks,
             float bottom_flow_feature_threshold,
             float bottom_flow_value_threshold);
    uint8_t compute_flow(uint8_t *image1, uint8_t *image2, uint32_t delta_time,
                 float x_rate, float y_rate, float *pixel_flow_x,
                 float *pixel_flow_y);
private:
    uint32_t _width;
    uint32_t _search_size;
    uint32_t _num_blocks;
    float _bottom_flow_feature_threshold;
    float _bottom_flow_value_threshold;
};
#endif
