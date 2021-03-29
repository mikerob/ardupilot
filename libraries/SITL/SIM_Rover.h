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
/*
  rover simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a rover simulator
 */
class SimRover : public Aircraft {
public:
    SimRover(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new SimRover(frame_str);
    }

private:
    float max_speed = 40.0f;
    float max_accel = 4.0f;
    float max_brake = 8.0f; // full brake deceleration
    float idle_decel = 1.0f; // deceleration due to drag / ground
    float max_wheel_turn = 35.0f;
    float turning_circle = 20.0f; // diameter
    float skid_turn_rate = 140.0f;
	float steering_unwind = 0.025f; // effective steering output reduced by this factor (e.g. output = calculated - calculated*lateral_acc * steering_unwind - loss of 2.5% of traction
    bool skid_steering;
	bool brake = true;

    float turn_circle(float steering) const;
    float calc_yaw_rate(float steering, float speed);
    float calc_lat_accel(float steering_angle, float speed);
};

} // namespace SITL
