/*!
 * Steering trajectory generation for quadrocopters
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once
// #include "CommonMath/Vec3.hpp"
// #include "CommonMath/Trajectory.hpp"
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"

namespace RapidQuadrocopterTrajectoryGenerator {

//! Quadrocopter steering trajectory
/*!
 * A quadrocopter steering trajectory derived from
 * the original quadrocopter state interception trajectory.
 */
class SteeringTrajectoryGenerator : public RapidTrajectoryGenerator{
 public:
  //! Constructor, user must define initial state, and the direction of gravity.
  SteeringTrajectoryGenerator(const CommonMath::Vec3 x0, const CommonMath::Vec3 v0,
                           const CommonMath::Vec3 a0,
                           const CommonMath::Vec3 gravity,
                           const float steering_amount = 0.0f) :
    RapidTrajectoryGenerator(x0, v0, a0, gravity), 
    _steering_amount(steering_amount) {
    };
  //! Return duration of the trajectory
  float get_steering() {
    return _steering_amount;
  }
  void set_steering(const float steering_amount) {
    _steering_amount = steering_amount;
  }
 private:
  float _steering_amount;  //! steering amount from the current heading [degrees]
};

}  //namespace
