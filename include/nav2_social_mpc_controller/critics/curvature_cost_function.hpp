// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__CURVATURE_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__CURVATURE_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace nav2_social_mpc_controller
{

class CurvatureCost
{
  /**
   * @class CurvatureCost
   * @brief Functor for computing curvature cost based on the states of three points.
   *
   * This class defines a cost function that computes the curvature cost based on the states of three points.
   * It checks if the angle between the vectors formed by these points exceeds a maximum allowed angle.
   * The cost is scaled by a weight factor.
   */
public:
  using CurvatureCostFunction = ceres::AutoDiffCostFunction<CurvatureCost, 1, 2, 2, 2>;
  CurvatureCost(double weight, double max_angle);

  /**
   * @brief Create a Ceres cost function for curvature cost.
   *
   * This function creates a Ceres cost function for curvature cost using automatic differentiation.
   * It initializes the cost function with the current instance of CurvatureCost.
   * @return A pointer to the created AutoDiffCostFunction instance.
   */
  inline static CurvatureCostFunction* Create(double weight, double max_angle)
  {
    return new CurvatureCost::CurvatureCostFunction(new CurvatureCost(weight, max_angle));
  }

  /**
   * @brief operator() computes the curvature cost based on the states of three points.
   *
   * This function computes the curvature cost based on the states of three points.
   * It calculates the angle between the vectors formed by these points and checks
   * if it exceeds the maximum allowed angle.
   *
   * @param state1 Pointer to the first state (x, y).
   * @param state2 Pointer to the second state (x, y).
   * @param state3 Pointer to the third state (x, y).
   * @param residual Pointer to store the computed residual.
   * @return True if successful, false otherwise.
   */

  template <typename T>
  bool operator()(const T* const state1, const T* const state2, const T* const state3, T* residual) const
  {
    T vector1[2] = { state2[0] - state1[0], state2[1] - state1[1] };
    T vector2[2] = { state2[0] - state3[0], state2[1] - state3[1] };

    T dot_product = (vector2[0] * vector1[0]) + (vector2[1] * vector1[1]);
    T norm_vector1 = sqrt((vector1[0] * vector1[0]) + (vector1[1] * vector1[1]));
    T norm_vector2 = sqrt((vector2[0] * vector2[0]) + (vector2[1] * vector2[1]));

    T angle = acos(dot_product / (norm_vector1 * norm_vector2));

    T bound1 = T(M_PI) - T(max_angle_);
    T bound2 = T(M_PI) + T(max_angle_);
    T bound = (bound1 + bound2) / (T)2.0;

    if ((angle < bound1) || (angle > bound2))
      residual[0] = (T)weight_ * exp(sqrt((angle - bound) * (angle - bound)));
    else
      residual[0] = T(0.0);

    return true;
  }

private:
  double weight_;
  double max_angle_;
};

}  // namespace nav2_social_mpc_controller

#endif