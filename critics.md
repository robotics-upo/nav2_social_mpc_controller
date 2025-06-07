# Critics Overview

This document describes the custom critics used in the `nav2_social_mpc_controller` package. Critics are cost functions that guide the robot's behavior during navigation, ensuring safety, efficiency, and social compliance.

---
## Optimization variables

The variables to optimize are the angular and linear speed of the robot in the subsequent time steps.
This is because the future position of the robot is optimized by inserting the subsequent speeds to project the state of the robot 
using the differential drive model.

---

## General passed parameters

**Weights:**
Each critic possesses its weight to determine its influence on the overall robot behavior.

**Robot params:**
The initial position and orientation of the robot is passed for the critics that require forward projection of the state.

**Path params:**
Some critics feature parameters containing the trajectorized path points, either positions or headings.

**Social params:**
The two social critics feature the information about the predicted agents positions and velocities.

---

## Agent Angle Cost Function

**Purpose:**  
Enforces an angular speed for the robot in certain situations.  
**Behavior:**  
If an agent is on the left side of the robot, the critic encourages steering to the right to follow social norms.

---

## Distance Cost Function

**Purpose:**  
Pushes the robot to follow points on the path.

**Usage:**  
- **Path Align:** If all trajectory points for different time steps are provided, aligns the robot to the path.
- **Path Follow:** If only the final point is given, acts as a path-follow critic.

---

## Goal Align Cost Function

**Purpose:**  
Reduces the angular difference between the robot's orientation and the goal pose orientation.

---

## Obstacle Cost Function

**Purpose:**  
Keeps the robot away from high-cost zones.

**Details:**  
Uses a bicubic interpolator to estimate the cost of a robot's position in future time steps using the local costmap.

---

## Social Work Cost Function

**Purpose:**  
Uses the Social Force Model (SFM) to consider social work as a cost, aiming to minimize the social impact of the robot.

---

## Velocity Cost Function

**Purpose:**  
Keeps the robot's velocity terms near the desired values.

---

## Velocity Feasibility Cost Function

**Purpose:**  
Prevents the optimizer from computing drastically different velocity terms in subsequent time steps.

---
## Example Configuration: `FollowPath`

Below is an example YAML configuration for the `FollowPath` behavior using the `nav2_social_mpc_controller::SocialMPCController` plugin:

```yaml
FollowPath:
    plugin: "nav2_social_mpc_controller::SocialMPCController"
    trajectorizer:
        omnidirectional: false
        desired_linear_vel: 0.6
        lookahead_dist: 2.0
        max_angular_vel: 1.4
        transform_tolerance: 0.5
        base_frame: "base_link"
        time_step: 0.05
        max_time: 1.5
    optimizer:
        linear_solver_type: "DENSE_SCHUR"
        param_tol: 1.0e-9
        fn_tol: 1.0e-5
        gradient_tol: 1.0e-8
        max_iterations: 40
        control_horizon: 18
        parameter_block_length: 6
        discretization: 1
        debug_optimizer: false
        current_path_weight: 1.0
        current_cmds_weight: 0.5
        weights:
            distance_weight: 20.0
            social_weight: 720.0 #400.0
            velocity_weight: 10.0
            angle_weight: 250.0
            agent_angle_weight: 40.0 #50.0
            proxemics_weight: 40.0
            velocity_feasibility_weight: 5.0
            goal_align_weight: 10.0
            obstacle_weight: 0.13 #0.15
            #obstacle_weight: 0.000005
```

This configuration sets parameters for trajectory generation and optimization, including weights for each cost function described above.

---


## License

See the repository for license information.
