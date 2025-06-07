# Nav2 Social MPC Controller (ongoing work)

This is a controller (local trajectory planner) for human-aware navigation based on MPC for optimizing a robot local path among people. 


This plugin implements the `nav2_core::Controller` interface allowing it to be used across the navigation stack as a local trajectory planner in the controller server's action server (`controller_server`).

## Dependencies

- This controller receives the people around through a topic publishing ros people_msgs.
At the moment of this development, people_msgs were not still available to be installed from the ros-foxy package apt server. You can get the package from here: https://github.com/wg-perception/people/tree/ros2
- obstacle_distance_manager: https://github.com/robotics-upo/obstacle_distance_manager 


## Configuration

* **Trajectorizer**

  * `omnidirectional`  Whether to consider omnidirectional robot locomotion or a differential
  * `desired_linear_vel`  The desired maximum linear velocity to use 
  * `lookahead_dist`  The lookahead distance to use to find the lookahead point 
  * `max_angular_vel`  Maximum allowable angular velocity 
  *  `transform_tolerance`  The TF transform tolerance 
  * `base_frame`  The frame of the robot (*base_link* by default) 
  * `time_step`  The time (seconds) to project the robot movement for each step (Default: *0.05 s*)


* **Optimizer (Ceres)**

  * `linear_solver_type`  Linear solver type to be used by optimizer. Valid values are *SPARSE_NORMAL_CHOLESKY* and *DENSE_QR* 
  * `param_tol`  Parameter tolerance optimization termination criterion (*1e-15* by default) 
  * `fn_tol`  Function tolerance optimization termination criterion (*1e-7* by default) 
  * `gradient_tol`  Gradient tolerance optimization termination criterion (*1e-10* by default) 
  * `max_iterations`  maximum iterations of the optimization (*100* by default) 
  * `debug_optimizer`  whether to active debugging 

* **Cost function**

  * ToDo...



Example fully-described XML with default parameter values:

```
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: "nav2_social_mpc_controller::SocialMPCController"
      trajectorizer:
        omnidirectional: false
        desired_linear_vel: 0.6
        lookahead_dist: 2.0
        max_angular_vel: 1.4
        transform_tolerance: 0.5
        base_frame: "base_link"
        time_step: 0.1
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
          social_weight: 120.0 # 120.0 # 400.0
          velocity_weight: 10.0
          angle_weight: 250.0
          agent_angle_weight: 40.0 #50.0
          velocity_feasibility_weight: 5.0
          goal_align_weight: 10.0
          obstacle_weight: 0.15
          proxemics_weight: 100.0
```
