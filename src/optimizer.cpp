#include "nav2_social_mpc_controller/optimizer.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2/utils.h"

namespace nav2_social_mpc_controller
{

/**
 * @brief OptimizerParams class constructor
 * @param node LifecycleNode to get parameters from
 * @param name Name of the optimizer parameters
 */

void OptimizerParams::get(rclcpp_lifecycle::LifecycleNode* node, const std::string& name)
{
  // Get the different parameters from the node, defined in the config file
  // if not defined, set to default values

  std::string trajectorizer = name + std::string(".trajectorizer.");
  std::string local_name = name + std::string(".optimizer.");
  std::string weights = local_name + std::string("weights.");

  // Optimizer params
  nav2_util::declare_parameter_if_not_declared(
      node, local_name + "linear_solver_type",
      rclcpp::ParameterValue("SPARSE_NORMAL_CHOLESKY"));  // SPARSE_NORMAL_CHOLESKY
                                                          // //DENSE_QR
  node->get_parameter(local_name + "linear_solver_type", linear_solver_type);
  if (solver_types.find(linear_solver_type) == solver_types.end())
  {
    std::stringstream valid_types_str;
    for (auto type = solver_types.begin(); type != solver_types.end(); type++)
    {
      if (type != solver_types.begin())
      {
        valid_types_str << ", ";
      }
      valid_types_str << type->first;
    }
    RCLCPP_ERROR(rclcpp::get_logger("optimizer"), "Invalid linear_solver_type. Valid values are %s",
                 valid_types_str.str().c_str());
    throw std::runtime_error("Invalid parameter: linear_solver_type");
  }
  nav2_util::declare_parameter_if_not_declared(node, local_name + "param_tol", rclcpp::ParameterValue(1e-15));
  node->get_parameter(local_name + "param_tol", param_tol);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "fn_tol", rclcpp::ParameterValue(1e-7));
  node->get_parameter(local_name + "fn_tol", fn_tol);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "gradient_tol", rclcpp::ParameterValue(1e-10));
  node->get_parameter(local_name + "gradient_tol", gradient_tol);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "max_iterations", rclcpp::ParameterValue(100));
  node->get_parameter(local_name + "max_iterations", max_iterations);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
  node->get_parameter(local_name + "debug_optimizer", debug);

  nav2_util::declare_parameter_if_not_declared(node, weights + "distance_weight", rclcpp::ParameterValue(3.0));
  node->get_parameter(weights + "distance_weight", distance_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "social_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(weights + "social_weight", socialwork_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "velocity_weight", rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "velocity_weight", velocity_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "angle_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "angle_weight", angle_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "agent_angle_weight", rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "agent_angle_weight", agent_angle_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "proxemics_weight", rclcpp::ParameterValue(90.0));
  node->get_parameter(weights + "proxemics_weight", proxemics_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "velocity_feasibility_weight",
                                               rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "velocity_feasibility_weight", velocity_feasibility_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "obstacle_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "obstacle_weight", obstacle_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "goal_align_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "goal_align_weight", goal_align_w_);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "control_horizon", rclcpp::ParameterValue(5));
  node->get_parameter(local_name + "control_horizon", control_horizon_);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "parameter_block_length", rclcpp::ParameterValue(5));
  node->get_parameter(local_name + "parameter_block_length", parameter_block_length_);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "current_path_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(local_name + "current_path_weight", current_path_w);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "current_cmds_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(local_name + "current_cmds_weight", current_cmds_w);
  node->get_parameter(trajectorizer + "max_time", max_time);
}
// constructor and destructor for Optimizer
Optimizer::Optimizer()
{
}
Optimizer::~Optimizer()
{
}

/**
 * @brief Initialization of the smoother
 * @param params OptimizerParam struct
 */
void Optimizer::initialize(const OptimizerParams params)
{
  // Initialize the optimizer with the parameters, getting the values from the
  // OptimizerParams struct
  debug_ = params.debug;
  obstacle_w_ = params.obstacle_w_;
  goal_align_w_ = params.goal_align_w_;
  velocity_feasibility_w_ = params.velocity_feasibility_w_;
  socialwork_w_ = params.socialwork_w_;
  distance_w_ = params.distance_w_;
  velocity_w_ = params.velocity_w_;
  angle_w_ = params.angle_w_;
  agent_angle_w_ = params.agent_angle_w_;
  proxemics_w_ = params.proxemics_w_;
  control_horizon_ = params.control_horizon_;
  parameter_block_length_ = params.parameter_block_length_;
  max_time = params.max_time;
  current_path_w = params.current_path_w;
  current_cmds_w = params.current_cmds_w;
  options_.linear_solver_type = params.solver_types.at(params.linear_solver_type);
  options_.max_num_iterations = params.max_iterations;
  options_.function_tolerance = params.fn_tol;
  options_.gradient_tolerance = params.gradient_tol;
  options_.parameter_tolerance = params.param_tol;
  if (debug_)
  {
    options_.minimizer_progress_to_stdout = true;
    options_.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
  }
  else
  {
    options_.logging_type = ceres::SILENT;
  }
  options_.max_solver_time_in_seconds = params.max_time;
}

/**
 * @brief main optimization function, where the trajectory is optimized around
 * a starting trajectory, given a set of people and a costmap.
 *
 * @param path starting trajectory to optimize
 * @param people_proj projected people in the field of view of the robot (filled by optimizer)
 * @param costmap costmap to use for the optimization
 * @param obstacles obstacles surrounding the robot, to fill the obstacles points for agents
 * @param cmds starting commands to optimize, which would result in the given path
 * @param people people that fall in the field of view of the robot, which will be projected
 * @param speed initial speed of the robot
 * @param time_step length of the time step for the forward projection of the trajectory
 * @return true if the optimization was successful, false otherwise
 */
bool Optimizer::optimize(nav_msgs::msg::Path& path, AgentsTrajectories& people_proj,
                         const nav2_costmap_2d::Costmap2D* costmap,
                         const obstacle_distance_msgs::msg::ObstacleDistance& obstacles,
                         std::vector<geometry_msgs::msg::TwistStamped>& cmds, const people_msgs::msg::People& people,
                         const geometry_msgs::msg::Twist& speed, const float time_step)
{
  // transfrom people to agent status factor
  AgentsStates init_people = people_to_status(people);

  // Path has always at least 2 points
  if (path.poses.size() < 2)
  {
    RCLCPP_WARN(rclcpp::get_logger("optimizer"), "Path has less than 2 points, cannot optimize");
    return false;
  }
  frame_ = path.header.frame_id;
  path_time_ = rclcpp::Time(path.header.stamp);

  // Create costmap grid
  costmap_grid_ = std::make_shared<ceres::Grid2D<u_char>>(costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0,
                                                          costmap->getSizeInCellsX());
  // create the bi-cubic interpolator for the costmap, to be used in the obstacles critic
  auto costmap_interpolator = std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(*costmap_grid_);
  // this should keep the previous optimized path and commands
  // in order to use them as a starting point for the optimization
  // i do not know if this works as expected, but it should
  auto& memory = TrajectoryMemory::getInstance();

  // if no previous path is set, set the current path and commands as the previous ones
  if (memory.previous_path.poses.size() == 0)
  {
    memory.previous_path = path;
    memory.previous_cmds = cmds;
    // Set initial commands
    // memory.is_initialized = true;
  }

  nav_msgs::msg::Path previous_path = memory.previous_path;
  std::vector<geometry_msgs::msg::TwistStamped> previous_cmds = memory.previous_cmds;

  // use the projected path to make it into the a parametrized format
  AgentsStates optim_status = format_to_optimize(path, previous_path, cmds, previous_cmds, speed, current_path_w,
                                                 current_cmds_w, max_time, time_step);
  // use the parametrized format to project the people in the field of view of the robot, using the obstacles and the
  // initial status of the people
  people_proj = project_people(init_people, optim_status, obstacles, max_time, time_step);

  // get different parameters from the initial status
  // and create the evolving poses, positions, headings and velocities
  std::vector<geometry_msgs::msg::PoseStamped> evolving_poses;
  std::vector<position> optim_positions;
  std::vector<heading> optim_headings;
  std::vector<vel> optim_velocities;
  std::vector<linear_velocity> optim_linear_velocities;
  std::vector<angular_velocity> optim_angular_velocities;

  for (auto a : optim_status)
  {
    position p;
    p.params[0] = a[0];  // x
    p.params[1] = a[1];  // y
    vel v;
    v.params[0] = a[4];  // lv
    v.params[1] = a[5];  // av
    linear_velocity lv;
    lv.params[0] = a[4];  // lv
    angular_velocity av;
    av.params[0] = a[5];  // av
    heading h;
    h.params[0] = a[3];  // t
    h.params[1] = a[2];  // yaw
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = a[0];
    pose.pose.position.y = a[1];
    pose.pose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, a[2]);  // yaw
    pose.pose.orientation = tf2::toMsg(quaternion);
    optim_positions.push_back(p);
    optim_velocities.push_back(v);
    optim_headings.push_back(h);
    optim_linear_velocities.push_back(lv);
    optim_angular_velocities.push_back(av);
    evolving_poses.push_back(pose);
  }
  Eigen::Matrix<double, 2, 1> final_trajectorized_point(optim_positions[optim_status.size() - 1].params[0],
                                                        optim_positions[optim_status.size() - 1].params[1]);

  optim_velocities.pop_back();
  double desired_linear_vel_ = 0.6;

  // setting ceres variables
  ceres::Problem problem;
  ceres::Solver::Summary summary;

  // set the value of control horizon, if size of velociteies is smaller than control horizon, set it to the size of
  // velocities also set the block length, if it is larger than the control horizon, set it to the control horizon
  double counter = 0.0;
  std::vector<double*> parameter_blocks;
  unsigned int control_horizon = std::min(control_horizon_, static_cast<unsigned int>(optim_velocities.size()));
  unsigned int block_length = std::min(parameter_block_length_, control_horizon);
  // start of optimization problem construction
  for (unsigned int i = 0; i < optim_velocities.size(); i++)  // i is the index of the current time step
  {
    counter = counter + 1.0;
    unsigned int block_used = i / block_length;

    // add the velocities to optimize
    if (i < control_horizon &&
        (parameter_blocks.empty() || parameter_blocks.back() != optim_velocities[block_used].params))
    {
      parameter_blocks.push_back(optim_velocities[block_used].params);
    }
    double counter_step = counter * time_step;
    if (people.people.size() != 0)
    {
      auto* social_work_function_f = SocialWorkCost::Create(socialwork_w_, people_proj[i + 1], evolving_poses[0].pose,
                                                            counter_step, i, time_step, control_horizon, block_length);
      auto* agent_angle_function_f = AgentAngleCost::Create(agent_angle_w_, people_proj[i + 1], evolving_poses[0].pose,
                                                            i, time_step, control_horizon, block_length);
      auto* proxemics_function_f = ProxemicsCost::Create(proxemics_w_, people_proj[i + 1], evolving_poses[0].pose,
                                                         counter_step, i, time_step, control_horizon, block_length);
      if (i < control_horizon)
      {
        for (unsigned int j = 0; j <= i / block_length; j++)
        {
          agent_angle_function_f->AddParameterBlock(2);
          social_work_function_f->AddParameterBlock(2);  // Each velocity block has 2 params (v, ω)
          proxemics_function_f->AddParameterBlock(2);    // Each velocity block has 2 params (v, ω)
        }
      }
      else
      {
        for (unsigned int j = 0; j <= (control_horizon - 1) / block_length; j++)
        {
          agent_angle_function_f->AddParameterBlock(2);
          social_work_function_f->AddParameterBlock(2);  // Each velocity block has 2 params (v, ω)
          proxemics_function_f->AddParameterBlock(2);    // Each velocity block has 2 params (v, ω)
        }
      }
      agent_angle_function_f->SetNumResiduals(1);
      social_work_function_f->SetNumResiduals(1);
      proxemics_function_f->SetNumResiduals(1);
      problem.AddResidualBlock(agent_angle_function_f, NULL, parameter_blocks);
      problem.AddResidualBlock(social_work_function_f, NULL, parameter_blocks);
      problem.AddResidualBlock(proxemics_function_f, NULL, parameter_blocks);
    }
    auto* velocity_function_f =
        VelocityCost::Create(velocity_w_, desired_linear_vel_, i, control_horizon, block_length);
    Eigen::Matrix<double, 2, 1> final_heading(optim_headings.back().params[0], optim_headings.back().params[1]);
    auto* goal_align_cost_function_f = GoalAlignCost::Create(goal_align_w_, final_heading, evolving_poses[0].pose, i,
                                                             time_step, control_horizon, block_length);
    if (i < control_horizon)
    {
      for (unsigned int j = 0; j <= i / block_length; j++)
      {
        // Each velocity block has 2 params (v, ω)
        velocity_function_f->AddParameterBlock(2);
        goal_align_cost_function_f->AddParameterBlock(2);
      }
    }
    else
    {
      for (unsigned int j = 0; j <= (control_horizon - 1) / block_length; j++)
      {
        // Each velocity block has 2 params (v, ω)
        velocity_function_f->AddParameterBlock(2);
        goal_align_cost_function_f->AddParameterBlock(2);
      }
    }

    velocity_function_f->SetNumResiduals(1);
    goal_align_cost_function_f->SetNumResiduals(1);

    problem.AddResidualBlock(velocity_function_f, NULL, parameter_blocks);
    problem.AddResidualBlock(goal_align_cost_function_f, NULL, parameter_blocks);

    // add the positions to optimize
    Eigen::Matrix<double, 2, 1> point(optim_positions[i + 1].params[0], optim_positions[i + 1].params[1]);

    // add the cost functions for the path following and alignment
    auto* path_follow_cost_function_f = DistanceCost::Create(
        distance_w_, final_trajectorized_point, evolving_poses[0].pose, i, time_step, control_horizon, block_length);
    // add the angle cost function, which is used to align the robot with the path
    auto* path_align_cost_function_f =
        DistanceCost::Create(angle_w_, point, evolving_poses[0].pose, i, time_step, control_horizon, block_length);

    // add the obstacle cost function, which is used to avoid obstacles
    // the obstacle cost function is used to avoid obstacles, it takes the costmap and the interpolator as parameters
    auto* obs_cost_function_f = ObstacleCost::Create(obstacle_w_, costmap, costmap_interpolator, evolving_poses[0].pose,
                                                     i, time_step, control_horizon, block_length);
    if (i < control_horizon)
    {
      for (unsigned int j = 0; j <= i / block_length; j++)
      {
        path_follow_cost_function_f->AddParameterBlock(2);  // Each velocity block has 2 params (v, ω)
        path_align_cost_function_f->AddParameterBlock(2);   // Each velocity block has 2 params (v, ω)
        obs_cost_function_f->AddParameterBlock(2);
      }
    }
    else
    {
      for (unsigned int j = 0; j <= (control_horizon - 1) / block_length; j++)
      {
        path_follow_cost_function_f->AddParameterBlock(2);  // Each velocity block has 2 params (v, ω)
        path_align_cost_function_f->AddParameterBlock(2);   // Each velocity block has 2 params (v, ω)
        obs_cost_function_f->AddParameterBlock(2);
      }
    }
    path_follow_cost_function_f->SetNumResiduals(1);
    path_align_cost_function_f->SetNumResiduals(1);
    obs_cost_function_f->SetNumResiduals(1);
    problem.AddResidualBlock(path_follow_cost_function_f, NULL, parameter_blocks);
    problem.AddResidualBlock(path_align_cost_function_f, NULL, parameter_blocks);
    problem.AddResidualBlock(obs_cost_function_f, NULL, parameter_blocks);
    if (i != 0 && i < control_horizon / block_length)
    {
      auto* velocity_feasibility_cost_function_f =
          VelocityFeasibilityCost::Create(velocity_feasibility_w_, i, control_horizon);
      problem.AddResidualBlock(velocity_feasibility_cost_function_f, NULL, optim_velocities[i].params,
                               optim_velocities[i - 1].params);
    }
  }

  for (unsigned int i = 0; i < control_horizon / block_length; i++)
  {
    problem.SetParameterLowerBound(optim_velocities[i].params, 0, 0.0);   // lower bound for linear velocity
    problem.SetParameterUpperBound(optim_velocities[i].params, 0, 0.6);   // upper bound for linear velocity
    problem.SetParameterLowerBound(optim_velocities[i].params, 1, -1.4);  // lower bound for angular velocity
    problem.SetParameterUpperBound(optim_velocities[i].params, 1, 1.4);   // upper bound for angular velocity
  }

  ceres::Solve(options_, &problem, &summary);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("optimizer"), "Brief report: " << summary.BriefReport() << std::endl);

  if (!summary.IsSolutionUsable())
  {
    RCLCPP_ERROR(rclcpp::get_logger("optimizer"), "Optimization failed!!!");
    return false;
  }

  for (unsigned int i = control_horizon / block_length; i < optim_velocities.size(); i++)
  {
    optim_velocities[i].params[0] = optim_velocities[(control_horizon - 1) / block_length].params[0];
    optim_velocities[i].params[1] = optim_velocities[(control_horizon - 1) / block_length].params[1];
  }
  std::vector<vel> saving_velocities;
  for (unsigned int i = 0; i < control_horizon; i++)
  {
    vel v;
    unsigned int block_idx = i / (block_length);
    v.params[0] = optim_velocities[block_idx].params[0];
    v.params[1] = optim_velocities[block_idx].params[1];
    saving_velocities.push_back(v);
  }
  for (unsigned int i = control_horizon; i < (optim_velocities.size() + 1); i++)
  {
    vel v;
    unsigned int block_idx = (i - 1);
    v.params[0] = optim_velocities[block_idx].params[0];
    v.params[1] = optim_velocities[block_idx].params[1];
    saving_velocities.push_back(v);
  }
  cmds.resize(saving_velocities.size());
  for (unsigned int i = 0; i < saving_velocities.size(); i++)
  {
    cmds[i].header = path.header;
    cmds[i].twist.linear.x = saving_velocities[i].params[0];
    cmds[i].twist.linear.y = 0.0;
    cmds[i].twist.angular.z = saving_velocities[i].params[1];
  }
  path.poses.clear();
  geometry_msgs::msg::PoseStamped pose_old;
  geometry_msgs::msg::PoseStamped previous_pose_old;
  pose_old.header = path.header;
  pose_old.pose.position.x = evolving_poses[0].pose.position.x;
  pose_old.pose.position.y = evolving_poses[0].pose.position.y;
  pose_old.pose.orientation = evolving_poses[0].pose.orientation;
  previous_pose_old.pose.position.x = evolving_poses[0].pose.position.x;
  previous_pose_old.pose.position.y = evolving_poses[0].pose.position.y;
  previous_pose_old.pose.orientation = evolving_poses[0].pose.orientation;

  for (auto vel : saving_velocities)
  {
    pose_old.pose.position.x = previous_pose_old.pose.position.x +
                               vel.params[0] * cos(tf2::getYaw(previous_pose_old.pose.orientation)) * time_step;
    pose_old.pose.position.y = previous_pose_old.pose.position.y +
                               vel.params[0] * sin(tf2::getYaw(previous_pose_old.pose.orientation)) * time_step;
    tf2::Quaternion uao;
    uao.setRPY(0, 0, tf2::getYaw(previous_pose_old.pose.orientation) + vel.params[1] * time_step);
    pose_old.pose.orientation = tf2::toMsg(uao);

    previous_pose_old.pose.position.x = pose_old.pose.position.x;
    previous_pose_old.pose.position.y = pose_old.pose.position.y;
    previous_pose_old.pose.orientation = pose_old.pose.orientation;

    path.poses.push_back(pose_old);
  }

  memory.previous_path = path;
  memory.previous_cmds = cmds;

  return true;
}

AgentsStates Optimizer::people_to_status(const people_msgs::msg::People& people)
{
  AgentsStates people_status;
  // the agents status contain 5 values:
  // x, y, yaw, timestamp, lv, av
  for (auto p : people.people)
  {
    double yaw = atan2(p.velocity.y, p.velocity.x);
    double lv = sqrt(p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y);
    AgentStatus st;
    st << (double)p.position.x, (double)p.position.y, yaw, 0.0, lv, (double)p.velocity.z;
    people_status.push_back(st);
  }
  // we add agents if needed
  while ((int)people_status.size() < 3)
  {
    AgentStatus st;
    // we fill with invalid agent: time=-1
    st << 0.0, 0.0, 0.0, -1.0, 0.0, 0.0;
    people_status.push_back(st);
  }
  // we remove agents if needed
  while ((int)people_status.size() > 3)
  {
    people_status.pop_back();
  }

  return people_status;
}

AgentTrajectory Optimizer::format_to_optimize(nav_msgs::msg::Path& path, const nav_msgs::msg::Path& previous_path,
                                              const std::vector<geometry_msgs::msg::TwistStamped>& cmds,
                                              const std::vector<geometry_msgs::msg::TwistStamped>& previous_cmds,
                                              const geometry_msgs::msg::Twist& speed, const float current_path_w,
                                              const float current_cmds_w, const float maxtime, const float timestep)
{
  // we check the timestep and the path size in order to cut the path
  // to a maximum duration.
  int maxsize = (int)round(maxtime / timestep);
  if ((int)path.poses.size() > maxsize)
  {
    std::vector<geometry_msgs::msg::PoseStamped> p(path.poses.begin(), (path.poses.begin() + (maxsize - 1)));
    path.poses = p;
  }

  AgentTrajectory robot_status;
  for (unsigned int i = 0; i < path.poses.size(); i++)
  {
    // double alpha = 1.0;  // weight for current path; (1-alpha) weight for previous pose
    //  Robot
    //  x, y, yaw, t, lv, av
    AgentStatus r;
    if (!previous_path.poses.empty() && i < previous_path.poses.size())
    {
      // blend current and previous poses for smoother transition
      geometry_msgs::msg::Pose smoothed;
      smoothed.position.x = current_path_w * path.poses[i].pose.position.x +
                            (1.0 - current_path_w) * previous_path.poses[i].pose.position.x;
      smoothed.position.y = current_path_w * path.poses[i].pose.position.y +
                            (1.0 - current_path_w) * previous_path.poses[i].pose.position.y;
      double yaw_current = tf2::getYaw(path.poses[i].pose.orientation);
      double yaw_prev = tf2::getYaw(previous_path.poses[i].pose.orientation);
      double smoothed_yaw = current_path_w * yaw_current + (1.0 - current_path_w) * yaw_prev;
      tf2::Quaternion q;
      q.setRPY(0, 0, smoothed_yaw);
      smoothed.orientation = tf2::toMsg(q);
      // update the current pose with the smoothed pose
      path.poses[i].pose = smoothed;
    }
    r(0, 0) = path.poses[i].pose.position.x;
    r(1, 0) = path.poses[i].pose.position.y;
    r(2, 0) = tf2::getYaw(path.poses[i].pose.orientation);
    r(3, 0) = i * timestep;

    // t += timestep;

    if (i == 0)
    {
      // Robot vel
      r(4, 0) = speed.linear.x;
      r(5, 0) = speed.angular.z;
    }
    else
    {
      geometry_msgs::msg::TwistStamped cmd_smoothed;
      cmd_smoothed.twist.linear.x =
          current_cmds_w * cmds[i - 1].twist.linear.x + (1.0 - current_cmds_w) * previous_cmds[i - 1].twist.linear.x;
      cmd_smoothed.twist.angular.z =
          current_cmds_w * cmds[i - 1].twist.angular.z + (1.0 - current_cmds_w) * previous_cmds[i - 1].twist.angular.z;
      // cmds[i-1] = cmd_smoothed;
      //  Robot vel
      r(4, 0) = cmd_smoothed.twist.linear.x;
      r(5, 0) = cmd_smoothed.twist.angular.z;
    }
    robot_status.push_back(r);
  }
  return robot_status;
}

// we project the people state for each time step of the robot path
AgentsTrajectories Optimizer::project_people(const AgentsStates& init_people, const AgentTrajectory& robot_path,
                                             const obstacle_distance_msgs::msg::ObstacleDistance& od,
                                             const float& maxtime, const float& timestep)
{
  float naive_goal_time = maxtime;  // secs
  // double people_desired_vel = 1.0;
  AgentsTrajectories people_traj;
  people_traj.push_back(init_people);

  // I NEED TO ADD THE CLOSER OBSTACLE POSITION TO EACH AGENT
  // FOR EACH STEP. THAT OBSTACLE POSITION MUST BE IN THE
  // SAME COORDINATE FRAME THAT THE AGENT POSITION.

  std::vector<sfm_controller::Agent> agents;

  // transform people to sfm agents
  for (unsigned int i = 0; i < init_people.size(); i++)
  {
    // if person not valid, skip it
    if (init_people[i][3] == -1)
      continue;

    sfm_controller::Agent a;
    a.id = i + 1;
    a.position << init_people[i][0], init_people[i][1];
    a.yaw = init_people[i][2];
    a.linearVelocity = init_people[i][4];
    a.angularVelocity = init_people[i][5];

    a.velocity << a.linearVelocity * cos(a.yaw), a.linearVelocity * sin(a.yaw);
    a.desiredVelocity = 0.5;  // people_desired_vel; // could be
                              // computed somehow???
    a.radius = 0.5;
    // compute goal with the Constant Velocity Model
    sfm_controller::Goal g;
    g.radius = 0.25;
    Eigen::Vector2d gpos = a.position + naive_goal_time * a.velocity;
    g.center = gpos;
    a.goals.push_back(g);
    // Fill the obstacles

    // check if the obstacle distance message is valid
    // if the map has 100x100 cells
    // TODO use the costmap to compute the obstacles
    if (od.info.width == 100 && od.info.height == 100)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("optimizer"),
                         "ObstacleDistance grid is NOT  valid with size: " << od.info.width << "x" << od.info.height);
      continue;
    }

    a.obstacles1.push_back(computeObstacle(a.position, od));
    agents.push_back(a);
  }

  // compute for each robot state of the path
  for (unsigned int i = 0; i < robot_path.size() - 1; i++)
  {
    // robot as sfm agent
    sfm_controller::Agent sfmrobot;
    sfmrobot.desiredVelocity = 0.6;
    sfmrobot.radius = 0.5;
    sfmrobot.id = 0;
    sfmrobot.position << robot_path[i][0], robot_path[i][1];
    sfmrobot.yaw = robot_path[i][2];
    sfmrobot.linearVelocity = robot_path[i][4];
    sfmrobot.angularVelocity = robot_path[i][5];
    // vx = linearVelocity * cos(yaw), vy = linearVelocity * sin(yaw)
    sfmrobot.velocity << sfmrobot.linearVelocity * cos(sfmrobot.yaw), sfmrobot.linearVelocity * sin(sfmrobot.yaw);
    sfm_controller::Goal g;
    g.radius = 0.25;
    Eigen::Vector2d gpos(robot_path.back()[0], robot_path.back()[1]);
    g.center = gpos;
    sfmrobot.goals.push_back(g);

    // add the robot to the agents
    agents.push_back(sfmrobot);

    // Compute Social Forces
    sfm_controller::SFM.computeForces(agents);
    // Project the people movement according to the SFM
    sfm_controller::SFM.updatePosition(agents, timestep);

    // remove the robot (last agent)
    agents.pop_back();

    // update agents obstacles
    for (unsigned int j = 0; j < agents.size(); j++)
    {
      agents[j].obstacles1.clear();
      agents[j].obstacles1.push_back(computeObstacle(agents[j].position, od));
    }

    // Take the people agents
    AgentsStates humans;
    for (auto p : agents)
    {
      AgentStatus as;
      as(0, 0) = p.position[0];
      as(1, 0) = p.position[1];
      as(2, 0) = p.yaw;
      as(3, 0) = (i + 1) * timestep;
      as(4, 0) = p.linearVelocity;
      as(5, 0) = p.angularVelocity;
      humans.push_back(as);
    }
    // fill with empty agents if needed
    while (humans.size() < init_people.size())
    {
      AgentStatus ag;
      ag.setZero();
      ag(3, 0) = -1.0;
      humans.push_back(ag);
    }
    people_traj.push_back(humans);
  }
  return people_traj;
}

Eigen::Vector2d Optimizer::computeObstacle(const Eigen::Vector2d& apos,
                                           const obstacle_distance_msgs::msg::ObstacleDistance& od)
{
  if (od.distances.empty() || od.indexes.empty())
  {
    throw std::runtime_error("ObstacleDistance grid is empty");
  }
  if (od.info.width <= 0 || od.info.height <= 0)
  {
    throw std::runtime_error("ObstacleDistance grid has invalid size");
  }
  if (od.info.resolution <= 0.0)
  {
    throw std::runtime_error("ObstacleDistance grid has invalid resolution");
  }

  // map point (person) to cell in the distance grid
  unsigned int xcell = (unsigned int)floor((apos[0] - od.info.origin.position.x) / od.info.resolution);
  unsigned int ycell = (unsigned int)floor((apos[1] - od.info.origin.position.y) / od.info.resolution);
  // cell to index of the array

  if (xcell >= (unsigned int)od.info.width || ycell >= (unsigned int)od.info.height)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("optimizer"), "ObstacleDistance grid cell out of bounds: xcell="
                                                             << xcell << ", ycell=" << ycell << ", width="
                                                             << od.info.width << ", height=" << od.info.height);
    throw std::runtime_error("ObstacleDistance grid cell out of bounds");
  }

  unsigned int index = xcell + ycell * od.info.width;

  float dist = od.distances[index];  // not used
  unsigned int ob_idx = od.indexes[index];

  if (ob_idx >= od.info.width * od.info.height)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("optimizer"),
                        "ObstacleDistance grid index out of bounds: ob_idx=" << ob_idx << ", width=" << od.info.width
                                                                             << ", height=" << od.info.height);
    throw std::runtime_error("ObstacleDistance grid index out of bounds");
  }
  // const div_t result = div(ob_idx, (int)od.info.width);
  ycell = floor(ob_idx / od.info.width);
  xcell = ob_idx % od.info.width;

  // cell to world point (obstacle)
  float x = xcell * od.info.resolution + od.info.origin.position.x;
  float y = ycell * od.info.resolution + od.info.origin.position.y;
  Eigen::Vector2d obstacle(x, y);

  // vector between person and obstacle
  Eigen::Vector2d diff = apos - obstacle;

  RCLCPP_DEBUG(rclcpp::get_logger("optimizer"), "Obstacle at (%f, %f) with distance %f", x, y, dist);
  return diff;
}

}  // namespace nav2_social_mpc_controller