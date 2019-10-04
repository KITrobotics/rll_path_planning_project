/*
 * This file is part of the Robot Learning Lab Path Planning Project
 *
 * Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
 * Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <planning_iface.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometric_shapes/shape_operations.h>
#include <visualization_msgs/Marker.h>

PlanningIfaceBase::PlanningIfaceBase(ros::NodeHandle nh, const std::string& srv_name)
  : RLLMoveIfaceBase<rll_planning_project::PlanToGoalAction, rll_planning_project::PlanToGoalGoal>(nh, srv_name)
{
  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose box_pose;
  visualization_msgs::Marker marker;
  float grasp_object_dim_x, grasp_object_dim_y, grasp_object_dim_z, start_pos_x, start_pos_y, start_pos_theta,
      goal_pos_x, goal_pos_y, goal_pos_theta;
  bool headless;

  ros::param::get(node_name_ + "/headless", headless);
  ros::param::get(node_name_ + "/start_pos_x", start_pos_x);
  ros::param::get(node_name_ + "/start_pos_y", start_pos_y);
  ros::param::get(node_name_ + "/start_pos_theta", start_pos_theta);
  ros::param::get(node_name_ + "/grasp_object_dim_x", grasp_object_dim_x);
  ros::param::get(node_name_ + "/grasp_object_dim_y", grasp_object_dim_y);
  ros::param::get(node_name_ + "/grasp_object_dim_z", grasp_object_dim_z);
  ros::param::get(node_name_ + "/goal_pos_x", goal_pos_x);
  ros::param::get(node_name_ + "/goal_pos_y", goal_pos_y);
  ros::param::get(node_name_ + "/goal_pos_theta", goal_pos_theta);

  // init start poses
  start_pose_2d.x = start_pos_x;
  start_pose_2d.y = start_pos_y;
  start_pose_2d.theta = start_pos_theta;
  pose2dToPose3d(start_pose_2d, start_pose_above);
  pose2dToPose3d(start_pose_2d, start_pose_grip);
  start_pose_grip.position.z = vert_grip_height;

  // init goal poses
  goal_pose_2d.x = goal_pos_x;
  goal_pose_2d.y = goal_pos_y;
  goal_pose_2d.theta = goal_pos_theta;
  pose2dToPose3d(goal_pose_2d, goal_pose_above);
  pose2dToPose3d(goal_pose_2d, goal_pose_grip);
  goal_pose_grip.position.z = vert_grip_height;

  // add the grasp object as a collision object into the scene
  grasp_object.header.frame_id = manip_move_group_.getPlanningFrame();
  grasp_object.id = "grasp_object";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = grasp_object_dim_x + 0.02;
  primitive.dimensions[1] = grasp_object_dim_y + 0.02;
  primitive.dimensions[2] = grasp_object_dim_z + 0.001;
  box_pose.position.x = start_pose_grip.position.x;
  box_pose.position.y = start_pose_grip.position.y;
  // ensure a vertical safety distance
  box_pose.position.z = primitive.dimensions[2] / 2 - 0.005;
  box_pose.orientation = start_pose_grip.orientation;
  grasp_object.primitives.push_back(primitive);
  grasp_object.primitive_poses.push_back(box_pose);
  grasp_object.operation = grasp_object.ADD;

  // Publish a marker at goal pose
  ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Frame ID and timestamp
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CUBE;
  // Marker pose = grasping object goal pose
  marker.pose = goal_pose_grip;
  marker.pose.position.z = box_pose.position.z;
  // Marker dimensions = grasping object dimensions
  marker.scale.x = primitive.dimensions[0];
  marker.scale.y = primitive.dimensions[1];
  marker.scale.z = primitive.dimensions[2];
  // color red
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (!headless && marker_pub.getNumSubscribers() < 1)
  {
    ROS_INFO_ONCE("Waiting for marker subscribers");
    ros::Duration(1.0).sleep();
  }
  marker_pub.publish(marker);

  planning_scene_interface_.applyCollisionObject(grasp_object);
  // allow these collisions for collision checks in the move iface
  disableCollision(grasp_object.id, "table");
  disableCollision(grasp_object.id, grasp_object.id);
  disableCollision(grasp_object.id, ns_ + "_gripper_finger_left");
  disableCollision(grasp_object.id, ns_ + "_gripper_finger_right");

  grasp_object_at_goal = false;
  allowed_to_plan = false;
}

void PlanningIfaceBase::runJob(const rll_msgs::JobEnvGoalConstPtr& /*goal*/, rll_msgs::JobEnvResult& result)
{
  bool run_three_times = false;
  int num_runs = 1;
  ros::Duration planning_time;
  std::vector<float> planning_times;
  float planning_time_final;
  RLLErrorCode error_code;

  ROS_INFO("got job running request");

  ros::param::get(node_name_ + "/run_three_times", run_three_times);
  if (run_three_times)
  {
    num_runs = 3;
  }

  for (int i = 0; i < num_runs; ++i)
  {
    bool success = runPlannerOnce(result, planning_time);
    if (!success)
    {
      return;
    }

    // idle is called separately after the last run
    if (run_three_times && i < 2)
    {
      error_code = idle();
      if (error_code.failed())
      {
        result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
        return;
      }
    }

    planning_times.push_back(planning_time.toSec());
  }

  if (run_three_times)
  {
    std::sort(planning_times.begin(), planning_times.end());
    planning_time_final = planning_times[1];
  }
  else
  {
    planning_time_final = planning_times[0];
  }

  rll_msgs::JobExtraField job_data;
  job_data.description = "duration";
  job_data.value = planning_time_final;
  result.job_data.push_back(job_data);

  result.job.status = rll_msgs::JobStatus::SUCCESS;
}

bool PlanningIfaceBase::runPlannerOnce(rll_msgs::JobEnvResult& result, ros::Duration& planning_time)
{
  rll_msgs::MovePTP::Request move_ptp_req;
  rll_msgs::MovePTP::Response move_ptp_resp;
  rll_msgs::PickPlace::Request pick_place_req;
  rll_msgs::PickPlace::Response pick_place_resp;

  // reset grasp object position
  bool success = planning_scene_interface_.applyCollisionObject(grasp_object);
  if (!success)
  {
    ROS_WARN("Failed to reset grasp object position");
  }
  else
  {
    ROS_INFO("Reset grasp object pos");
  }

  // pick up the grasp object
  move_ptp_req.pose = start_pose_above;
  move_ptp_req.pose.position.z = pose_z_above_maze;
  RLLErrorCode error_code = movePTP(move_ptp_req, move_ptp_resp);
  if (error_code.failed())
  {
    ROS_FATAL("Moving PTP above grasp object failed");
    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  pick_place_req.pose_above = start_pose_above;
  pick_place_req.pose_grip = start_pose_grip;
  pick_place_req.gripper_close = true;
  pick_place_req.grasp_object = grasp_object.id;
  error_code = pickPlace(pick_place_req, pick_place_resp);
  if (error_code.failed())
  {
    ROS_FATAL("Failed to pick up the grasp object!");
    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  // check_path needs this to have a proper start state
  robot_state::RobotState start_state = getCurrentRobotState(true);
  check_path_start_state = &start_state;

  if (!action_client_ptr_->waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("planning service not available");
    result.job.status = rll_msgs::JobStatus::FAILURE;
    return false;
  }

  rll_planning_project::PlanToGoalGoal plan_req;
  plan_req.start.x = start_pose_2d.x;
  plan_req.start.y = start_pose_2d.y;
  plan_req.start.theta = start_pose_2d.theta;
  plan_req.goal.x = goal_pose_2d.x;
  plan_req.goal.y = goal_pose_2d.y;
  plan_req.goal.theta = goal_pose_2d.theta;
  allowed_to_plan = true;
  ROS_INFO("calling the planning service\n");
  action_client_ptr_->sendGoal(plan_req);

  ros::Time planning_start = ros::Time::now();
  action_client_ptr_->waitForResult(ros::Duration(plan_service_timeout));
  ros::Time planning_end = ros::Time::now();
  planning_time = planning_end - planning_start;

  ROS_INFO("called the planning service, planning and moving took %d minutes and %d seconds",
           int(planning_time.toSec() / 60), int(fmod(planning_time.toSec(), 60)));
  allowed_to_plan = false;

  success = checkGoalState();
  if (!success)
  {
    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  if (!grasp_object_at_goal)
  {
    result.job.status = rll_msgs::JobStatus::FAILURE;
    return false;
  }

  return true;
}

RLLErrorCode PlanningIfaceBase::idle()
{
  rll_msgs::MovePTP::Request move_ptp_req;
  rll_msgs::MovePTP::Response move_ptp_resp;
  rll_msgs::PickPlace::Request pick_place_req;
  rll_msgs::PickPlace::Response pick_place_resp;
  RLLErrorCode error_code;
  ROS_INFO("got idle request");

  if (grasp_object_at_goal)
  {
    // pick up the grasp object
    move_ptp_req.pose = goal_pose_above;
    move_ptp_req.pose.position.z = pose_z_above_maze;
    error_code = movePTP(move_ptp_req, move_ptp_resp);
    if (error_code.failed())
    {
      ROS_ERROR("Moving PTP above goal pos for reset failed");
      return error_code;
    }

    pick_place_req.pose_above = goal_pose_above;
    pick_place_req.pose_grip = goal_pose_grip;
    pick_place_req.gripper_close = true;
    pick_place_req.grasp_object = grasp_object.id;
    error_code = pickPlace(pick_place_req, pick_place_resp);
    if (error_code.failed())
    {
      ROS_ERROR("Failed to pick up the grasp object at the goal!");
      return error_code;
    }

    grasp_object_at_goal = false;
  }

  error_code = resetToStart();
  if (!error_code.failed())
  {
    return error_code;
  }

  ROS_INFO("moved to idle position\n");
  return RLLErrorCode::SUCCESS;
}

RLLErrorCode PlanningIfaceBase::beforeMovementServiceCall(const std::string& srv_name)
{
  RLLErrorCode error_code = RLLMoveIfaceBase::beforeMovementServiceCall(srv_name);
  if (error_code.failed())
  {
    return error_code;
  }

  if (!allowed_to_plan)
  {
    ROS_WARN("Not allowed to plan and therefore not allowed to send move commands");
    return RLLErrorCode::SERVICE_CALL_NOT_ALLOWED;
  }

  return error_code;
}

void PlanningIfaceBase::generateRotationWaypoints(const geometry_msgs::Pose2D& pose2d_start, float rot_step_size,
                                                  std::vector<geometry_msgs::Pose>& waypoints)
{
  geometry_msgs::Pose pose3d_rot;
  geometry_msgs::Pose2D pose2d_rot = pose2d_start;
  for (int i = 0; i < 9; ++i)
  {
    pose2d_rot.theta += rot_step_size;
    pose2dToPose3d(pose2d_rot, pose3d_rot);
    waypoints.push_back(pose3d_rot);
  }
}

bool PlanningIfaceBase::moveSrv(rll_planning_project::Move::Request& req, rll_planning_project::Move::Response& resp)
{
  geometry_msgs::Pose pose3d_goal;
  geometry_msgs::Pose2D pose2d_cur;
  std::vector<geometry_msgs::Pose> waypoints;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  float move_dist, dist_rot;
  const double EEF_STEP = 0.0005;
  const double JUMP_THRESHOLD = 4.5;
  const std::string SRV_NAME = "move";

  RLLErrorCode error_code = beforeMovementServiceCall(SRV_NAME);
  if (error_code.failed())
  {
    resp.success = false;
    return true;
  }

  // enforce a lower bound for move command to ensure that Moveit has enough waypoints
  diffCurrentState(req.pose, move_dist, dist_rot, pose2d_cur);
  if ((move_dist < 0.005 && move_dist > 0.0005) || (move_dist < 0.0005 && dist_rot < 20 * M_PI / 180))
  {
    ROS_WARN("move commands that cover a distance between 0 and 5 mm or sole rotations less than 20 degrees are not "
             "supported!");
    resp.success = false;
    return true;
  }

  if (move_dist < 0.0005)
  {
    // generate more rotation points here to ensure that there are at least ten
    // for the continuity check
    float rot_step_size = (pose2d_cur.theta - req.pose.theta) / 10;
    generateRotationWaypoints(pose2d_cur, rot_step_size, waypoints);
  }

  ROS_INFO("move request to pos x=%.3f y=%.3f theta=%.3f", req.pose.x, req.pose.y, req.pose.theta);
  manip_move_group_.setStartStateToCurrentState();
  pose2dToPose3d(req.pose, pose3d_goal);
  waypoints.push_back(pose3d_goal);
  double achieved = manip_move_group_.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);
  if (achieved < 1)
  {
    ROS_WARN("aborting, only achieved to compute %f of the requested move path", achieved);
    abortDueToCriticalFailure();
    resp.success = false;
    return true;
  }

  if (trajectory.joint_trajectory.points.size() < 10)
  {
    ROS_ERROR("trajectory has not enough points to check for continuity, only got %lu",
              trajectory.joint_trajectory.points.size());
    ROS_ERROR("move dist %f, dist rot %f", move_dist, dist_rot);
    ROS_WARN("aborting...");
    abortDueToCriticalFailure();
    resp.success = false;
    return true;
  }

  bool success = modifyPtpTrajectory(trajectory);
  if (!success)
  {
    ROS_ERROR("move service: time parametrization failed");
    abortDueToCriticalFailure();
    resp.success = false;
    return true;
  }

  my_plan.trajectory_ = trajectory;
  success = (manip_move_group_.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_ERROR("path execution failed, aborting");
    abortDueToCriticalFailure();
    resp.success = false;
    return true;
  }

  error_code = afterMovementServiceCall(SRV_NAME, error_code);

  resp.success = error_code.succeeded();
  return true;
}

void PlanningIfaceBase::abortDueToCriticalFailure()
{
  RLLMoveIfaceBase::abortDueToCriticalFailure();
  allowed_to_plan = false;
}

bool PlanningIfaceBase::checkPathSrv(rll_planning_project::CheckPath::Request& req,
                                     rll_planning_project::CheckPath::Response& resp)
{
  geometry_msgs::Pose pose3d_start, pose3d_goal;
  std::vector<geometry_msgs::Pose> waypoints;
  moveit_msgs::RobotTrajectory trajectory;
  const double EEF_STEP = 0.0005;
  const double JUMP_THRESHOLD = 4.5;

  if (!allowed_to_plan)
  {
    ROS_WARN("Not allowed to request path checks");
    return true;
  }

  // enforce a lower bound for check_path requests to ensure that Moveit has enough waypoints
  float move_dist = sqrt(pow(req.pose_start.x - req.pose_goal.x, 2) + pow(req.pose_start.y - req.pose_goal.y, 2));
  float dist_rot = fabs(req.pose_start.theta - req.pose_goal.theta);
  if ((move_dist < 0.005 && move_dist != 0) || (move_dist == 0 && dist_rot < 20 * M_PI / 180))
  {
    ROS_WARN("check path requests that cover a distance between 0 and 5 mm or sole rotations less than 20 degrees are "
             "not supported!");
    ROS_WARN("Moving distance would have been %f m", move_dist);
    resp.valid = false;
    return true;
  }

  if (move_dist == 0)
  {
    // generate more points here to ensure that there are at least ten
    // for the continuity check
    float rot_step_size = (req.pose_start.theta - req.pose_goal.theta) / 10;
    generateRotationWaypoints(req.pose_start, rot_step_size, waypoints);
  }

  pose2dToPose3d(req.pose_start, pose3d_start);
  pose2dToPose3d(req.pose_goal, pose3d_goal);

  bool start_pose_valid =
      check_path_start_state->setFromIK(manip_joint_model_group_, pose3d_start, manip_move_group_.getEndEffectorLink());

  if (!start_pose_valid)
  {
    // ROS_WARN("path check: start pose not valid");
    resp.valid = false;
    return true;
  }

  manip_move_group_.setStartState(*check_path_start_state);

  waypoints.push_back(pose3d_goal);
  double achieved = manip_move_group_.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);
  if (achieved < 1)
  {
    // ROS_WARN("only achieved to compute %f of the requested path", achieved);
    resp.valid = false;
    return true;
  }
  resp.valid = true;

  if (trajectory.joint_trajectory.points.size() < 10)
  {
    ROS_ERROR("trajectory has not enough points to check for continuity, only got %lu",
              trajectory.joint_trajectory.points.size());
    ROS_ERROR("move dist %f, dist rot %f", move_dist, dist_rot);
    resp.valid = false;
  }

  return true;
}

bool PlanningIfaceBase::checkGoalState()
{
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose2D pose2d_cur;
  moveit_msgs::RobotTrajectory trajectory;
  const double EEF_STEP = 0.0001;
  const double JUMP_THRESHOLD = 4.5;
  rll_msgs::PickPlace::Request pick_place_req;
  rll_msgs::PickPlace::Response pick_place_resp;
  rll_msgs::MoveLin::Request move_lin_req;
  rll_msgs::MoveLin::Response move_lin_resp;
  float dist_goal_trans, diff_goal_angle;
  RLLErrorCode error_code;

  // check if we are close enough to the goal
  diffCurrentState(goal_pose_2d, dist_goal_trans, diff_goal_angle, pose2d_cur);

  if (dist_goal_trans < goal_tolerance_trans && diff_goal_angle < goal_tolerance_rot)
  {
    ROS_INFO("distance to goal: %f, orientation diff: %f", dist_goal_trans, diff_goal_angle);

    geometry_msgs::Pose reset_above_goal = goal_pose_above;
    reset_above_goal.position.z = pose_z_above_maze;
    waypoints.push_back(reset_above_goal);
    double achieved = manip_move_group_.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);
    if (achieved < 1)
    {
      ROS_WARN("You came close to the goal, but unfortunately, it's not reachable.\n");
      grasp_object_at_goal = false;
      return true;
    }

    ROS_INFO("Goal was reached, good job!\n");

    // place the object at the goal
    pick_place_req.pose_above = goal_pose_above;
    pick_place_req.pose_grip = goal_pose_grip;
    pick_place_req.gripper_close = false;
    pick_place_req.grasp_object = grasp_object.id;

    error_code = pickPlace(pick_place_req, pick_place_resp);
    if (error_code.failed())
    {
      ROS_FATAL("Failed to place the grasp object at the goal!");
      return false;
    }

    grasp_object_at_goal = true;

    // move a little higher after placing the object
    move_lin_req.pose = reset_above_goal;
    moveLin(move_lin_req, move_lin_resp);
    if (error_code.failed())
    {
      ROS_FATAL("Moving above maze for reset to home failed");
      return false;
    }

    error_code = resetToHome();
    if (error_code.failed())
    {
      ROS_FATAL("Failed to reset to home from goal");
      return false;
    }
  }
  else
  {
    ROS_WARN("Goal was not reached");
    ROS_WARN("distance to goal: %f, orientation diff: %f\n", dist_goal_trans, diff_goal_angle);
    grasp_object_at_goal = false;
  }

  return true;
}

void PlanningIfaceBase::diffCurrentState(const geometry_msgs::Pose2D pose_des, float& diff_trans, float& diff_rot,
                                         geometry_msgs::Pose2D& pose2d_cur)
{
  geometry_msgs::Pose current_pose = manip_move_group_.getCurrentPose().pose;

  diff_trans = sqrt(pow(current_pose.position.x - pose_des.x, 2) + pow(current_pose.position.y - pose_des.y, 2));

  tf::Quaternion q_current, q_orig, q_rot;
  // orientation of eef in home position
  q_orig[0] = q_orig[2] = q_orig[3] = 0;
  q_orig[1] = 1;
  quaternionMsgToTF(current_pose.orientation, q_current);
  q_rot = q_current * q_orig;
  tf::Matrix3x3 m(q_rot);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  if (yaw <= -0.001)
  {
    yaw += 2 * M_PI;
  }
  // almost 360°
  if (yaw > 6.26)
  {
    yaw = 0;
  }
  diff_rot = fabs(yaw - pose_des.theta);

  pose2d_cur.x = current_pose.position.x;
  pose2d_cur.y = current_pose.position.y;
  pose2d_cur.theta = yaw;
}

RLLErrorCode PlanningIfaceBase::resetToStart()
{
  rll_msgs::MoveLin::Request move_lin_req;
  rll_msgs::MoveLin::Response move_lin_resp;
  rll_msgs::MovePTP::Request move_ptp_req;
  rll_msgs::MovePTP::Response move_ptp_resp;
  rll_msgs::PickPlace::Request pick_place_req;
  rll_msgs::PickPlace::Response pick_place_resp;
  geometry_msgs::Pose current_pose = manip_move_group_.getCurrentPose().pose;

  // set this a little higher to make sure we are moving above the maze
  current_pose.position.z = pose_z_above_maze;
  move_lin_req.pose = current_pose;
  RLLErrorCode error_code = moveLin(move_lin_req, move_lin_resp);
  if (error_code.failed())
  {
    ROS_FATAL("Moving above maze for reset failed");
    return error_code;
  }

  move_ptp_req.pose = start_pose_above;
  move_ptp_req.pose.position.z = pose_z_above_maze;
  if (!poseGoalTooClose(current_pose, move_ptp_req.pose))
  {
    error_code = movePTP(move_ptp_req, move_ptp_resp);
    if (error_code.failed())
    {
      ROS_FATAL("Moving above start pos for reset failed");
      return error_code;
    }
  }

  pick_place_req.pose_above = start_pose_above;
  pick_place_req.pose_above.position.z = pose_z_above_maze;
  pick_place_req.pose_grip = start_pose_grip;
  pick_place_req.gripper_close = false;
  pick_place_req.grasp_object = grasp_object.id;
  error_code = pickPlace(pick_place_req, pick_place_resp);
  if (error_code.failed())
  {
    ROS_FATAL("Failed to place the grasp object at the start pos");
    return error_code;
  }

  error_code = resetToHome();
  if (error_code.failed())
  {
    ROS_FATAL("Failed to reset to start from start pos");
    return error_code;
  }

  return RLLErrorCode::SUCCESS;
}

void PlanningIfaceBase::pose2dToPose3d(geometry_msgs::Pose2D& pose2d, geometry_msgs::Pose& pose3d)
{
  tf::Quaternion q_orig, q_rot, q_new;

  pose3d.position.x = pose2d.x;
  pose3d.position.y = pose2d.y;
  pose3d.position.z = vert_ground_clearance;

  // orientation of eef in home position
  q_orig[0] = q_orig[2] = q_orig[3] = 0;
  q_orig[1] = 1;

  q_rot = tf::createQuaternionFromRPY(0, 0, pose2d.theta);
  q_new = q_rot * q_orig;
  q_new.normalize();
  quaternionTFToMsg(q_new, pose3d.orientation);
}

void PlanningIfaceBase::startServicesAndRunNode(ros::NodeHandle& nh)
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  PlanningIfaceBase* iface_ptr = this;
  RLLMoveIface* move_iface_ptr = iface_ptr;

  iface_ptr->resetToHome();

  RLLMoveIface::JobServer server_job(nh, RLLMoveIface::RUN_JOB_SRV_NAME,
                                     boost::bind(&RLLMoveIface::runJobAction, move_iface_ptr, _1, &server_job), false);
  server_job.start();
  RLLMoveIface::JobServer server_idle(nh, RLLMoveIface::IDLE_JOB_SRV_NAME,
                                      boost::bind(&RLLMoveIface::idleAction, move_iface_ptr, _1, &server_idle), false);
  server_idle.start();
  ros::ServiceServer move = nh.advertiseService("move", &PlanningIfaceBase::moveSrv, iface_ptr);
  ros::ServiceServer check_path = nh.advertiseService("check_path", &PlanningIfaceBase::checkPathSrv, iface_ptr);
  ros::ServiceServer robot_ready = nh.advertiseService("robot_ready", &RLLMoveIface::robotReadySrv, move_iface_ptr);

  ROS_INFO("RLL Planning Interface started\n");
  ros::waitForShutdown();
}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
