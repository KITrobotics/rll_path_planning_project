/*
 * This file is part of the Robot Learning Lab Path Planning Project
 *
 * Copyright (C) 2018-2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#include <geometric_shapes/shape_operations.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <rll_planning_project/planning_iface.h>

PlanningIfaceBase::PlanningIfaceBase(const ros::NodeHandle& nh) : RLLMoveIfaceBase(nh)
{
  float start_pos_x, start_pos_y, start_pos_theta, goal_pos_x, goal_pos_y, goal_pos_theta;

  ros::param::get(node_name_ + "/start_pos_x", start_pos_x);
  ros::param::get(node_name_ + "/start_pos_y", start_pos_y);
  ros::param::get(node_name_ + "/start_pos_theta", start_pos_theta);
  ros::param::get(node_name_ + "/goal_pos_x", goal_pos_x);
  ros::param::get(node_name_ + "/goal_pos_y", goal_pos_y);
  ros::param::get(node_name_ + "/goal_pos_theta", goal_pos_theta);

  // init start poses
  start_pose_2d_.x = start_pos_x;
  start_pose_2d_.y = start_pos_y;
  start_pose_2d_.theta = start_pos_theta;
  pose2dToPose3d(start_pose_2d_, &start_pose_above_);
  pose2dToPose3d(start_pose_2d_, &start_pose_grip_);
  start_pose_grip_.position.z = VERT_GRIP_HEIGHT;

  // init goal poses
  goal_pose_2d_.x = goal_pos_x;
  goal_pose_2d_.y = goal_pos_y;
  goal_pose_2d_.theta = goal_pos_theta;
  pose2dToPose3d(goal_pose_2d_, &goal_pose_above_);
  pose2dToPose3d(goal_pose_2d_, &goal_pose_grip_);
  goal_pose_grip_.position.z = VERT_GRIP_HEIGHT;

  insertGraspObject();

  grasp_object_at_goal_ = false;
  registerPermissions();
}

void PlanningIfaceBase::insertGraspObject()
{
  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose box_pose;
  visualization_msgs::Marker marker;
  float grasp_object_dim_x, grasp_object_dim_y, grasp_object_dim_z;
  bool headless;

  ros::param::get(node_name_ + "/headless", headless);
  ros::param::get(node_name_ + "/grasp_object_dim_x", grasp_object_dim_x);
  ros::param::get(node_name_ + "/grasp_object_dim_y", grasp_object_dim_y);
  ros::param::get(node_name_ + "/grasp_object_dim_z", grasp_object_dim_z);

  // add the grasp object as a collision object into the scene
  grasp_object_.header.frame_id = manip_move_group_.getPlanningFrame();
  grasp_object_.id = "grasp_object";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = grasp_object_dim_x + 0.02;
  primitive.dimensions[1] = grasp_object_dim_y + 0.02;
  primitive.dimensions[2] = grasp_object_dim_z + 0.001;
  box_pose.position.x = start_pose_grip_.position.x;
  box_pose.position.y = start_pose_grip_.position.y;
  // ensure a vertical safety distance
  box_pose.position.z = primitive.dimensions[2] / 2 - 0.005;
  box_pose.orientation = start_pose_grip_.orientation;
  grasp_object_.primitives.push_back(primitive);
  grasp_object_.primitive_poses.push_back(box_pose);
  grasp_object_.operation = grasp_object_.ADD;

  // Publish a marker at goal pose
  ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Frame ID and timestamp
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CUBE;
  // Marker pose = grasping object goal pose
  marker.pose = goal_pose_grip_;
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

  planning_scene_interface_.applyCollisionObject(grasp_object_);
  // allow these collisions for collision checks in the move iface
  disableCollision(grasp_object_.id, "table");
}

void PlanningIfaceBase::runJob(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult* result)
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
    bool success = runPlannerOnce(goal, result);
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
        result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
        return;
      }
    }

    planning_times.push_back(job_result_.getJobDuration().toSec());
  }

  if (run_three_times)
  {
    // take the average
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
  result->job_data.push_back(job_data);

  result->job.status = rll_msgs::JobStatus::SUCCESS;
}

bool PlanningIfaceBase::runPlannerOnce(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult* result)
{
  rll_msgs::MovePTP::Request move_ptp_req;
  rll_msgs::MovePTP::Response move_ptp_resp;
  rll_msgs::PickPlace::Request pick_place_req;
  rll_msgs::PickPlace::Response pick_place_resp;

  // reset grasp object position
  bool success = planning_scene_interface_.applyCollisionObject(grasp_object_);
  if (!success)
  {
    ROS_WARN("Failed to reset grasp object position");
  }
  else
  {
    ROS_INFO("Reset grasp object pos");
  }

  // pick up the grasp object
  move_ptp_req.pose = start_pose_above_;
  move_ptp_req.pose.position.z = POSE_Z_ABOVE_MAZE;
  RLLErrorCode error_code = movePTP(move_ptp_req, &move_ptp_resp);
  if (error_code.failed())
  {
    ROS_FATAL("Moving PTP above grasp object failed");
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  pick_place_req.pose_above = start_pose_above_;
  pick_place_req.pose_grip = start_pose_grip_;
  pick_place_req.gripper_close = RLL_SRV_TRUE;
  pick_place_req.grasp_object = grasp_object_.id;
  error_code = pickPlace(pick_place_req, &pick_place_resp);
  if (error_code.failed())
  {
    ROS_FATAL("Failed to pick up the grasp object!");
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  // check_path needs this to have a proper start state
  robot_state::RobotState start_state = getCurrentRobotState(true);
  check_path_start_state_ = &start_state;

  permissions_.storeCurrentPermissions();
  permissions_.updateCurrentPermissions(plan_permission_, true);
  ROS_INFO("calling the planning service\n");
  success = runClient(goal, result);
  permissions_.restorePreviousPermissions();

  if (success)
  {
    ROS_INFO("successfully called the planning service, planning and moving took %d minutes and %d seconds",
             int(job_result_.getJobDuration().toSec() / 60), int(fmod(job_result_.getJobDuration().toSec(), 60)));
  }
  else
  {
    ROS_WARN("called the planning service with error");
  }

  success = checkGoalState();
  if (!success)
  {
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  if (!grasp_object_at_goal_)
  {
    result->job.status = rll_msgs::JobStatus::FAILURE;
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

  if (grasp_object_at_goal_)
  {
    // pick up the grasp object
    move_ptp_req.pose = goal_pose_above_;
    move_ptp_req.pose.position.z = POSE_Z_ABOVE_MAZE;
    error_code = movePTP(move_ptp_req, &move_ptp_resp);
    if (error_code.failed())
    {
      ROS_ERROR("Moving PTP above goal pos for reset failed");
      return error_code;
    }

    pick_place_req.pose_above = goal_pose_above_;
    pick_place_req.pose_grip = goal_pose_grip_;
    pick_place_req.gripper_close = RLL_SRV_TRUE;
    pick_place_req.grasp_object = grasp_object_.id;
    error_code = pickPlace(pick_place_req, &pick_place_resp);
    if (error_code.failed())
    {
      ROS_ERROR("Failed to pick up the grasp object at the goal!");
      return error_code;
    }

    grasp_object_at_goal_ = false;
  }

  error_code = resetToStart();
  if (error_code.failed())
  {
    return error_code;
  }

  ROS_INFO("moved to idle position\n");
  return RLLErrorCode::SUCCESS;
}

void PlanningIfaceBase::registerPermissions()
{
  plan_permission_ = permissions_.registerPermission("allowed_to_plan", false);

  permissions_.setDefaultRequiredPermissions(plan_permission_);
  // robot ready service check is always allowed
  permissions_.setRequiredPermissionsFor(RLLMoveIfaceServices::ROBOT_READY_SRV_NAME,
                                         Permissions::NO_PERMISSION_REQUIRED);
}

void PlanningIfaceBase::generateRotationWaypoints(const geometry_msgs::Pose2D& pose2d_start, float rot_step_size,
                                                  std::vector<geometry_msgs::Pose>* waypoints)
{
  geometry_msgs::Pose pose3d_rot;
  geometry_msgs::Pose2D pose2d_rot = pose2d_start;
  for (int i = 0; i < 9; ++i)
  {
    pose2d_rot.theta += rot_step_size;
    pose2dToPose3d(pose2d_rot, &pose3d_rot);
    waypoints->push_back(pose3d_rot);
  }
}

bool PlanningIfaceBase::getStartGoalSrv(rll_planning_project::GetStartGoal::Request& /*req*/,
                                        rll_planning_project::GetStartGoal::Response& resp)
{
  RLLErrorCode error_code = beforeNonMovementServiceCall(GET_START_GOAL_SRV_NAME);

  if (error_code.succeeded())
  {
    resp.start = start_pose_2d_;
    resp.goal = goal_pose_2d_;
  }

  error_code = afterNonMovementServiceCall(GET_START_GOAL_SRV_NAME, error_code);
  resp.error_code = error_code.value();
  resp.success = error_code.succeededSrv();
  return true;
}

bool PlanningIfaceBase::moveSrv(rll_planning_project::Move::Request& req, rll_planning_project::Move::Response& resp)
{
  return controlledMovementExecution(req, &resp, MOVE_SRV_NAME, &PlanningIfaceBase::move);
}

RLLErrorCode PlanningIfaceBase::move(const rll_planning_project::Move::Request& req,
                                     rll_planning_project::Move::Response* /*resp*/)
{
  geometry_msgs::Pose pose3d_goal;
  geometry_msgs::Pose2D pose2d_cur;
  std::vector<geometry_msgs::Pose> waypoints;
  moveit_msgs::RobotTrajectory trajectory;
  float move_dist, dist_rot;

  // enforce a lower bound for move command to ensure that Moveit has enough waypoints
  diffCurrentState(req.pose, &move_dist, &dist_rot, &pose2d_cur);
  if ((move_dist < 0.005 && move_dist > DEFAULT_LINEAR_EEF_STEP) ||
      (move_dist < DEFAULT_LINEAR_EEF_STEP && dist_rot < 20 * M_PI / 180))
  {
    ROS_WARN("move commands that cover a distance between 0 and 5 mm or sole rotations less than 20 degrees are not "
             "supported!");
    return RLLErrorCode::TOO_FEW_WAYPOINTS;
  }

  if (move_dist < DEFAULT_LINEAR_EEF_STEP)
  {
    // generate more rotation points here to ensure that there are at least ten
    // for the continuity check
    float rot_step_size = (pose2d_cur.theta - req.pose.theta) / LINEAR_MIN_STEPS_FOR_JUMP_THRESH;
    generateRotationWaypoints(pose2d_cur, rot_step_size, &waypoints);
  }

  ROS_INFO("move request to pos x=%.3f y=%.3f theta=%.3f", req.pose.x, req.pose.y, req.pose.theta);
  manip_move_group_.setStartStateToCurrentState();
  pose2dToPose3d(req.pose, &pose3d_goal);
  waypoints.push_back(pose3d_goal);
  RLLErrorCode error_code = computeLinearPath(waypoints, &trajectory);
  if (error_code.failed())
  {
    ROS_ERROR("computing path failed, move dist %f, dist rot %f", move_dist, dist_rot);
    return error_code;
  }

  return runLinearTrajectory(trajectory);
}

bool PlanningIfaceBase::checkPathSrv(rll_planning_project::CheckPath::Request& req,
                                     rll_planning_project::CheckPath::Response& resp)
{
  RLLErrorCode error_code = beforeNonMovementServiceCall(CHECK_PATH_SRV_NAME);
  if (error_code.succeeded())
  {
    checkPath(req, &resp);
  }

  error_code = afterNonMovementServiceCall(CHECK_PATH_SRV_NAME, error_code);

  if (error_code.failed())
  {
    ROS_INFO("checkPathSrv call failed with: %s", error_code.message());
    resp.valid = RLL_SRV_FALSE;
  }

  return true;
}

bool PlanningIfaceBase::checkPath(const rll_planning_project::CheckPath::Request& req,
                                  rll_planning_project::CheckPath::Response* resp)
{
  geometry_msgs::Pose pose3d_start, pose3d_goal;
  std::vector<geometry_msgs::Pose> waypoints;
  moveit_msgs::RobotTrajectory trajectory;
  const double POSITIVE_ZERO = 1E-07;

  // enforce a lower bound for check_path requests to ensure that Moveit has enough waypoints
  float move_dist = sqrt(pow(req.pose_start.x - req.pose_goal.x, 2) + pow(req.pose_start.y - req.pose_goal.y, 2));
  float dist_rot = fabs(req.pose_start.theta - req.pose_goal.theta);
  if ((move_dist < 0.005 && move_dist > POSITIVE_ZERO) || (move_dist <= POSITIVE_ZERO && dist_rot < 20 * M_PI / 180))
  {
    ROS_WARN("check path requests that cover a distance between 0 and 5 mm or sole rotations less than 20 degrees are "
             "not supported!");
    ROS_WARN("Moving distance would have been %f m", move_dist);
    resp->valid = RLL_SRV_FALSE;
    return true;
  }

  if (move_dist <= POSITIVE_ZERO)
  {
    // generate more points here to ensure that there are at least ten
    // for the continuity check
    float rot_step_size = (req.pose_start.theta - req.pose_goal.theta) / LINEAR_MIN_STEPS_FOR_JUMP_THRESH;
    generateRotationWaypoints(req.pose_start, rot_step_size, &waypoints);
  }

  pose2dToPose3d(req.pose_start, &pose3d_start);
  pose2dToPose3d(req.pose_goal, &pose3d_goal);

  bool start_pose_valid = check_path_start_state_->setFromIK(manip_joint_model_group_, pose3d_start,
                                                             manip_move_group_.getEndEffectorLink());

  if (!start_pose_valid)
  {
    resp->valid = RLL_SRV_FALSE;
    return true;
  }

  manip_move_group_.setStartState(*check_path_start_state_);

  waypoints.push_back(pose3d_goal);
  double achieved = manip_move_group_.computeCartesianPath(waypoints, DEFAULT_LINEAR_EEF_STEP,
                                                           DEFAULT_LINEAR_JUMP_THRESHOLD, trajectory);
  if (achieved < 1)
  {
    resp->valid = RLL_SRV_FALSE;
    return true;
  }

  if (trajectory.joint_trajectory.points.size() < LINEAR_MIN_STEPS_FOR_JUMP_THRESH)
  {
    ROS_ERROR("trajectory has not enough points to check for continuity, only got %lu",
              trajectory.joint_trajectory.points.size());
    ROS_ERROR("move dist %f, dist rot %f", move_dist, dist_rot);
    resp->valid = RLL_SRV_FALSE;
    return true;
  }

  resp->valid = RLL_SRV_TRUE;
  return true;
}

bool PlanningIfaceBase::checkGoalState()
{
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose2D pose2d_cur;
  moveit_msgs::RobotTrajectory trajectory;
  const double EEF_STEP = 0.0001;
  rll_msgs::PickPlace::Request pick_place_req;
  rll_msgs::PickPlace::Response pick_place_resp;
  rll_msgs::MoveLin::Request move_lin_req;
  rll_msgs::MoveLin::Response move_lin_resp;
  float dist_goal_trans, diff_goal_angle;
  RLLErrorCode error_code;

  // check if we are close enough to the goal
  diffCurrentState(goal_pose_2d_, &dist_goal_trans, &diff_goal_angle, &pose2d_cur);

  if (dist_goal_trans < GOAL_TOLERANCE_TRANS && diff_goal_angle < GOAL_TOLERANCE_ROT)
  {
    ROS_INFO("distance to goal: %f, orientation diff: %f", dist_goal_trans, diff_goal_angle);

    geometry_msgs::Pose reset_above_goal = goal_pose_above_;
    reset_above_goal.position.z = POSE_Z_ABOVE_MAZE;
    waypoints.push_back(reset_above_goal);
    double achieved =
        manip_move_group_.computeCartesianPath(waypoints, EEF_STEP, DEFAULT_LINEAR_JUMP_THRESHOLD, trajectory);
    if (achieved < 1)
    {
      ROS_WARN("You came close to the goal, but unfortunately, it's not reachable.\n");
      grasp_object_at_goal_ = false;
      return true;
    }

    ROS_INFO("Goal was reached, good job!\n");

    // place the object at the goal
    pick_place_req.pose_above = goal_pose_above_;
    pick_place_req.pose_grip = goal_pose_grip_;
    pick_place_req.gripper_close = RLL_SRV_FALSE;
    pick_place_req.grasp_object = grasp_object_.id;

    error_code = pickPlace(pick_place_req, &pick_place_resp);
    if (error_code.failed())
    {
      ROS_FATAL("Failed to place the grasp object at the goal!");
      return false;
    }

    grasp_object_at_goal_ = true;

    // move a little higher after placing the object
    move_lin_req.pose = reset_above_goal;
    moveLin(move_lin_req, &move_lin_resp);
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
    grasp_object_at_goal_ = false;
  }

  return true;
}

void PlanningIfaceBase::diffCurrentState(const geometry_msgs::Pose2D& pose_des, float* diff_trans, float* diff_rot,
                                         geometry_msgs::Pose2D* pose2d_cur)
{
  geometry_msgs::Pose current_pose = manip_move_group_.getCurrentPose().pose;

  *diff_trans = sqrt(pow(current_pose.position.x - pose_des.x, 2) + pow(current_pose.position.y - pose_des.y, 2));

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
  *diff_rot = fabs(yaw - pose_des.theta);

  pose2d_cur->x = current_pose.position.x;
  pose2d_cur->y = current_pose.position.y;
  pose2d_cur->theta = yaw;
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
  current_pose.position.z = POSE_Z_ABOVE_MAZE;
  move_lin_req.pose = current_pose;
  RLLErrorCode error_code = moveLin(move_lin_req, &move_lin_resp);
  if (error_code.failed())
  {
    ROS_FATAL("Moving above maze for reset failed");
    return error_code;
  }

  move_ptp_req.pose = start_pose_above_;
  move_ptp_req.pose.position.z = POSE_Z_ABOVE_MAZE;
  if (!poseGoalTooClose(move_ptp_req.pose))
  {
    error_code = movePTP(move_ptp_req, &move_ptp_resp);
    if (error_code.failed())
    {
      ROS_FATAL("Moving above start pos for reset failed");
      return error_code;
    }
  }

  pick_place_req.pose_above = start_pose_above_;
  pick_place_req.pose_above.position.z = POSE_Z_ABOVE_MAZE;
  pick_place_req.pose_grip = start_pose_grip_;
  pick_place_req.gripper_close = RLL_SRV_FALSE;
  pick_place_req.grasp_object = grasp_object_.id;
  error_code = pickPlace(pick_place_req, &pick_place_resp);
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

void PlanningIfaceBase::pose2dToPose3d(const geometry_msgs::Pose2D& pose2d, geometry_msgs::Pose* pose3d)
{
  tf::Quaternion q_orig, q_rot, q_new;

  pose3d->position.x = pose2d.x;
  pose3d->position.y = pose2d.y;
  pose3d->position.z = VERT_GROUND_CLEARANCE;

  // orientation of eef in home position
  q_orig[0] = q_orig[2] = q_orig[3] = 0;
  q_orig[1] = 1;

  q_rot = tf::createQuaternionFromRPY(0, 0, pose2d.theta);
  q_new = q_rot * q_orig;
  q_new.normalize();
  quaternionTFToMsg(q_new, pose3d->orientation);
}

void PlanningIfaceBase::startServicesAndRunNode(ros::NodeHandle* nh)
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  PlanningIfaceBase* iface_ptr = this;
  RLLMoveIfaceServices* move_iface_ptr = iface_ptr;
  RLLMoveIfaceBase* base_iface_ptr = iface_ptr;

  iface_ptr->resetToHome();

  RLLMoveIfaceBase::JobServer server_job(  // NOLINT clang-analyzer-optin.cplusplus.VirtualCall
      *nh, RLLMoveIfaceBase::RUN_JOB_SRV_NAME, boost::bind(&RLLMoveIfaceBase::runJobAction, iface_ptr, _1, &server_job),
      false);
  server_job.start();
  RLLMoveIfaceBase::JobServer server_idle(*nh, RLLMoveIfaceBase::IDLE_JOB_SRV_NAME,
                                          boost::bind(&RLLMoveIfaceBase::idleAction, iface_ptr, _1, &server_idle),
                                          false);
  server_idle.start();
  ros::ServiceServer move = nh->advertiseService(MOVE_SRV_NAME, &PlanningIfaceBase::moveSrv, iface_ptr);
  ros::ServiceServer check_path =
      nh->advertiseService(CHECK_PATH_SRV_NAME, &PlanningIfaceBase::checkPathSrv, iface_ptr);
  ros::ServiceServer get_start_goal =
      nh->advertiseService(GET_START_GOAL_SRV_NAME, &PlanningIfaceBase::getStartGoalSrv, iface_ptr);
  ros::ServiceServer robot_ready = nh->advertiseService(RLLMoveIfaceServices::ROBOT_READY_SRV_NAME,
                                                        &RLLMoveIfaceServices::robotReadySrv, move_iface_ptr);
  ros::ServiceServer job_finished =
      nh->advertiseService(RLLMoveIfaceBase::JOB_FINISHED_SRV_NAME, &RLLMoveIfaceBase::jobFinishedSrv, base_iface_ptr);

  ROS_INFO("RLL Planning Interface started\n");
  ros::waitForShutdown();
}
