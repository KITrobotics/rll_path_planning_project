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

#ifndef RLL_PLANNING_PROJECT_PLANNING_IFACE_H
#define RLL_PLANNING_PROJECT_PLANNING_IFACE_H

#include <rll_move/move_iface_base.h>
#include <rll_planning_project/CheckPath.h>
#include <rll_planning_project/GetStartGoal.h>
#include <rll_planning_project/Move.h>

class PlanningIfaceBase : public RLLMoveIfaceBase
{
public:
  explicit PlanningIfaceBase(const ros::NodeHandle& nh);

  // NOLINTNEXTLINE google-runtime-references
  bool moveSrv(rll_planning_project::Move::Request& req, rll_planning_project::Move::Response& resp);
  // NOLINTNEXTLINE google-runtime-references
  bool checkPathSrv(rll_planning_project::CheckPath::Request& req, rll_planning_project::CheckPath::Response& resp);
  void startServicesAndRunNode(ros::NodeHandle* nh) override;

protected:
  RLLErrorCode idle() override;
  void runJob(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult* result) override;
  bool getStartGoalSrv(  // NOLINTNEXTLINE google-runtime-references
      rll_planning_project::GetStartGoal::Request& req, rll_planning_project::GetStartGoal::Response& resp);
  bool checkPath(const rll_planning_project::CheckPath::Request& req, rll_planning_project::CheckPath::Response* resp);
  RLLErrorCode move(const rll_planning_project::Move::Request& req, rll_planning_project::Move::Response* /*resp*/);

  void registerPermissions();

private:
  const std::string GET_START_GOAL_SRV_NAME = "get_start_goal";
  const std::string MOVE_SRV_NAME = "move";
  const std::string CHECK_PATH_SRV_NAME = "check_path";

  const float GOAL_TOLERANCE_TRANS = 0.04;
  const float GOAL_TOLERANCE_ROT = 10 * M_PI / 180;
  const float VERT_GROUND_CLEARANCE = 0.05;
  const float VERT_GRIP_HEIGHT = 0.01;
  const float POSE_Z_ABOVE_MAZE = 0.15;

  bool grasp_object_at_goal_;
  Permissions::Index plan_permission_;
  moveit_msgs::CollisionObject grasp_object_;
  geometry_msgs::Pose start_pose_grip_, start_pose_above_;
  geometry_msgs::Pose goal_pose_grip_, goal_pose_above_;
  geometry_msgs::Pose2D start_pose_2d_, goal_pose_2d_;
  robot_state::RobotState* check_path_start_state_;

  void insertGraspObject();
  RLLErrorCode resetToStart();
  bool runPlannerOnce(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult* result);
  bool checkGoalState();
  void diffCurrentState(const geometry_msgs::Pose2D& pose_des, float* diff_trans, float* diff_rot,
                        geometry_msgs::Pose2D* pose2d_cur);
  void pose2dToPose3d(const geometry_msgs::Pose2D& pose2d, geometry_msgs::Pose* pose3d);
  void generateRotationWaypoints(const geometry_msgs::Pose2D& pose2d_start, float rot_step_size,
                                 std::vector<geometry_msgs::Pose>* waypoints);
};

#endif  // RLL_PLANNING_PROJECT_PLANNING_IFACE_H

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
