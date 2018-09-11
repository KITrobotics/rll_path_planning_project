/*
 * This file is part of the Robot Learning Lab Path Planning Project
 *
 * Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#ifndef RLL_PLANNING_PROJECT_IFACE_H
#define RLL_PLANNING_PROJECT_IFACE_H

#include <rll_move/move_iface.h>
#include <rll_planning_project/Move.h>
#include <rll_planning_project/CheckPath.h>
#include <rll_planning_project/PlanToGoalAction.h>

class PlanningIface
{
public:
	explicit PlanningIface(ros::NodeHandle nh);

	RLLMoveIface move_iface;

	void run_job(const rll_msgs::JobEnvGoalConstPtr &goal,
		     RLLMoveIface::JobServer *as);
	void job_idle(const rll_msgs::JobEnvGoalConstPtr &goal,
		      RLLMoveIface::JobServer *as);
	bool move(rll_planning_project::Move::Request &req,
		  rll_planning_project::Move::Response &resp);
	bool check_path(rll_planning_project::CheckPath::Request &req,
			rll_planning_project::CheckPath::Response &resp);

	~PlanningIface();

private:
	const float goal_tolerance_trans = 0.04;
	const float goal_tolerance_rot = 10 * M_PI / 180;
	const float vert_ground_clearance = 0.05;
	const float vert_grip_height = 0.01;
	const float pose_z_above_maze = 0.15;
	const float plan_service_timeout = 8 * 60;

	bool grasp_object_at_goal;
	bool allowed_to_plan;
	moveit_msgs::CollisionObject grasp_object;
	geometry_msgs::Pose start_pose_grip, start_pose_above;
	geometry_msgs::Pose goal_pose_grip, goal_pose_above;
	geometry_msgs::Pose2D start_pose_2d, goal_pose_2d;
	robot_state::RobotState* check_path_start_state;
	const robot_state::JointModelGroup* manip_joint_model_group;
	std::string eef_link;

	ros::NodeHandle nh_;
	std::string node_name;
	actionlib::SimpleActionClient<rll_planning_project::PlanToGoalAction>* plan_client_ptr;

	bool run_planner_once(RLLMoveIface::JobServer *as,
			      actionlib::SimpleActionClient<rll_planning_project::PlanToGoalAction> &plan_client,
			      ros::Duration &planning_time);
	bool idle();
	bool reset_to_start();
	bool check_goal_state();
	void diff_current_state(const geometry_msgs::Pose2D pose_des, float &diff_trans, float &diff_rot,
				geometry_msgs::Pose2D &pose2d_cur);
	void pose2d_to_pose3d(geometry_msgs::Pose2D &pose2d, geometry_msgs::Pose &pose3d);
};

#endif  // RLL_PLANNING_PROJECT_IFACE_H

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
