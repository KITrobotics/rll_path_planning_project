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

#include <planning_iface.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <visualization_msgs/Marker.h>

PlanningIface::PlanningIface(ros::NodeHandle nh)
	: nh_(nh)
{
	moveit_msgs::CollisionObject maze;
	shape_msgs::SolidPrimitive primitive;
	geometry_msgs::Pose box_pose, maze_pose;
	visualization_msgs::Marker marker;
	float grasp_object_dim_x, grasp_object_dim_y, grasp_object_dim_z,
		start_pos_x, start_pos_y, start_pos_theta,
		goal_pos_x, goal_pos_y, goal_pos_theta;
	bool headless;

	node_name = ros::this_node::getName();
	nh.getParam(node_name + "/headless", headless);
	nh.getParam(node_name + "/start_pos_x", start_pos_x);
	nh.getParam(node_name + "/start_pos_y", start_pos_y);
	nh.getParam(node_name + "/start_pos_theta", start_pos_theta);
	nh.getParam(node_name + "/grasp_object_dim_x", grasp_object_dim_x);
	nh.getParam(node_name + "/grasp_object_dim_y", grasp_object_dim_y);
	nh.getParam(node_name + "/grasp_object_dim_z", grasp_object_dim_z);
	nh.getParam(node_name + "/goal_pos_x", goal_pos_x);
	nh.getParam(node_name + "/goal_pos_y", goal_pos_y);
	nh.getParam(node_name + "/goal_pos_theta", goal_pos_theta);

	// init start poses
	start_pose_2d.x = start_pos_x;
	start_pose_2d.y = start_pos_y;
	start_pose_2d.theta = start_pos_theta;
	pose2d_to_pose3d(start_pose_2d, start_pose_above);
	pose2d_to_pose3d(start_pose_2d, start_pose_grip);
	start_pose_grip.position.z = vert_grip_height;
	// init goal poses
	goal_pose_2d.x = goal_pos_x;
	goal_pose_2d.y = goal_pos_y;
	goal_pose_2d.theta = goal_pos_theta;
	pose2d_to_pose3d(goal_pose_2d, goal_pose_above);
	pose2d_to_pose3d(goal_pose_2d, goal_pose_grip);
	goal_pose_grip.position.z = vert_grip_height;

	// add the grasp object as a collision object into the scene
	grasp_object.header.frame_id = move_iface.manip_move_group.getPlanningFrame();
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
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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

	move_iface.planning_scene_interface.applyCollisionObject(grasp_object);

	maze.header.frame_id = move_iface.manip_move_group.getPlanningFrame();
	maze.id = "maze";
	maze.meshes.resize(1);
	maze.mesh_poses.resize(1);
	shapes::Mesh* m = shapes::createMeshFromResource("package://rll_planning_project/meshes/collision/Maze.stl");
	shape_msgs::Mesh maze_mesh;
	shapes::ShapeMsg maze_mesh_msg;
	shapes::constructMsgFromShape(m, maze_mesh_msg);
	maze_mesh = boost::get<shape_msgs::Mesh>(maze_mesh_msg);
	maze.meshes[0] = maze_mesh;
	maze.meshes.push_back(maze_mesh);
	maze_pose.position.x = maze_pose.position.y = maze_pose.position.z = 0;
	maze.mesh_poses[0] = maze_pose;
	maze.mesh_poses.push_back(maze_pose);
	maze.operation = maze.ADD;
	std_msgs::ColorRGBA maze_color;
	maze_color.r = maze_color.g = maze_color.a = 1;
	maze_color.b = 0;
	move_iface.planning_scene_interface.applyCollisionObject(maze, maze_color);

	move_iface.reset_to_home();

	grasp_object_at_goal = false;
	allowed_to_plan = false;

	manip_joint_model_group = move_iface.manip_model->getJointModelGroup(move_iface.manip_move_group.getName());
	eef_link = move_iface.manip_move_group.getEndEffectorLink();
}

void PlanningIface::run_job(const rll_msgs::JobEnvGoalConstPtr &goal,
			    RLLMoveIface::JobServer *as)
{
	actionlib::SimpleActionClient<rll_planning_project::PlanToGoalAction> plan_client("plan_to_goal", true);
	bool run_three_times = false;
	int num_runs = 1;
	rll_msgs::JobEnvResult result;
	ros::Duration planning_time;
	std::vector<float> planning_times;
	float planning_time_final;

	ROS_INFO("got job running request");

	nh_.getParam(node_name + "/run_three_times", run_three_times);
	if (run_three_times)
		num_runs = 3;

	for (int i = 0; i < num_runs; ++i) {
		bool success = run_planner_once(as, plan_client, planning_time);
		if (!success)
			return;

		// idle is called separately after the last run
		if (run_three_times && i < 2) {
			success = idle();
			if (!success) {
				result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
				as->setSucceeded(result);
				return;
			}
		}

		planning_times.push_back(planning_time.toSec());
	}

	if (run_three_times) {
		std::sort(planning_times.begin(), planning_times.end());
		planning_time_final = planning_times[1];
	} else {
		planning_time_final = planning_times[0];
	}

	rll_msgs::JobExtraField job_data;
	job_data.description = "duration";
	job_data.value = planning_time_final;
	result.job_data.push_back(job_data);

	result.job.status = rll_msgs::JobStatus::SUCCESS;
	as->setSucceeded(result);
}

void PlanningIface::job_idle(const rll_msgs::JobEnvGoalConstPtr &goal,
			     RLLMoveIface::JobServer *as)
{
	rll_msgs::JobEnvResult result;

	ROS_INFO("got job idle request");

	bool success = idle();
	if (!success)
		result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
	else
		result.job.status = rll_msgs::JobStatus::SUCCESS;

	as->setSucceeded(result);
}

bool PlanningIface::run_planner_once(RLLMoveIface::JobServer *as,
				     actionlib::SimpleActionClient<rll_planning_project::PlanToGoalAction> &plan_client,
				     ros::Duration &planning_time)
{
	rll_msgs::JobEnvResult result;
	rll_msgs::MovePTP::Request move_ptp_req;
	rll_msgs::MovePTP::Response move_ptp_resp;
	rll_msgs::PickPlace::Request pick_place_req;
	rll_msgs::PickPlace::Response pick_place_resp;

	// pick up the grasp object
	move_ptp_req.pose = start_pose_above;
	move_ptp_req.pose.position.z = pose_z_above_maze;
	move_iface.move_ptp(move_ptp_req, move_ptp_resp);
	if (!move_ptp_resp.success) {
		ROS_FATAL("Moving PTP above grasp object failed");
		result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
		as->setSucceeded(result);
		return false;
	}

	pick_place_req.pose_above = start_pose_above;
	pick_place_req.pose_grip = start_pose_grip;
	pick_place_req.gripper_close = true;
	pick_place_req.grasp_object = grasp_object.id;
	move_iface.pick_place(pick_place_req, pick_place_resp);
	if (!pick_place_resp.success) {
		ROS_FATAL("Failed to pick up the grasp object!");
		result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
		as->setSucceeded(result);
		return false;
	}

	// check_path needs this to have a proper start state
	static planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
		std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
	planning_scene_monitor->requestPlanningSceneState("get_planning_scene");
	planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor);
	planning_scene->getCurrentStateNonConst().update();
	robot_state::RobotState start_state = planning_scene->getCurrentState();
	check_path_start_state = &start_state;

	if (!plan_client.waitForServer(ros::Duration(2.0))) {
		ROS_ERROR("planning service not available");
		result.job.status = rll_msgs::JobStatus::FAILURE;
		as->setSucceeded(result);
		return false;
	}

	plan_client_ptr = &plan_client;

	rll_planning_project::PlanToGoalGoal plan_req;
	plan_req.start.x = start_pose_2d.x;
	plan_req.start.y = start_pose_2d.y;
	plan_req.start.theta = start_pose_2d.theta;
	plan_req.goal.x = goal_pose_2d.x;
	plan_req.goal.y = goal_pose_2d.y;
	plan_req.goal.theta = goal_pose_2d.theta;
	allowed_to_plan = true;
	ROS_INFO("calling the planning service\n");
	plan_client_ptr->sendGoal(plan_req);

	ros::Time planning_start = ros::Time::now();
	plan_client_ptr->waitForResult(ros::Duration(plan_service_timeout));
	ros::Time planning_end = ros::Time::now();
	planning_time = planning_end - planning_start;

	ROS_INFO("called the planning service, planning and moving took %d minutes and %d seconds",
		 int(planning_time.toSec() / 60), int(fmod(planning_time.toSec(), 60)));
	allowed_to_plan = false;

	bool success = check_goal_state();
	if (!success) {
		result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
		as->setSucceeded(result);
		return false;
	}

	if (!grasp_object_at_goal) {
		result.job.status = rll_msgs::JobStatus::FAILURE;
		as->setSucceeded(result);
		return false;
	}

	return true;
}

bool PlanningIface::idle()
{
	rll_msgs::MovePTP::Request move_ptp_req;
	rll_msgs::MovePTP::Response move_ptp_resp;
	rll_msgs::PickPlace::Request pick_place_req;
	rll_msgs::PickPlace::Response pick_place_resp;

	ROS_INFO("got idle request");

	if (grasp_object_at_goal) {
		// pick up the grasp object
		move_ptp_req.pose = goal_pose_above;
		move_ptp_req.pose.position.z = pose_z_above_maze;
		move_iface.move_ptp(move_ptp_req, move_ptp_resp);
		if (!move_ptp_resp.success) {
			ROS_ERROR("Moving PTP above goal pos for reset failed");
			return false;
		}

		pick_place_req.pose_above = goal_pose_above;
		pick_place_req.pose_grip = goal_pose_grip;
		pick_place_req.gripper_close = true;
		pick_place_req.grasp_object = grasp_object.id;
		move_iface.pick_place(pick_place_req, pick_place_resp);
		if (!pick_place_resp.success) {
			ROS_ERROR("Failed to pick up the grasp object at the goal!");
			return false;
		} else {
			grasp_object_at_goal = false;
		}
	}

	bool success = reset_to_start();
	if (!success)
		return false;

	ROS_INFO("moved to idle position\n");
	return true;
}

bool PlanningIface::move(rll_planning_project::Move::Request &req,
			 rll_planning_project::Move::Response &resp)
{
	geometry_msgs::Pose pose3d_goal;
	geometry_msgs::Pose2D pose2d_cur;
	std::vector<geometry_msgs::Pose> waypoints;
	moveit_msgs::RobotTrajectory trajectory;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	float move_dist, dist_rot;
	const double eef_step = 0.0005;
	const double jump_threshold = 4.5;

	if (!allowed_to_plan) {
		ROS_WARN("Not allowed to send move commands");
		resp.success = false;
		return true;
	}

	// enforce a lower bound for move command to ensure that Moveit has enough waypoints
	diff_current_state(req.pose, move_dist, dist_rot, pose2d_cur);
	if ((move_dist < 0.005 && move_dist > 0.0005) || (move_dist < 0.0005 && dist_rot < 20 * M_PI / 180)) {
		ROS_WARN("move commands that cover a distance between 0 and 5 mm or sole rotations less than 20 degrees are not supported!");
		resp.success = false;
		return true;
	} else if (move_dist < 0.0005) {
		// generate more rotation points here to ensure that there are at least ten
		// for the continuity check
		float rot_step_size = (pose2d_cur.theta - req.pose.theta) / 10;
		geometry_msgs::Pose pose3d_rot;
		geometry_msgs::Pose2D pose2d_rot = pose2d_cur;
		for (int i = 0; i < 9; ++i) {
			pose2d_rot.theta += rot_step_size;
			pose2d_to_pose3d(pose2d_rot, pose3d_rot);
			waypoints.push_back(pose3d_rot);
		}
	}

	ROS_INFO("move request to pos x=%.3f y=%.3f theta=%.3f", req.pose.x, req.pose.y, req.pose.theta);
	move_iface.manip_move_group.setStartStateToCurrentState();
	pose2d_to_pose3d(req.pose, pose3d_goal);
	waypoints.push_back(pose3d_goal);
	double achieved = move_iface.manip_move_group.computeCartesianPath(waypoints,
									   eef_step, jump_threshold, trajectory);
	if (achieved < 1) {
		ROS_WARN("aborting, only achieved to compute %f of the requested move path", achieved);
		plan_client_ptr->cancelGoal();
		allowed_to_plan = false;
		resp.success = false;
		return true;
	} else if (trajectory.joint_trajectory.points.size() < 10) {
		ROS_ERROR("trajectory has not enough points to check for continuity, only got %lu", trajectory.joint_trajectory.points.size());
		ROS_ERROR("move dist %f, dist rot %f", move_dist, dist_rot);
		ROS_WARN("aborting...");
		plan_client_ptr->cancelGoal();
		allowed_to_plan = false;
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;
	bool success = (move_iface.manip_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed, aborting");
		plan_client_ptr->cancelGoal();
		allowed_to_plan = false;
		resp.success = false;
		return true;
	}

	resp.success = true;
	return true;
}

bool PlanningIface::check_path(rll_planning_project::CheckPath::Request &req,
			       rll_planning_project::CheckPath::Response &resp)
{
	geometry_msgs::Pose pose3d_start, pose3d_goal;
	std::vector<geometry_msgs::Pose> waypoints;
	moveit_msgs::RobotTrajectory trajectory;
	const double eef_step = 0.0005;
	const double jump_threshold = 4.5;

	if (!allowed_to_plan) {
		ROS_WARN("Not allowed to request path checks");
		return true;
	}

	// enforce a lower bound for check_path requests to ensure that Moveit has enough waypoints
	float move_dist = sqrt(pow(req.pose_start.x - req.pose_goal.x, 2)
			       + pow(req.pose_start.y - req.pose_goal.y, 2));
	float dist_rot = fabs(req.pose_start.theta - req.pose_goal.theta);
	if ((move_dist < 0.005 && move_dist != 0) || (move_dist == 0 && dist_rot < 20 * M_PI / 180)) {
		ROS_WARN("check path requests that cover a distance between 0 and 5 mm or sole rotations less than 20 degrees are not supported!");
		ROS_WARN("Moving distance would have been %f m", move_dist);
		resp.valid = false;
		return true;
	} else if (move_dist == 0) {
		// generate more points here to ensure that there are at least ten
		// for the continuity check
		float rot_step_size = (req.pose_start.theta - req.pose_goal.theta) / 10;
		geometry_msgs::Pose pose3d_rot;
		geometry_msgs::Pose2D pose2d_rot = req.pose_start;
		for (int i = 0; i < 9; ++i) {
			pose2d_rot.theta += rot_step_size;
			pose2d_to_pose3d(pose2d_rot, pose3d_rot);
			waypoints.push_back(pose3d_rot);
		}
	}

	pose2d_to_pose3d(req.pose_start, pose3d_start);
	pose2d_to_pose3d(req.pose_goal, pose3d_goal);

	bool start_pose_valid = check_path_start_state->setFromIK(manip_joint_model_group, pose3d_start,
								  eef_link);
	if (!start_pose_valid) {
		// ROS_WARN("path check: start pose not valid");
		resp.valid = false;
		return true;
	}

	move_iface.manip_move_group.setStartState(*check_path_start_state);

	waypoints.push_back(pose3d_goal);
	double achieved = move_iface.manip_move_group.computeCartesianPath(waypoints,
									   eef_step, jump_threshold, trajectory);
	if (achieved < 1) {
		// ROS_WARN("only achieved to compute %f of the requested path", achieved);
		resp.valid = false;
		return true;
	} else {
		resp.valid = true;
	}
	if (trajectory.joint_trajectory.points.size() < 10) {
		ROS_ERROR("trajectory has not enough points to check for continuity, only got %lu", trajectory.joint_trajectory.points.size());
		ROS_ERROR("move dist %f, dist rot %f", move_dist, dist_rot);
		resp.valid = false;
	}

	return true;
}

bool PlanningIface::check_goal_state()
{
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose2D pose2d_cur;
	moveit_msgs::RobotTrajectory trajectory;
	const double eef_step = 0.0001;
	const double jump_threshold = 4.5;
	rll_msgs::PickPlace::Request pick_place_req;
	rll_msgs::PickPlace::Response pick_place_resp;
	float dist_goal_trans, diff_goal_angle;

	// check if we are close enough to the goal
	diff_current_state(goal_pose_2d, dist_goal_trans, diff_goal_angle, pose2d_cur);

	if (dist_goal_trans < goal_tolerance_trans && diff_goal_angle < goal_tolerance_rot) {
		ROS_INFO("distance to goal: %f, orientation diff: %f", dist_goal_trans, diff_goal_angle);

		geometry_msgs::Pose reset_above_goal = goal_pose_above;
		reset_above_goal.position.z = pose_z_above_maze;
		waypoints.push_back(reset_above_goal);
		double achieved = move_iface.manip_move_group.computeCartesianPath(waypoints,
										   eef_step, jump_threshold, trajectory);
		if (achieved < 1) {
			ROS_WARN("You came close to the goal, but unfortunately, it's not reachable.\n");
			grasp_object_at_goal = false;
			return true;
		} else {
			ROS_INFO("Goal was reached, good job!\n");
		}

		// place the object at the goal
		pick_place_req.pose_above = goal_pose_above;
		pick_place_req.pose_grip = goal_pose_grip;
		pick_place_req.gripper_close = false;
		pick_place_req.grasp_object = grasp_object.id;
		move_iface.pick_place(pick_place_req, pick_place_resp);
		if (!pick_place_resp.success) {
			ROS_FATAL("Failed to place the grasp object at the goal!");
			return false;
		} else {
			grasp_object_at_goal = true;
		}

		bool success = move_iface.reset_to_home();
		if (!success) {
			ROS_FATAL("Failed to reset to home from goal");
			return false;
		}
	} else {
		ROS_WARN("Goal was not reached");
		ROS_WARN("distance to goal: %f, orientation diff: %f\n", dist_goal_trans, diff_goal_angle);
		grasp_object_at_goal = false;
	}

	return true;
}

void PlanningIface::diff_current_state(const geometry_msgs::Pose2D pose_des, float &diff_trans, float &diff_rot,
				       geometry_msgs::Pose2D &pose2d_cur)
{
	geometry_msgs::Pose current_pose = move_iface.manip_move_group.getCurrentPose().pose;

	diff_trans = sqrt(pow(current_pose.position.x - pose_des.x, 2)
			  + pow(current_pose.position.y - pose_des.y, 2));

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
		yaw += 2*M_PI;
	// almost 360°
	if (yaw > 6.26)
		yaw = 0;
	diff_rot = fabs(yaw - pose_des.theta);

	pose2d_cur.x = current_pose.position.x;
	pose2d_cur.y = current_pose.position.y;
	pose2d_cur.theta = yaw;
}

bool PlanningIface::reset_to_start()
{
	rll_msgs::MoveLin::Request move_lin_req;
	rll_msgs::MoveLin::Response move_lin_resp;
	rll_msgs::MovePTP::Request move_ptp_req;
	rll_msgs::MovePTP::Response move_ptp_resp;
	rll_msgs::PickPlace::Request pick_place_req;
	rll_msgs::PickPlace::Response pick_place_resp;
	geometry_msgs::Pose current_pose = move_iface.manip_move_group.getCurrentPose().pose;

	// set this a little higher to make sure we are moving above the maze
	current_pose.position.z = pose_z_above_maze;
	move_lin_req.pose = current_pose;
	move_iface.move_lin(move_lin_req, move_lin_resp);
	if (!move_lin_resp.success) {
		ROS_FATAL("Moving above maze for reset failed");
		return false;
	}

	move_ptp_req.pose = start_pose_above;
	move_ptp_req.pose.position.z = pose_z_above_maze;
	move_iface.move_ptp(move_ptp_req, move_ptp_resp);
	if (!move_ptp_resp.success) {
		ROS_FATAL("Moving above start pos for reset failed");
		return false;
	}

	pick_place_req.pose_above = start_pose_above;
	pick_place_req.pose_above.position.z = pose_z_above_maze;
	pick_place_req.pose_grip = start_pose_grip;
	pick_place_req.gripper_close = false;
	pick_place_req.grasp_object = grasp_object.id;
	move_iface.pick_place(pick_place_req, pick_place_resp);
	if (!pick_place_resp.success) {
		ROS_FATAL("Failed to place the grasp object at the start pos");
		return false;
	}

	bool success = move_iface.reset_to_home();
	if (!success) {
		ROS_FATAL("Failed to reset to start from start pos");
		return false;
	}

	return true;
}

void PlanningIface::pose2d_to_pose3d(geometry_msgs::Pose2D &pose2d, geometry_msgs::Pose &pose3d)
{
	tf::Quaternion q_orig, q_rot, q_new;

	pose3d.position.x = pose2d.x;
	pose3d.position.y = pose2d.y;
	pose3d.position.z = vert_ground_clearance;

	// orientation of eef in home position
	q_orig[0] = q_orig[2] = q_orig[3] = 0;
	q_orig[1] = 1;

	q_rot = tf::createQuaternionFromRPY(0, 0, pose2d.theta);
	q_new = q_rot*q_orig;
	q_new.normalize();
	quaternionTFToMsg(q_new, pose3d.orientation);
}

PlanningIface::~PlanningIface() {}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planning_iface");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	PlanningIface plan_iface(nh);

	RLLMoveIface::JobServer server_job(nh, "job_env", boost::bind(&PlanningIface::run_job, &plan_iface, _1, &server_job), false);
	server_job.start();
	RLLMoveIface::JobServer server_idle(nh, "job_idle", boost::bind(&PlanningIface::job_idle, &plan_iface, _1, &server_idle), false);
	server_idle.start();
	ros::ServiceServer move = nh.advertiseService("move", &PlanningIface::move, &plan_iface);
	ros::ServiceServer check_path = nh.advertiseService("check_path", &PlanningIface::check_path, &plan_iface);

	ROS_INFO("RLL Planning Project Interface started\n");

	ros::waitForShutdown();

	return 0;
}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
