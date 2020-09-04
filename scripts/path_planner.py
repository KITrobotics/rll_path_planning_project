#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose2D
from rll_planning_project_iface.client import RLLPlanningProjectClient


def plan_to_goal(client):
    # type: (RLLPlanningProjectClient) -> bool

    """ Plan a path from Start to Goal """

    rospy.loginfo("Got a planning request")

    # Input: map dimensions, start pose, and goal pose
    # retrieving input values
    map_width = 1.0
    map_length = 1.4
    start_pose, goal_pose = client.get_start_goal()

    # printing input values
    rospy.loginfo("map dimensions: width=%1.2fm, length=%1.2fm",
                  map_width, map_length)
    rospy.loginfo("start pose: x %f, y %f, theta %f",
                  start_pose.x, start_pose.y, start_pose.theta)
    rospy.loginfo("goal pose: x %f, y %f, theta %f",
                  goal_pose.x, goal_pose.y, goal_pose.theta)

    ###############################################
    # Implement your path planning algorithm here #
    ###############################################

    path = []

    # example motions for the gripper
    motions = [
        # movement by 0.1m in positive or negative x-direction
        [0.1, 0, 0], [-0.1, 0, 0],
        # movement by 0.1m in positive or negative y-direction
        [0, 0.1, 0], [0, -0.1, 0],
        # rotation on the spot by 90°, clockwise or counterclockwise
        [0, 0, 1.57], [0, 0, 1.57],
        # rotation by 90° and movement into y or x direction (grinding curves)
        [0, 0.1, 1.57], [0.1, 0, -1.57]]

    ###############################################
    # Example on how to use check_path functionality
    ################################################
    for motion in motions:

        newx = start_pose.x + motion[0]
        newy = start_pose.y + motion[1]
        newtheta = start_pose.theta + motion[2]
        new_pose = Pose2D(newx, newy, newtheta)
        if client.check_path(start_pose, new_pose):
            path.append(new_pose)

    if path is not None:
        rospy.loginfo("A path was found, now trying to execute it")
        for point in path:
            client.move(point)

        return True

    rospy.loginfo("No path to goal found")
    return False


def main():
    rospy.init_node('path_planner')

    # Inititalize planning client with plan_to_goal as function to execute
    # Set verbose=True for more debugging output
    client = RLLPlanningProjectClient(plan_to_goal, verbose=False)
    client.spin()


if __name__ == "__main__":
    main()
