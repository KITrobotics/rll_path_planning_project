#! /usr/bin/env python

import rospy
from rll_planning_project.srv import GetStartGoal, Move, CheckPath
from geometry_msgs.msg import Pose2D

from rll_move_client.client import RLLDefaultMoveClient


# TODO: create a planning client and rewrite the hello world
#       accordingly
def plan_to_goal(_):
    """ Plan a path from Start to Goal """
    pose_start = Pose2D()
    pose_goal = Pose2D()
    pose_check_start = Pose2D()
    pose_check_goal = Pose2D()
    pose_move = Pose2D()

    rospy.loginfo("Got a planning request")

    get_start_goal_srv = rospy.ServiceProxy('get_start_goal', GetStartGoal)
    start_goal = get_start_goal_srv()

    pose_start = start_goal.start
    pose_goal = start_goal.goal

    move_srv = rospy.ServiceProxy('move', Move)
    check_srv = rospy.ServiceProxy('check_path', CheckPath, persistent=True)

    ###############################################
    # Implement your path planning algorithm here #
    ###############################################

    # printing input values
    rospy.loginfo("start pose: x %f, y %f, theta %f",
                  pose_start.x, pose_start.y, pose_start.theta)
    rospy.loginfo("goal pose: x %f, y %f, theta %f",
                  pose_goal.x, pose_goal.y, pose_goal.theta)

    # Output: movement commands
    pose_check_start = pose_start
    pose_check_goal = pose_goal
    # checking if the arm can move to the goal pose
    resp = check_srv(pose_check_start, pose_check_goal)
    if resp.valid:
        rospy.loginfo("Valid pose")
        pose_move = pose_goal
        # executing a move command towards the goal pose
        resp = move_srv(pose_move)
    else:
        rospy.loginfo("Invalid pose")

    ###############################################
    # End of Algorithm #
    ###############################################

    return True


def main():
    rospy.init_node('path_planner')
    client = RLLDefaultMoveClient(plan_to_goal)
    client.spin()


if __name__ == "__main__":
    main()
