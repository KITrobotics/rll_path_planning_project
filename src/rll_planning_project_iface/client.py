from typing import Tuple  # pylint: disable=unused-import
import rospy
from geometry_msgs.msg import Pose2D  # pylint: disable=unused-import
from rll_move_client.client import RLLBasicMoveClient, RLLMoveClientListener
from rll_move_client.error import RLLErrorCode
from rll_move_client.formatting import override_formatting_for_ros_types
from rll_planning_project.srv import CheckPath, GetStartGoal, Move


class RLLPlanningProjectClient(RLLBasicMoveClient, RLLMoveClientListener):
    CHECK_PATH_SRV_NAME = "check_path"
    GET_START_GOAL_SRV_NAME = "get_start_goal"
    MOVE_SRV_NAME = "move"

    def __init__(self, execute=None, verbose=True):
        self.verbose = verbose
        RLLBasicMoveClient.__init__(self)
        RLLMoveClientListener.__init__(self, execute)

        self.get_start_goal_srv = rospy.ServiceProxy('get_start_goal',
                                                     GetStartGoal)
        self.move_srv = rospy.ServiceProxy('move', Move)
        self.check_srv = rospy.ServiceProxy(
            'check_path', CheckPath, persistent=True)

        RLLErrorCode.set_error_code_details(
            RLLErrorCode.PROJECT_SPECIFIC_RECOVERABLE_1,
            "PREVIOUS_MOVE_FAILED",
            "A previous move service call failed. As a result all further move"
            " service calls will be rejected. Please verify that a movement is"
            " possible by calling check_path first.")

        RLLErrorCode.set_error_code_details(
            RLLErrorCode.PROJECT_SPECIFIC_INVALID_1,
            "TOO_LITTLE_MOVEMENT",
            "Requests that cover a distance between 0 and 5 mm or sole "
            "rotations less than 20 degrees are not supported!")

        RLLErrorCode.set_error_code_details(
            RLLErrorCode.PROJECT_SPECIFIC_INVALID_2, "NO_PATH_FOUND", None)

        override_formatting_for_ros_types()

    def move(self, pose):
        # type: (Pose2D) -> bool
        return self._call_service_with_error_check(
            self.move_srv,
            self.MOVE_SRV_NAME,
            "%s requested with: %s",
            self._handle_response_error_code,
            pose)

    def get_start_goal(self, ):
        # type: () -> Tuple[Pose2D, Pose2D]

        def handle_return_values(resp):
            return [resp.start, resp.goal]

        return self._call_service_with_error_check(
            self.get_start_goal_srv,
            self.GET_START_GOAL_SRV_NAME,
            "%s requested",
            self._handle_resp_with_values(handle_return_values)
        )

    def check_path(self, pose_a, pose_b):
        # type: (Pose2D, Pose2D) -> bool

        return self._call_service_with_error_check(
            self.check_srv, self.CHECK_PATH_SRV_NAME,
            "%s requested from '%s' to '%s'", self._handle_response_error_code,
            pose_a, pose_b)
