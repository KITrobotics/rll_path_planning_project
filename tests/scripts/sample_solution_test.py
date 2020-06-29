#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab Path Planning Project
#
# Copyright (C) 2020 Mark Weinreuter <mark.weinreuter@kit.edu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import unittest
from os.path import abspath, dirname
import sys
import rospy

from rll_tools.run import run_project_in_background
from rll_move_client.client import RLLDefaultMoveClient

# Hackish: add the path_planner script to the path manually
SCRIPT_PATH = abspath(dirname(abspath(__file__)) + "../../../scripts/")
sys.path.append(SCRIPT_PATH)
import path_planner  # noqa:E402 pylint: disable=wrong-import-position


class TestSampleSolution(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestSampleSolution, self).__init__(*args, **kwargs)

    def _run_script_for_result(self, client):
        # run the main script -> only assert the result
        result = path_planner.plan_to_goal(client)

        # Initiate ROS shutdown before we assert the result, which might raise
        rospy.Timer(rospy.Duration(3), lambda _: rospy.signal_shutdown(
            "Test completed"), oneshot=True)
        self.assertTrue(result)
        return result

    def test_run_script(self):
        client = RLLDefaultMoveClient(self._run_script_for_result)
        run_project_in_background(8)
        client.spin()
