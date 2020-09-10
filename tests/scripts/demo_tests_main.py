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

import rospy
import rosunit

from rll_move_client.tests_demo_launcher import create_test_class


def main():
    rospy.init_node("demo_tests_python")

    to_import = [("rll_planning_project", "scripts/"),
                 ("rll_planning_project", "src")]

    klass = create_test_class(to_import, "path_planner", "plan_to_goal",
                              "rll_planning_project_iface.client",
                              "RLLPlanningProjectClient")

    rosunit.unitrun("rll_planning_project", "demo_tests_python", klass)


if __name__ == "__main__":
    main()
