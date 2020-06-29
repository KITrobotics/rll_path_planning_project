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

from sample_solution_test import TestSampleSolution


def main():
    rospy.init_node("tests_python")

    rosunit.unitrun("rll_planning_project",
                    "solution_test", TestSampleSolution)


if __name__ == "__main__":
    main()
