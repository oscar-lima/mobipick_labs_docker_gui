#!/usr/bin/env python3
"""
Move arm to transport pose, drive to table 2, perceive the table area,
then place the object currently held in the gripper on table_2.
"""

import math
import mobipick_api

# From moelk_tables_demo.yaml:
# base_table_2_pose: [[11.85, 2.45, 0.0], [0.0, 0.0, 1.0, 0.0]]
# Quaternion [0, 0, 1.0, 0.0] → yaw ≈ pi
BASE_TABLE_2_POSE = (11.85, 2.45, math.pi)  # x [m], y [m], yaw [rad]


def go_to_table_2_and_place():
    robot = mobipick_api.Robot("mobipick")

    # Move arm to a safe transport pose
    robot.arm.move("transport")

    # Drive base to table 2
    x, y, yaw = BASE_TABLE_2_POSE
    robot.base.move(x, y, yaw)

    # Perceive the table 2 area with arm camera
    # robot.arm_cam.perceive(
    #     observation_list=["observe100cm_right"]
    # )

    # Place currently held object on table_2, no extra observation
    robot.arm.place_object("table_2", observe_before_place=True)


if __name__ == "__main__":
    go_to_table_2_and_place()
