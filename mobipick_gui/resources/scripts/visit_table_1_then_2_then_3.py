#!/usr/bin/env python3
"""
Drive mobipick to table 1, then table 2, then table 3.

Pose data taken from moelk_tables_demo.yaml:

base_pick_pose_name:  "base_table_2_pose"
base_place_pose_name: "base_table_3_pose"

poses:
  # geometry_msgs Pose [x, y, z], [qx, qy, qz, qw]
  # base_handover_pose: [[10.16, 1.76, 0.0], [0.0, 0.0, 0.707, 0.707]]
  # base_home_pose:     [[12.00, 2.00, 0.0], [0.0, 0.0, 0.0  , 1.0  ]]
  # base_table_1_pose:  [[12.21, 2.10, 0.0], [0.0, 0.0, 0.707, 0.707]]
  # base_table_2_pose:  [[11.85, 2.45, 0.0], [0.0, 0.0, 1.0  , 0.0  ]]
  # base_table_3_pose:  [[10.25, 2.45, 0.0], [0.0, 0.0, 1.0  , 0.0  ]]
"""

import math
import mobipick_api


# Quaternion [0, 0, 0.707, 0.707] corresponds to yaw ≈ pi / 2
# Quaternion [0, 0, 1.0,   0.0  ] corresponds to yaw ≈ pi
TABLE_BASE_POSES = {
    "base_table_1_pose": (12.21, 2.10, math.pi / 2.0),
    "base_table_2_pose": (11.85, 2.45, math.pi),
    "base_table_3_pose": (10.25, 2.45, math.pi),
}

TABLE_VISIT_SEQUENCE = [
    "base_table_1_pose",
    "base_table_2_pose",
    "base_table_3_pose",
]


def visit_tables_1_2_3():
    robot = mobipick_api.Robot("mobipick")

    # Optional: start from a known safe arm pose
    robot.arm.move("transport")

    for pose_name in TABLE_VISIT_SEQUENCE:
        x, y, yaw = TABLE_BASE_POSES[pose_name]
        robot.base.move(x, y, yaw)


if __name__ == "__main__":
    visit_tables_1_2_3()
