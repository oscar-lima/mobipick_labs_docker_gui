#!/usr/bin/env python3
"""
Drive mobipick to table_1 and pick up klt_2.

Requires:
  - mobipick_api (rae-upom-mobipick branch)
  - moelk_tables_demo.yaml with base_table_1_pose defined
"""

import math
import mobipick_api


# Poses taken from moelk_tables_demo.yaml
# base_table_1_pose: [[12.21, 2.10, 0.0], [0.0, 0.0, 0.707, 0.707]]
# Quaternion [0, 0, 0.707, 0.707] corresponds to yaw ≈ pi / 2
TABLE_POSES = {
    "table_1": (12.21, 2.10, math.pi / 2.0),  # x [m], y [m], yaw [rad]
}


def drive_to_table_1_and_pick_klt_2():
    # Get the mobipick robot object (namespace "mobipick")
    mobipick = mobipick_api.Robot("mobipick")

    # Navigate base to table_1 using pose from moelk_tables_demo.yaml
    x, y, yaw = TABLE_POSES["table_1"]
    mobipick.base.move(x, y, yaw)

    # Run perception with the arm camera
    mobipick.arm_cam.perceive(
        observation_list=["observe100cm_right"]
    )

    # Ensure klt_2 was detected
    if not mobipick.arm_cam.is_object_inside_pose_selector("klt_2"):
        raise RuntimeError("klt_2 was not detected near table_1")

    # Pick klt_2 from table_1
    mobipick.arm.pick_object(
        "klt_2",              # object id
        "table_1",            # support surface name
        planning_scene_ignore_list=[],
        timeout=50.0,
    )


if __name__ == "__main__":
    drive_to_table_1_and_pick_klt_2()
