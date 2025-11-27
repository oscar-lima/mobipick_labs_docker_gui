#!/usr/bin/env python3
"""
Pick klt_3 from table_3 while ignoring multimeter_1 in the planning scene.
Assumes perception and planning scene are already set up.
"""

import mobipick_api


def pick_klt_3_ignoring_multimeter_1():
    mobipick = mobipick_api.Robot("mobipick")

    # Optional but usually safer: move arm to a known pose before picking
    mobipick.arm.move("transport")

    mobipick.arm.pick_object(
        "klt_3",
        "table_3",
        planning_scene_ignore_list=["multimeter_1"],
        timeout=50.0,
    )


if __name__ == "__main__":
    pick_klt_3_ignoring_multimeter_1()
