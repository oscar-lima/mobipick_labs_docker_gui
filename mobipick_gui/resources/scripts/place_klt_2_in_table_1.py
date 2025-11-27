#!/usr/bin/env python3
"""
Place the currently held object on table_1 without an additional observation step.
"""

import mobipick_api


def place_on_table_1():
    robot = mobipick_api.Robot("mobipick")
    # Place object on support surface "table_1" without observing beforehand
    robot.arm.place_object("table_1", observe_before_place=False)


if __name__ == "__main__":
    place_on_table_1()
