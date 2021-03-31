#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The files should be layouted as following:
    - All generated CSV files should be placed somewhere within the 'Output' folder
    - Every set of CSV files that belong to one Tester should be placed in a separate subfolder
    for example, if a user Ahmet ran the simulation 10 times, then 10 CSV files were generated,
    we place all of them in a folder called 'Ahmet' or some random number, it doesn't really matter.
    - Basically every subfolder represents one set of training runs. One person yani
    - The generated CSV files should be named with their timestamped
    so that there filenames are can be sorted in increasing order.
#todo: deal with different simulation scenarios (with vs without axis alignment)
"""

import os
from datetime import datetime


# OUTPUT_FOLDER = "../Output"  # take it as a rosparam
OUTPUT_FOLDER = "/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/logging/Output"  # take it as a rosparam
OUTPUT_FILENAME_PREFIX = "gizem_test"


ID_ELAPSED_TIME = 'elapsed_time'
ID_TEE_POS_X = 'tool_pos_x'
ID_TEE_POS_Y = 'tool_pos_y'
ID_TEE_POS_Z = 'tool_pos_z'
ID_TEE_ORI_X = 'tool_ori_x'
ID_TEE_ORI_Y = 'tool_ori_y'
ID_TEE_ORI_Z = 'tool_ori_z'
ID_TEE_ORI_W = 'tool_ori_w'
# ID_PITCH = 'human_pitch'
# ID_ROLL = 'human_roll'
# ID_YAW = 'human_yaw'
ID_GAIN = 'adaptive_gain'

DATA_LABELS = (ID_ELAPSED_TIME, ID_TEE_POS_X, ID_TEE_POS_Y, ID_TEE_POS_Z, ID_TEE_ORI_X, ID_TEE_ORI_Y, ID_TEE_ORI_Z, ID_TEE_ORI_W, ID_GAIN)
DATA_INDICES = {
								ID_ELAPSED_TIME: 0,
								ID_TEE_POS_X: 1,
								ID_TEE_POS_Y: 2,
								ID_TEE_POS_Z: 3,
								ID_TEE_ORI_X: 4,
								ID_TEE_ORI_Y: 5,
								ID_TEE_ORI_Z: 6,
								ID_TEE_ORI_W: 7,
								ID_GAIN: 8
								}


def get_new_filename():
    # postfix = datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + ".csv"
    # filename = OUTPUT_FILENAME_PREFIX + "_" + postfix
    filename = "gizem_test.csv"
    # if folder doesn't exist, create it
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
    filename = os.path.join(OUTPUT_FOLDER, filename)
    return filename
