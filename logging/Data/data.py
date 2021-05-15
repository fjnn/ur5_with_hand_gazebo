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
OUTPUT_FOLDER = "/home/gizem/Documents/Gitkraken/arm-paper/DataOutput"  # take it as a rosparam
OUTPUT_FILENAME_PREFIX = "2arms_ur5e"

# ID_ELAPSED_TIME = 'elapsed_time'

# ID_LHAND_POS_X = 'lhand_pos_x'
# ID_LHAND_POS_Y = 'lhand_pos_y'
# ID_LHAND_POS_Z = 'lhand_pos_z'
# ID_LHAND_ORI_X = 'lhand_ori_x'
# ID_LHAND_ORI_Y = 'lhand_ori_y'
# ID_LHAND_ORI_Z = 'lhand_ori_z'
# ID_LHAND_ORI_W = 'lhand_ori_w'

# ID_RHAND_POS_X = 'rhand_pos_x'
# ID_RHAND_POS_Y = 'rhand_pos_y'
# ID_RHAND_POS_Z = 'rhand_pos_z'
# ID_RHAND_ORI_X = 'rhand_ori_x'
# ID_RHAND_ORI_Y = 'rhand_ori_y'
# ID_RHAND_ORI_Z = 'rhand_ori_z'
# ID_RHAND_ORI_W = 'rhand_ori_w'

# ID_TGOAL_POS_X = 'tgoal_pos_x'
# ID_TGOAL_POS_Y = 'tgoal_pos_y'
# ID_TGOAL_POS_Z = 'tgoal_pos_z'
# ID_TGOAL_ORI_X = 'tgoal_ori_x'
# ID_TGOAL_ORI_Y = 'tgoal_ori_y'
# ID_TGOAL_ORI_Z = 'tgoal_ori_z'
# ID_TGOAL_ORI_W = 'tgoal_ori_w'

# ID_TACTUAL_POS_X = 'tactual_pos_x'
# ID_TACTUAL_POS_Y = 'tactual_pos_y'
# ID_TACTUAL_POS_Z = 'tactual_pos_z'
# ID_TACTUAL_ORI_X = 'tactual_ori_x'
# ID_TACTUAL_ORI_Y = 'tactual_ori_y'
# ID_TACTUAL_ORI_Z = 'tactual_ori_z'
# ID_TACTUAL_ORI_W = 'tactual_ori_w'

# ID_TEE_POS_X = 'tool_pos_x'
# ID_TEE_POS_Y = 'tool_pos_y'
# ID_TEE_POS_Z = 'tool_pos_z'
# ID_TEE_ORI_X = 'tool_ori_x'
# ID_TEE_ORI_Y = 'tool_ori_y'
# ID_TEE_ORI_Z = 'tool_ori_z'
# ID_TEE_ORI_W = 'tool_ori_w'

DATA_LABELS = ('ID_ELAPSED_TIME',
				'ID_LHAND_POS_X',
				'ID_LHAND_POS_Y',
				'ID_LHAND_POS_Z',
				'ID_LHAND_ORI_X',
				'ID_LHAND_ORI_Y',
				'ID_LHAND_ORI_Z',
				'ID_LHAND_ORI_W',
				'ID_RHAND_POS_X',
				'ID_RHAND_POS_Y',
				'ID_RHAND_POS_Z',
				'ID_RHAND_ORI_X',
				'ID_RHAND_ORI_Y',
				'ID_RHAND_ORI_Z',
				'ID_RHAND_ORI_W',
				'ID_HAND_POS_X',
				'ID_HAND_POS_Y',
				'ID_HAND_POS_Z',
				'ID_HAND_ORI_X',
				'ID_HAND_ORI_Y',
				'ID_HAND_ORI_Z',
				'ID_HAND_ORI_W',
				'ID_TGOAL_POS_X',
				'ID_TGOAL_POS_Y',
				'ID_TGOAL_POS_Z',
				'ID_TGOAL_ORI_X',
				'ID_TGOAL_ORI_Y',
				'ID_TGOAL_ORI_Z',
				'ID_TGOAL_ORI_W',
				'ID_TACTUAL_POS_X',
				'ID_TACTUAL_POS_Y',
				'ID_TACTUAL_POS_Z',
				'ID_TACTUAL_ORI_X',
				'ID_TACTUAL_ORI_Y',
				'ID_TACTUAL_ORI_Z',
				'ID_TACTUAL_ORI_W')

DATA_INDICES = list(range(0,len(DATA_LABELS)))


def get_new_filename():
    # postfix = datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + ".csv"
    # filename = OUTPUT_FILENAME_PREFIX + "_" + postfix
    filename = "gizem_test.csv"
    # if folder doesn't exist, create it
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
    filename = os.path.join(OUTPUT_FOLDER, filename)
    return filename
