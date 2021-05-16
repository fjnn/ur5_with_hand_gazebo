#!/usr/bin/env python
# -*- coding: utf-8 -*-

import plotting
import loading
import matplotlib.pyplot as plt
import os
import pandas as pd

# %%
# Load Data
# my_trials = loading.load_folder('../../Output')

folder = '/home/gizem/Documents/Gitkraken/arm-paper/DataOutput/csv'
df = []

files = sorted(os.listdir(folder))
for file in files:
    csv_filename = os.path.join(folder, file)
    csv_file = loading.pd.read_csv(csv_filename)
    df.append(csv_file)

print df[0]

# %% Plot individual motions
# fig, axes = plt.subplots(nrows = 1, ncols = 3)
# plt.tight_layout()
# axes[0].plot(df[0]['elapsed_time'], df[0]['y_deg'])
# axes[1].plot(df[0]['elapsed_time'], df[0]['p_deg'])
# axes[2].plot(df[0]['elapsed_time'], df[0]['r_deg'])

# # %% Plot learning curve
# plt.figure()  # create new plot window
# plotting.plot_learning_curve(my_trials)
#
# # %% Plot pedal presses
# plt.figure()  # create new plot window
# plotting.plot_pedal_press_counts_per_trial(my_trials)
