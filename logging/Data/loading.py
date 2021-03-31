#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import data

def load_file(filename):
    """ Load the CSV file specified
    @param filename: The name of the file or the relative path to the file you wish to load
    @returns A pandas DataFrame containing the data loaded from the specified file
    """
    return pd.read_csv(filename)

def load_folder(folder):
    """ Loads all the files located in the given folder.
    Make sure that the folder only contains csv files!
    @param folder: The relative path to the folder to load files from
    @returns A list of pandas DataFrames. The list contains one DataFrame per file
    """
    # get list of files in folder
    files = sorted(os.listdir(folder))
    # make sure folder is not empty
    if len(files) < 1:
        raise ValueError("Given folder %s is empty!" % folder)
    # read files
    dataframes = []
    for file in files:
        csv_filename = os.path.join(folder, file)
        csv_file = load_file(csv_filename)
        dataframes.append(csv_file)
    # return the list of pandas DataFrames
    return dataframes


def merge_dataframes(dataframe_list):
    """ Given a list of DataFrames, this function merges them into one DataFrame
    @param dataframe_list: A list of dataframes to merge into one dataframe
    @returns The merged DataFrames
    """
    merged_dataframe = dataframe_list[0]
    for i in range(1, len(dataframe_list)):
        dataframe = dataframe_list[i]
        merged_dataframe = merged_dataframe.append(dataframe)
    return merged_dataframe


def clean_dataframe_from_initial_values(dataframe):
    """ Removes the redundant 
    @param dataframe: The dataframe to remove the data from
    @returns A copy of the given dataframe, stripped and cleaned from the initial rows
    """
    # round all the angles to 3 decimal places
    rounded = dataframe.loc[:, (data.DATA_LABELS[1:4])].round()
    # True for all values where at least one of the angles is non 0
    comparison = (rounded == 0).all(axis=1)
    # Get the index of the first element that failed the previous comparison
    # (the first angles that are not all 0s)
    idx = comparison.idxmin()
    # drop all elements up to that point
    stripped_dataframe = dataframe.iloc[idx:].copy()
    # reset indices of dataframe
    stripped_dataframe.reset_index(inplace=True, drop=True)
    # now consider the first row in the dataframe as our intiial point
    # recalculate the time and pedal values of the all the remaining data
    # by setting the first row as our reference point
    # recalculate time:
    time_offset = stripped_dataframe[data.ID_TIME].iloc[0]
    stripped_dataframe.loc[:, data.ID_TIME] -= time_offset
    # recalculate pedal:
    pedal_offset = stripped_dataframe[data.ID_PEDAL].iloc[0] - 1
    stripped_dataframe.loc[:, data.ID_PEDAL] -= pedal_offset

    return stripped_dataframe


def clean_many_dataframes_from_initial_values(dataframe_list):
    """ Runs the clean_dataframe_from_intiial_values function on a list of dataframes """
    cleaned_dataframes = []
    for dataframe in dataframe_list:
        cleaned_dataframes.append(clean_dataframe_from_initial_values(dataframe))
    return cleaned_dataframes
