#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This is a data_logger class. Subscribes the IMU readings and save them into a CSV fileself.

"""

# imports
from queue import Queue, Empty
import threading
import csv
import data


# Private Globals
_DATA = Queue()
_DATA_LOGGER = None
_logging_enabled = False


# Functions
def log_metrics(te, tg, pitch, roll, yaw, mark):
    if _DATA_LOGGER.running:
        new_data = [tg, te, pitch, roll, yaw, mark]
        _DATA.put(new_data)
        print new_data
    else:
        return


def enable_logging():
    global _DATA_LOGGER
    global _logging_enabled
    _logging_enabled = True
    _DATA_LOGGER = DataLogger()  # thread here
    _DATA_LOGGER.start()
    print "enable_logging"


def disable_logging():
    global _logging_enabled
    if _logging_enabled:
        if _DATA_LOGGER.running:
            _DATA_LOGGER.stop()
        _logging_enabled = False


class DataLogger(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
        self.filename = data.get_new_filename()
        self.fp = open(self.filename, 'w')
        self.writer = csv.writer(self.fp, lineterminator='\n')
        # write the header of the CSV file (the labels of each field/feature)
        self.writer.writerow(data.DATA_LABELS)
        self.running = True

    def run(self):
        while self.running:
            try:
                row = _DATA.get(timeout=1)
                self.writer.writerow(row)
            except Empty:
                continue
        self.close()

    def close(self):
        if self.fp is not None:
            self.fp.close()
            self.fp = None

    def stop(self):
        self.running = False

    def __del__(self):
        self.close()
