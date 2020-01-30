#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
from fibre import Logger, Event
from odrive.utils import OperationAbortedException
from fibre.protocol import ChannelBrokenException
import sys
import numpy as np
import matplotlib.pyplot as plt

def get_odrive(shutdown_token):
    """
    Look for and return an odrive connected via usb
    """

    print('Looking for ODrive...')
    odrv = odrive.find_any(search_cancellation_token=app_shutdown_token, channel_termination_token=app_shutdown_token)
    print('Found.')
    return odrv

def main(app_shutdown_token):
    """
    !! Main program !!
    Looks for odrive, then calibrates, then sets gains, then tests motors
    WARNING: Saving more than twice per boot will cause a reversion of all changes
    """

    odrv0 = get_odrive(app_shutdown_token)

    input("Press Enter to continue with bandwidth test...")

    ##### Loop Parameters #####
    tspan = 5.0

    update_freq = 800.0 # -> 500Hz gives 0.002s to 0.00207s period
    n = int(tspan*update_freq)
    start_time = time.time()
    intervals = np.zeros((n,))
    prev_time = time.time()
    i=0

    ##### Current control parameters #####
    current_commands = np.zeros((n,))
    current_amplitude = 4.0 # amps
    min_current = 4.0
    current_average = current_amplitude + min_current
    signal_freq = 3.0 # Hz

    while(i<n):
        now = time.time()
        elapsed = now - start_time
        if now >= start_time + (i+1)*1.0/update_freq:
            period = now - prev_time
            prev_time = now     
            intervals[i] = period

            current_command = current_average + current_amplitude * np.sin(2*np.pi*signal_freq*elapsed)
            current_command = round(current_command,3)
            current_commands[i] = current_command


            odrv0.axis0.controller.set_current_setpoint(current_command)

            i += 1
            if i%10 == 0:
                print("I=%s \t t=%s \tdt=%s"%(current_command,elapsed,period))

    odrv0.axis0.controller.set_current_setpoint(0.0)
    # plt.figure()
    # plt.plot(current_commands)
    # plt.figure()
    # plt.plot(intervals)
    # plt.show()



app_shutdown_token = Event()
try:
    main(app_shutdown_token)
    # init_odrive(odrv0)
except OperationAbortedException:
    logger.info("Operation aborted.")
finally:
    app_shutdown_token.set()

# encoder calibration
# set use_index ot true
# request encoder_index_search
# pre_calibrated to True
# then change startup to encoder_index_search true