#! /usr/bin/env python
# -*- coding: utf-8 -*-

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Copyright 2016 Achim Sack, Christian Scholz


import numpy as np  # numpy is used to save the data to a file
import matplotlib.pyplot as plt  # pyplot is used to plot the result
import time

import MieScattering as Mie  # the main libraryto control the experiment

# Setup
# The angles are angles after calibration
start_angle = -90.  # -90...90
stop_angle = 90.  # -90...90
step_angle = 1.		# 1...180

# how many samples to average during normal measurement
NumAverage = 200

# File to save the final data
outfile_parallel = "Mie_scattering_parallel.dat"
outfile_perpendicular = "Mie_scattering_perpendicular.dat"

#####################################################################
# Open Mie scattering device:
device = Mie.MieScattering()
plt.ion()  # set pyplot to interactive mode (allows zooming)


# Calibrate lower servo: Enable Laser to help align the grating
device.LaserOn()  # enable LASER
# wait for user to hit return
input("Place 1.5 um grating in beam, align\nsuch that pattern is horizontally.\nHit return to start calibration!")
device.LaserOff()  # disable LASER
device.calibrate_angle()  # perform a calibration of the lower servo


# Calibrate upper servo: Enable Laser to help aligning
device.LaserOn()  # enable LASER
device.moveTo(x_pos=91)  # move detector close to center position
input("Insert a scattering cell\nHit return to start polarizer calibration!")
device.LaserOff()  # disable LASER
# perform a calibration of the polarizer
b, intensity_pol = device.calibrate_polarizer()
plt.plot(intensity_pol)
# I did not find a good way to reliably determin the 0°, 90° and
# open position from these measurements, therefore ask the user
# to enter this data

device.Cal_Y_perpend = float(input(
    "Find the minimum of the sinusoidal part on the plot and enter the x-value: "))
device.Cal_Y_parallel = float(input(
    "Find the maximum of the sinusoidal part on the plot and enter the x-value: "))

# Main measurement

# manually choose one polarizer orientation:
# device.polarizer_parallel()
# device.polarizer_perpendicular()
# device-moveTo(y_pos=111.)


# move polarizer parallel to detector plane
device.polarizer_parallel_to_detector()

device.LaserOn()  # enable LASER
# wait for user to hit return
input("Place sample in beam.\nHit return to start measurement!")
device.LaserOff()  # disable LASER


print("Measuring parallel component of scattering intensity")

# integrate over #NumAverage samples per position
device.setNumsamples(NumAverage)
time.sleep(1)
# measure scattering intensity
intensity = device.scan_angle(start_angle, stop_angle, step_angle)

np.savetxt(outfile_parallel, intensity,
           header="#angle\tintensity\n#[deg]\t arb.u.")  # save to file

plt.cla()  # clear axis
# plot parallel scattering intensity
plt.plot(intensity[:, 0], intensity[:, 1])

print("Measuring perpendicular component of scattering intensity")

# move polarizer perpendicular to detector plane
device.polarizer_perpendicular_to_detector()
# measure scattering intensity
intensity = device.scan_angle(stop_angle, start_angle, step_angle)

np.savetxt(outfile_perpendicular, intensity,
           header="#angle\tintensity\n#[deg]\t arb.u.")  # save to file

# plot perpendicular scattering intensity
plt.plot(intensity[:, 0], intensity[:, 1])

device.moveTo(x_pos=90)  # move detector back to center

input()
