
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
# Copyright 2016 Achim Sack

"""
                                -*- Notes -*-
1 - don't forget to compile and upload the arduino file to the 
    uno cache before attempting to run any python code.

"""


import platform
import glob
import numpy as np
import matplotlib.pyplot as plt
import time
import serial

ArduMie_baudrate = 9600


class MieScattering:

    def __init__(self, ComPort=None):
        self.ComHndl = None
        if ComPort == None:  # if no ComPort given: scan for devices
            device = self.find_device()
            if device == "":
                print("No device found.")
                exit()
            ComPort = device  # choose first device if multiple are found

        self.ComHndl = serial.Serial(
            ComPort, ArduMie_baudrate, timeout=5, stopbits=serial.STOPBITS_TWO, interCharTimeout=0.1)
        time.sleep(2)  # wait for Arduino to exit reset state

        # calibration data
        self.Cal_X_success = False
        self.Cal_X_offset = 0.
        self.Cal_X_scaling = 1.

        self.Cal_Y_success = False
        self.Cal_Y_parallel = 0.
        self.Cal_Y_perpend = 0.

        self.NumSamples = None
        # set device to average over ten sample at a time
        self.setNumsamples(10)
        # how long to wait for one sample (windows timeout bug)
        self.wait_for_sample = 1e-2

        # position data
        self.Xpos = 90
        self.Ypos = 180  # assume: 180 is the clear position
        self.moveTo(self.Xpos, self.Ypos)  # update position

    def __del__(self):
        if not self.ComHndl == None:
            self.ComHndl.close()

    def find_device(self, ping_string=str.encode("ArduMie")):
        OperatingSystem = platform.system()  # return a string like "Linux", etc
        if OperatingSystem == "Windows":
            ports = [f'COM{(i + 1)}s' for i in range(256)]  # after change!!
        elif OperatingSystem == "Linux":
            ports = glob.glob("/dev/tty[A-Za-z]*")
        else:
            raise EnvironmentError("Unsupported platform")
        device = ""
        for port in ports:
            # Try to open the port
            try:
                ser = serial.Serial(port, ArduMie_baudrate, timeout=0.2)
                # print port
                time.sleep(2)  # wait for Arduino to exit reset state
                ser.write(b"p\r")
                ping = ser.readline()[:-1]
                ser.close()
                # print ping
                if ping_string in ping:
                    device = port
                    break
            except (OSError, serial.SerialException):
                pass
        return device

    def ask(self, query):
        # attach terminate char to string and send
        self.ComHndl.write(str.encode(query+"\r"))
        time.sleep(.05)
        if platform.system() == "Windows" and query == "m":
            # pyserial Windows bug?
            time.sleep(self.NumSamples * self.wait_for_sample)
        answer = self.ComHndl.readline()  # read answer from Arduino
        # print "q:",query," a:",answer, "l:", len(answer)
        return answer[:-1]  # return answer without newline

    def moveTo(self, x_pos=None, y_pos=None):
        if not x_pos == None:
            self.ask(f"x{int(x_pos+.5)}i")  # update x-position
            self.Xpos = int(x_pos+.5)

        if not (x_pos == None or y_pos == None):
            time.sleep(.5)

        if not y_pos == None:
            self.ask(f"y{int(y_pos+.5)}i")  # update y-position
            self.Ypos = int(y_pos+.5)

    def setNumsamples(self, N):  # average over how many samples (arduino-internally)
        self.NumSamples = N
        self.ask(f"N{int(N)}i")  # update y-position

    def measure(self):  # return sum over N sampes
        answer = self.ask("m")			# query measurement
        # print answer
        data = float(answer.split(b":")[-1])		# extract data from answer
        return data

    def LaserOn(self):  # enable the laser
        answer = self.ask("O")

    def LaserOff(self):  # disable laser
        answer = self.ask("o")

    def scan_angle(self, start_pos, stop_pos, step):
        # if a previous calibration was successfull, I assume all *_pos parameters are given in real world angles [-90°..90°]
        # so convert them to the servo coordinates:
        # x_pos_angle = np.arange(start_pos, stop_pos, np.sign(stop_pos-start_pos)*step)		# generate intermediate steps
        if self.Cal_X_success == True:				# convert to servo coordinates
            start_pos = start_pos/self.Cal_X_scaling + self.Cal_X_offset
            stop_pos = stop_pos/self.Cal_X_scaling + self.Cal_X_offset
            step = step/self.Cal_X_scaling

        x_pos = np.arange(start_pos, stop_pos, np.sign(
            stop_pos-start_pos)*step)		# generate intermediate steps
        a = np.zeros((len(x_pos), 2))					# array for measured data
        self.moveTo(x_pos=start_pos)				# move to initial position
        time.sleep(0.5)						# and wait for it to get there

        for i, x in enumerate(x_pos):			# for each point:
            self.moveTo(x_pos=x)				# update x-position
            time.sleep(0.1)					# wait a while for servo to move there

            data = self.measure() 				# query measurement
         # a[i] = x_pos_angle[i],data					# save position and data in output array
            # save position and data in output array
            a[i] = (float(int(x+.5))-self.Cal_X_offset) * \
                self.Cal_X_scaling, data

            # <
            # !!>
            print("Pos: %.1f\tInten: %i" %
                  (a[i, 0], a[i, 1])) 	# output current data

        return a				# return data

    def scan_polarizer(self, start_pos, stop_pos, step):
        # if a previous calibration was successfull, I assume all *_pos parameters are given in real world angles [-90°..90°]
        # so convert them to the servo coordinates:
        if self.Cal_Y_success == True:
            start_pos = start_pos/self.Cal_Y_scaling + self.Cal_Y_offset
            stop_pos = stop_pos/self.Cal_Y_scaling + self.Cal_Y_offset
            step = step/self.Cal_Y_scaling

        # generate intermediate steps
        y_pos = np.arange(start_pos, stop_pos, step)
        a = np.zeros((len(y_pos), 2))					# array for measured data
        self.moveTo(y_pos=start_pos)				# move to initial position
        time.sleep(0.5)						# and wait for it to get there

        for i, y in enumerate(y_pos):			# for each point:
            self.moveTo(y_pos=y)				# update x-position
            time.sleep(0.1)					# wait a while for servo to move there

            data = self.measure() 				# query measurement
            a[i] = y, data					# save position and data in output array

            # <make it python3!!>
            print("Pos: %.1f\tInten: %i" %
                  (a[i, 0], a[i, 1])) 	# output current data

        return a				# return data

    def calibrate_angle(self):
        self.Cal_X_success = False			# signal that no valid calibration is available
        self.Cal_X_offset = 0.			#
        self.Cal_X_scaling = 1.			#
        self.setNumsamples(20)			# integrate 10 samples per position
        intensity = self.scan_angle(0, 180, 1)  # do a quick, full scan
        intensity[intensity[:, 1] < 0, 1] = 0
        smoothed = np.convolve(intensity[:, 1], np.ones(5)/5., mode="same")

        # determine min and max value
        m, M = np.min(smoothed), np.max(smoothed)
        # find positions where threshold is reached
        peaks = np.where(smoothed > (M-m)*0.05)[0]
        print(peaks)

        i = 0							# M
        f = []						# A
        for j in range(len(peaks)):				# G
            # print "i: %i, j: %i"%(i,j)			# I
            # C areful: do not access peaks[j+1] if j indexes already the last element in peaks
            if (j+1 == len(peaks)) or (peaks[j+1] > peaks[j]+1):
                f = np.append(f, [peaks[i], peaks[j]])		#
                i = j+1						#
        f = f.reshape([-1, 2])					#
        f = np.mean(f, 1)					# f contains the mid-positions of the peaks

        # find value closest to 90deg, this is the center peak
        g = abs(f-90.)
        h = np.where(g == min(g))[0]				#
        print(f) 						#

        # if we have at least 3 peaks and h indexes an "inner" peak
        if (h > 0) and (len(f) >= 3) and (h < len(f)):
            print(f"Found {(len(f))}i peaks")

            # < make it python3 >
            print("using peaks at: %.1f and %.1f" % (f[h-1], f[h+1]))
            # print f, h

            angle_measured = abs(f[h-1]-f[h+1])
            angle_theor = np.arcsin(635e-9/1.5e-6)*(360/(2*np.pi))*2
            self.Cal_X_scaling = angle_theor/angle_measured
            self.Cal_X_offset = (f[h-1]+f[h]+f[h+1])/3.
            self.Cal_X_success = True

            # <make it python3>
            print("offset: %.3f" % (self.Cal_X_offset))
            print("scaling: %.3f" % (self.Cal_X_scaling))
        else:
            print("Could not detect enough peaks")
            print(f"Found {(len(f))}i peaks")

        return [self.Cal_X_offset, self.Cal_X_scaling], intensity, smoothed

    def calibrate(self, data):
        calibrated_data = map(
            lambda x: [(x[0]-self.Cal_X_offset)*self.Cal_X_scaling, x[1]], data)
        return np.array(calibrated_data)

    def calibrate_polarizer(self):
        self.Cal_Y_success = False						# signal that no valid calibration is available
        self.setNumsamples(20)						# integrate 20 samples per position
        if self.Cal_X_success == True:
            # Move detector ~4 deg off axis to avoid main beam
            self.moveTo(x_pos=-4/self.Cal_X_scaling + self.Cal_X_offset)
        else:
            self.moveTo(x_pos=80)						# else: goto the uncalibrated position
        time.sleep(0.5)						# and wait for it to get there
        intensity = self.scan_polarizer(0, 180, 1)				# do a quick, full scan
        intensity[intensity[:, 1] < 0, 1] = 0
        smoothed = np.convolve(intensity[:, 1], np.ones(5)/5., mode="valid")

        m, M = np.min(smoothed[2:120]), np.max(
            smoothed[2:130])		# determine min and max value
        k, K = np.where(smoothed == m)[0], np.where(smoothed == M)[
            0]		# find positions where threshold is reached

        print(m, M, k, K)

        self.Cal_Y_parallel = K
        self.Cal_Y_perpend = k

        return [k, K], smoothed

    def polarizer_parallel(self):
        self.moveTo(y_pos=self.Cal_Y_parallel)

    def polarizer_perpendicular(self):
        self.moveTo(y_pos=self.Cal_Y_perpend)

    def polarizer_parallel_to_detector(self):
        self.moveTo(y_pos=(self.Cal_Y_parallel-0.5 *
                    abs(self.Cal_Y_parallel-self.Cal_Y_perpend)) % 180)

    def polarizer_perpendicular_to_detector(self):
        self.moveTo(y_pos=(self.Cal_Y_parallel+0.5 *
                    abs(self.Cal_Y_parallel-self.Cal_Y_perpend)) % 180)
