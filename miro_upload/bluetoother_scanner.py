#!/usr/bin/env python
# Tom Jarman
# Beacon MAC Addresses:
# Beacon 1 - dc:0d:30:01:14:95
# Beacon 2 - dc:0d:30:01:14:cd
# Beacon 3 - dc:0d:30:01:14:d2
from __future__ import division
from bluepy.btle import Scanner, DefaultDelegate
from sys import stdout
from collections import deque


class BluetootherScanner:

    # Measurement calculation parameters
    SAMPLES = 30
    N = 3.8
    RSSI_0 = -59
    SCAN_DURATION = 0.1

    # Beacon MAC addresses
    BEACON_1 = "dc:0d:30:01:14:95"
    BEACON_2 = "dc:0d:30:01:14:cd"
    BEACON_3 = "dc:0d:30:01:14:d2"

    # Base distances
    BEACON_1_POS = (0,0)
    BEACON_2_POS = (1.8,0)
    BEACON_3_POS = (0.9,1.8)

    # Variables
    rolling_averages = [deque(),deque(),deque()]

    def __init__(self):
        self.scanner = Scanner()


    def compute_average(self):
        devices = self.scanner.scan(self.SCAN_DURATION)
        avg_dist_1 = 0
        avg_dist_2 = 0
        avg_dist_3 = 0
        
        for dev in devices:
            if (dev.addr == self.BEACON_1):
                self.rolling_averages[0].append(10**((self.RSSI_0-dev.rssi)/(10*self.N)))
                if len(self.rolling_averages[0]) > self.SAMPLES:
                    self.rolling_averages[0].popleft()

            elif (dev.addr == self.BEACON_2):
                self.rolling_averages[1].append(10**((self.RSSI_0-dev.rssi)/(10*self.N)))
                if len(self.rolling_averages[1]) > self.SAMPLES:
                    self.rolling_averages[1].popleft()

            elif (dev.addr == self.BEACON_3):
                self.rolling_averages[2].append(10**((self.RSSI_0-dev.rssi)/(10*self.N)))
                if len(self.rolling_averages[2]) > self.SAMPLES:
                    self.rolling_averages[2].popleft()

        if (len(self.rolling_averages[0]) > 0):
            avg_dist_1 = sum(self.rolling_averages[0]) / len(self.rolling_averages[0])

        if (len(self.rolling_averages[1]) > 0):
            avg_dist_2 = sum(self.rolling_averages[1]) / len(self.rolling_averages[1])

        if (len(self.rolling_averages[2]) > 0):
            avg_dist_3 = sum(self.rolling_averages[2]) / len(self.rolling_averages[2])

        # Finish this
        return [avg_dist_1, avg_dist_2, avg_dist_3]

    def get_position(self):
        distances = self.compute_average()

        a = distances[0]
        b = distances[1]
        c = distances[2]

        p = self.BEACON_2_POS[0]
        q = self.BEACON_3_POS[0]
        r = self.BEACON_3_POS[1]

        x = ((a**2)-(b**2)+(p**2))/(2*p)
        y = (((a**2)-(c**2)+(q**2)+(r**2))/(2*r))-((q/r)*x)

        return (x,y)


if __name__ == "__main__":
    scanner = BluetootherScanner()
    while True:
        print scanner.get_position()