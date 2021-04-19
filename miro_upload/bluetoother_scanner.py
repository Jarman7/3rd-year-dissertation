#!/usr/bin/env python
# Tom Jarman
# Beacon MAC Addresses:
# Beacon 1 - dc:0d:30:01:14:95
# Beacon 2 - dc:0d:30:01:14:cd
# Beacon 3 - dc:0d:30:01:14:d2
from __future__ import division
from bluepy.btle import Scanner, DefaultDelegate
from sys import stdout


class BluetootherScanner:

    # Measurement calculation parameters
    SAMPLES = 10
    N = 3.8
    RSSI_0 = -59
    SCAN_DURATION = 0.3

    # Beacon MAC addresses
    BEACON_1 = "dc:0d:30:01:14:95"
    BEACON_2 = "dc:0d:30:01:14:cd"
    BEACON_3 = "dc:0d:30:01:14:d2"

    def __init__(self):
        self.scanner = Scanner()


    def compute_average(self):
        avg_distance = [0,0,0]
            
        for i in range(self.SAMPLES):
            devices = self.scanner.scan(self.SCAN_DURATION)
            
            for dev in devices:
                if (dev.addr == self.BEACON_1):
                    avg_distance[0] += 10**((self.RSSI_0-dev.rssi)/(10*self.N))

                elif (dev.addr == self.BEACON_2):
                    avg_distance[1] += 10**((self.RSSI_0-dev.rssi)/(10*self.N))

                elif (dev.addr == self.BEACON_3):
                    avg_distance[2] += 10**((self.RSSI_0-dev.rssi)/(10*self.N))

        avg_distance[0] /= self.SAMPLES
        avg_distance[1] /= self.SAMPLES
        avg_distance[2] /= self.SAMPLES

        return avg_distance


if __name__ == "__main__":
    scanner = BluetootherScanner()
    print(scanner.compute_average())