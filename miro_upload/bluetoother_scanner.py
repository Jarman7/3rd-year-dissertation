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
import math


class BluetootherScanner:

    # Measurement calculation parameters
    SAMPLES = 100
    #N = 3.5
    #RSSI_0 = -65
    #N = [1.91, 3.73, 1.68]
    #N = [1.91, 3.73, 3]
    N = [2.3, 3.73, 3.5]
    
    RSSI_0 = [-63.4, -65.7, -59.7]
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
                self.rolling_averages[0].append(10 ** ((self.RSSI_0[0] - dev.rssi) / (10 * self.N[0])))
                if len(self.rolling_averages[0]) > self.SAMPLES:
                    self.rolling_averages[0].popleft()

            elif (dev.addr == self.BEACON_2):
                self.rolling_averages[1].append(10 ** ((self.RSSI_0[1] - dev.rssi) / (10 * self.N[1])))
                if len(self.rolling_averages[1]) > self.SAMPLES:
                    self.rolling_averages[1].popleft()

            elif (dev.addr == self.BEACON_3):
                self.rolling_averages[2].append(10 ** ((self.RSSI_0[2] - dev.rssi) / (10 * self.N[2])))
                if len(self.rolling_averages[2]) > self.SAMPLES:
                    self.rolling_averages[2].popleft()

        if (len(self.rolling_averages[0]) > 0):
            avg_dist_1 = round(sum(self.rolling_averages[0]) / len(self.rolling_averages[0]), 2)

        if (len(self.rolling_averages[1]) > 0):
            avg_dist_2 = round(sum(self.rolling_averages[1]) / len(self.rolling_averages[1]), 2)

        if (len(self.rolling_averages[2]) > 0):
            avg_dist_3 = round(sum(self.rolling_averages[2]) / len(self.rolling_averages[2]), 2)

        # Finish this
        return [avg_dist_1, avg_dist_2, avg_dist_3]


    def get_position(self):
        distances = self.compute_average()

        a = distances[0]
        #print "Distance 1 -" + str(a)
        b = distances[1]
        #print "Distance 2 -" + str(b)
        c = distances[2]
        #print "Distance 3 -" + str(c)

        p = self.BEACON_2_POS[0]
        q = self.BEACON_3_POS[0]
        r = self.BEACON_3_POS[1]

        x = ((a**2)-(b**2)+(p**2))/(2*p)
        y = (((a**2)-(c**2)+(q**2)+(r**2))/(2*r))-((q/r)*x)

        return (round(x, 2),round(y, 2))


    def get_beacon_rssi_avg(self, mac_addr):
        SAMPLES = 50
        rssi_avg = deque()

        for i in range(SAMPLES):
            devices = self.scanner.scan(self.SCAN_DURATION)

            for dev in devices:
                if dev.addr == mac_addr:
                    rssi_avg.append(dev.rssi)

        return sum(rssi_avg) / len(rssi_avg)


    def train_parameters(self):
        rssi_0 = []
        n = []
        distances = [0.25, 0.75, 1.25, 1.75]
        #distances = [0.25, 0.75]
        beacons = [self.BEACON_1, self.BEACON_2, self.BEACON_3]

        samples_per_distance = 4

        print "Parameter training:"
        print "The system will specify the beacon and distance measure"
        print "There will be " + str(samples_per_distance) + " measurements per distance"
        print "There will be " + str(distances) + " distance checks\n"

        for beacon in range(3):
            n_values = []
            rssi_0_measurements = []

            print "Beacon " + str(beacon + 1)
            print "First lets get an RSSI_0 measurement"

            for sample in range(samples_per_distance):
                print "Distance check 1m"
                print "Sample " + str(sample + 1)
                raw_input("Enter when ready")
                rssi_0_measurements.append(self.get_beacon_rssi_avg(beacons[beacon]))

            rssi_0.append(sum(rssi_0_measurements) / samples_per_distance)
            print "RSSI_0 = " + str(rssi_0[beacon])

            print "Now to train the exponent"
            
            for distance in range(len(distances)):
                n_values_per_dist = []
                print "Beacon " + str(beacon + 1)
                print "Distance check " + str(distances[distance])

                for sample in range(samples_per_distance):
                    print "Sample " + str(sample + 1) + "\n"
                    raw_input("Enter when ready")
                    n_val = (rssi_0[beacon] - self.get_beacon_rssi_avg(beacons[beacon])) / (10 * math.log10(distances[distance]))
                    print(n_val)
                    n_values_per_dist.append(n_val)
                
                n_values.append(sum(n_values_per_dist) / len(n_values_per_dist))
            n.append(sum(n_values) / len(n_values))

        print "Results"
        print "Beacon 1:\nn = " + str(n[0]) + "\nRSSI_0 = " + str(rssi_0[0])
        print "Beacon 2:\nn = " + str(n[1]) + "\nRSSI_0 = " + str(rssi_0[1])     
        print "Beacon 3:\nn = " + str(n[2]) + "\nRSSI_0 = " + str(rssi_0[2])     


if __name__ == "__main__":
    scanner = BluetootherScanner()
    while True:
        print scanner.compute_average() , scanner.get_position()