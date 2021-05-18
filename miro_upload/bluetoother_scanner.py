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

    # Number of samples used in rolling average
    SAMPLES = 100

    # Signal decay exponent
    #N = [1.91, 3.73, 3] # Home Configuration
    N = [2.3, 3.73, 3.5] # DiaLab Configuration
    # Reference Distances
    RSSI_0 = [-63.4, -65.7, -59.7]
    # Signal scan period
    SCAN_DURATION = 0.1

    # Beacon MAC addresses
    BEACON_1 = "dc:0d:30:01:14:95"
    BEACON_2 = "dc:0d:30:01:14:cd"
    BEACON_3 = "dc:0d:30:01:14:d2"

    # Fixed beacon coordinates, meters
    BEACON_1_POS = (0,0)
    BEACON_2_POS = (1.8,0)
    BEACON_3_POS = (0.9,1.8)

    # Rolling average variables
    rolling_averages = [deque(),deque(),deque()]

    # Setup
    def __init__(self):
        self.scanner = Scanner()


    def compute_average(self):
        # Uses RSSI decay and rolling averages to ouput distances to each beacon

        # Scan for devices
        devices = self.scanner.scan(self.SCAN_DURATION)

        # Outputs 0 by default if beacon isn't detectable
        avg_dist_1 = 0
        avg_dist_2 = 0
        avg_dist_3 = 0
        
        # Iterate through detected devices
        for dev in devices:
            # Beacon 1 update rolling averages
            # Calculates distance to beacon using RSSI_0, RSSI and N
            # Adds sample to rolling average queue
            # Reduces queue size if size limit is reached
            if (dev.addr == self.BEACON_1):
                self.rolling_averages[0].append(10 ** ((self.RSSI_0[0] - dev.rssi) / (10 * self.N[0])))
                if len(self.rolling_averages[0]) > self.SAMPLES:
                    self.rolling_averages[0].popleft()

            # Beacon 2 update rolling averages
            # Calculates distance to beacon using RSSI_0, RSSI and N
            # Adds sample to rolling average queue
            # Reduces queue size if size limit is reached
            elif (dev.addr == self.BEACON_2):
                self.rolling_averages[1].append(10 ** ((self.RSSI_0[1] - dev.rssi) / (10 * self.N[1])))
                if len(self.rolling_averages[1]) > self.SAMPLES:
                    self.rolling_averages[1].popleft()

            # Beacon 3 update rolling averages
            # Calculates distance to beacon using RSSI_0, RSSI and N
            # Adds sample to rolling average queue
            # Reduces queue size if size limit is reached
            elif (dev.addr == self.BEACON_3):
                self.rolling_averages[2].append(10 ** ((self.RSSI_0[2] - dev.rssi) / (10 * self.N[2])))
                if len(self.rolling_averages[2]) > self.SAMPLES:
                    self.rolling_averages[2].popleft()

        # Calculates average of queue elements and assigns to output variable
        if (len(self.rolling_averages[0]) > 0):
            avg_dist_1 = round(sum(self.rolling_averages[0]) / len(self.rolling_averages[0]), 2)

        if (len(self.rolling_averages[1]) > 0):
            avg_dist_2 = round(sum(self.rolling_averages[1]) / len(self.rolling_averages[1]), 2)

        if (len(self.rolling_averages[2]) > 0):
            avg_dist_3 = round(sum(self.rolling_averages[2]) / len(self.rolling_averages[2]), 2)

        return [avg_dist_1, avg_dist_2, avg_dist_3]


    def get_position(self):
        # Uses circle geometry and trilateration to calculate robot position
        # in relative coordiante system

        # Gets beacon distances
        distances = self.compute_average()

        # Sets distances to appropriate algebra variables
        a = distances[0]
        b = distances[1]
        c = distances[2]

        # Sets distances to appropriate algebra variables
        p = self.BEACON_2_POS[0]
        q = self.BEACON_3_POS[0]
        r = self.BEACON_3_POS[1]

        # Calculates relative (x,y) coordinate using circle geometry
        x = ((a**2)-(b**2)+(p**2))/(2*p)
        y = (((a**2)-(c**2)+(q**2)+(r**2))/(2*r))-((q/r)*x)

        # Rounds to 2 dp
        return (round(x, 2),round(y, 2))


    def get_beacon_rssi_avg(self, mac_addr):
        # Used to get beacon RSSI average for single beacon
        SAMPLES = 50
        rssi_avg = deque()

        # For SAMPLES iterations
        for i in range(SAMPLES):
            # Detect Devices
            devices = self.scanner.scan(self.SCAN_DURATION)
            # Find device with mac_addr MAC address and add rssi reading to avg
            for dev in devices:
                if dev.addr == mac_addr:
                    rssi_avg.append(dev.rssi)

        # Output average
        return sum(rssi_avg) / len(rssi_avg)


    def train_parameters(self):
        # Training program used to calculate parameters for distance reading
        # Initial reference distances are taken
        # Then for each beacon a set of distances are used to get rssi readings
        # At each distance a certain number of samples is taken
        # Parameters are calculated for each beacon and output

        # Training parameters
        rssi_0 = []
        n = []
        # Distances measured at (m)
        distances = [0.25, 0.75, 1.25, 1.75]
        beacons = [self.BEACON_1, self.BEACON_2, self.BEACON_3]
        # Number of measurements taken at each distance
        samples_per_distance = 4

        # Output configuration
        print "Parameter training:"
        print "The system will specify the beacon and distance measure"
        print "There will be " + str(samples_per_distance) + " measurements per distance"
        print "There will be " + str(distances) + " distance checks\n"

        # For each beacon
        for beacon in range(3):
            # Reset measurement variables
            n_values = []
            rssi_0_measurements = []

            # Get reference reading
            print "Beacon " + str(beacon + 1)
            print "First lets get an RSSI_0 measurement"
            for sample in range(samples_per_distance):
                print "Distance check 1m"
                print "Sample " + str(sample + 1)
                raw_input("Enter when ready")
                rssi_0_measurements.append(self.get_beacon_rssi_avg(beacons[beacon]))

            # Calculate average RSSI_0 reference reading
            rssi_0.append(sum(rssi_0_measurements) / samples_per_distance)
            print "RSSI_0 = " + str(rssi_0[beacon])

            # Training the exponent
            print "Now to train the exponent"
            # For each distance
            for distance in range(len(distances)):
                n_values_per_dist = []
                print "Beacon " + str(beacon + 1)
                print "Distance check " + str(distances[distance])
                # For the number of samples per distance
                for sample in range(samples_per_distance):
                    print "Sample " + str(sample + 1) + "\n"
                    raw_input("Enter when ready")
                    # Calculate exponent
                    n_val = (rssi_0[beacon] - self.get_beacon_rssi_avg(beacons[beacon])) / (10 * math.log10(distances[distance]))
                    # Add to storage variable
                    n_values_per_dist.append(n_val)
                # Calculate average for this distance
                n_values.append(sum(n_values_per_dist) / len(n_values_per_dist))
            # Calculate average for beacon
            n.append(sum(n_values) / len(n_values))

        # Output results
        print "Results"
        print "Beacon 1:\nn = " + str(n[0]) + "\nRSSI_0 = " + str(rssi_0[0])
        print "Beacon 2:\nn = " + str(n[1]) + "\nRSSI_0 = " + str(rssi_0[1])     
        print "Beacon 3:\nn = " + str(n[2]) + "\nRSSI_0 = " + str(rssi_0[2])     


# Test area
if __name__ == "__main__":
    scanner = BluetootherScanner()
    while True:
        print scanner.compute_average() , scanner.get_position()