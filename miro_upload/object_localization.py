#!/usr/bin/env python
# Tom Jarman

from bluetoother_scanner import BluetootherScanner

# ROS Imports
import rospy
from std_msgs.msg import UInt16
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

# Miro Imports
import miro2 as miro2
from miro2.utils import wheel_speed2cmd_vel
from miro2.interface import vision

import os
import math
import time


class Client:

    # Program constants
    # Collision detection variables
    touched_body = False
    touched_head = False
    object_close = False
    resolve_collision = False
    RESOLVE_COLLISION_ITERATIONS = 300
    resolve_collision_counter = RESOLVE_COLLISION_ITERATIONS
    # Successful navigation
    success = False
    # Rate of topic broadcast
    BROADCAST_RATE = 50
    # Caps wheel speed
    MAX_WHEEL_SPEED = 0.25
    
    # Stores sonar distance reading
    sonar_reading = None

    # Angle update timestamp
    last_angle_update = time.time()

    # Fixed beacon positions
    BEACON_1_POS = (0, 0)
    BEACON_2_POS = (1.8, 0)
    BEACON_3_POS = (0.9, 1.8)

    # ROS Constants
    TICK = 0.01    
    ROBOT_NAME = '/' + os.getenv('MIRO_ROBOT_NAME')

    # Holds robots current heading relative to origin
    angle_to_origin = 0
    
    # Target beacon
    target_beacon = 1

    
    def touch_body_callback(self, data):
        # Updates touched_body to reflect if touch body sensor is active
        if data.data > 0:
            self.touched_body = True

    def touch_head_callback(self, data):
        # Updates touched_head to reflect if touch head sensor is active
        if data.data > 0:
            self.touched_head = True

    def sonar_callback(self, data):
        # Updates object_close to reflect if sonar reads less than 5cm
        if data.range < 0.05:
            self.object_close = True
        # Updates sonar_reading to hold sonar reading
        self.sonar_reading = data.range
        

    def body_vel_callback(self, data):
        # Updates robot angle prediction by body velocity times interval
        self.angle_to_origin += (-data.twist.angular.z) * (180 / math.pi) / self.BROADCAST_RATE

        # Keeps angle positive
        if self.angle_to_origin < 0:
            self.angle_to_origin = 360 - abs(self.angle_to_origin)
        # Caps angle at 360
        else:
            self.angle_to_origin %= 360
        

    def detect_collision(self):
        # Checks if a collision has occured by checking
        # 1. Has body been touched
        # 2. Has head been touched
        # 3. Is object too close
        # Does this by checking flags set by subscribers
        if self.touched_body or self.touched_head or self.object_close:
            return True
        return False


    def reset_collision_markers(self):
        # Resets collision variables to reflect no collision
        self.touched_body = False
        self.touched_head = False
        self.object_close = False
        self.resolve_collision_counter = self.RESOLVE_COLLISION_ITERATIONS


    def reset_success_vars(self):
        # Sets success time stamp
        self.success_start = time.time()


    def move_miro(self, vel_left, vel_right):
        # Moves miro based on velocity supplied for left and right wheel
        # Velocity is given in m/s
        velocity_msg = TwistStamped()
        # Converts to cmd_vel command format
        (dr, dtheta) = wheel_speed2cmd_vel([vel_left, vel_right])
        # Sets linear velocity
        velocity_msg.twist.linear.x = dr
        # Sets angular velocity
        velocity_msg.twist.angular.z = dtheta
        # Publishes velocity message
        self.velocity.publish(velocity_msg)
    

    def resolve_collision(self):
        # Resolved collision by reversing for set number of iterations
        # Checks if set number of iterations has been reached
        if self.resolve_collision_counter > 0:
            # Reverse
            self.move_miro(-0.1, -0.1)
            self.resolve_collision_counter -= 1
        
        # If the number of iterations has been reached then stop avoiding collision
        else:
            self.move_miro(0,0)
            self.resolve_collision_counter = 0
            self.resolve_collision = False


    def compute_magnitude_and_angle(self, coords, beacon):
        # Uses robot position and beacon coordinates passed to calculate vector between
        # beacon and robot

        # Calculates difference in components
        x_diff = coords[0] - beacon[0]
        y_diff = coords[1] - beacon[1]

        # Calculates magnitude of difference between positions (hypotinuse)
        mag = math.sqrt((x_diff ** 2) + (y_diff ** 2))
        # Calculates angle between beacon and robot using trigonometry
        ang = math.asin(abs(x_diff) / mag) * (180 / math.pi)

        # Map angle onto angle from origin from MiRO's position
        if (x_diff > 0):
            if (y_diff < 0):
                ang = 360 - ang

            elif (y_diff > 0):
                ang += 180
            
        if (x_diff < 0):
            if (y_diff > 0):
                ang = 180 - ang

        return [mag, ang]


    def transform_magnitudes(self, vectors):
        # Uses y = 1 / (8x + 2) equation to transform magnitudes of vectors

        scaled_vectors = []
        
        # For each vector passed
        for i in range(len(vectors)):
            # Scale the magnitude
            scaled_mag = 1 / ((8 * vectors[i][0]) + 2)
            
            # Cap magnitude at 0
            if i == self.target_beacon:
                scaled_vectors.append([max(scaled_mag, 0), vectors[i][1]])

            # Cap magnitude at 0.25
            else:
                scaled_vectors.append([min(scaled_mag, 0.25), vectors[i][1]])

        return scaled_vectors   


    def get_resultant_vector(self, vectors, position):
        # Uses potential fields to calculate the resultant vector on the robot
        x = 0
        y = 0
        # For each vector
        for vector in vectors:
            # Normalise the angle to be less than 90
            normalised_ang = vector[1] % 90
            # Use trigonometry to calculate the x and y components
            comp_1 = math.sin(normalised_ang) * vector[0]
            comp_2 = math.cos(normalised_ang) * vector[0]
            
            # Give the components direction based on angle of effect on robot
            if vector[1] > 90 and vector[1] <= 180:
                x += comp_2
                y -= comp_1

            elif vector[1] > 180 and vector[1] <= 270:
                x -= comp_1
                y -= comp_2

            elif vector[1] > 270:
                x -= comp_2
                y += comp_1

            else:
                x += comp_1
                y += comp_2

        # Return the magnitude and angle of the resultant vector
        return self.compute_magnitude_and_angle([x,y], position)


    def find_turn_angle(self, turn):
        # Takes in a turn angle and converts it to an angle less than 180
        # Corresponding to right or left turn
        # Gets rid of minus signs
        if turn > 180:
            turn_angle = turn - 180

        elif turn < 180 and turn > 0:
            turn_angle = turn

        elif turn < 0 and turn > -180:
            turn_angle = abs(turn)

        elif turn < -180:
            turn_angle = abs(turn + 180)
                
        return turn_angle


    def calculate_wheel_speed(self):
        # Calculates left and right wheel velocity based on angle
        position = self.scanner.get_position()

        beacon_1_vector = self.compute_magnitude_and_angle(position, self.BEACON_1_POS)
        beacon_2_vector = self.compute_magnitude_and_angle(position, self.BEACON_2_POS)
        beacon_3_vector = self.compute_magnitude_and_angle(position, self.BEACON_3_POS)
        beacon_vectors = [beacon_1_vector, beacon_2_vector, beacon_3_vector]


        vectors = self.transform_magnitudes([beacon_1_vector, beacon_2_vector, beacon_3_vector])

        # Use potential field navigation with unit vectors
        #resultant_vector = self.get_resultant_vector(vectors, position) 

        # Use heading navigation
        turn = beacon_vectors[self.target_beacon][1] - self.angle_to_origin
        distance_to_target = beacon_vectors[0]
        # Calculate turn angle
        turn_angle = self.find_turn_angle(turn) 

        # Turn right
        if (turn < 0  and turn > -180) or turn > 180:
            # Modifies left wheel velocity based on turn angle
            left_speed = (0.25 - round(0.25 * (turn_angle / 180), 2)) - 0.12
            right_speed = 0.25

        # Turn right
        else: 
            # Modifies right wheel velocity based on turn angle
            right_speed = (0.25 - round(0.25 * (turn_angle / 180), 2)) - 0.12
            left_speed = 0.25

        return (left_speed, right_speed), distance_to_target


    def take_navigation_input(self):
        # Takes navigation input from the console
        self.target_beacon = int(input('Which beacon would you like to navigate to? (1, 2, 3)')) - 1
        while self.target_beacon not in [0, 1, 2]:
            self.target_beacon = int(input('Which beacon would you like to navigate to? (1, 2, 3)')) - 1
            print self.target_beacon
        self.success = False


    def check_arrived(self, distance_to_target):
        # Checks if sonar reading is less than 20cm or distance to target beacon is less than 20cm
        if self.sonar_reading < 0.2 or distance_to_target < 0.2: 
            self.success = True
            self.reset_success_vars()


    def __init__(self):
        # Sets up ros node
        rospy.init_node('object_localization', anonymous=True)
        rospy.sleep(2.0)
        print('Initializing')

        # Sensor subscribers
        self.touch_body = rospy.Subscriber(self.ROBOT_NAME + '/sensors/touch_body', UInt16, self.touch_body_callback, queue_size=1, tcp_nodelay=True)
        self.touch_head = rospy.Subscriber(self.ROBOT_NAME + '/sensors/touch_head', UInt16, self.touch_head_callback, queue_size=1, tcp_nodelay=True)
        self.sonar = rospy.Subscriber(self.ROBOT_NAME + '/sensors/sonar', Range, self.sonar_callback, queue_size=1, tcp_nodelay=True)
        self.body_vel = rospy.Subscriber(self.ROBOT_NAME + '/sensors/body_vel', TwistStamped, self.body_vel_callback, queue_size=1, tcp_nodelay=True)

        # Control subscribers
        self.velocity = rospy.Publisher(self.ROBOT_NAME + '/control/cmd_vel', TwistStamped, queue_size=1, tcp_nodelay=True)

        # Create scanner object
        self.scanner = BluetootherScanner()


    def main_loop(self):
        # Robot Mainloop
        # Debug to show looping
        print "Looping"

        # While the rospy code isn't shutdown, run mainloop
        while not rospy.core.is_shutdown():

            # If collision is detected, resolve
            if self.detect_collision():
                # Debug print state
                print "Collision"
                self.reset_collision_markers()
                self.resolve_collision = True

            # Resolve collision
            elif self.resolve_collision:
                # Debug print state
                print "Resolving Collision"
                self.resolve_collision()

            # If successful navigation
            elif self.success:
                # Debug print state
                print "Success"
                # Turn robot to face away from beacon
                if (time.time() - self.success_start) < 2:
                    self.move_miro(-0.25, 0.25)

                else:
                    # Change target beacon
                    # Takes input from user
                    #self.take_navigation_input()
                    # Automatically changes target beacon
                    self.target_beacon += 1
                    self.target_beacon %= 3
                    self.success = False

            else:
                # Debug print state
                print "Navigating"
                # Calculate wheel speed
                (left_speed, right_speed), distance_to_target = self.calculate_wheel_speed()
                # Move miro
                self.move_miro(left_speed, right_speed)
                # Check if MiRO is at target beacon
                self.check_arrived(distance_to_target)

            # Small pause
            rospy.sleep(self.TICK)
            

# Run program
if __name__ == "__main__":
    client = Client()
    client.main_loop()