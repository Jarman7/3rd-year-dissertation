from bluetoother_scanner import BluetootherScanner

import rospy
import miro2 as miro2

from miro2.utils import wheel_speed2cmd_vel
from miro2.interface import vision

from std_msgs.msg import UInt16
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import os
import math
import time

from bluetoother_scanner import BluetootherScanner

class Client:

    TOUCH_BODY = False
    TOUCH_HEAD = False
    OBJECT_CLOSE = False
    RESOLVE_COLLISION = False
    RESOLVE_COLLISION_ITERATIONS = 300
    SUCCESS = False
    BROADCAST_RATE = 50
    MAX_WHEEL_SPEED = 0.25

    sonar_reading = None

    last_angle_update = time.time()
    counter = 0

    BEACON_1_POS = (0, 0)
    BEACON_2_POS = (1.8, 0)
    BEACON_3_POS = (0.9, 1.8)

    TICK = 0.01    
    ROBOT_NAME = '/' + os.getenv('MIRO_ROBOT_NAME')

    resolve_collision_counter = RESOLVE_COLLISION_ITERATIONS
    angle_to_origin = 0
    target_beacon = 1

    def touch_body_callback(self, data):
        if data.data > 0:
            self.TOUCH_BODY = True

    def touch_head_callback(self, data):
        if data.data > 0:
            self.TOUCH_HEAD = True

    def sonar_callback(self, data):
        if data.range < 0.05:
            self.OBJECT_CLOSE = True

        self.sonar_reading = data.range
        

    def body_vel_callback(self, data):
        self.counter += 1
        self.angle_to_origin += (-data.twist.angular.z) * (180 / math.pi) / self.BROADCAST_RATE

        if self.angle_to_origin < 0:
            self.angle_to_origin = 360 - abs(self.angle_to_origin)

        else:
            self.angle_to_origin %= 360

        #print "Angle to origin - " + str(self.angle_to_origin)
        

    def detect_collision(self):
        if self.TOUCH_BODY or self.TOUCH_HEAD or self.OBJECT_CLOSE:
            return True
        return False

    def reset_collision_markers(self):
        self.TOUCH_BODY = False
        self.TOUCH_HEAD = False
        self.OBJECT_CLOSE = False
        self.resolve_collision_counter = self.RESOLVE_COLLISION_ITERATIONS


    def reset_success_vars(self):
        self.success_start = time.time()


    def move_miro(self, vel_left, vel_right):
        velocity_msg = TwistStamped()
        (dr, dtheta) = wheel_speed2cmd_vel([vel_left, vel_right])

        velocity_msg.twist.linear.x = dr
        velocity_msg.twist.angular.z = dtheta

        self.velocity.publish(velocity_msg)
    

    def resolve_collision(self):
        if self.resolve_collision_counter > 0:
            self.move_miro(-0.1, -0.1)
            self.resolve_collision_counter -= 1
        
        else:
            self.move_miro(0,0)
            self.resolve_collision_counter = 0
            self.RESOLVE_COLLISION = False


    def compute_magnitude_and_angle(self, coords, beacon):
        x_diff = coords[0] - beacon[0]
        y_diff = coords[1] - beacon[1]

        mag = math.sqrt((x_diff ** 2) + (y_diff ** 2))

        ang = math.asin(abs(x_diff) / mag) * (180 / math.pi)

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
        # y = 1 / (8x + 2)
        scaled_vectors = []
        for i in range(len(vectors)):

            scaled_mag = 1 / ((8 * vectors[i][0]) + 2)
            
            if i == self.target_beacon:
                scaled_vectors.append([min(scaled_mag, 0), vectors[i][1]])

            else:
                scaled_vectors.append([max(scaled_mag, 0.25), vectors[i][1]])

        return scaled_vectors   


    def get_resultant_vector(self, vectors, position):
        x = 0
        y = 0
        counter = 0
        for vector in vectors:
            counter += 1
            normalised_ang = vector[1] % 90
            comp_1 = math.sin(normalised_ang) * vector[0]
            comp_2 = math.cos(normalised_ang) * vector[0]
            
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

        return self.compute_magnitude_and_angle([x,y], position)

    def find_turn_angle(self, turn):
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
        position = self.scanner.get_position()

        beacon_1_vector = self.compute_magnitude_and_angle(position, self.BEACON_1_POS)
        beacon_2_vector = self.compute_magnitude_and_angle(position, self.BEACON_2_POS)
        beacon_3_vector = self.compute_magnitude_and_angle(position, self.BEACON_3_POS)
        beacon_vectors = [beacon_1_vector, beacon_2_vector, beacon_3_vector]


        vectors = self.transform_magnitudes([beacon_1_vector, beacon_2_vector, beacon_3_vector])

        #resultant_vector = self.get_resultant_vector(vectors, position) 
        #print "Resultant Vector - " + str(resultant_vector)

        turn = beacon_vectors[self.target_beacon][1] - self.angle_to_origin
        turn_angle = self.find_turn_angle(turn) 

        # Turn right
        if (turn < 0  and turn > -180) or turn > 180:
            left_speed = (0.25 - round(0.25 * (turn_angle / 180), 2)) - 0.12
            right_speed = 0.25

        # Turn right
        else: 
            right_speed = (0.25 - round(0.25 * (turn_angle / 180), 2)) - 0.12
            left_speed = 0.25

        return (left_speed, right_speed)

    def take_navigation_input(self):
        self.target_beacon = int(input('Which beacon would you like to navigate to? (1, 2, 3)')) - 1
        while self.target_beacon not in [0, 1, 2]:
            self.target_beacon = int(input('Which beacon would you like to navigate to? (1, 2, 3)')) - 1
            print self.target_beacon
        self.SUCCESS = False


    def check_arrived(self):
        if self.sonar_reading < 0.2: 
            self.SUCCESS = True
            self.reset_success_vars()


    def __init__(self):
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

        self.scanner = BluetootherScanner()

    def main_loop(self):
        print('Looping')
        while not rospy.core.is_shutdown():
            if self.detect_collision():
                print "Collision"
                print "Touched Body:", self.TOUCH_BODY
                print "Touched Head:", self.TOUCH_HEAD
                print "Collision", self.OBJECT_CLOSE
                self.reset_collision_markers()
                self.RESOLVE_COLLISION = True

            elif self.RESOLVE_COLLISION:
                self.resolve_collision()

            elif self.SUCCESS:
                print "Success"
                #self.take_navigation_input()
                if (time.time() - self.success_start) < 2:
                    self.move_miro(-0.25, 0.25)

                else:    
                    self.target_beacon += 1
                    self.target_beacon %= 3
                    self.SUCCESS = False

            else:
                #print "-------------------------------"
                #print "Angle to origin - " + str(self.angle_to_origin)
                (left_speed, right_speed) = self.calculate_wheel_speed()
                #print "Wheel speed - " + str(left_speed) + "," + str(right_speed)
                #print "-------------------------------"
                self.move_miro(left_speed, right_speed)
                self.check_arrived()


            rospy.sleep(self.TICK)
            
            #print self.angle_to_origin
            #self.calculate_wheel_speed()
            #print self.scanner.get_position()

if __name__ == "__main__":
    client = Client()
    client.main_loop()

# Points to check:
# Correct angles
# Correct magnitudes