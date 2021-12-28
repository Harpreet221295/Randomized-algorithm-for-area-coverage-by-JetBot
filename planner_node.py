#!/usr/bin/env python

import rospy
import cv2
import apriltag
from std_msgs.msg import Float32MultiArray, Bool
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np
import time
import pickle
import networkx as nx
import math
from numpy import arccos, array
from numpy.linalg import norm

ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)
vel_pub = rospy.Publisher('/vel', Float32MultiArray, queue_size=2)
toggle_detection_pub = rospy.Publisher('/toggle_detection', Bool, queue_size=1)


robo_pose_updated = False
is_new_detection = False
closest_reached = False


class Robot:
    def __init__(self, state_pos):
        self.pose = np.array(state_pos)
          
    def update_pose(self, new_pose):
        self.pose = np.array(new_pose)
        

        
robot = Robot([0,0,0])

#movements_in_circle = None

def turn_90_clockwise():
    move = 0
    stop = 1
    
    # First leg
    
    speed_l = -0.85
    speed_r = 0.9
    
    start_detect_msg = False
    toggle_detection_pub.publish(start_detect_msg)
        
    for i in range(1):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        #time.sleep(0.2)
            
    start_detect_msg = True
    toggle_detection_pub.publish(start_detect_msg)
    vel_msg = Float32MultiArray()
    dist = 0
    angle = 90
    vel_msg.data = [dist/0.6, angle/0.6,0.6]
    vel_pub.publish(vel_msg)
    time.sleep(3)
    
        
def move_in_circle():
    move = 0
    stop = 1
    
    speed_l = -0.78
    speed_r = -0.9
    
    for j in range(19):
        
        start_detect_msg = False
        toggle_detection_pub.publish(start_detect_msg)
        
        for i in range(3):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctrl_pub.publish(msg)
            time.sleep(0.2)
            msg = Float32MultiArray()
            msg.data = [stop, speed_l, speed_r]
            ctrl_pub.publish(msg)
            #time.sleep(0.2)
            
        start_detect_msg = True
        toggle_detection_pub.publish(start_detect_msg)
        vel_msg = Float32MultiArray()
        dist = 0.24
        angle = 20
        vel_msg.data = [dist/0.6, angle/0.6,0.6]
        vel_pub.publish(vel_msg)
        time.sleep(10)
        
def move_in_multiple_circles(num_circles):
    
    for iter in range(num_circles):
        move_in_circle()
       

    
def calibrate_with_vel():
    move = 0
    stop = 1
    
    speed_l = -0.78
    speed_r = -0.9
    
    start_detect_msg = False
    toggle_detection_pub.publish(start_detect_msg)
        
        
    for i in range(3):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.2)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        #time.sleep(0.2)
            
    start_detect_msg = True
    toggle_detection_pub.publish(start_detect_msg)
    vel_msg = Float32MultiArray()
    dist = 0
    angle = 0
    vel_msg.data = [dist/0.6, angle/0.6,0.6]
    vel_pub.publish(vel_msg)

def correct_orientation(current_orientation, target_orientation):
    angular_diff = target_orientation - current_orientation
    pass
    
def set_robot_pose_callback(msg):
    global robot
    robot.update_pose(msg.data)
    print("robot's pose updated")

def set_is_new_detection_callback(msg):
    global is_new_detection
    is_new_detection = True
    
def set_closest_callback(msg):
    global closest_reached
    closest_reached = msg
    
def theta(v, w): 
    return arccos(v.dot(w)/(norm(v)*norm(w)))

def cover_waypoints(waypoints):
    global robot
    num_waypoint = len(waypoints)
    
    print("robot: {0}".format(robot.pose))
    for waypt in waypoints:
        #print(waypt)
        #print(robot.pose[:3])
        #print(np.array(waypt))
        movement_vector = np.array(waypt) - robot.pose[:2]
        movement_vector = movement_vector.astype('float')
        target_distance = np.linalg.norm(movement_vector)
        x_axis_vector = np.array([1,0]).astype('float')
        target_orientation = math.degrees(theta(movement_vector, x_axis_vector))
        
        if np.cross(x_axis_vector,movement_vector)/np.linalg.norm(movement_vector)  < 0:
            target_orientation = -1 * target_orientation
        
        angle = target_orientation - robot.pose[2]
        angle = get_shorter_angle(angle)
        angle = round(angle, 2)
        print("{0} -> {1}  dist: {2}    angle: {3}".format(robot.pose[:2].round(2), waypt, np.linalg.norm(movement_vector).round(2), round(angle,2)))
        
        dist = np.linalg.norm(movement_vector).round(2)
        rotate_robot(angle)
        translate_robot(dist)
        
        robot.update_pose([waypt[0],waypt[1], robot.pose[2]+angle])
        
        
        #correct_orientation(robot.pose[2], target_orientation)
        #cover_linear_distance(np.linalg.norm(movement_vector))

def translate():
    move = 0
    stop = 1
    # + 30 - 40 degrees
    speed_l = -0.95
    speed_r = -0.95
    
    for i in range(2):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        #msg = Float32MultiArray()
        #msg.data = [move, speed_l, speed_r]
        #ctrl_pub.publish(msg)
        #time.sleep(0.3)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(2)
        
def rotate():
    move = 0
    stop = 1
    # + 30 - 40 degrees
    speed_l = 0.75
    speed_r = -0.75
    
    for i in range(1):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        msg = Float32MultiArray()
        #msg.data = [move, speed_l, speed_r]
        #ctrl_pub.publish(msg)
        #time.sleep(0.3)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.5)
    '''
    for i in range(1):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        #time.sleep(0.2)
    '''        
    #start_detect_msg = True
    #toggle_detection_pub.publish(start_detect_msg)

def move_robot_util(l,r,iters):
    #Utility function to move a robot using given left and right motor speeds for 0.3 seconds and then give 0.6 seconds localization window
    move = 0
    stop = 1
    for i in range(iters):
        start_detect_msg = False
        toggle_detection_pub.publish(start_detect_msg)
        msg = Float32MultiArray()
        msg.data = [move, l, r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        
        
        msg = Float32MultiArray()
        msg.data = [stop, l, r]
        start_detect_msg = True
        toggle_detection_pub.publish(start_detect_msg)
        ctrl_pub.publish(msg)
        time.sleep(0.6)
    
    
def get_shorter_angle(angle):
    if abs(angle) > 180:
        if angle > 0:
            angle = 360 - angle
            angle = -1 * angle
        else:
            angle = 360 - abs(angle)
            #angle = -1 * angle
    return angle

def rotate_robot(angle):
    
    angle = get_shorter_angle(angle)
    
    if angle > 0:
        #current = angle
        if angle > 10 and angle < 30:
            move_robot_util(0.65, -0.65, 1)
            publish_vel(0, 30, 0.9)
            
        if angle > 30 and angle < 40:
            move_robot_util(0.75, -0.75, 1)
            publish_vel(0, 35, 0.9)
            
        if angle > 40 and angle<= 50:
            move_robot_util(0.75, -0.75, 1)
            publish_vel(0, 35, 0.9)
            
        if angle > 60 and angle < 70:
            move_robot_util(0.75, -0.75, 2)
            publish_vel(0, 35, 1.8)
            
        if angle >70 and angle <= 90:
            move_robot_util(0.8, -0.8, 1)
            move_robot_util(0.7, -0.7, 1)
            publish_vel(0, 35, 1.8)
            
            
        if angle >90 and angle<=130:
            move_robot_util(0.75, -0.75, 2)
            move_robot_util(0.75, -0.75, 2)
            publish_vel(0, 35, 3.6)
            
        if angle > 140 and angle < 180:
            move_robot_util(0.8, -0.8, 1)
            move_robot_util(0.7, -0.7, 1)
            move_robot_util(0.8, -0.8, 1)
            move_robot_util(0.7, -0.7, 1)
            publish_vel(0, 35, 3.6)
    
    else:
        angle = abs(angle)
        if angle > 10 and angle < 30:
            move_robot_util(-0.72, 0.72, 1)
            publish_vel(0, 35, 0.9)
            
        if angle > 30 and angle < 40:
            move_robot_util(-0.75, 0.75, 1)
            publish_vel(0, 35, 0.9)
            
        if angle > 40 and angle<= 50:
            move_robot_util(-0.75, 0.75, 1)
            publish_vel(0, 35, 0.9)
            
        if angle > 60 and angle < 70:
            move_robot_util(0.73, -0.73, 2)
            publish_vel(0, 35, 1.8)
            
        if angle >70 and angle <= 90:
            move_robot_util(-0.75, 0.75, 1)
            move_robot_util(-0.75, 0.75, 1)
            publish_vel(0, 35, 1.8)
            
        if angle >90 and angle<=130:
            move_robot_util(0.73, -0.73, 2)
            move_robot_util(0.73, -0.73, 2)
            publish_vel(0, 35, 3.6)
            
        if angle > 140 and angle < 180:
            move_robot_util(-0.75, 0.75, 1)
            move_robot_util(-0.75, 0.75, 1)
            move_robot_util(-0.75, 0.75, 1)
            move_robot_util(-0.75, 0.75, 1)
            publish_vel(0, 35, 3.6)

def move_front(dist):
    if dist <0.15:
         move_robot_util(-0.85, -0.85, 1)
    
    if dist < 0.27:
        move_robot_util(-0.95, -0.95, 2)
    else:
        move_robot_util(-0.92, -0.95, 2)
        move_robot_util(-0.72, -0.75, 1)
        
def move_back(dist):
    if dist <0.15:
         move_robot_util(-0.85, -0.85, 1)
    
    if dist < 0.27:
        move_robot_util(-0.95, -0.95, 2)
    else:
        move_robot_util(-0.95, -0.9, 2)
        move_robot_util(-0.75, -0.7, 1)

def turn_left_sided():
    move_robot_util(0, -0.95, 3)
    move_robot_util(0, -0.85, 2)
    
def turn_right_sided():
    move_robot_util(-0.95, 0,  4)
    move_robot_util(-0.85, 0,  1)
    
def translate_robot(dist):
    if dist <0.15:
         move_robot_util(-0.85, -0.85, 1)
    
    if dist < 0.27:
        move_robot_util(-0.95, -0.95, 2)
    else:
        move_robot_util(-0.95, -0.95, 2)
        move_robot_util(-0.75, -0.75, 1)
            
def compute_eucledian_distance(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)


def random_rotate2():
    angle_list = [45, 90, 130, 160]
    select_angle = np.random.choice(angle_list)
    if np.random.rand() > 0.5:
        rotate_robot(select_angle)
        print("rotated robot by {0}".format(select_angle))
    else:
        rotate_robot(-1 * select_angle)
        print("rotated robot by {0}".format(-1 * select_angle))

        
def publish_vel(dist, angle, timet):
    vel_msg = Float32MultiArray()
    vel_msg.data = [dist/timet, angle/timet,timet]
    vel_pub.publish(vel_msg)

def random_rotate():
    rnd = np.random.rand()
    if rnd < 0.25:
        rotate_robot(65)
        print("rotating robot by 65")
    elif rnd < 0.5:
        rotate_robot(65)
        rotate_robot(65)
        print("rotating robot by 130")
    elif rnd < 0.75:
        rotate_robot(-65)
        print("rotating robot by -65")
    else:
        rotate_robot(-65)
        rotate_robot(-65)
        print("rotating robot by -130")

if __name__ == "__main__":
    
    rospy.init_node('planner_node')
    rospy.Subscriber("/pose_robot", Float32MultiArray, set_robot_pose_callback)
    rospy.Subscriber("/closest", Bool, set_closest_callback)
    rospy.Subscriber("/is_new_detection", Bool, set_is_new_detection_callback)
    
    
    
    
    
    
    sleep_time = 5
    iters = 100
    
    #global closest_reached
    
    for i in range(iters):
         move_front(0.3)
         publish_vel(0.3, 0, 2.7)
         print("Move front 0.3m")
         time.sleep(sleep_time)
         if is_new_detection:
            print("Yes Detected")
            if closest_reached:
                move_front(0.1)
                publish_vel(0.1, 0, 0.9)
                print("Moved 0.1m")
                
                print("Closest Reached")
                
                random_rotate2()
                
                closest_reached = False
            is_new_detection = False
            
         else:
            print("not detected....gonna rotate")
            for i in range(5):
                print("Gonna explore by rotating 35")
                rotate_robot(35)
                time.sleep(sleep_time)
                if is_new_detection is True:
                    print("Yes Detected")
                    if closest_reached:
                        move_front(0.1)
                        print("Moved 0.1m")
                        print("Closest Reached")
                        
                        random_rotate2()
                        
                        closest_reached = False
                    is_new_detection = False
                    break
    
    
    
    
    
   
    