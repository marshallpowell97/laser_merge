#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import roslib
import tf
from copy import deepcopy


def mergeval(kinect, lidar, dist):
    # the variables needed to calculate the
    # angle and distance relative to the lidar
    k = 3  # array location number in kinect array
    count = 1  # adjusts k to match the angles more closely
    deg = -31  # keeps track of what degree we are looking at
    theta = deg  # used in the math for calculating new angle/distance
    phi = 0  # this is the angle relative to 0 from
    rangeval = 0  # stores the kinect array value
    rp1 = 0  # stores the converted connect value relative to rplidar
    #checks every degree from -31 to 31 in increments of degrees (not sure this is needed)

    for i in range (1, 640):
        if(i==k):
            rangeval = kinect[k]
        # print(k,",",deg,",",rangeval)
            theta = deg
            if(k<319):
                theta=theta+180
                theta = math.radians(theta)
                rp1 = math.sqrt(dist**2 + rangeval**2 - (2*dist*rangeval*math.cos(theta)))
                phi = (math.asin(((rangeval*(math.sin(theta)))/rp1)))
                phi = math.degrees(phi)
                if (math.isnan(phi)):
                    phi = 0
                phi = int(phi)
                if (rp1 < lidar[180-phi]):
                    lidar = list(lidar)
                    lidar[180 - phi] = rp1
            else:
                theta=180-theta
                theta = math.radians(theta)
                rp1 = math.sqrt(dist**2 + rangeval**2 - (2*dist*rangeval*math.cos(theta)))
                phi = (math.asin(((rangeval*(math.sin(theta)))/rp1)))
                phi = math.degrees(phi)
                if (math.isnan(phi)):
                    phi = 0
                phi = int(phi)
                if(rp1<lidar[180+phi]):
                    lidar = list(lidar)
                    lidar[180 + phi] = rp1

            #print(rp1,phi, k)
            deg =deg + 1
            k=k+10
            count=count+1
            if(count == 5):
                k+=1
                count = 1

    return lidar

def callback_lidar(msg):
    global lidar_flag
    global kinect_flag
    global merged_scan
    global lidar_temp
    global kinect_temp
    global base_scan

    base_scan = deepcopy(msg)

    lidar_flag = 1
    lidar_temp = list(msg.ranges)

    if ((kinect_flag == 1) and (lidar_flag == 1)):
        merged_scan = deepcopy(base_scan)
        lidar_temp = mergeval(kinect_temp, lidar_temp, 0.37084)
        merged_scan.ranges = lidar_temp
        pub_scan.publish(merged_scan)
        lidar_flag = 0
        kinect_flag = 0

def callback_kinect(msg):
    global kinect_flag
    global lidar_flag
    global merged_scan
    global lidar_temp
    global kinect_temp
    global base_scan

    kinect_flag = 1
    kinect_temp = list(msg.ranges)


    if ((kinect_flag == 1) and (lidar_flag == 1)):
        merged_scan = deepcopy(base_scan)
        lidar_temp = mergeval(kinect_temp, lidar_temp, 0.37084)
        merged_scan.ranges = lidar_temp
        pub_scan.publish(merged_scan)
        lidar_flag = 0
        kinect_flag = 0

lidar_flag = 0
kinect_flag = 0
lidar_temp = []
kinect_temp = []
merged_scan = LaserScan()
merged_scan.header.frame_id = "scan_2"
pub_scan = rospy.Publisher('/scan', LaserScan)

base_scan = LaserScan()

def laser_scan():
    global merged_scan
    rospy.init_node('scan_values')
    sub_lidar = rospy.Subscriber('/scan_lidar', LaserScan, callback_lidar)
    sub_kinect = rospy.Subscriber('/scan_kinect', LaserScan, callback_kinect)
    rospy.spin()

if __name__ == '__main__':
    try:
        laser_scan()
    except rospy.ROSInterruptException:
        pass

    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "scan_2",
                         "turtle1")
        rate.sleep()
