#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import time

from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot

from geometry_msgs.msg import PoseStamped

mc = None

def callback(data):
    global mc
  
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    res = mc.get_coords()
    time.sleep(0.1)
    
    x= data.pose.position.x*10 + res[0]
    y= data.pose.position.y*10 + res[2]
    z= data.pose.position.z*10 + res[1]
    rx= data.pose.orientation.x + res[3]
    ry = data.pose.orientation.y + res[5]
    rz= data.pose.orientation.z + res[4]
  
    
    data_list = [x, y, z, rx, ry, rz]
        
    for index, value in enumerate(data_list):
        data_list[index] = round(value, 3)
    
    data_list = data_list[:6]
    print("coord:%s"%data_list[:6])
    mc.send_coords(data_list[:6], 50, 0)
      
    time.sleep(1)
    
def listener():
    global mc
   
    rospy.init_node("control_slider", anonymous=True)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback) #for coord control 
    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot(port, baud)
 
    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
