#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Autovendors talker script 

import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg  import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from random import seed
from random import randint

pub = rospy.Publisher('navMsg', Float32, queue_size=10)
turtlebotPub =  rospy.Publisher('destinationGoal', PoseStamped, queue_size=10)
navMsgNum = 1 #robot at home
scaleFactor = .072039
global boxState
global order


def webOrdersCallback(data):
    global navMsgNum
    global order
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    order = data.data #order variable has room number
    navMsgNum = 4 #delivery requested

def webCommandsCallback(data):
    global navMsgNum
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    commands = data.data
    if commands == 'unlock':
        navMsgNum = 0 #emergency open

def boxStateCallback(data):
    global navMsgNum
    global boxState
    global order
    destinationHeader =  Header(10,rospy.get_rostime(),"map")
    destinationMsg = PoseStamped(destinationHeader, Pose(Point(0,0,0), Quaternion(0,0,0,1)))
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    boxState = data.data
    if boxState == 1 and ('order' in globals()): #box is closed and filled
        if order == 'C456':
            position = Point(268 * scaleFactor,32 * scaleFactor, 0*scaleFactor)
            destinationMsg.pose.position = position
        elif order == 'C444D':
            position =  Point(274 *scaleFactor,450 *scaleFactor,0*scaleFactor)
            destinationMsg.pose.position = position
        elif order == 'W417':
            position =  Point(653*scaleFactor,231 * scaleFactor,0 *scaleFactor)
            destinationMsg.pose.position = position
        elif order == 'E474':
            position =  Point(46*scaleFactor,258 *scaleFactor,0*scaleFactor)
            destinationMsg.pose.position = position
        elif order == 'E463A':
            position =  Point(139*scaleFactor,450*scaleFactor,0*scaleFactor)
            destinationMsg.pose.position = position
        else:
            order = 'NO ORDER'

        if order != 'NO ORDER':
            navMsgNum = 3 #robot in transit
            rospy.loginfo("Going to goal")
            rospy.loginfo(navMsgNum)
            pub.publish(navMsgNum)
            sendToGoal(destinationMsg)



    if boxState == 2: #box is closed and empty
        #currently home location is C440A
        position =  Point(336*scaleFactor,541*scaleFactor,0*scaleFactor)
        destinationMsg.pose.position = position
        navMsgNum = 3 #robot in transit
        rospy.loginfo("Going to home")
        rospy.loginfo(navMsgNum)
        pub.publish(navMsgNum)
        sendToGoal(destinationMsg)



def sendToGoal(data):
    goalMsg = MoveBaseGoal(data)
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    print("Recieved order with destination")
    move_base.wait_for_server(rospy.Duration(10))
    print("Moving")
    move_base.send_goal(goalMsg)
    print("waiting")
    move_base.wait_for_result()
    print("I made it")
    arrivedCallback(move_base.get_state())

def arrivedCallback(data):
    global navMsgNum
    global boxState
    #test = data
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.status_list[0].status)
    #arrived = data.status_list[0].status
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    print("Everything is working?")
    if data == 3 and boxState == 1: #arrived to order destination
        navMsgNum = 2
        rospy.set_param('unlock_code', randint(1000,9999))

    if data == 3 and boxState == 2: #arrived home
        navMsgNum = 1

def talker():
    rospy.init_node('talker', anonymous=True)

    rospy.Subscriber('webOrders', String, webOrdersCallback) #room number from website
    #rospy.Subscriber('webCommands', String, webCommandsCallback) #unlock and reset from website
    rospy.Subscriber('boxState', Float32, boxStateCallback) #state of box (empty, filled,open)

    #ROS Subscriber for robot completing movement either home or to destination
    #rospy.Subscriber('move_base/status', GoalStatusArray, arrivedCallback)

    #get ip address of laptop
    with open('/home/turtlebot/Documents/ip.txt') as fp:
        ip = fp.readline()
    ip_list = ip.split('.')

    #set parameters for laptop IP and unlock code
    rospy.set_param('ip_zero',int(ip_list[0]))
    rospy.set_param('ip_one',int(ip_list[1]))
    rospy.set_param('ip_two',int(ip_list[2]))
    rospy.set_param('ip_three',int(ip_list[3]))
    rospy.set_param('unlock_code', 4873)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rospy.loginfo(navMsgNum)
        pub.publish(navMsgNum)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
