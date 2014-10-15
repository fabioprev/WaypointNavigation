#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import datetime
import time
import threading
import math
from math import fabs,sqrt,pi,degrees,radians,atan2,tan,sin,cos

PKG = 'dis_navigation'

import roslib; roslib.load_manifest(PKG)

import rospy
import tf

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from dis_navigation.srv import RobotPose


global robotname
robotname='no_robot'

global tfl
tfl=0

def getRobotPose_srv(V=[]):
    global robotname
  
    try:
        if (robotname=='no_robot'):
            robotname = rospy.get_param("/robotname")
    except:
        print 'Exception in get_param robotname'
        return False
    sname = '/'+robotname+'/getpose'
    rospy.wait_for_service(sname)
    getpose_f = rospy.ServiceProxy(sname, RobotPose)
    res = 0
    try:
      res = getpose_f(robotname)
    except rospy.ServiceException, e:
      print "Service getpose did not process request: %s" % str(e)
      return False

    print "Service getpose:: robot pose %s = %.2f %.2f %.2f " % (robotname, res.x, res.y, degrees(res.th))
    if (res.th>360):
      return False
    if (len(V)>0):
      V[0] = res.x
      V[1] = res.y
      if (len(V)>2):
        V[2] = res.th
    return True


def getRobotPose_tf(V=[]):
  global tfl
  global robot_pose
  global robotname
  
  try:
    if (robotname=='no_robot'):
      robotname = rospy.get_param("/robotname")
  except:
    print 'Exception in get_param'
    return False
    
  source = '/map'
  dest = '/'+robotname+'/base_link'
    
  if (tfl==0):
    tfl = tf.TransformListener()
    
    try:
      timetf = tfl.getLatestCommonTime(source,dest)
      print 'waitForTransform 1'
      tfl.waitForTransform(source, dest, rospy.Time(0), rospy.Duration(1.0))
      print 'waitForTransform 1 OK'
    except:
      print 'waitForTransform failed 1'
      time.sleep(1.0)
      try:
        print 'waitForTransform 2'
        tfl.waitForTransform(source, dest, rospy.Time.now(), rospy.Duration(2.0))
        print 'waitForTransform 2 OK'
      except:
        print 'waitForTransform failed 2'
        time.sleep(1.0)
        try:
          print 'waitForTransform 3'
          tfl.waitForTransform(source, dest, rospy.Time.now(), rospy.Duration(3.0))
          print 'waitForTransform 3 OK'
        except:
          print 'waitForTransform failed 3'
      

  found = False
  sl=0.20
  while (not found and not rospy.is_shutdown()):
    try:
      timetf = tfl.getLatestCommonTime(source,dest)
      #print '*** lookup transform: '+source+' to '+dest      
      timenow = rospy.Time(0) #.now()
      sl = sl+0.05
      (trans,rot) = tfl.lookupTransform(source, dest, timetf) #rospy.Time(0))
      robot_pose = PoseStamped()
      robot_pose.header.stamp = timetf
      robot_pose.pose.position.x = trans[0]
      robot_pose.pose.position.y = trans[1]
      robot_pose.pose.position.z = trans[2]
      robot_pose.pose.orientation.x = rot[0]
      robot_pose.pose.orientation.y = rot[1]
      robot_pose.pose.orientation.z = rot[2]
      robot_pose.pose.orientation.w = rot[3]
      (r, p, y) = euler_from_quaternion(rot)
      #print '*** T = ',trans
      #print '*** R = ',rot,'  yaw = ',degrees(y),' deg'
      #print 'Robot pose ',robot_pose.pose.position.x,robot_pose.pose.position.y
      found = True
    except:
      time.sleep(sl)
      print '*** exception in getRobotPose: ',robotname
      print '*** lookup transform: '+source+' to '+dest 
      print '*** sleep time: %s'%sl 
      print sys.exc_info()[0]
      robot_pose=0
      continue
  if (robot_pose==0):
    return False
  else:
    if (len(V)>0):
      V[0] = robot_pose.pose.position.x
      V[1] = robot_pose.pose.position.y
      if (len(V)>2):
        V[2] = y
    return True


def getRobotPose(V=[]):
    return getRobotPose_tf(V)


