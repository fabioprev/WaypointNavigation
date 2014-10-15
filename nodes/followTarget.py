#! /usr/bin/env python

PKG = 'WaypointNavigation'

import roslib; roslib.load_manifest(PKG)
import rospy

import actionlib
from geometry_msgs.msg import Twist

from math import degrees, radians, pi, atan2, asin

from threading import Thread, Lock

import robotpose
import tf
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from WaypointNavigation.msg import *

global robotPoseX
global robotPoseY
global robotPoseTheta

global mutex

mutex = Lock()

MAX_TRANS_VEL   = 1.0
MAX_ANG_VEL     = 1.0
MIN_VEL = 0.25
MID_ANG = 30

#######################################################################################
### Support Functions

def norm180(a):
  while (a>180):
    a = a-360
  while (a<=-180):
    a = a+360
  return a
  
def norm360(a):
  while (a>=360):
    a = a-360
  while (a<=-360):
    a = a+360
  return a

def roundPI2(a):
    if ((a>=-pi/4 and a<=pi/4) or (a>=7*pi/4 and a<=2*pi)):
        return 0
    elif (a>=pi/4 and a<=3*pi/4):
        return pi/2
    elif ((a>=3*pi/4 and a<=5*pi/4) or (a>=-pi and a<=-3*pi/4)):
        return pi
    elif ((a>=5*pi/4 and a<=7*pi/4) or (a>=-3*pi/4 and a<=-pi/4)):
        return -pi/2;

#######################################################################################
### FollowTarget Action Server

class FollowTargetAction(object):
  _feedback             = FollowTargetFeedback()
  _result               = FollowTargetResult()
  robotname             = rospy.get_param("/robotname", "robot_0")
  pub_cmd               = rospy.Publisher('/'+robotname+'/desired_cmd_vel', Twist)
  
  def __init__(self, name):
    self._action_name =  name    
    self._as = actionlib.SimpleActionServer(self._action_name, WaypointNavigation.msg.FollowTargetAction, execute_cb=self.execute_cb, auto_start = False)  
    self._as.start()
    
    rospy.loginfo('%s: FollowTarget started' % self._action_name)
  
  def execute_cb(self, goal):
            
    GX  = goal.target_pose.pose.position.x
    GY  = goal.target_pose.pose.position.y
    
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, followTarget [%s,%s]...' % (self._action_name, GX, GY))
    
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
      self.preempt()
      return
    
    # feedback
    self._feedback.feedback = "Following target"
    self._as.publish_feedback(self._feedback)
    
    # reach the target position (GX,GY)
    self.do_followTarget(GX, GY)
    
    if (not self._as.is_preempt_requested()):
      self._result.result = "Succeeded"
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
    else:
      self._result.result = "Aborted"
      rospy.loginfo('%s: Aborted' % self._action_name)
      self._as.set_succeeded(self._result)

  def do_followTarget(self, GX, GY):
    global mutex, robotPoseX, robotPoseY, robotPoseTheta
    
    # getting robot pose
    rp = [0,0,0]
    
    #while (not robotpose.getRobotPose(rp)):
    #  rospy.sleep(0.25)
    
    mutex.acquire()
    
    rp[0] = robotPoseX
    rp[1] = robotPoseY
    rp[2] = robotPoseTheta
    
    mutex.release()
    
    cmd_vel = Twist()
    I_err = 0
    prev_err = 0
    X = rp[0]
    Y = rp[1]
    
    a = atan2(GY-Y,GX-X)
    GTh = degrees(roundPI2(a))

    limX=1.0
    limY=1.0
    
    #if (GTh==0 or GTh==180): 
      #limX=0.5
      #limY=1.0
    #elif (GTh==90 or GTh==270 or GTh==-90): 
      #limX=1.0
      #limY=0.5
    #else:
      #limX=0.5
      #limY=0.5
    
    while ((not self._as.is_preempt_requested()) and (abs(GX-X)>limX or abs(GY-Y)>limY)):
      #if (not robotpose.getRobotPose(rp)):
      #  continue
      
      mutex.acquire()
      
      rp[0] = robotPoseX
      rp[1] = robotPoseY
      rp[2] = robotPoseTheta
      
      mutex.release()
      
      X = rp[0]
      Y = rp[1]
      yaw = rp[2]
      
      print 'Current pose [%.1f,%.1f,%.1f] ...' %(X, Y, degrees(yaw))
      print 'Target pose [%.1f,%.1f,%.1f] ...' %(GX, GY, GTh)
      
      self._feedback.feedback = 'Current pose [%s,%s,%s] ...' % (X, Y, degrees(yaw))
      self._as.publish_feedback(self._feedback)
      
      a = atan2(GY-Y,GX-X)
      GTh = degrees(a)

      Th        = norm360(degrees(yaw))
      adist     = norm180(GTh-Th)

      print 'Th = %.1f - GTh = %.1f - adist = %.1f ...' %(Th,GTh,adist)
       
      if (abs(adist)>90):
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = MAX_ANG_VEL
      elif (abs(adist)>MID_ANG):
        cmd_vel.linear.x = MIN_VEL * (90-abs(adist))/(90-MID_ANG) 
        cmd_vel.angular.z = MAX_ANG_VEL
      else:
        cmd_vel.linear.x = (MAX_TRANS_VEL-MIN_VEL) * (MID_ANG-abs(adist))/MID_ANG + MIN_VEL
        cmd_vel.angular.z = MAX_ANG_VEL/2
      if (abs(adist)>40 and abs(adist)<50):
        I_err = 0
      
      # PID controller
      err = radians(adist)
      P_err = err
      I_err += err
      D_err = err - prev_err
      prev_err = err
      Kp = 0.60
      Ki = 0.000
      Kd = 0.01
      v = Kp * P_err + Ki * I_err + Kd * D_err

      print 'PID [%.3f,%.3f,%.3f] -> %.3f \n' %(P_err, I_err, D_err, v)

      cmd_vel.angular.z *= v
      #if ((GTh==90) or (GTh==270)):
      #    tdist = abs(GY-Y)
      #else:
      #    tdist = abs(GX-X)
      
      #w = MAX_TRANS_VEL * max(0.5,min(1.0,tdist-1.0))
      #cmd_vel.linear.x *= w

      print 'Command [%.3f,%.3f] \n' %(cmd_vel.linear.x, cmd_vel.angular.z)
      
      self.pub_cmd.publish(cmd_vel)
      rospy.sleep(0.1)

    self.stop_follow()
  
  def stop_follow(self):
    cmd_vel               = Twist()
    cmd_vel.linear.x      = 0
    cmd_vel.linear.y      = 0
    cmd_vel.linear.z      = 0
    cmd_vel.angular.x     = 0
    cmd_vel.angular.y     = 0
    cmd_vel.angular.z     = 0
    self.pub_cmd.publish(cmd_vel)

def updateRobotPose(data):
  global mutex, robotPoseX, robotPoseY, robotPoseTheta
  
  mutex.acquire()
  
  robotPoseX = -data.pose.pose.position.y;
  robotPoseY = data.pose.pose.position.x;
  
  theta = atan2((-2 * data.pose.pose.orientation.x * data.pose.pose.orientation.y) + (2 * data.pose.pose.orientation.z * data.pose.pose.orientation.w),
                  1 - (2 * data.pose.pose.orientation.y * data.pose.pose.orientation.y) - (2 * data.pose.pose.orientation.z * data.pose.pose.orientation.z))
  
  robotPoseTheta = norm180((theta + (pi / 2)))
    
  mutex.release()

#######################################################################################
### Main 

if __name__ == '__main__':
    rospy.init_node('followTarget')
    robotname     = rospy.get_param("/robotname", "robot_0")
    
    rospy.Subscriber("/"+ robotname +"/base_pose_ground_truth",Odometry,updateRobotPose)
    
    FollowTargetAction("/"+ robotname +"/followTarget")
    rospy.spin()
