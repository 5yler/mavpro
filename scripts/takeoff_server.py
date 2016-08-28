#! /usr/bin/env python

# takeoff_server
#
# Action server for autonomous takeoff.
# 
# @author   Syler Wagner	<syler@mit.edu>
#
# @date	 2016-08-11	  creation

import roslib
roslib.load_manifest('mavpro')
import rospy
import actionlib
import sys

from mavpro.msg import TakeoffAction #, TakeoffServer

import mavros
from mavros_msgs.srv import CommandBool  # for landing and arming
from mavros_msgs.srv import SetMode

from mavros import command  # for arming

from mavros_msgs.srv import ParamGet	# for checking sys_id match

# for AUTO takeoff
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.msg import OverrideRCIn

from std_msgs.msg import Float64	# for relative altitude

from mavpro.msg import *


class TakeoffServer(object):

	# create messages that are used to publish feedback/result
	_feedback = TakeoffFeedback()
	_result   = TakeoffResult()


	def __init__(self):
		mavros.set_namespace() # TODO: handle namespaces
		self.rate = rospy.Rate(rospy.get_param('~rate', 1)) # 1 Hz rate default

		self.server = actionlib.SimpleActionServer('takeoff', TakeoffAction, self.execute, False)

		self.mavros_ns = rospy.get_param('~mavros_ns', 'mavros')

		rospy.logwarn("Starting takeoff_server...")

		# relative altitude subscriber
		self.alt_sub = rospy.Subscriber(self.mavros_ns+'/global_position/rel_alt', Float64, self.rel_alt_callback)
		
		# RC override publisher
		self.rc_pub = rospy.Publisher(self.mavros_ns+'/rc/override/', OverrideRCIn, queue_size=5)

		self.server.start()

		print " [   ] Ready to take off! Use the Takeoff.action with the desired goal altitude."

	def check_gcs_id_match(self):

		disable_param_checks = rospy.get_param('~disable_param_checks', False) # double negatives are great!
		if not disable_param_checks:
			print "Param checks enabled"
			mavros_sys_id = rospy.get_param(self.mavros_ns+'/system_id')
			# uav_gcs_id = 

			rospy.wait_for_service(self.mavros_ns+'/param/get')
			get_fcu_param = rospy.ServiceProxy(self.mavros_ns+'/param/get', ParamGet)

			try:
				response = get_fcu_param("SYSID_MYGCS")
				if response.success:
					uav_gcs_id = response.value.integer
					if mavros_sys_id == uav_gcs_id:
						return True
					else:
						rospy.logerr("mavros system_id: %d", mavros_sys_id)
						rospy.logerr("fcu SYSID_MYGCS: %d", uav_gcs_id)

			except rospy.ServiceException, e:
				print " [!!!] ParamGet service call failed: %s"%e

			return False
		else:
			return True

	def execute(self, goal):
		# Do lots of awesome groundbreaking robot stuff here
		rospy.logwarn("Received takeoff goal with altitude %4.2f...", goal.altitude)

		# check that mavros system_id and SYSID_MYGCS match
		# because RC override will not work otherwise
		if not self.check_gcs_id_match():
			self._result.success = False
			self.server.set_aborted(self._result)
			rospy.logerr("The mavros system_id parameter and fcu SYSID_MYGCS parameter do not match. Exiting.")
			sys.exit(-1)

		self.takeoff_alt = goal.altitude
		self._feedback.altitude = 0

		# rospy.sleep(1)

		self.request_mode('STABILIZE') # need to be in an armable mode
		# also can't push missions while in AUTO mode

		# workaround using a takeoff AUTO mission
		if self.push_takeoff_mission():
			self.override_throttle(1000)	# throttle to ~lowest setting
			
			rospy.sleep(0.1)				# sleep so RC override kicks in
			command.arming(True)			# arm UAV
			
			self._result.success = self.auto_takeoff()	# try autonomous takeoff
			if self._result.success:
				self.server.set_succeeded(self._result)
			else:
				rospy.logerr("Goal aborted, mission pushed but auto takeoff failed.")
				self.server.set_aborted(self._result)
		else:
			rospy.logerr("Goal aborted, takeoff mission failed to push.")
			self._result.success = False
			self.server.set_aborted(self._result)

	def auto_takeoff(self):

		self.request_mode('AUTO')	
		self.override_throttle(1400)	# if throttle is at lowest it will not take off

		self.at_goal = False

		while not self.at_goal and not rospy.is_shutdown():
			
			self.server.publish_feedback(self._feedback)
			self.rate.sleep()
			
			if self._feedback.altitude > self.takeoff_alt:	# check altitude

				rospy.logwarn("Takeoff altitude reached!")
				self.override_throttle(0)			# clear RC override, return full manual control
				self.request_mode('GUIDED')
				at_goal = True
			
		return True

	def override_throttle(self, pwm):
		
		rospy.logwarn("Overriding throttle to PWM %4.0f", pwm)

		rc_override = OverrideRCIn()
		NO_CHANGE = 65535 	# leaves RC channel value unchanged 
		RELEASE = 0			# releases RC channel

		# throttle is channel 3
		if pwm == 0:	# clear previous override values
			rc_override.channels = [RELEASE, RELEASE, RELEASE, RELEASE, RELEASE, RELEASE, RELEASE, RELEASE]
			# TODO: 
		else:			# set override for throttle channel
			rc_override.channels = [NO_CHANGE, NO_CHANGE, pwm, NO_CHANGE, NO_CHANGE, NO_CHANGE, NO_CHANGE, NO_CHANGE]

		self.rc_pub.publish(rc_override)


	# relative altitude message callback method
	def rel_alt_callback(self, rel_alt_msg):
		self._feedback.altitude = rel_alt_msg.data


	def create_takeoff_mission(self, altitude):
		waypoints = []

		#$ add dummy start waypoint
		dummy = Waypoint()
		dummy.frame = 0
		dummy.command = 16 # MAV_CMD_WAYPOINT
		dummy.is_current = False
		dummy.autocontinue = True
		dummy.param1 = 0
		dummy.param2 = 0
		dummy.param3 = 0
		dummy.param4 = 0
		dummy.x_lat = 0
		dummy.y_long = 0
		dummy.z_alt = 0
		waypoints.append(dummy)

		#$ add takeoff takeoff
		takeoff = Waypoint()
		takeoff.frame = 3 # 3 MAV_FRAME_GLOBAL_RELATIVE_ALT
		takeoff.command = 22 # 22 MAV_CMD_NAV_TAKEOFF  
		takeoff.is_current = True
		takeoff.autocontinue = True
		takeoff.param1 = 5 # min_pitch
		takeoff.param2 = 0
		takeoff.param3 = 0
		takeoff.param4 = 0
		takeoff.x_lat = 0
		takeoff.y_long = 0
		takeoff.z_alt = altitude
		waypoints.append(takeoff)

		return waypoints

	def push_takeoff_mission(self):

		rospy.wait_for_service(self.mavros_ns+'/mission/push')
		set_takeoff_mission = rospy.ServiceProxy(self.mavros_ns+'/mission/push', WaypointPush)

		waypoints = self.create_takeoff_mission(self.takeoff_alt + 2) # make it 2m higher so it will overreach

		try:
			response = set_takeoff_mission(waypoints)
			print " [>  ] Calling waypoints service..."
			if response.success:
				print " [>>>] Service call successfully added", response.wp_transfered-1, "waypoints!"
				return True
			else:
				print " [!!!] Service call failed to add ", response.wp_transfered-1, " waypoints!"
		except rospy.ServiceException, e:
			print " [!!!] Service call failed: %s"%e
		
		return False

	def request_mode(self, mode):
		rospy.wait_for_service(self.mavros_ns+'/set_mode')
		try:
			request_mode = rospy.ServiceProxy(self.mavros_ns+'/set_mode', SetMode)
			response = request_mode(custom_mode=mode)
			return response.success
		except rospy.ServiceException, e:
			print ' [!!!] Mode change request failed: %s' % e


if __name__ == '__main__':
	rospy.init_node('takeoff_server')
	server = TakeoffServer()
	rospy.spin()