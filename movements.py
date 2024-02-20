'''
Written for Plushie, HELPER project 5 jan 2024 by Danique Damen
'''

import rospy
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from nav_msgs.msg import Odometry

import geometry_msgs
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Imu

import math
import numpy as np
import time
import sys
import os
import threading

import miro2 as miro

from stream_audio import streamer

################################################################

# constants
max_fwd_spd = 0.4
T_ramp_up_down = 2.0

################################################################

def fmtflt(f):

	return "{:.3f}".format(f)

################################################################

class controller:

	def callback_kin(self, msg):

		# ignore until active
		if not self.active:
			return

		# report
		self.kin_sensor = msg.position

	def callback_package(self, msg):

		# ignore until active
		if not self.active:
			return

		# storeprint("thread_alive?:", threadAudio.is_alive())
		self.sensors = msg

	def imp_report_wheels(self, msg_wheels):

		if self.report_wheels and not self.sensors is None:
			opto = self.sensors.wheel_speed_opto.data
			emf = self.sensors.wheel_speed_back_emf.data
			pwm = self.sensors.wheel_effort_pwm.data
			msg = fmtflt(msg_wheels.twist.linear.x) + " " + \
				fmtflt(msg_wheels.twist.angular.z) + " " + \
				fmtflt(opto[0]) + " " + fmtflt(opto[1]) + " " + \
				fmtflt(emf[0]) + " " + fmtflt(emf[1]) + " " + \
				fmtflt(pwm[0]) + " " + fmtflt(pwm[1])
			print (msg)

	def loop(self, stopttime_action, stopttime_audio):

		# pars
		f_kin = 0.25
		f_cos = 1.0

		# state
		t_now = 0.0

		# message
		msg_kin = JointState()
		msg_kin.position = [0.0, np.radians(30.0), 0.0, 0.0]

		# message
		msg_wheels = TwistStamped()
		msg_push = miro.msg.push()

		# message
		msg_cos = Float32MultiArray()
		msg_cos.data = [0.5, 0.5, 0.0, 0.0, 0.5, 0.5]

		# message
		msg_illum = UInt32MultiArray()
		msg_illum.data = [0, 0, 0, 0, 0, 0]


		curr_time = 0
		exit_event = threading.Event()

		# loop
		while self.active and not rospy.core.is_shutdown():

			# compute drive signals
			xk = math.sin(t_now * f_kin * 2 * math.pi)
			xc = math.sin(t_now * f_cos * 2 * math.pi)
			xcc = math.cos(t_now * f_cos * 2 * math.pi)
			xc2 = math.sin(t_now * f_cos * 1 * math.pi)

			# at special time instances:
			#at zero start audio thread

			if curr_time == 0:
				threadAudio = threading.Thread(target=self.audio.loop, args=(exit_event,))
				threadAudio.start()

			elif stopttime_action == 1000 and not threadAudio.is_alive():
				for i in [2, 3]:
						msg_cos.data[i] = 0
				self.pub_cos.publish(msg_cos)
				return
		
			if stopttime_audio != 1000 and curr_time == stopttime_audio: # if action > audio but audio max is reached, stop audio

				exit_event.set()
			
			if curr_time == max(stopttime_action, stopttime_audio): # if max length is reached, stop
				for i in [2, 3]:
						msg_cos.data[i] = 0

				for i in range(1,3):
					msg_kin.position[i] = 0

				self.pub_kin.publish(msg_kin)
				self.pub_cos.publish(msg_cos)

				return
		
			#otherwise continue with movement code
			#curr_time += 1
			# send kin
			self.kin = ""
			if len(self.kin):
				msg_kin.position[1] = np.radians(30.0)
				msg_kin.position[2] = np.radians(0.0)
				msg_kin.position[3] = np.radians(0.0)
				if "l" in self.kin:
					msg_kin.position[1] = xk * np.radians(20.0) + np.radians(30.0)
				if "y" in self.kin:
					t = xk * np.radians(45.0)
					msg_kin.position[2] = t
				if "p" in self.kin:
					msg_kin.position[3] = xk * np.radians(15.0) + np.radians(-7.0)
				self.pub_kin.publish(msg_kin)


			# send cos
			self.cos = ""
			if len(self.cos):
				sc = 0.5
				if "h" in self.cos:
					for i in range(2, 6):
						msg_cos.data[i] = xc * sc + 0.5
				if "l" in self.cos:
					for i in [2, 4]:
						msg_cos.data[i] = xc * sc + 0.5
				if "r" in self.cos:
					for i in [3, 5]:
						msg_cos.data[i] = xc * sc + 0.5
				if "y" in self.cos:
					for i in [2, 3]:
						msg_cos.data[i] = xc * sc + 0.5
				if "e" in self.cos:
					for i in [4, 5]:
						msg_cos.data[i] = xc * sc + 0.5
				if "w" in self.cos:
					msg_cos.data[1] = xc * 0.5 + 0.5
				if "d" in self.cos:
					msg_cos.data[0] = xc * 0.5 + 0.5
				if "x" in self.cos:
					if xc2 >= 0:
						msg_cos.data[1] = xc * 0.5 + 0.5
					else:
						msg_cos.data[0] = xc * 0.5 + 0.5
				self.pub_cos.publish(msg_cos)

			# send wheels
			if not self.wheels is None:
				v = 0.0
				Tq = 0.2
				T = T_ramp_up_down
				t1 = Tq
				t2 = t1 + T
				t3 = t2 + T
				t4 = t3 + Tq
				if t_now < t1:
					v = 0.0
				elif t_now < t2:
					v = (t_now - t1) / T
				elif self.forever:
					v = 1.0
				elif t_now < t3:
					v = 1.0 - (t_now - t2) / T
				elif t_now < t4:
					v = 0.0
				else:
					self.active = False
				msg_wheels.twist.linear.x = v * self.wheels
				msg_wheels.twist.angular.z = 0.0
				self.pub_wheels.publish(msg_wheels)
				self.imp_report_wheels(msg_wheels)

			# send wheels
			if not self.wheelsf is None:
				msg_wheels.twist.linear.x = self.wheelsf
				msg_wheels.twist.angular.z = 0.0
				self.pub_wheels.publish(msg_wheels)
				self.imp_report_wheels(msg_wheels)

			# send wheels
			if not self.spin is None:
				v = 0.0
				Tq = 0.2
				T = 1.0
				t1 = Tq
				t2 = t1 + T
				t3 = t2 + T
				t4 = t3 + Tq
				if t_now < t1:
					v = 0.0
				elif t_now < t2:
					v = (t_now - t1) / T
				elif self.forever:
					v = 1.0
				elif t_now < t3:
					v = 1.0 - (t_now - t2) / T
				elif t_now < t4:
					v = 0.0
				else:
					self.active = False
				msg_wheels.twist.linear.x = 0.0
				msg_wheels.twist.angular.z = v * 6.2832 * self.spin
				self.pub_wheels.publish(msg_wheels)
				self.imp_report_wheels(msg_wheels)

			# send push
			if self.push:
				msg_push.link = miro.constants.LINK_HEAD
				msg_push.flags = miro.constants.PUSH_FLAG_VELOCITY
				msg_push.pushpos = geometry_msgs.msg.Vector3(miro.constants.LOC_NOSE_TIP_X, miro.constants.LOC_NOSE_TIP_Y, miro.constants.LOC_NOSE_TIP_Z)
				msg_push.pushvec = geometry_msgs.msg.Vector3(0.0, 0.2 * xk, 0.0)
				self.pub_push.publish(msg_push)

			self.illum = True
			# send illum
			if self.illum:
				q = int(xcc * -127 + 128)
				if t_now >= 4.0:
					self.active = False
					q = 0
				for i in range(0, 3):
					msg_illum.data[i] = (q << ((2-i) * 8)) | 0xFF000000 
				for i in range(3, 6):
					msg_illum.data[i] = (q << ((i-3) * 8)) | 0xFF000000

				self.pub_illum.publish(msg_illum)
			# state
			time.sleep(0.02)
			self.count = self.count + 1
			t_now = t_now + 0.02

		# end loop

	def __init__(self, input):

		rospy.init_node("Action_from_interaction", anonymous=True)

		# state
		self.count = 0
		self.active = False
		self.forever = False

		# input
		#self.report_input = True
		self.sensors = None

		self.kin_sensor = None

		# options
		self.report_wheels = False
		#self.report_file = None
		self.wheels = None
		self.wheelsf = None
		self.spin = None
		self.kin = ""
		self.cos = ""
		self.illum = False
		self.push = False

		self.audio = streamer(input)

		# move ears
		if input =="ph1_intro":			# eyes
			self.cos = "y"
			self.kin = ""
		elif input == "ph1_same":		# wag
			self.cos = "w"
		elif input =="ph1_shake":		# workout
			self.cos = "lrx"
			self.kin = "lyp"
		elif input == "ph1_hold": 		# wag
			self.cos = "w"
		elif input =="ph1_talk":		# ears
			self.cos = "e"
		elif input == "ph1_squeeze":	# spin
			self.spin = 2.0
		elif input == "ph1_outro":		# eyes and ears
			self.cos = "ey"


		if input == "ph2_congratulations":
			self.illum = True
			self.cos = "lrx"
			self.kin = "lyp"
		elif input == "ph2_music1" or input == "ph2_music2":
			self.cos = "lrx"
			self.kin = "lyp"
		elif input == "ph2_sh_donkey" or input == "ph2_sh_dog" or input == "ph2_sh_cat" or input == "ph2_sh_cow" or input == "ph2_sh_lion":
			self.cos = "w" # wag
			self.kin = ""
		elif input == "ph2_cat" or input == "ph2_dog":
			self.cos = "d"
			self.kin = ""
		elif input == "ph2_donkey" or "ph2_cow":
			self.cos = "dey" # droop
		elif input == "ph2_lion":
			self.cos = "lrx"
			self.kin = "lyp"

		if input == "ph3_intro":
			self.cos = "e"
			self.kin = ""
		elif input == "ph3_1":
			self.cos = "d"
			self.kin = ""
		elif input == "ph3_3":
			self.cos = "w"
		elif input == "ph3_2" or input == "ph3_5":
			self.cos = "yew"
			self.kin = ""
		elif input == "ph3_6" or input == "ph3_4":
			self.cos = "lrx"
			self.kin = "lyp"
			self.illum = True

				# simon says
		'''
			"ph2_intro_g3" : "ph2_26.mp3",
			"ph2_a" : "ph2_27.mp3",
			"ph2_blab" : "ph2_28.mp3",
			"ph2_hold" : "ph2_29.mp3",
			"ph2_shake" : "ph2_30.mp3",
			"ph2_squeeze" : "ph2_31.mp3",
			"ph2_goodjob" : "ph2_32.mp3",
			"ph2_next" : "ph2_33.mp3",
		'''

		# if sad story, droop

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/control/cmd_vel"
		print ("publish", topic)
		self.pub_wheels = rospy.Publisher(topic, TwistStamped, queue_size=0)

		# publish
		topic = topic_base_name + "/control/kinematic_joints"
		print ("publish", topic)
		self.pub_kin = rospy.Publisher(topic, JointState, queue_size=0)

		# publish
		topic = topic_base_name + "/control/cosmetic_joints"
		print ("publish", topic)
		self.pub_cos = rospy.Publisher(topic, Float32MultiArray, queue_size=0)

		# publish
		topic = topic_base_name + "/control/illum"
		print ("publish", topic)
		self.pub_illum = rospy.Publisher(topic, UInt32MultiArray, queue_size=0)

		# publish
		topic = topic_base_name + "/core/mpg/push"
		print ("publish", topic)
		self.pub_push = rospy.Publisher(topic, miro.msg.push, queue_size=0)

		# publish
		topic = topic_base_name + "/control/flags"
		print ("publish", topic)
		self.pub_flags = rospy.Publisher(topic, UInt32, queue_size=0)

		# subscribe
		topic = topic_base_name + "/sensors/package"
		print ("subscribe", topic)
		self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package, queue_size=5, tcp_nodelay=True)

		# wait for connect
		print ("wait for connect...")
		time.sleep(1)

		# set to active
		self.active = True

if __name__ == "__main__":

	# normal singular invocation
	main = controller("ph1_intro")
	main.loop(1000,0)
