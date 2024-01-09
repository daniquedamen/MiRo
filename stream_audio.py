import rospy
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray, Int16MultiArray, String
import time
import sys
import os
import numpy as np
import hashlib

# messages larger than this will be dropped by the receiver
MAX_STREAM_MSG_SIZE = (4096 - 48)

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 2000 samples will
# buffer for quarter of a second, for instance.
BUFFER_STUFF_BYTES = 4000

# media source list
DIR_SOURCE = [
	"../../bin/shared/HELPER/audio" #"../../share/media"
	]

# list directories belonging to other releases
DIR_ROOT="../../../"
DIR_MDK = []
subdirs = os.listdir(DIR_ROOT)
for d in subdirs:
	if len(d) < 3:
		continue
	if d[0:3] != "mdk":
		continue
	DIR_MDK.append(d)

# rsort them so we prioritise more recent ones
DIR_MDK.sort()
DIR_MDK.reverse()

# and append to media source list
for d in DIR_MDK:
	DIR_SOURCE.append(DIR_ROOT + d + "/bin/shared/HELPER/audio") # "/share/media")

# append dev directories
DIR_SOURCE.append(os.getenv('HOME') + "/lib/miro2x/mdk/share/media")

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

# index
index = []
index_special = []

# not digits
digits = None

# for each source directory
for dir in DIR_SOURCE:

	# if is directory
	if os.path.isdir(dir):

		# get files
		files = [f for f in os.listdir(dir) if os.path.isfile(os.path.join(dir, f)) and f.endswith(".mp3")]

		# add to array
		for file in files:

			# special files prepended with underscore
			if file[0] == '_':

				if file[1] != '_':
					index_special.append([file, os.path.join(dir, file)])

			# normal files
			else:

				# append to index
				index.append([file, os.path.join(dir, file)])

		# if any were found, we stop there, because the source
		# directories are intended not to union each other, but
		# to override each other, in the order specified in
		# DIR_SOURCE
		if len(index) > 0:
			print ("reading from:", dir)
			break

# sort array
index.sort()

################################################################

class streamer:

	def callback_log(self, msg):

		sys.stdout.write(msg.data)
		sys.stdout.flush()

	def callback_stream(self, msg):

		self.buffer_space = msg.data[0]
		self.buffer_total = msg.data[1]

	def loop(self, event):

		state_file = None

		# periodic reports
		count = 0

		# safety dropout if receiver not present
		dropout_data_r = -1
		dropout_count = 3

		# loop
		while not rospy.core.is_shutdown() and not event.is_set():

			# check state_file
			if not state_file is None:
				if not os.path.isfile(state_file):
					break

			# if we've received a report
			if self.buffer_total > 0:

				# compute amount to send
				buffer_rem = self.buffer_total - self.buffer_space
				n_bytes = BUFFER_STUFF_BYTES - buffer_rem
				n_bytes = max(n_bytes, 0)
				n_bytes = min(n_bytes, MAX_STREAM_MSG_SIZE)

				# if amount to send is non-zero
				if n_bytes > 0:

					msg = Int16MultiArray(data = self.data[self.data_r:self.data_r+n_bytes])
					self.pub_stream.publish(msg)
					self.data_r += n_bytes

			# break
			if self.data_r >= len(self.data):
				print("completely played")
				return 1

			# report once per second
			if count == 0:
				count = 10
				print ("streaming:", self.data_r, "/", len(self.data), "bytes")

				# check at those moments if we are making progress, also
				if dropout_data_r == self.data_r:
					if dropout_count == 0:
						print ("dropping out because of no progress...")
						break
					print ("dropping out in", str(dropout_count) + "...")
					dropout_count -= 1
				else:
					dropout_data_r = self.data_r

			# count tenths
			count -= 1
			time.sleep(0.1)


	def __init__(self, action):
		rospy.init_node("Action_from_interaction", anonymous=True)

		action_tracks = {
			"ph1_intro" : "ph1_1.mp3",
			"ph1_same" : "ph1_2.mp3",
			"ph1_shake" : "ph1_3.mp3",
			"ph1_hold" : "ph1_4.mp3",
			"ph1_talk" : "ph1_5.mp3",
			"ph1_squeeze" : "ph1_6.mp3",
			"ph1_outro" : "ph1_7.mp3",

			"ph2_intro" : "ph2_8.mp3",
			"ph2_choose" : "ph2_9.mp3",

			"ph2_intro_g1" : "ph2_10.mp3",
			"ph2_music1" : "ph2_11.mp3",
			"ph2_music2" : "ph2_12.mp3",
			"ph2_congratulations" : "ph2_13.mp3",
			"ph2_nexttime" : "ph2_14.mp3",

			"ph2_intro_g2" : "ph2_15.mp3",
			"ph2_sh_donkey" : "ph2_16.mp3",
			"ph2_sh_dog" : "ph2_17.mp3",
			"ph2_sh_cat" : "ph2_18.mp3",
			"ph2_sh_cow" : "ph2_19.mp3",
			"ph2_sh_lion" : "ph2_20.mp3",
			"ph2_cat" : "ph2_21.mp3",
			"ph2_cow" : "ph2_22.mp3",
			"ph2_dog" : "ph2_23.mp3",
			"ph2_donkey" : "ph2_24.mp3",
			"ph2_lion" : "ph2_25.mp3",

			"ph2_intro_g3" : "ph2_26.mp3",
			"ph2_a" : "ph2_27.mp3",
			"ph2_blab" : "ph2_28.mp3",
			"ph2_hold" : "ph2_29.mp3",
			"ph2_shake" : "ph2_30.mp3",
			"ph2_squeeze" : "ph2_31.mp3",
			"ph2_goodjob" : "ph2_32.mp3",
			"ph2_next" : "ph2_33.mp3",

			"ph3_intro" : "ph3_34.mp3",
			"ph3_1" : "ph3_35.mp3",
			"ph3_2" : "ph3_36.mp3",
			"ph3_3" : "ph3_37.mp3",
			"ph3_4" : "ph3_38.mp3",
			"ph3_5" : "ph3_39.mp3",
			"ph3_6" : "ph3_40.mp3",

		}

		TRACK_FILE = action_tracks.get(action)

		print(TRACK_FILE)
		
		TRACK_PATH = "../../../mdk/bin/shared/HELPER/audio/" + TRACK_FILE


		# decode mp3
		file = "/tmp/" + TRACK_FILE + ".decode"
		print("track_file:", TRACK_FILE)
		print("track path:", TRACK_PATH)
		if not os.path.isfile(file):
			cmd = "ffmpeg -y -i \"" + TRACK_PATH + "\" -f s16le -acodec pcm_s16le -ar 8000 -ac 1 \"" + file + "\""
			os.system(cmd)
			if not os.path.isfile(file):
				error('failed decode mp3')

		# load wav
		with open(file, 'rb') as f:
			dat = f.read()
		self.data_r = 0

		# convert to numpy array
		dat = np.fromstring(dat, dtype='int16').astype(np.int32)

		# normalise wav
		#dat = dat.astype(np.float)
		sc = 32767.0 / np.max(np.abs(dat))
		dat = dat * sc
		dat = dat.astype(np.int16)
		dat = dat.tolist()


		# these are for _digits_and_dot_female.mp3
		digits_and_dot = [
			[550, 5348],
			[6100, 11470],
			[13649, 18257],
			[21551, 26383],
			[29512, 33906],
			[37896, 42336],
			[46378, 50548],
			[54434, 58762],
			[61995, 66019],
			[69027, 73927],
			[76593, 80695],
			]

		# handle digits
		if digits:

			# start with some blank so we don't miss beginning
			dat_ = dat
			dat = [dat_[0] * 0] * 8000
			gap = [dat_[0] * 0] * 50

			# now add the digits
			for d in digits:
				if d in "0123456789":
					s = digits_and_dot[int(d) - int("0")]
					dat += dat_[s[0]:s[1]]
					dat += gap
				if d == ".":
					s = digits_and_dot[10]
					dat += dat_[s[0]:s[1]]
					dat += gap

		# store
		self.data = dat

		# state
		self.buffer_space = 0
		self.buffer_total = 0

		# get robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/control/stream"
		print ("publish", topic)
		self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

		# subscribe
		topic = topic_base_name + "/platform/log"
		print ("subscribe", topic)
		self.sub_log = rospy.Subscriber(topic, String, self.callback_log, queue_size=5, tcp_nodelay=True)

		# subscribe
		topic = topic_base_name + "/sensors/stream"
		print ("subscribe", topic)
		self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)


