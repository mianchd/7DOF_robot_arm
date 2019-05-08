
import sys
import os
import time
import psutil

import numpy as np
import math
import random
from pprint import pprint
import matplotlib.pyplot as plt
import cv2
from PIL import Image

sys.path.append(os.path.join(sys.path[0],'vrep_boilerplate'))
import vrep


"""
V-REP Installation folder here. The Extracted archive of V-REP installation files
"""
vrep_dir = "/home/ubuntu/Downloads/V-REP_PRO_EDU_V3_6_1_Ubuntu18_04/"

blocking = vrep.simx_opmode_blocking
oneshot = vrep.simx_opmode_oneshot
streaming = vrep.simx_opmode_streaming
buffering = vrep.simx_opmode_buffer
oneshot_wait = vrep.simx_opmode_oneshot_wait

def wait(duration=5, str_=''):
	print(str_, end="")
	for _ in range(duration):
		print(".", end="", flush=True)
		time.sleep(1)
	print()

if not 'vrep' in [p.name() for p in psutil.process_iter()]:
	try:
		scene_file = os.path.join(sys.path[0], 'scenes/7dof_arm_sim.ttt')
		command_to_execute = "gnome-terminal -x " + vrep_dir + "vrep.sh " + scene_file
		print(f"Executing: {command_to_execute}")
		os.system(command_to_execute)
		wait(4, 'Launching V-REP.')
		print('Loading Scene File')
	except:
		print("VREP Scene file not found at /Scenes/7dof_arm_sim.ttt")
		sys.exit(0)
else:
	print("V-REP instance detected.")

# print(sys.path[0])
# print(os.path.dirname(sys.path[0]))

class joints():
	"""[class for controlling joints in V-REP]

	Variables:

	"""
	def __init__(self, vrep_name):
		self.vrep_name = vrep_name
		self.handle = self.get_Handle()


	def get_Handle(self):
		returnCode, jointHandle = vrep.simxGetObjectHandle(clientID, self.vrep_name, blocking)
		if returnCode == vrep.simx_return_ok:
			return jointHandle
		else: return None

	def set_angle(self, angle):
		# accepts angle in degrees
		assert self.handle is not None, "{} Handle does not exist".format(self.vrep_name)
		rc = vrep.simxSetJointPosition(clientID, self.handle, math.radians(angle), vrep.simx_opmode_streaming)
		return rc

	def set_angle_target(self, angle):
		# Works only if joint is in torque/force mode
		# Physics engine figures out the torque required to take joint to the target angle
		# Should have a true loop which evaluates if the position has been reached.
		# We can query the torque being applied with vrep.simxGetJointForce (Differnt physics engine returns different values)
		vrep.simxSetJointTargetPosition(clientID, self.handle, math.radians(angle), streaming)

	def set_max_torque(self, torque):
		# Set Maximum torque for a joint
		rc = vrep.simxSetJointForce(clientID, self.handle, torque, oneshot)

	def set_target_velocity(self, vel):
		# Control joint by velocity - VREP applies approprite torque to reach the velocity
		rc = vrep.simxSetJointTargetVelocity(clientID, self.handle, vel, oneshot)

	def get_angle(self, first=False):
		assert self.handle is not None, "{} Handle does not exist".format(self.vrep_name)
		mode = streaming if first else buffering
		rc, angle = vrep.simxGetJointPosition(clientID, self.handle, mode)
		# print("{} is currently {}".format(self.vrep_name, math.degrees(angle)))
		return round(math.degrees(angle), 3)

	def get_torque(self, first=False):
		"""[Returns the torque exerted on the joint]

		[Different physics engine will return different values]

		Keyword Arguments:
			first {bool} -- [If true, the function is called in streaming mode,
			The first call to the function should always be as first = True] (default: {False})

		Returns:
			[float] -- [torque or force exerted on the joint]
		"""

		assert self.handle is not None, "{} Handle does not exist".format(self.vrep_name)
		mode = streaming if first else buffering
		rc, torque = vrep.simxGetJointForce(clientID, self.handle, mode)
		return round(torque, 3)

	def get_Transformation_Matrix(self, first=False):
		assert self.handle is not None, "{} Handle does not exist".format(self.vrep_name)
		mode = streaming if first else buffering
		rc, matrix = vrep.simxGetJointMatrix(clientID, self.handle, mode)
		return matrix
		"""
		(the last row of the 4x4 matrix (0,0,0,1) is not needed)
		The x-axis of the orientation component is (matrix[0],matrix[4],matrix[8])
		The y-axis of the orientation component is (matrix[1],matrix[5],matrix[9])
		The z-axis of the orientation component is (matrix[2],matrix[6],matrix[10])
		The translation component is (matrix[3],matrix[7],matrix[11])
		"""

	# --------------------------------------------------------------------------------------------------------------------- Jog
	def jog_with_torque(self, step=1):
		# if random_dir:
		# 	rand = random.random()
		# 	if rand > 0.5:
		# 		step = abs(step)
		# 	else:
		# 		step = -abs(step)
		curr_angle = self.get_angle() # returned in degrees
		new_angle = curr_angle + step
		rc = self.set_angle_target(new_angle)

class robot_RL_Env(object):
	def __init__(self):

		vrep.simxFinish(-1)
		self.client_id = vrep.simxStart(
			'127.0.0.1', 19997, True, True, 5000, 5)

		if self.client_id != -1:  # check if client connection successful
			print('Connected to remote API server.')
		else:
			print('Connection not successful')
			sys.exit('Could not connect')

		global clientID
		clientID = self.client_id

		self.dict_name_to_handle = self.get_all_objects_data(name_to_handle=True)
		self.dict_handle_to_name = self.get_all_objects_data(name_to_handle=False)

		self.sensor_handle = None
		self.dist_handle = None
		self.start_sim()
		self.joint_dict = {}							# Dictionary containing objects of joint class as values
		self.get_handles()
		self.initialize_target_position_tracking()
		self.initialize_distance_calculation()
		self.initialize_joint_values()
		self.initialize_proximity_sensor()
		self.initialize_camera_image('base_camera', 'EE_camera')


	def reset(self):
		stop = vrep.simxStopSimulation(
			self.client_id, oneshot_wait)
		print("Resetting Simulation")
		self.start_sim()

	def reset_for_episode(self):
		self.move_to_default_config()
		self.proceed_simulation()
		ob, r, d = self.step()
		return ob, r, d

	def start_sim(self):
		start = vrep.simxStartSimulation(
			self.client_id, oneshot_wait)
		rc = vrep.simxSynchronous(self.client_id, True)
		print("Starting Simulation.")

	def destroy(self):
		vrep.simxStopSimulation(self.client_id, blocking)
		vrep.simxFinish(self.client_id)
		print("Stopping Simulation")

	def get_handles(self):
		for i in range(1,8):
			joint_index = i - 1
			self.joint_dict['joint' + str(joint_index)] = joints('redundantRob_joint' + str(i))

	def get_angles(self, print_it = False):
		list_of_angles = []
		for key, joint in self.joint_dict.items():
			list_of_angles.append(joint.get_angle())

		if print_it:
			for key, joint in self.joint_dict.items():
				print("{} has Angle\t{}".format(key, joint.get_angle()))

		return list_of_angles

	def move_to_default_config(self):
		default_config = [0,0,0,0,0,-180,0]

		for (key, joint), default_angle in zip(self.joint_dict.items(), default_config):
			joint.set_angle_target(default_angle)
			self.proceed_simulation()
		print("Default Joint Configuration.")

	def read_dist_to_target(self):
		if self.dist_handle is None:
			print("Please Initilise distance calculation first...")
			return
		rc, dist_to_EE = vrep.simxReadDistance(clientID, self.dist_handle, buffering)
		if rc == 0:
			return round(dist_to_EE, 3)
		else:
			return 0
			print("Couldn't calculate Distance")

	def initialize_distance_calculation(self):
		rc, dist_handle_ = vrep.simxGetDistanceHandle(clientID, 'EE_to_target', blocking)
		if rc == 0:
			# print("Distance calculation started.")
			rc, dist_to_EE = vrep.simxReadDistance(clientID, dist_handle_, streaming)
			self.dist_handle = dist_handle_

	def initialize_joint_values(self):
		for key, joint in self.joint_dict.items():
			joint.get_angle(first=True)

	def initialize_target_position_tracking(self):
		rc, target_position = vrep.simxGetObjectPosition(
												self.client_id,
												self.dict_name_to_handle['redundantRob_manipSphere'],
												self.dict_name_to_handle['redundantRob_link0'], # -1 for absolute position
												streaming
												)

	def proceed_simulation(self):
		vrep.simxSynchronousTrigger(self.client_id)

	def get_joint_handles_as_list(self):
		joint_handles = []
		for key, joint in self.joint_dict.items():
			joint_handles.append(joint.handle)
		return joint_handles

	def get_object_handles(self):
		rc, arr_Handles = vrep.simxGetObjects(self.client_id, vrep.sim_object_joint_type, blocking)
		return arr_Handles

	def get_data_multiple_objects(self):
		rc, handles, intData, floatData, strData = vrep.simxGetObjectGroupData(self.client_id, vrep.sim_object_joint_type, 15, blocking)
		# 15 = returns 2 float values, [position and torques], Can get all sort of data, check documentation.

		# print("Handles: " + str(handles))
		# print("Int Data: " + str(intData))
		# print("Float Data: " + str(floatData))
		# print("String Data: " + str(strData))

		for i in range(len(floatData)):
			print(i)
			if i % 2 == 0: # If Even
				degrees = math.degrees(floatData[i])
				degrees = round(degrees, 3)
				print("Position: {}".format(degrees))
			else:
				torque = round(floatData[i], 3)
				print("Torque: " +  str(torque))

		# Could be all objects, a specific object type, or a user-defined collection

	def change_joints_mode(self, mode):
		# Not working, no signal received on V-REP side
		# for key, value in self.joint_dict.items():
		# 	msg = value.vrep_name + '_' + mode
		rc, intData, floatData, strData, bufferData = vrep.simxCallScriptFunction(
			self.client_id,
			'redundantRobot',
			vrep.sim_scripttype_childscript,
			'displayText_function',
			[],
			[],
			'Hello world from Mian',
			[],
			blocking)
			# vrep.simxSetStringSignal(self.client_id, "jointModeCmd", joint.vrep_name+'_'+mode, oneshot)


	def jog_all_joints(self, list_of_steps):
		for (key, joint), step_size in zip(self.joint_dict.items(), list_of_steps):
			joint.jog_with_torque(step=step_size)
			# print("{} is stepping by angle: {}".format(joint.vrep_name, step_size))

	def get_handle(self, object_str, print_it=False):
		rc, tempObject = vrep.simxGetObjectHandle(self.client_id, object_str, blocking)
		if tempObject == 0:
			print("Coudn't get Handle for '" + object_str +  "'\t--Check the Name String!")
			self.destroy()
			sys.exit(0)
		if print_it:
			print(object_str + " handle is: " + str(tempObject))
		return tempObject

	def initialize_proximity_sensor(self):
		self.sensor_handle = self.get_handle('Proximity_sensor')
		if self.sensor_handle is not None:
			vrep.simxReadProximitySensor(self.client_id, self.sensor_handle, streaming)
		else:
			print("Coudln't get the proximity sensor handle.")

	def get_all_objects_data(self, name_to_handle):
		temp_dict = {}
		rc, handles, intData, floatData, stringData = vrep.simxGetObjectGroupData(
														self.client_id,
														vrep.sim_appobj_object_type,
														0,
														blocking)
		if name_to_handle:
			for i, j in zip(stringData, handles):
				temp_dict[i] = j
		else:
			for i, j in zip(handles, stringData):
				temp_dict[i] = j

		return temp_dict

	def read_proximity_sensor(self):
		rc, detectionState, detectedPoint, detectedObjectHandle, DSNV = vrep.simxReadProximitySensor(self.client_id, self.sensor_handle, buffering)
		distance = round(detectedPoint[2], 3)
		if detectionState:
			print("Detected Object: {}\t|\tDistance: {}\tMeters".format(self.dict_handle_to_name[detectedObjectHandle], distance))

	def initialize_camera_image(self, *cameras):
		for camera in cameras:
			rc, resolution, image = vrep.simxGetVisionSensorImage(
				self.client_id,
				self.dict_name_to_handle[camera],
				0,  # options=0 -> RGB
				streaming,
			)

	def get_vision_image(self, visionSensorName):
		rc, resolution, image = vrep.simxGetVisionSensorImage(
			self.client_id,
			self.dict_name_to_handle[visionSensorName],
			0,  # options=0 -> RGB
			buffering
		)

		if rc == vrep.simx_return_ok:
			return resolution, image
		elif rc == vrep.simx_return_novalue_flag:
			print ("\nNo image yet\n")
		else:
			print (rc)

	def get_processed_camera_image(self, camera_name):
		resolution, image = self.get_vision_image(camera_name)
		image_byte_array = np.array(image, dtype='uint8')
		image_byte_array.resize([resolution[0], resolution[1], 3])
		image_byte_array = cv2.flip(image_byte_array, 0)
		image_byte_array = cv2.cvtColor(image_byte_array, cv2.COLOR_BGR2RGB)
		return image_byte_array

	def stream_camera_images(self, *cameras):

		while True:
			for camera in cameras:
				image_byte_array = self.get_processed_camera_image(camera)
				cv2.imshow('OpenCV V-REP Camera Images - ' + camera, image_byte_array)

			if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
				break
			self.proceed_simulation()

		print("Keyboard Interrupt --> |")
		cv2.destroyAllWindows()

	def apply_action_with_index(self, index):
		joint_index = index // 2

		step = 2
		if index % 2 != 0:		# if Odd
			step = -step

		self.joint_dict['joint' + str(joint_index)].jog_with_torque(step)


	def step(self):
		"""Observes the current state of the simulation
		[Returns the Observation, such as sensor data and reward
		For reward calculation need to step the simulation 1 time or need to get previous obsrevation.
		"""

		""" Episode Termination Check"""
		self.proceed_simulation()
		done = False
		""" Observation """
		joint_angles = self.get_angles()
		rc, target_position = vrep.simxGetObjectPosition(self.client_id,
											self.dict_name_to_handle['redundantRob_manipSphere'],
											self.dict_name_to_handle['redundantRob_link0'], # -1 for absolute position
											buffering)
		# image_base_camera = get_processed_camera_image(base_camera)
		observation = [joint_angles + target_position]

		# observation = [round(t, 3) for t in observation[0]]

		observation = np.array(observation)
		# Stack > 1 observations together to form one observation. This will capture direction, velocity data
		# To get just the robot joint configuration and location of cube this is not required. (Theses attest)

		reward = -1

		if self.read_dist_to_target() < 0.02:
			done = True
			reward = 100

		# reward = -self.read_dist_to_target()
		return observation, reward, done

class agent_Random(object):
	def __init__(self, env_):
		self.env = env_

	def get_action(self):
		return [random.randint(-5, 5) for x in range(7)]

	def move_randomly(self, number=100):
		for _ in range(number):
			for (key, joint), step in zip(self.env.joint_dict.items(), self.get_action()):
				joint.jog_with_torque(step)
				self.env.proceed_simulation()			# Speed of the simulation and smoothness changes when we update every timestep.
			print(self.env.read_dist_to_target())


class agent_RL():

	def __init__(self, env_):
		self.env = env_				# Environment agent is acting in

		self.epsilon = 1			# Epsilon Greedy with Decay
		self.epsilon_decay = 0.999 	# Decay by 10th of a percent
		self.epsilon_min = 0.01 	# One percent randomisation as minimum

		self.episodes = 1000		# Total number of Episodes
		self.alpha = 0.1 			# Learning Rate
		self.gamma = 0.9 			# Discount Factor
		self.learning_rate = 0.5

		self.model = self.setupNN()
		self.observation = None

	def train(self):

		for e in range(self.episodes):

			print('=======================================================================')
			print("Episode: {}/{}".format(e, self.episodes))
			print('=======================================================================')

			self.observation, reward, done = self.env.reset_for_episode()

			# I = self.model.predict(self.observation)
			# action_index = np.argmax(I) # return index as well.

			reward_score = 0
			while not done:
				new_observation, reward, done = self.env.step()
				print("Obsrevation: {}\nReward: {}\n".format(self.observation[0], reward))

				reward_score += reward
				print("Total Reward: {}".format(reward_score))
				action_index = self.choose_action()
				print('------------------------------------------------------------------')

				self.env.apply_action_with_index(action_index)
				# self.update_q(self.observation, action_list, reward, new_observation)
				self.observation = new_observation

			# Offline Learning -- BackPropagation happens only when episode completes
			print("\nEE reached to Goal.\nDistance to Target: {}\nJoint Configuration: {}".format(self.env.read_dist_to_target(), self.env.get_angles()))
			# cost = reward + self.gamma * np.max(self.model.predict(self.observation))
			# I[index] = cost
			# target = I
			# self.model.fit(y=target)

	def choose_action(self):
		if random.random() > self.epsilon:
			predictions = self.model.predict(self.observation) # Exploit
			action_index = np.argmax(predictions)
			print("Index of Predicted action: {}".format(action_index))
		else:
			action_index = random.randint(0, 13) # Explore
			print("Index of Randomly choosen action: {}".format(action_index))

		if self.epsilon > self.epsilon_min:
			self.epsilon = self.epsilon * self.epsilon_decay
			print("Epsilon reduced to: {}".format(self.epsilon))
		return action_index

	def update_q(self, state, action, reward, state_new):
		# stays for reference
		q_sa = self.q_table[state, action]
		td_error = reward + self.gamma * np.max(self.q_table[state_new]) - q_sa
		self.q_table[state, action] = q_sa + self.alpha * td_error

	def setupNN(self):
		"""Configures and returns a NN model
		[returns a model model that can be trained and evaluated]
		"""
		os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
		from keras.models import Sequential
		from keras.layers import Dense, Activation
		from keras.optimizers import Adam

		# Dense = Fully connected (flattened) layer
		# input_shape(number of rows[timesteps], number of columns [features])

		model = Sequential()
		# Might have to use input_shape as it is first layer
		model.add(Dense(units=32, input_dim=10))		# Model will take array of shape (* rows, 10 columns) as input
														# The Layer will output array of shape (*, 32) 32 is arbitrary hyperparameter
		model.add(Activation('relu'))

		model.add(Dense(units=24))						# Deep Layer. 24 Nuerons is arbitrary
		model.add(Activation('relu'))

		model.add(Dense(units=14))						# Output of the last layer (corresponding to probability of actions)
		model.add(Activation('linear'))

		obj_optimizer = Adam(lr=self.learning_rate)

		model.compile(optimizer=obj_optimizer, loss='mse')

		return model

def main():
	robotEnv = robot_RL_Env()
	robotEnv.move_to_default_config()
	# robotEnv.stream_camera_images('EE_camera', 'base_camera')

	#random_agent = agent_Random(robotEnv)
	#random_agent.move_randomly()

	myagent = agent_RL(robotEnv)
	myagent.train()


	robotEnv.destroy()


if __name__ == '__main__':
		main()
