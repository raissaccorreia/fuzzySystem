import sys, time
sys.path.insert(0, '../lib')
import vrep

class Robot():
	def __init__(self):
		self.ROBOT_WIDTH = 0.381
		self.WHEEL_RADIUS = 0.195/2.0
		self.SERVER_IP = "127.0.0.1"
		self.SERVER_PORT = 25000
		self.clientID = self.start_sim()
		self.us_handle, self.vision_handle, self.laser_handle = self.start_sensors()
		self.motors_handle = self.start_motors()
		self.robot_handle = self.start_robot()

	def start_sim(self):
		"""
			Function to start the simulation. The scene must be running before running this code.
		    Returns:
		        clientID: This ID is used to start the objects on the scene.
		"""
		vrep.simxFinish(-1)
		clientID = vrep.simxStart(self.SERVER_IP, self.SERVER_PORT, True, True, 2000, 5)
		if clientID != -1:
			print("Connected to remoteApi server.")
		else:
			vrep.simxFinish(clientID)
			sys.exit("\033[91m ERROR: Unable to connect to remoteApi server. Consider running scene before executing script.")

		return clientID

	def get_connection_status(self):
		"""
			Function to inform if the connection with the server is active.
			Returns:
				connectionId: -1 if the client is not connected to the server.
				Different connection IDs indicate temporary disconections in-between.
		"""
		return vrep.simxGetConnectionId(self.clientID)

	def start_sensors(self):
		"""
			Function to start the sensors.
		    Returns:
		        us_handle: List that contains each ultrassonic sensor handle ID.
				vision_handle: Contains the vision sensor handle ID.
				laser_handle: Contains the laser handle ID.
		"""
		#Starting ultrassonic sensors
		us_handle = []
		sensor_name=[]
		for i in range(0,15):
			sensor_name.append("Pioneer_p3dx_ultrasonicSensor" + str(i+1))

			res, handle = vrep.simxGetObjectHandle(self.clientID, sensor_name[i], vrep.simx_opmode_oneshot_wait)
			if(res != vrep.simx_return_ok):
				print ("\033[93m "+ sensor_name[i] + " not connected.")
			else:
				print ("\033[92m "+ sensor_name[i] + " connected.")
				us_handle.append(handle)

		#Starting vision sensor
		res, vision_handle = vrep.simxGetObjectHandle(self.clientID, "Vision_sensor", vrep.simx_opmode_oneshot_wait)
		if(res != vrep.simx_return_ok):
			print ("\033[93m Vision sensor not connected.")
		else:
			print ("\033[92m Vision sensor connected.")

		#Starting laser sensor
		res, laser_handle = vrep.simxGetObjectHandle(self.clientID, "fastHokuyo", vrep.simx_opmode_oneshot_wait)
		if(res != vrep.simx_return_ok):
			print ("\033[93m Laser not connected.")
		else:
			print ("\033[92m Laser connected.")

		return us_handle, vision_handle, laser_handle

	def start_motors(self):
		"""
			Function to start the motors.
		    Returns:
		        A dictionary that contains both motors handle ID.
		"""

		res, left_handle = vrep.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
		if(res != vrep.simx_return_ok):
			print("\033[93m Left motor not connected.")
		else:
			print("\033[92m Left motor connected.")

		res, right_handle = vrep.simxGetObjectHandle(self.clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait)
		if(res != vrep.simx_return_ok):
			print("\033[93m Right motor not connected.")
		else:
			print("\033[92m Right motor connected.")

		return {"left": left_handle, "right":right_handle}

	def start_robot(self):
		"""
			Function to start the robot.
			Returns:
				robot_handle: Contains the robot handle ID.
		"""
		res, robot_handle = vrep.simxGetObjectHandle(self.clientID, "Pioneer_p3dx", vrep.simx_opmode_oneshot_wait)
		if(res != vrep.simx_return_ok):
			print("\033[93m Robot not connected.")
		else:
			print("\033[92m Robot connected.")

		return robot_handle

	def read_ultrassonic_sensors(self):
		"""
			Reads the distances from the 16 ultrassonic sensors.
			Returns:
				distances: List with the distances in meters.
		"""
		distances = []
		noDetectionDist = 5.0 #Here we define the maximum distance as 5 meters

		for sensor in self.us_handle:
			res, status, distance,_,_ = vrep.simxReadProximitySensor(self.clientID, sensor, vrep.simx_opmode_streaming)
			while(res != vrep.simx_return_ok):
				res, status, distance,_,_ = vrep.simxReadProximitySensor(self.clientID, sensor, vrep.simx_opmode_buffer)

			if(status != 0):
				distances.append(distance[2])
			else:
				distances.append(noDetectionDist)


		return distances

	def read_vision_sensor(self):
		"""
			Reads the image raw data from vrep vision sensor.
			Returns:
				resolution: Tuple with the image resolution.
				image: List with the image data.
		"""
		res, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, self.vision_handle, 0, vrep.simx_opmode_streaming)
		while(res != vrep.simx_return_ok):
			res, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, self.vision_handle, 0, vrep.simx_opmode_buffer)

		return resolution, image

	def read_laser(self):
		"""
			Gets the 572 points read by the laser sensor. Each reading contains 3 values (x, y, z) of the point relative to the sensor position.
			Returns:
				laser: List with 1716 values of x, y and z from each point.
		"""
		res, laser = vrep.simxGetStringSignal(self.clientID,"LasermeasuredDataAtThisTime", vrep.simx_opmode_streaming)
		laser = vrep.simxUnpackFloats(laser)
		while(res != vrep.simx_return_ok):
			res, laser = vrep.simxGetStringSignal(self.clientID,"LasermeasuredDataAtThisTime", vrep.simx_opmode_buffer)
			laser = vrep.simxUnpackFloats(laser)

		return laser

	def stop(self):
		"""
			Sets the motors velocities to 0.
		"""
		vrep.simxSetJointTargetVelocity(self.clientID, self.motors_handle["left"], 0, vrep.simx_opmode_streaming)
		vrep.simxSetJointTargetVelocity(self.clientID, self.motors_handle["right"], 0, vrep.simx_opmode_streaming)
		time.sleep(0.1)

	def set_left_velocity(self, vel):
		"""
			Sets the velocity on the left motor.
			Args:
				vel: The velocity to be applied in the motor (rad/s)
		"""
		vrep.simxSetJointTargetVelocity(self.clientID, self.motors_handle["left"], vel, vrep.simx_opmode_streaming)

	def set_right_velocity(self, vel):
		"""
			Sets the velocity on the right motor.
			Args:
				vel: The velocity to be applied in the motor (rad/s)
		"""
		vrep.simxSetJointTargetVelocity(self.clientID, self.motors_handle["right"], vel, vrep.simx_opmode_streaming)

	def set_velocity(self, V, W):
		"""
			Sets a linear and a angular velocity on the robot.
			Args:
				V: Linear velocity (m/s) to be applied on the robot along its longitudinal axis.
				W: Angular velocity (rad/s) to be applied on the robot along its axis of rotation, positive in the counter-clockwise direction.
		"""
		left_vel = (V - W*(self.ROBOT_WIDTH/2))/self.WHEEL_RADIUS
		right_vel = (V + W*(self.ROBOT_WIDTH/2))/self.WHEEL_RADIUS
		vrep.simxSetJointTargetVelocity(self.clientID, self.motors_handle["left"], left_vel, vrep.simx_opmode_streaming)
		vrep.simxSetJointTargetVelocity(self.clientID, self.motors_handle["right"], right_vel, vrep.simx_opmode_streaming)

	def get_current_position(self):
		"""
			Gives the current robot position on the environment.
			Returns:
				position: Array with the (x,y,z) coordinates.
		"""
		res, position = vrep.simxGetObjectPosition(self.clientID, self.robot_handle, -1, vrep.simx_opmode_streaming)
		while(res != vrep.simx_return_ok):
			res, position = vrep.simxGetObjectPosition(self.clientID, self.robot_handle, -1, vrep.simx_opmode_streaming)

		return position

	def get_current_orientation(self):
		"""
			Gives the current robot orientation on the environment.
			Returns:
				orientation: Array with the euler angles (alpha, beta and gamma).
		"""
		res, orientation = vrep.simxGetObjectOrientation(self.clientID, self.robot_handle, -1, vrep.simx_opmode_streaming)
		while(res != vrep.simx_return_ok):
			res, orientation = vrep.simxGetObjectOrientation(self.clientID, self.robot_handle, -1, vrep.simx_opmode_streaming)

		return orientation
