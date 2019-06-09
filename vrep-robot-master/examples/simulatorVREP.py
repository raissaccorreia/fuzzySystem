import vrep
import time
import sys
import numpy as np
import math
import matplotlib.pyplot as mpl
import os
from enum import Enum

class Quadrants(Enum):
	FirstQuarter = 1
	SecondQuarter = 2
	ThirdQuarter = 3
	FourthQuarter = 4

class Simulator(object):

	def __init__(self, ip, portNumber):
		self.ip = ip
		self.portNumber = portNumber
		self.id = -1

	def connect(self):
		self.id = 1
		self.id = vrep.simxStart(self.ip, self.portNumber, True, True, 2000, 5)
		print('teste')
		if (self.id == -1):
			print("Unable to connect to V-REP Server")

		return self.id

	def getId(self):
		return self.id

	def disconnect(self):
		if (self.id != -1):
			vrep.simxFinish(self.id)

	def pause(self):
		if (self.id != -1):
			vrep.simxPauseCommunication(self.id, 0)

	def resume(self):
		if (self.id != -1):
			vrep.simxPauseCommunication(self.id, 1)

	def getHandle(self,name):

		if (self.id != -1):
			errorCode,handle = vrep.simxGetObjectHandle(self.id, name , vrep.simx_opmode_oneshot_wait)
			if (errorCode != vrep.simx_return_ok):
				print("Problems to get handle")
				return
			else:
				return handle

	def readProximitySensor(self,handle):

		if (self.id != -1):
			errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(self.id, handle, vrep.simx_opmode_buffer)
			if (errorCode != vrep.simx_return_ok):
			 	print('Problems to get proximity')
			 	return
			else:
				return (detectionState,detectedPoint)

	def getObjectPosition(self,handle):

		if (self.id != -1):
			errorCode,position = vrep.simxGetObjectPosition(self.id, handle, -1, vrep.simx_opmode_streaming)
			# if (errorCode != vrep.simx_return_ok):
			# 	print("Problems to get object position")
			# 	return
			# else:
			# print(position)
			return (position)

	def getObjectOrientation(self,handle):

		if (self.id != -1):
			errorCode,orientation = vrep.simxGetObjectOrientation(self.id, handle, -1, vrep.simx_opmode_streaming)
			# if (errorCode != vrep.simx_return_ok):
			# 	print("Problems to get object orientation")
			# 	return
			# else:
			return (orientation)

	def getJointPosition (self,jointHandle):

		if (self.id != -1):
			errorCode, position = vrep.simxGetJointPosition(self.id, jointHandle, vrep.simx_opmode_streaming)
			if (errorCode != vrep.simx_return_ok):
				print("Problems to get joint position")
				return
			else:
				print(position)
				return (position)

	def getVisionSensorImage(self, visionHandle):

		if (self.id != -1):
			errorCode, resolution, image = vrep.simxGetVisionSensorImage(self.id, visionHandle, 0, vrep.simx_opmode_oneshot_wait)
			if (errorCode != vrep.simx_return_ok):
				print("Problems to get vision image")
				return
			else:
				return (image)

	def setJointTargetVelocity (self,jointHandle, velocity):

		if (self.id != -1):
			errorCode = vrep.simxSetJointTargetVelocity(self.id, jointHandle, velocity, vrep.simx_opmode_streaming)
			# if (errorCode != vrep.simx_return_ok):
			# 	print("Problems to set joint position")
			# 	return 0
			# else:
			return 1

	def initializeSensor(self,sensorHandle):

		if (self.id != -1):
			errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(self.id, sensorHandle, vrep.simx_opmode_streaming)
			# if (errorCode != vrep.simx_return_ok):
			# 	print("Problems to  initialize sensor")
			# 	print(errorCode)
			# 	return 0
			# else:
			return 1


class Robot(object):

	def __init__(self, simulator, name):

		#file creation
		self.xGTFile = self.createFile('RobotGTx')
		self.yGTFile = self.createFile('RobotGTy')
		self.xMapFile = self.createFile('MapX')
		self.yMapFile = self.createFile('MapY')
		self.xGTOdometryFile = self.createFile('RobotGTxOdometry')
		self.yGTOdometryFile = self.createFile('RobotGTyOdometry')

		# constants
		self.L = 0.381
		self.R = 0.0975

		self.sim = simulator
		self.name = name
		self.handle = self.sim.getHandle(name)

		PI = math.pi
		self.sensorAngles = [(PI/2),(10*PI/36),(PI/6),(PI/18),(-PI/18),(-PI/6),(-10*PI/36),(-PI/2)]


		self.encoderHandle = []
		self.motorHandle = []
		self.sensorValues = []
		self.sensorLastValues = []
		# get encoder handles
		leftWheelHandle = self.sim.getHandle('Pioneer_p3dx_leftWheel')
		rightWheelHandle = self.sim.getHandle('Pioneer_p3dx_rightWheel')

		self.encoderHandle.append(leftWheelHandle)
		self.encoderHandle.append(rightWheelHandle)

		# get motor handles
		leftMotorHandle = self.sim.getHandle('Pioneer_p3dx_leftMotor')
		rightMotorHandle = self.sim.getHandle('Pioneer_p3dx_rightMotor')

		self.motorHandle.append(leftMotorHandle)
		self.motorHandle.append(rightMotorHandle)

		# get sonars handles
		self.sonarHandle = []
		for x in range(1,16+1):
			sensor_handle =  self.sim.getHandle('Pioneer_p3dx_ultrasonicSensor'+str(x))
			self.sonarHandle.append(sensor_handle)
			self.sim.initializeSensor(sensor_handle)
			print('conected sensor' + str(x))

			self.sensorValues.append(-1)
			self.sensorLastValues.append(-1)

		#get visionHandle
		# self.visionHandle = self.sim.getHandle('Vision_sensor')

		# get initial position and orientation
		self.actualPosition = self.sim.getObjectPosition(self.handle)
		self.actualOrientation = self.sim.getObjectOrientation(self.handle)

		# initial inforamtion to odometry
		self.actualOdometryPosition = self.actualPosition
		self.actualOdometryOrientation = self.actualOrientation
		self.actualOdometryOrientation
		self.firstUpdate = 0
		self.wheelR = 0.0975 # raio da roda
		self.rightWheelAngle = self.sim.getJointPosition(self.motorHandle[1])
		self.leftWheelAngle = self.sim.getJointPosition(self.motorHandle[0])


		self.robotLastPosition = self.actualPosition

		# testes
		print('initial position')
		print(self.actualPosition)
		print('initial orientation')
		print(self.actualOrientation)

	def stop(self):
		self.sim.setJointTargetVelocity(self.motorHandle[0],0)
		self.sim.setJointTargetVelocity(self.motorHandle[1],0)


	def vRToDrive(self, vLinear, vAngular):
		return (((2*vLinear)+(self.L*vAngular))/2*self.R);

	def vLToDrive(self, vLinear, vAngular):
		return (((2*vLinear)-(self.L*vAngular))/2*self.R);

	def drive(self, vLinear, vAngular):
		self.sim.setJointTargetVelocity(self.motorHandle[0], self.vLToDrive(vLinear,vAngular))
		self.sim.setJointTargetVelocity(self.motorHandle[1], self.vRToDrive(vLinear,vAngular))

	def setVelocity(self,vLeft,vRight):
		self.sim.setJointTargetVelocity(self.motorHandle[0], vLeft)
		self.sim.setJointTargetVelocity(self.motorHandle[1], vRight)


	def turnR(self, degrees):
		self.drive(0,-degrees)

	def turnL(self,degrees):
		self.drive(0,degrees)

	def update(self):

		self.updatePose()
		self.updateSensors()

		# self.setVelocity(2,5)

		#P CONTROL, AVOID OBSTACLES
		# self.avoidObstacles()
		self.wallFollowing()

	def wallFollowing(self):

		#references
		frontRef = 0.5
		latRef = 0.35

		kLateral	= 5.5
		kFront 		= 10

		vFront = 3
		vLateral = 3

		value = self.sensorValues[3:7]
		for x in range(0,3):
			if (value[x] == 10.0):
				print('entrou')
				value[x] = 0

		print(value)

		frontDist	= value[0]
		latDist		= value[3]
		quinDist	= value[2]

		frontError = frontDist
		latError = latRef - latDist

		if (latError < - 2): #sem parede ao lado
			latError = - 0.1

		# print('frontError = ',frontError)
		# print('latError = ',latError)

		if ((frontDist < frontRef ) and frontDist != 0):
			# print('Front Dist ', frontDist)
			#controle para virar pois obstaculo esta na frente
			leftVelocity	= (vFront - kFront * frontError)
			rightVelocity	= (vFront + kFront * frontError)

		else:
			# print('seguir')
			#controle para seguir parede
			leftVelocity 	= ((vLateral - kLateral * latError))
			rightVelocity 	= ((vLateral + kLateral * latError))


		print('L vel : ', leftVelocity)
		print('R vel : ', rightVelocity)

		self.setVelocity(leftVelocity,rightVelocity)


	def avoidObstacles(self):

		min_ind = np.where(self.sensorValues[0:8]==np.min(self.sensorValues[0:8]))
		min_ind = min_ind[0][0]

		print('min ind', min_ind)

		minValue = self.sensorValues[min_ind]

		if minValue<0.3 and self.sensorValues[min_ind] != 0:
			value = 1 - minValue
			if (min_ind > 3 ):
				value = -value

			steer = (1/self.sensorAngles[min_ind]) + value
		else:
			steer=0

		v=3	#forward velocity
		kp=0.6	#steering gain
		vl=v + kp*steer
		vr=v - kp*steer
		print ("V_l =",vl)
		print ("V_r =",vr)

		self.setVelocity(vl,vr)


	def updateSensors(self):
		self.sensorLastValues = self.sensorValues
		for x in range (1,16):
			# print('updateSensors' + str(x))
			state,pos = self.sim.readProximitySensor(self.sonarHandle[x-1])
			if (state):
				self.sensorValues[x-1] = pos[2]
				# print(self.sensorValues[x-1])
			else:
				self.sensorValues[x-1] = 10

		for x in range (0,8):
			# print('getting distance from sensor ' + str(x+1) + ' : '+ str(self.sensorValues[x]))
			if(self.sensorValues[x] != 10):
				pos = self.getObstaclePosition(self.sensorValues[x],self.sensorAngles[x])
				# print('position of sensor' + str(x+1))
				# print(pos)

				xMapString = str(pos[0]) + ' '
				yMapString = str(-pos[1]) + ' ' #negativo para plottar o gráfico correto, eixo está refletido

				self.writeOnFile(self.xMapFile,xMapString)
				self.writeOnFile(self.yMapFile,yMapString)



	def updatePose(self):
		self.robotLastPosition = self.actualPosition

		self.actualPosition = self.sim.getObjectPosition(self.handle)
		self.actualOrientation = self.sim.getObjectOrientation(self.handle)
		self.updateOdometryPosition()

		#save this on file

		xGTString = str(self.actualPosition[0])
		yGTString = str(-self.actualPosition[1]) #negativo para plottar o gráfico correto, eixo está refletido
		xGTOdometryString = str(self.actualOdometryPosition[0])
		yGTOdometryString = str(-self.actualOdometryPosition[1])

		if (float(xGTString) > 10):
			xGTString = str(self.robotLastPosition[0])

		if (float(yGTString) > 10):
			yGTString = str(-self.robotLastPosition[1])

		if (float(xGTOdometryString) > 10):
			xGTString = str(self.lastOdometryPosition[0])

		if (float(yGTOdometryString) > 10):
			yGTString = str(-self.lastOdometryPosition[1])

		xGTString = xGTString + ' '
		yGTString = yGTString + ' '
		xGTOdometryString = xGTOdometryString + ' '
		yGTOdometryString = yGTOdometryString + ' '

		self.writeOnFile(self.xGTFile,xGTString)
		self.writeOnFile(self.yGTFile,yGTString)
		self.writeOnFile(self.xGTOdometryFile,xGTOdometryString)
		self.writeOnFile(self.yGTOdometryFile,yGTOdometryString)

		# self.printPose()

	def updateOdometryPosition(self):
		if (self.firstUpdate < 2): # get first ground value to plot reference
			self.lastOdometryPosition = self.actualOdometryPosition
			self.actualOdometryPosition = self.actualPosition
			self.actualOdometryOrientation = self.actualOrientation[2]
			self.getWheelUpdate()
			self.firstUpdate = self.firstUpdate + 1
		else:
			self.lastOdometryPosition = self.actualOdometryPosition
			self.lastOdometryOrientation = self.actualOdometryOrientation
			self.calcDislocation()

		print("////////////ODOMETRY/////////////")
		print('x:' + str(self.actualOdometryPosition[0]) + ' y: ' + str(self.actualOdometryPosition[1]))
		print('orientation angle: ' + str(self.actualOdometryOrientation))
		print("/////////////////////////////////")
		self.printPose()

	def getWheelUpdate(self):
		self.lastRightWheelAngle = self.rightWheelAngle
		self.lastLeftWheelAngle = self.leftWheelAngle
		self.rightWheelAngle = self.sim.getJointPosition(self.motorHandle[1])
		self.leftWheelAngle = self.sim.getJointPosition(self.motorHandle[0])


	def calcDislocation(self):
		self.getWheelUpdate()

		PI = math.pi

		leftAngle = self.ang(self.leftWheelAngle, self.lastLeftWheelAngle)

		rightAngle = self.ang(self.rightWheelAngle, self.lastRightWheelAngle)

		# calculate dislocation for each wheel
		leftWheelDislocation = self.wheelR * (leftAngle)
		rightWheelDislocation = self.wheelR * (rightAngle)

		# update position
		self.actualOdometryPosition[0] = (np.sin(self.lastOdometryOrientation)*(leftWheelDislocation+rightWheelDislocation)) + self.lastOdometryPosition[0]
		self.actualOdometryPosition[1] = (np.cos(self.lastOdometryOrientation)*(leftWheelDislocation+rightWheelDislocation)) + self.lastOdometryPosition[1]

		# update orientation
		self.actualOdometryOrientation = self.actualOrientation[2] # TODO orientation odometry


		print("/////////////////////////////////")
		print('left wheel angle: ' + str(self.leftWheelAngle))
		print('right wheel angle: ' + str(self.rightWheelAngle))
		print('leftWheelDislocation: ' + str(leftWheelDislocation))
		print('rightWheelDislocation: ' + str(rightWheelDislocation))
		print("/////////////////////////////////")


	def ang(self, ang1, ang2):
		total = 0

		PI = math.pi

		if (ang1 < ang2):

			angle1 = PI/2 - ang1
			angle2 = PI/2 - abs(ang2)
			total = angle1 + angle2

		else:
			total = ang1 - ang2

		return total

	def printPose(self):
		print('Position: ')
		print('x:' + str(self.actualPosition[0]) + ' y: ' + str(self.actualPosition[1]))
		print('orientation angle: ' + str(self.actualOrientation[2]))

	def getObstaclePosition(self,dist,angle):
		PI = math.pi
		pos = []
		realDist = dist + self.R
		realAngle = angle + (self.actualOrientation[2])
		# print('Angle: ' + str(angle) + 'actualOri: ' + str(self.actualOrientation[2]) + 'total angle: ' + str(realAngle) )

		if (realAngle < -PI):
			realAngle = realAngle + 2 * PI
			# print('new angle = ' + str (realAngle))

		elif (realAngle > PI):
			realAngle = realAngle - 2 * PI
			# print('new angle = ' + str (realAngle))


		x = realDist * math.cos(realAngle)
		y = realDist * math.sin(realAngle)
		
		pos.append(self.actualPosition[0] + (x))
		pos.append(self.actualPosition[1] + (y))

		return pos

	def createFile(self, fileName):

		file = fileName + '.txt'

		if (os.path.exists(file)):
			open(file,'w').close()
			return open(file, 'r+')
		else:
			return open(file,'x+')

	def openFile(self,fileName):

		file = fileName + '.txt'

		return open(file,'r')

	def writeOnFile(self,file,text):

		file.write(text)

	def saveFile(self,file):

		file.close()

	def plotGraphs(self):

		self.saveFile(self.xGTFile)
		self.saveFile(self.yGTFile)
		self.saveFile(self.xGTOdometryFile)
		self.saveFile(self.yGTOdometryFile)
		self.saveFile(self.xMapFile)
		self.saveFile(self.yMapFile)

		self.xGTFile = self.openFile('RobotGTx')
		self.yGTFile = self.openFile('RobotGTy')
		self.xGTOdometryFile = self.openFile('RobotGTxOdometry')
		self.yGTOdometryFile = self.openFile('RobotGTyOdometry')
		self.xMapFile = self.openFile('MapX')
		self.yMapFile = self.openFile('MapY')

		xGT = self.fromStringToArray(self.xGTFile.read())
		yGT = self.fromStringToArray(self.yGTFile.read())
		xGTOdometry = self.fromStringToArray(self.xGTOdometryFile.read())
		yGTOdometry = self.fromStringToArray(self.yGTOdometryFile.read())
		yMap = self.fromStringToArray(self.yMapFile.read())
		xMap = self.fromStringToArray(self.xMapFile.read())

		size1 = xGT.shape[0]
		size2 = yGT.shape[0]
		size1Odometry = xGTOdometry.shape[0]
		size2Odometry = yGTOdometry.shape[0]
		xMapSize = xMap.shape[0]
		yMapSize = yMap.shape[0]

		if (size1 > size2):
			dif = size1 - size2
			xGT = xGT[:-dif]
		elif (size2 > size1):
			dif = size2 - size1
			yGT = yGT[:-dif]

		print('xGT shape :' + str(xGT.shape[0]))
		print('yGT shape :' + str(yGT.shape[0]))

		xGT,yGT = self.filterData(xGT,yGT)

		if (size1Odometry > size2Odometry):
			difOdometry = size1Odometry - size2Odometry
			xGTOdometry = xGTOdometry[:-difOdometry]
		elif (size2Odometry > size1Odometry):
			difOdometry = size2Odometry - size1Odometry
			yGTOdometry = yGTOdometry[:-difOdometry]

		print('xGTOdometry shape :' + str(xGTOdometry.shape[0]))
		print('yGTOdometry shape :' + str(yGTOdometry.shape[0]))

		xGTOdometry,yGTOdometry = self.filterData(xGTOdometry,yGTOdometry)

		if (xMapSize > yMapSize):
			dif = xMapSize - yMapSize
			xMap = xMap[:-dif]
		elif (yMapSize > xMapSize):
			dif = yMapSize - xMapSize
			yMap = yMap[:-dif]

		print('xMap shape :' + str(xMap.shape[0]))
		print('yMap shape :' + str(yMap.shape[0]))

		xMap, yMap = self.filterData(xMap,yMap)

		mpl.plot(yGT,xGT,'r.') #INVERTIDO POIS O VREP TEM OUTRAS COORDENADAS
		# mpl.plot(yGTOdometry,xGTOdometry,'y.') #DESCOMENTAR PARA VER ODOMETRIA
		mpl.plot(yMap,xMap,'b.')

		mpl.show()


	def fromStringToArray(self,string):
		splited = string.split()
		numbersArray = np.array(splited)
		numbersArray = np.delete(numbersArray,0)
		print('STRING SIZE')
		print(numbersArray.shape[0])
		return numbersArray

	def filterData(self,xArray, yArray):
		sizeX = xArray.shape[0]
		sizeY = yArray.shape[0]
		print('Tamanho do array que vai ser filtrado ' + str(sizeX) + ' ' + str(sizeY))
		for x in range(0,sizeX-1):
			if (float(xArray[x]) > 10.0 or float(yArray[x]) > 10.0):
				xArray = np.delete(xArray,x)
				yArray = np.delete(yArray,x)

		print('X value = ' + str(x))

		return (xArray,yArray)

	def getQuadrant(self,angle):
		PI = math.pi
		if (angle <= 0 and angle > (-PI/2)):
			#primeiro quadrante
			return Quadrants.FirstQuarter
		elif (angle > 0 and angle <= (PI/2)):
			#segundo quadrante
			return Quadrants.SecondQuarter
		elif (angle >= (-PI) and angle < (-PI/2)):
			#quarto quadrante
			return Quadrants.FourthQuarter
		else:
			return Quadrants.ThirdQuarter



print('Started')
sim = Simulator("127.0.0.1",25000)
if (sim.connect() == -1):
	print ('Failed to connect')
else:
	print('Connected')
print('simulator check')
robot = Robot(sim, 'Pioneer_p3dx')
print('robot check')

for x in range(0,50):
	robot.update()
	# print(x)
	time.sleep(0.2)

# robot.saveFile(robot.xGTFile)
# robot.saveFile(robot.yGTFile)
# robot.saveFile(robot.xGTOdometryFile)
# robot.saveFile(robot.yGTOdometryFile)
# robot.saveFile(robot.xMapFile)
# robot.saveFile(robot.yMapFile)


robot.stop()
sim.disconnect()

print('Disconnect and ended')

robot.plotGraphs()


