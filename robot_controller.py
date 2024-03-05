import time
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2
from TRSensors_rev22 import TRSensor
import logging

# implement TRsensors here for reading sensors

class RobotController:
	def __init__(self):
		self.robot = AlphaBot2()
		self.sensors = TRSensor()
		self.sensor_DR = 16
		self.sensor_DL = 19
		self.breaking_time = 0.2
		self.lf_obstacle_counter = 0
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.sensor_DR, GPIO.IN, GPIO.PUD_UP)
		GPIO.setup(self.sensor_DL, GPIO.IN, GPIO.PUD_UP)
		self.sensors.calibratedMin = [960, 958, 962, 961, 959]
		self.sensors.calibratedMax = [126, 61, 100, 103, 101]

		self.green_exploration_counter = 0

	def move_forward(self):
		self.robot.forward()

	def move_backward(self):
		self.robot.backward()

	def stop(self):
		self.robot.stop()

	def rotate(self, sense, steps=1000):

		if sense == "left":
			self.robot.left()
		elif sense == "right":
			self.robot.right()
		
		### Waits time to make the movement
		i = 0
		while True:
			time.sleep(0.00001)
			i  = i + 1
			if(i<steps):
				pass
			else:
				self.robot.stop()
				break

	def rotation_exploration(self):
		logging.info('rotation exploration')
		# Uses the function rotates to explore for the green. The bot will alternatively rotate clockwise and counterclockwise.
		# the number of steps of every rotation increase to cover more space.

		self.green_exploration_counter =+ 1

		logging.info(f'Rotation explotation, counter = {self.green_exploration_counter}')

		if self.green_exploration_counter % 2 == 0:
			self.rotate('left', 1000*self.green_exploration_counter)
		else:
			self.rotate('right', 1000*self.green_exploration_counter)

	def green_finder(self, prev_pos_x):
		if prev_pos_x < 0.5:
			self.rotate("left")
		elif prev_pos_x > 0.5:
			self.rotate("right")

		
	def line_follow_sensors_calibration(self):
		for i in range(0,100):
			if(i<25 or i>= 75):
				self.robot.right()
			else:
				self.robot.left()
			self.sensors.calibrate()
		self.robot.stop()

	def read_linefollow_sensors(self):
		line_pos, sensor_values = self.sensors.readLine()
		return line_pos, sensor_values
	

	def read_obstacle_sensors(self):
		DR_status = GPIO.input(self.sensor_DR)
		DL_status = GPIO.input(self.sensor_DL)
		# Implement reading and returning sensor values
		return DR_status, DL_status
	
	def hard_brake(self, maxDC):
		timeout2 = time.time() + self.breaking_time
		self.robot.setPWMA(maxDC/4)
		self.robot.setPWMB(maxDC/4)
		self.robot.backward()

		while time.time() < timeout2:
			pass
		self.robot.setPWMA(0)
		self.robot.setPWMB(0)
		self.robot.forward()

	def brake(self):
		self.robot.setPWMA(0)
		self.robot.setPWMB(0)

	def cleanup(self):
		self.robot.stop()
		self.robot.close()