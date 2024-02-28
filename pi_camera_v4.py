import picamera
import picamera.array
import RPi.GPIO as GPIO
from robot_controller import RobotController
from line_following import LineFollowingController
from obstacle_detection import ObstacleDetectionController
from green_object_detection import GreenObjectDetectionController
from camera_controller import CameraObjectController
import settings
import numpy as np
import time
import cv2
import logging

# Configure the logging module
logging.basicConfig(filename='bot_log.txt', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', filemode='a')

# Create a console handler and set its level
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)

# Create a formatter and add it to the console handler
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)

# Add the console handler to the root logger
logging.getLogger().addHandler(console_handler)

# Now both file and console logging are configured
logging.info("This message should be displayed in the console.")


# Configuration settings
robot_config = {
	"line_following_settings": {"P": 0.03, "I": 0, "D": 1, "maxDC": 25},
	"green_following_settings": {"P": settings.green_P, "I": settings.green_I, "D": settings.green_D, "maxDC": settings.green_maxDC}
}

# Initialize hardware components
robot_controller = RobotController()

# Initialize controllers
line_follower = LineFollowingController(robot_controller, robot_config['line_following_settings'])
green_follower = GreenObjectDetectionController(robot_controller, robot_config['green_following_settings'])

# camera = picamera.PiCamera()
camera_controller = CameraObjectController()
# camera, stream = camera_controller.adjust_settings()
camera_controller.adjust_settings()

def main_and_jam_detection_fulltime():
	'''
	This function gives priority to the line detection over the green detection.
	'''

	# ID of pictures taken from bot
	pic_id = 0

	try:
		while True:

			pic_id += 1

			if line_follower.line_detected():

				robot_controller.move_forward()
				line_follower.follow_line_evacuations()

				# every time a line is detected this counter is reseted
				green_follower.green_detection_counter = 0

			if not line_follower.line_detected(): # if there is no line, search for green using the camera

				# Capture image
				camera_controller.camera.capture(camera_controller.stream, format='bgr', use_video_port=True)

				# save the picture taken for possible analysis
				# camera_controller.save_image(pic_id)

				frame = camera_controller.stream
				
				# Crop the upper portion of the frame (adjust the values to your desired crop size)
				# TODO: check later, because is not used and could be useful to compare just the green part in images
				# althoug if the is a jam and bots are not pointing to the green it might not be solved
				cropped_frame = frame.array[settings.crop_pos:, :]

				# Follow exit using the image
				green_follower.evacuations_v3(cropped_frame)
				
				# TODO: think about this conditional, is it really necessary ???
				# If the bot has detected green after leaving the line
				if green_follower.green_detection_counter > 1:

					# look for jams and react to them
					green_follower.break_jam(prev_frame, prev_mask, cropped_frame)

				prev_frame = cropped_frame
				prev_mask = green_follower.green_mask

				# Clear the stream in preparation for the next frame
				camera_controller.stream.truncate(0)

	except KeyboardInterrupt:
		# Handle Ctrl+C gracefully
		pass
	
	finally:
		# Clean up GPIO and camera resources
		GPIO.cleanup()
		camera_controller.camera.close()

		# Close the log file
		logging.shutdown()
		
		# pass

if __name__ == "__main__":

	main_and_jam_detection_fulltime()

	# main()
	# main_save_image()
	# move_if_in_jam()
	# main_and_jam_detection()


### Not being used at the moment

def main():
	try:
		while True:
			if line_follower.line_detected():
				robot_controller.move_forward()
				line_follower.follow_line_evacuations()
				green_follower.green_detection_counter = 0

			if not line_follower.line_detected():

				# Capture image
				camera_controller.camera.capture(camera_controller.stream, format='bgr', use_video_port=True)
				frame = camera_controller.stream
				
				# Crop the upper portion of the frame (adjust the values to your desired crop size)
				cropped_frame = frame.array[settings.crop_pos:, :]

				# Follow exit using the image
				green_follower.evacuations_v4(cropped_frame)
				
				# Clear the stream in preparation for the next frame
				camera_controller.stream.truncate(0)

	except KeyboardInterrupt:
		# Handle Ctrl+C gracefully
		pass
	
	finally:
		# Clean up GPIO and camera resources
		GPIO.cleanup()
		camera_controller.camera.close()
		# pass

def main_and_jam_detection():

	try:
		while True:
			if line_follower.line_detected():
				robot_controller.move_forward()
				line_follower.follow_line_evacuations()
				green_follower.green_detection_counter = 0

			if not line_follower.line_detected():

				# Capture image
				camera_controller.camera.capture(camera_controller.stream, format='bgr', use_video_port=True)
				frame = camera_controller.stream
				
				# Crop the upper portion of the frame (adjust the values to your desired crop size)
				cropped_frame = frame.array[settings.crop_pos:, :]

				if green_follower.green_detection_counter == 0:
					prev_frame = cropped_frame

					# Record start time
					start_time = time.time()

					first_control = False

				# Follow exit using the image
				green_follower.evacuations_v4(cropped_frame)

				# Record end time
				end_time = time.time()

				# Calculate elapsed time
				elapsed_time = end_time - start_time
					
				if elapsed_time >= 10 and first_control == False: # in seconds

					movement_magnitude = green_follower.calculate_displacement(prev_frame, cropped_frame)
					
					if movement_magnitude < 40000:
						print('You are in a jam!', movement_magnitude)
						robot_controller.line_follow_sensors_calibration()
						pass
					else:
						print('You are moving!', movement_magnitude)
						pass

					prev_frame = cropped_frame
					
					# Record start time
					start_time = time.time()
					
					first_control = True

				if elapsed_time >= 2 and first_control == True:

					movement_magnitude = green_follower.calculate_displacement(prev_frame, cropped_frame)
					
					if movement_magnitude < 40000:
						print('You are in a jam!', movement_magnitude)
						robot_controller.line_follow_sensors_calibration()
						pass
					else:
						print('You are moving!', movement_magnitude)
						pass

					prev_frame = cropped_frame

					# Record start time
					start_time = time.time()

				# Clear the stream in preparation for the next frame
				camera_controller.stream.truncate(0)

	except KeyboardInterrupt:
		# Handle Ctrl+C gracefully
		pass
	
	finally:
		# Clean up GPIO and camera resources
		GPIO.cleanup()
		camera_controller.camera.close()
		# pass

def main_save_image():
	# Capture image
	camera_controller.camera.capture(camera_controller.stream, format='bgr', use_video_port=True)
	camera_controller.save_image()

def move_if_in_jam():
	# prev_frame = np.zeros((480,720,3))

	counter = 0

	try:
		while True:

			# # Record start time
			# start_time = time.time()

			# Capture image
			camera_controller.camera.capture(camera_controller.stream, format='bgr', use_video_port=True)
			frame = camera_controller.stream
			
			# Crop the upper portion of the frame (adjust the values to your desired crop size)
			cropped_frame = frame.array[settings.crop_pos:, :]

			if counter == 0:
				prev_frame = cropped_frame

				# Clear the stream in preparation for the next frame
				camera_controller.stream.truncate(0)
				counter += 1
				continue
			else:
				pass

			if counter%1 == 0:
				# Record start time
				start_time = time.time()
				movement_magnitude = green_follower.detect_movement(prev_frame, cropped_frame, 20)
				# Record end time
				end_time = time.time()

				prev_frame = cropped_frame
				
				if np.max(movement_magnitude) < 20:
					# print('You are in a jam!', np.max(movement_magnitude))
					# move !!!
					pass
				else:
					# print('You are moving!',np.max(movement_magnitude))
					pass
			counter += 1
				
			# Clear the stream in preparation for the next frame
			camera_controller.stream.truncate(0)		
			
			# # Record end time
			# end_time = time.time()

			# Calculate elapsed time
			elapsed_time = end_time - start_time

			print(elapsed_time)

	except KeyboardInterrupt:
		# Handle Ctrl+C gracefully
		pass
	
	finally:
		# Clean up GPIO and camera resources
		GPIO.cleanup()
		camera_controller.camera.close()
		# pass	

