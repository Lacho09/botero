import time
import settings
import RPi.GPIO as GPIO

class ObstacleDetectionController:
    def __init__(self, robot_controller):
        self.robot_controller = robot_controller
        self.obstacle_counter = 0
        self.braking_counter = 0

    def detect_obstacle(self):
        # Read obstacle status from sensors or GPIO pins
        DR = 16
        DL = 19
        DR_status = GPIO.input(DR)  # Read the state of GPIO pin 16. Returns 1 if an obstacle is in the detection range, 0 if not.
        DL_status = GPIO.input(DL)  # Read the state of GPIO pin 19. Returns 1 if an obstacle is in the detection range, 0 if not.

        if DR_status == 0 and DL_status == 0:
            self.obstacle_counter += 1

            if self.obstacle_counter == 1 and self.braking_counter > 50:
                # Implement obstacle avoidance actions
                # For example, brake and wait for a certain time
                timeout = time.time() + settings.breaking_time
                self.robot_controller.backward()
                while time.time() < timeout:
                    pass
                self.robot_controller.stop()

            else:
                self.robot_controller.stop()  # Implement other actions if needed

        else:
            self.obstacle_counter = 0
