import settings
import logging
import random

class LineFollowingController:
    def __init__(self, robot_controller, line_following_settings):
        self.robot_controller = robot_controller
        self.P = line_following_settings["P"]
        self.I = line_following_settings["I"]
        self.D = line_following_settings["D"]
        self.maxDC = line_following_settings["maxDC"]
        self.sensor_threshold = settings.sensor_threshold
        self.prev_error = 0
        self.integral = 0
        self.lf_counter = 0
        self.lf_total_counter = 0
        self.lf_obstacle_counter = 0
        self.preference = 'right'
        self.bifurcation_counter = 0
        self.bifurcation_flag = 1
        self.random_float = 0

    def line_detected(self):
        
        # Read values from all 5 sensors
        position, sensor_values = self.robot_controller.read_linefollow_sensors()  # Replace with actual function to read sensor values

        # print(sensor_values)

        # Check if any of the sensors detects a line
        for value in sensor_values:
            if value > self.sensor_threshold:
                return True  # Line detected

        return False  # No line detected

    def follow_line(self):

        self.robot_controller.robot.setPWMA(self.maxDC)
        self.robot_controller.robot.setPWMB(self.maxDC)
        
        # Implement line-following logic using sensor input
        position, sensor_values = self.robot_controller.read_linefollow_sensors()
        # print(position)

        # Detects any bifurcation and acts consequently
        position = self.detect_bifurcation_random(position, sensor_values)

        # Calculate the error based on sensor values
        error = position - 2000

        # Implement the PID control algorithm
        self.integral += error
        derivative = error - self.prev_error

        # Calculate the motor control output using PID
        power_difference = (error * self.P) + (self.integral * self.I) + (derivative * self.D)
        # print(power_difference)

        if (power_difference > self.maxDC):
            power_difference = self.maxDC
        if (power_difference < - self.maxDC):
            power_difference = - self.maxDC

        # Adjust motor speeds based on the power difference
        # Implement motor control logic using power_difference        
        if (power_difference < 0):          # If power_difference is negative, the left motor speed (PWMA) is increased, while the right motor speed (PWMB) remains at the maximum duty cycle (self.maxDC). This directs the robot to the left.
            self.robot_controller.robot.setPWMA(self.maxDC + power_difference)
            self.robot_controller.robot.setPWMB(self.maxDC)
        else:         # If power_difference is positive, the left motor speed remains at the maximum duty cycle (self.maxDC), and the right motor speed is reduced by power_difference. This directs the robot to the right
            self.robot_controller.robot.setPWMA(self.maxDC)
            self.robot_controller.robot.setPWMB(self.maxDC - power_difference)

        self.prev_error = error

    def follow_line_evacuations(self, green_ever_detected):
        lsensor_status, rsensor_status = self.robot_controller.read_obstacle_sensors()

        self.lf_total_counter = self.lf_total_counter + 1
        logging.info(f'{self.lf_total_counter}')
        
        if self.lf_total_counter < settings.lf_ignore_obstacle and green_ever_detected != 0: 
            logging.info('Line detected. Following it no matter what.')
            self.robot_controller.move_forward()
            self.follow_line()
            
        else:
            if lsensor_status != 0 and rsensor_status != 0:
                self.robot_controller.move_forward()
                self.follow_line()
                self.lf_counter =+ 1
                self.lf_obstacle_counter = 0

            else:
                if self.lf_obstacle_counter == 1 and self.lf_counter > 50:
                    self.robot_controller.hard_brake(self.maxDC)
                    self.lf_obstacle_counter =+ 1
                    self.lf_counter = 0
                else:
                    self.robot_controller.brake()

    def detect_bifurcation(self, position, sensors):

        if (sensors[0] > settings.sensor_threshold and sensors[4] > settings.sensor_threshold) and sensors[2] < settings.sensor_threshold:
            logging.info(f'bifurcation {self.preference}')
            
            self.bifurcation_counter = self.bifurcation_counter + 1

            if self.bifurcation_counter == 1:

                if self.preference == 'right':
                    self.preference = 'left'
                    position = settings.bif_right
                else:
                    self.preference = 'right'
                    position = settings.bif_left
            else:
                if self.preference == 'right':
                    position = settings.bif_right
                else:
                    position = settings.bif_left
            
        return position
    
    def detect_bifurcation_random_old(self, position, sensors):

        if (sensors[0] > settings.sensor_threshold and sensors[4] > settings.sensor_threshold) and sensors[2] < settings.sensor_threshold:
            logging.warning(f'bifurcation {self.preference}')

            random_float = random.random()

            if random_float >= 0.5:
                self.preference = 'right'
                position = settings.bif_right
            else:
                self.preference = 'left'
                position = settings.bif_left 
            
        return position
            
    def detect_bifurcation_random_old2(self, position, sensors):

        if (sensors[0] > settings.sensor_threshold and sensors[4] > settings.sensor_threshold) and sensors[2] < settings.sensor_threshold:

            self.bifurcation_counter = self.bifurcation_counter + 1

            if self.bifurcation_counter == 5:

                random_float = random.random()

                if random_float >= 0.5:
                    self.preference = 'right'
                    position = settings.bif_right
                else:
                    self.preference = 'left'
                    position = settings.bif_left
                    
                logging.warning(f'bifurcation {self.preference}')
        else:
            # print('no bifurcation')
            self.bifurcation_counter = 0

        # print(position)
                
        return position

    def detect_bifurcation_random(self, position, sensors):
        """
        Detects a potential bifurcation based on sensor readings and randomly chooses a direction to proceed.
        Asummes that the bifurcation is detected continuously until it is not detected anymore.
        It won't work if bifurcation and no bifurcations are detected.  

        Args:
            position (int): Current position of the line.
            sensors (list): List of sensor readings.

        Returns:
            int: Updated position after potentially navigating the bifurcation.
        """

        if (sensors[0] > settings.sensor_threshold and sensors[4] > settings.sensor_threshold) and sensors[2] < settings.sensor_threshold:

            if self.bifurcation_flag == 1:

                self.random_float = random.random()

                if self.random_float >= 0.5:
                    self.preference = 'right'
                else:
                    self.preference = 'left'

                logging.warning(f'bifurcation {self.preference}')
                self.bifurcation_flag = 0

            if self.preference == 'right':
                position = position + 1500
            elif self.preference == 'left':
                position = position - 1500

        else:
            # logging.warning(f'no bifurcation, flag goes to 1')
            self.bifurcation_flag = 1
                   
        return position
        

