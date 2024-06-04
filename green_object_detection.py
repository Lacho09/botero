import cv2
import numpy as np
import settings  # Import relevant settings for green object detection
import logging

class GreenObjectDetectionController:
    def __init__(self, robot_controller, green_following_settings):
        self.robot_controller = robot_controller
        self.P = green_following_settings["P"]
        self.I = green_following_settings["I"]
        self.D = green_following_settings["D"]
        self.maxDC = green_following_settings["maxDC"]
        self.prev_error = 0
        self.integral = 0
        self.green_mask = np.array(0)
        self.pos_x = 0.5
        self.prev_pos_x = 0.5
        self.green_detection_counter = 0
        self.green_ever_detected = 0
        self.green_recently_detected = 0
        self.gf_counter = 0
        self.gf_obstacle_counter = 0
        self.objective = 0.5

    def detect_green_object(self, frame):

        # Define thresholds for the green color in RGB
        lower_green = np.array([settings.RGBchannel1Min, settings.RGBchannel2Min, settings.RGBchannel3Min])
        upper_green = np.array([settings.RGBchannel1Max, settings.RGBchannel2Max, settings.RGBchannel3Max])

        # Create a mask for the green color
        self.green_mask = cv2.inRange(frame, lower_green, upper_green)
        # cv2.imshow('frame', mask)
        # cv2.waitKey(0)

        # # Find the centroid of the biggest green contour
        # self.pos_x = self.centroid_biggest_green_contour(self.green_mask, settings.area)

        # # Find the centroid of the green
        self.pos_x = self.green_centroid(self.green_mask, frame)

        # if settings.squared_exit:
        # if self.pos_x != 0.5 and self.pos_x != -1:
            # print('self.pos_x != 0.5 and self.pos_x != -1')
        # return mask

    def detect_green_object_hsv(self, frame):
        # Convert the frame from BGR to HSV color space
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define thresholds for the green color in HSV
        lower_green = np.array([settings.channel1Min, settings.channel2Min, settings.channel3Min])
        upper_green = np.array([settings.channel1Max, settings.channel2Max, settings.channel3Max])

        # Create a mask for the green color
        self.green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        # cv2.imshow('frame', mask)
        # cv2.waitKey(0)

        # # Find the centroid of the biggest green contour
        # self.pos_x = self.centroid_biggest_green_contour(self.green_mask, settings.area)

        # # Find the centroid of the green
        self.pos_x = self.green_centroid(self.green_mask, frame)

        # if settings.squared_exit:
        # if self.pos_x != 0.5 and self.pos_x != -1:
            # print('self.pos_x != 0.5 and self.pos_x != -1')
        # return mask

    def centroid_biggest_green_contour(self, mask, threshold_area):
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (largest green object)
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the centroid of the largest contour
            M = cv2.moments(largest_contour)
            try:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_prop = centroid_x / mask.shape[1]
            except Exception:
                logging.warning('Problems with green centroid')
                pass

            # Calculate the area of the largest contour
            contour_area = cv2.contourArea(largest_contour)

            if contour_area > threshold_area:
                return centroid_prop
            else:
                return -1
        else:
            return -1
        
    def green_centroid(self, mask, frame):
        """
        Calculates the centroid of green pixels within the provided mask image.

        Args:
            mask (numpy.ndarray): An image mask indicating regions of interest.
            frame (numpy.ndarray): The original frame/image.

        Returns:
            float: Normalized centroid position relative to the width of the mask.
            Returns -1 if centroid calculation fails or green pixels are insufficient.
        """
        
        white_pixel_count = cv2.countNonZero(mask)

        if white_pixel_count >= settings.pixel_count_threshold:
            
            # Calculate moments of the mask
            M = cv2.moments(mask)

            try:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_prop = centroid_x / mask.shape[1]
                self.green_recently_detected = 1
                return centroid_prop
            
            except Exception:
                self.green_recently_detected = 0
                logging.warning('Problems with green centroid')
                return -1
        else:
            self.green_recently_detected = 0

            # Save the image
            output_path = "/home/pi/W/pi_cam/pi_cam_upgraded/under_threshold"+".jpg"  # Replace with the desired output path
            cv2.imwrite(output_path, frame)

            logging.warning('Green pixels are not enough to consider them as the exit')
            return -1
 
    def region_of_interest(self, mask, frame):

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Create a black image of the same size as the frame
            black_image = np.zeros_like(frame)
            
            # Draw the green region on the black image
            cv2.drawContours(black_image, [largest_contour], 0, (255, 255, 255), thickness=cv2.FILLED)
            
            # Bitwise AND the original frame with the black image to keep only the green region
            result = cv2.bitwise_and(frame, black_image)
            
            return result
        else:
            logging.warning('No green found in the image')
            return frame  # Return the original frame if no green region is found

    def detect_movement_flow(self, image1, image2, threshold):

        # Convert images to grayscale
        gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

        # Calculate optical flow using Gunnar Farneback's algorithm
        flow = cv2.calcOpticalFlowFarneback(gray1, gray2, None, 0.5, 3, 15, 3, 5, 1.2, 0) # (prev, next, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags)
        # documentation: https://docs.opencv.org/3.4/dc/d6b/group__video__track.html#ga5d10ebbd59fe09c5f650289ec0ece5af

        # Calculate magnitude of the optical flow vectors
        magnitude = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)

        # # Threshold the magnitude to identify regions with movement
        # movement_mask = magnitude > threshold

        return magnitude
    
    def jam_identification(self, counter, cropped_frame):
             
        if counter == 0:
            prev_frame = cropped_frame

            counter += 1
            return 0
        else:
            pass

        if counter%1 == 0:
            # Record start time
            # start_time = time.time()

            movement_magnitude = self.detect_movement_flow(prev_frame, cropped_frame, 20)
            
            # Record end time
            # end_time = time.time()

            prev_frame = cropped_frame
            
            if np.max(movement_magnitude) < 20:
                # print('You are in a jam!', np.max(movement_magnitude))
                # move !!!
                pass
            else:
                # print('You are moving!',np.max(movement_magnitude))
                pass
        counter += 1

    def calculate_quadratic_difference(self, image1, image2):
        """
        Computes the sum of squared differences between the green channel of two images.

        Args:
            image1 (numpy.ndarray): The first input image.
            image2 (numpy.ndarray): The second input image.

        Returns:
            int: The sum of squared differences between the green channels of the input images.
        """

        # # Ensure images have the same dimensions
        # if image1.shape != image2.shape:
        #     raise ValueError("Images must have the same dimensions")
        
        green_image1 = image1[:,:,1]
        green_image2 = image2[:,:,1]

        # Compute absolute difference
        diff = cv2.absdiff(green_image1, green_image2)

        # Compute the sum of elements raised to the power of 2
        sum_squared = np.sum(diff**2)

        return sum_squared

    def calculate_difference_in_pixels(self, image1, image2):
        """
        Calculates the number of different pixels between two images.

        Args:
            self: Reference to the object instance.
            image1 (numpy.ndarray): The first input image.
            image2 (numpy.ndarray): The second input image.

        Returns:
            int: The number of different pixels between the two images.
        """

        # # Ensure images have the same dimensions
        # if image1.shape != image2.shape:
        #     raise ValueError("Images must have the same dimensions")

        diff = cv2.absdiff(image1, image2)

        # Count non-zero pixels in the green channel of the difference image
        different_pixels = cv2.countNonZero(diff[:,:,1])

        return different_pixels

    def calculate_displacement(self, image1, prev_mask, image2):
        '''This code assumes that the displacement is relatively small between the two images. 
        If the displacement is large or involves rotation, more advanced techniques like image registration or 
        optical flow might be necessary, but they could also be more computationally expensive.'''

        # # Record start time
        # start_time = time.time()

        image1 = self.region_of_interest(prev_mask, image1)
        image2 = self.region_of_interest(self.green_mask, image2)

        # Convert images to grayscale
        gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

        # Compute absolute difference
        diff = cv2.absdiff(gray1, gray2)

        # cv2.imshow('frame', diff)
        # cv2.waitKey(0)

        # Thresholding
        _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

        # cv2.imshow('frame', thresh)
        # cv2.waitKey(0)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:

            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the area of the largest contour
            area = cv2.contourArea(largest_contour)

            # Calculate centroid of the largest contour
            
            # M = cv2.moments(largest_contour)
            # cx = int(M['m10'] / M['m00'])
            # cy = int(M['m01'] / M['m00'])

            # # Record start time
            # end_time = time.time()

            # print('tiempo de procesamiento ',  end_time-start_time)
        else:
            # end_time = time.time()

            # print('tiempo de procesamiento ', end_time-start_time)

            logging.info('No difference found in contiguous images')
            area = 0

        return area
    
    def break_jam(self, image1, prev_mask, image2):

        area = self.calculate_displacement(image1, prev_mask, image2)
                
        if area < 1000: # This value has been obtained empirically

            logging.warning(f'Jam detected! - {area}')

            # This command makes the bot move backward, but if there is no line in the next iteration AND
            # there are no obstacles, the bot will start moving forward and repeat the cycle.
            self.robot_controller.move_backward()

            # This is another way to try to brake the jam
            # self.robot_controller.line_follow_sensors_calibration()
            pass

        else:
            logging.info(f'No jam detected - {area}')
            pass

    def break_jam_new(self, image1, image2):
        """
        Checks for a potential jam based on the difference between two consecutive images.

        Args:
            image1 (numpy.ndarray): The first input image.
            image2 (numpy.ndarray): The second input image.

        Returns:
            None
        """

        # thres = self.calculate_difference_in_pixels(image1, image2)
        diff = self.calculate_quadratic_difference(image1, image2)
  
        if diff < settings.diff_thres: # This value has been obtained empirically

            logging.warning(f'Jam detected! - {diff}')

            # This command makes the bot move backward, but if there is no line in the next iteration AND
            # there are no obstacles, the bot will start moving forward and repeat the cycle.
            self.robot_controller.move_backward()

            # This is another way to try to brake the jam
            # self.robot_controller.line_follow_sensors_calibration()
            pass

        else:
            logging.warning(f'No jam detected - {diff}')
            pass

    def follow_green(self, objective = 0.5):
        # Implement green-following logic using sensor input
        # position, sensor_values = self.robot_controller.read_linefollow_sensors()

        # Calculate the error based on sensor values
        error = self.pos_x - objective

        # Implement the PID control algorithm
        self.integral += error
        derivative = error - self.prev_error

        # Calculate the motor control output using PID
        power_difference = (error * self.P) + (self.integral * self.I) + (derivative * self.D)

        if (power_difference > self.maxDC):
            power_difference = self.maxDC
        if (power_difference < - self.maxDC):
            power_difference = - self.maxDC

        # Adjust motor speeds based on the power difference
        # Implement motor control logic using power_difference        
        if (power_difference < 0):
            self.robot_controller.robot.setPWMA(self.maxDC + power_difference)
            self.robot_controller.robot.setPWMB(self.maxDC)
        else:
            self.robot_controller.robot.setPWMA(self.maxDC)
            self.robot_controller.robot.setPWMB(self.maxDC - power_difference)      

        self.prev_error = error

    def evacuations_single_obstacle_v1(self, frame, lsensor_status):
        '''Follows the exit in presence of obstacles. 
        If it loses the green, it stops'''

        self.detect_green_object(frame)

        if self.pos_x != -1: # if green is detected
            self.prev_pos_x = self.pos_x
            self.robot_controller.move_forward()

            if lsensor_status == 0:
                self.pos_x = 2            # v1 works
                self.follow_green()       # v1 works
                # self.follow_green(-2.5)
            else:
                self.follow_green()       # v1 works
                # self.follow_green(0.5)
                
            self.green_detection_counter += 1
            self.gf_counter =+ 1
            self.gf_obstacle_counter = 0

        else: 
            logging.info('green lost')
            if self.green_detection_counter == 0:
                # self.robot_controller.stop()
                self.robot_controller.rotate("left")
            else:
                if self.gf_obstacle_counter == 1 and self.gf_counter > 50:
                    self.robot_controller.hard_brake(self.maxDC)
                    self.gf_obstacle_counter =+ 1
                    self.gf_counter = 0
                else:
                    self.robot_controller.brake()

    def evacuations_single_obstacle_v2(self, frame, lsensor_status):
        '''Follows the exit in presence of obstacles. 
        If it loses the green, it rectifies and forget of the obstacle'''

        self.detect_green_object(frame)

        if self.pos_x != -1:
            self.prev_pos_x = self.pos_x
            self.robot_controller.move_forward()

            if lsensor_status == 0:
                # self.pos_x = 2            # v1 works
                # self.follow_green()       # v1 works
                self.follow_green(-2.5)
            else:
                # self.follow_green()       # v1 works
                self.follow_green(0.5)
            self.green_detection_counter += 1
            self.gf_counter =+ 1
            self.gf_obstacle_counter = 0

        else:
            logging.info('green lost')
            if self.green_detection_counter == 0:
                # self.robot_controller.stop()
                self.robot_controller.rotate("left")
            else:
                if self.prev_pos_x < 0.5:
                    # self.follow_green()       # v1 works
                    self.follow_green(0.5)
                elif self.prev_pos_x > 0.5:
                    # self.pos_x = 2            # v1 works
                    # self.follow_green()       # v1 works
                    self.follow_green(-2.5)              

    def evacuations_v1(self, frame):
        '''Follows the exit. If it loses the green, rotates until detection.
            No detection considered.
        '''

        self.detect_green_object(frame)

        if self.pos_x != -1:    # green feature has been found in frame
            self.prev_pos_x = self.pos_x
            self.robot_controller.move_forward()
            self.follow_green()
            self.green_detection_counter += 1

        else:
            if self.green_detection_counter == 0:
                self.robot_controller.rotate("left")

            else:
                if self.prev_pos_x < 0.5:
                    self.robot_controller.rotate("left")
                elif self.prev_pos_x > 0.5:
                    self.robot_controller.rotate("right")
    
    def evacuations_v2_old(self, frame):
        '''
        Follows the exit.
        Controls two main situations when the green is not found in the current frame
            - and has never been found -> rotates searching for it
            - and was found before -> rotates in the proper sense based on the previous green detection
        '''

        self.detect_green_object(frame)

        if self.pos_x != -1:    # green feature has been found
            self.prev_pos_x = self.pos_x
            self.robot_controller.move_forward()
            self.follow_green()
            self.green_detection_counter += 1
            self.green_ever_detected = 1

            # this is to restart the counter used for rotation_exploration
            self.robot_controller.green_exploration_counter = 0

        else: # no green found
                # if green never has been found or the green exploration is on the go
                if self.green_detection_counter == 0 or self.robot_controller.green_exploration_counter != 0: 

                    logging.warning('Green never detected. Doing rotation exploration')

                    # self.robot_controller.stop()

                    # TODO: sería interesante preguntarse cuales son las mejores maneras para explorar
                    
                    # Rotate to find the green
                    self.robot_controller.rotate('left')

                else: # if green was found before helps to decide the rotation sense based on last position it was found
                    logging.info('Green lost')
                    
                    if self.prev_pos_x < 0.5:
                        # self.follow_green()       # in v1 works
                        self.follow_green(0.5)
                    
                    elif self.prev_pos_x > 0.5:
                        # self.pos_x = 2            # in v1 works
                        # self.follow_green()       # in v1 works
                        self.follow_green(-2.5)
    
    def evacuations_v2(self, frame):
        '''
        Follows the exit.
        Controls two main situations when the green is not found in the current frame
            - and has never been found -> rotates searching for it
            - and was found before -> rotates in the proper sense based on the previous green detection
        '''

        self.detect_green_object(frame)

        if self.pos_x != -1:    # green feature has been found
            self.prev_pos_x = self.pos_x
            self.robot_controller.move_forward()
            self.follow_green()
            self.green_detection_counter += 1
            self.green_ever_detected = 1

            # this is to restart the counter used for rotation_exploration
            self.robot_controller.green_exploration_counter = 0

        else: # no green found
            
            if self.gf_counter % 5 == 0:
                self.robot_controller.move_backward()
            else:
                if self.prev_pos_x < 0.5:
                    # self.follow_green()       # in v1 works
                    # self.follow_green(0.5)
                    self.robot_controller.rotate('left')
                
                elif self.prev_pos_x > 0.5:
                    # self.pos_x = 2            # in v1 works
                    # self.follow_green()       # in v1 works
                    # self.follow_green(-2.5)
                    self.robot_controller.rotate('right')
                
                else: #TODO: randomize
                    self.robot_controller.rotate('left')
                
            
            # if self.green_detection_counter == 0 or self.robot_controller.green_exploration_counter != 0: # if green never has been found or the green exploration is on the go
            #     logging.warning('Green never detected. Doing rotation exploration')

            #     # self.robot_controller.stop()

            #     # TODO: sería interesante preguntarse cuales son las mejores maneras para explorar 
            #     # Rotate to find the green
            #     self.robot_controller.rotate('left')

            # else: # if green was found before helps to decide the rotation sense based on last position it was found
           

# TODO: se podría hacer un caso en que se considere continuar avanzando cuando sea solo un sensor el que detecta obstaculo.
                    
    def evacuations_v3(self, frame):
        '''
        Gives priority to obstacle detections.
        If there is an obstacle it stops no matter what
        '''

        lsensor_status, rsensor_status = self.robot_controller.read_obstacle_sensors()

        if lsensor_status != 0 and rsensor_status != 0: ### if no obstacles
            self.robot_controller.move_forward()

            # decides what to do if there is green, or not, in the current frame and, 
            # depending on previous detections, it decides what to do to find it
            self.evacuations_v2(frame)

            # gf_counter counts how many CONTINUOUS steps there is a green following
            self.gf_counter =+ 1
            # gf_obstacle_counter goes to zero when the bot is following the green
            self.gf_obstacle_counter = 0

        else: # if obstacles
            if self.pos_x == -1: # and green feature is not been seen, then find it
                # logging.warning('Obstacle detected. Forgetting about it and doing rotation exploration')
                # self.robot_controller.rotation_exploration()

                logging.info('Obstacle detected. Forgetting about it and doing green finder')
                self.robot_controller.green_finder(self.prev_pos_x)

            else: # if there is green, then stop

                if self.gf_obstacle_counter == 1 and self.gf_counter > 50:

                    self.robot_controller.hard_brake(self.maxDC)
                    
                    # counts how many CONTINUOUS steps the bot is stoped (check)
                    self.gf_obstacle_counter =+ 1
                    # goes to zero because green following was interrupted
                    self.gf_counter = 0
                else:
                    self.robot_controller.brake()

    def evacuations_v4(self, frame):
        '''Follow the exit. If it loses the green, it keeps moving rectifying the trayectory.
        If there is an obstacle it's avoided'''

        lsensor_status, rsensor_status = self.robot_controller.read_obstacle_sensors()

        if lsensor_status != 0 and rsensor_status != 0: ### if no obstacles
            self.robot_controller.move_forward()
            self.evacuations_v2(frame)
            self.gf_counter =+ 1
            self.gf_obstacle_counter = 0

        elif lsensor_status == 0 and rsensor_status == 0: # if double detection
            if self.gf_obstacle_counter == 1 and self.gf_counter > 50:
                self.robot_controller.hard_brake(self.maxDC)
                self.gf_obstacle_counter =+ 1
                self.gf_counter = 0
            else:
                self.robot_controller.brake()
        
        else:  # single detection
            self.evacuations_single_obstacle_v1(frame, lsensor_status)

    def evacuations_v5(self, frame):
        '''Follow the exit. If it loses the green, it keeps moving rectifying the trayectory.
        If there is an obstacle it's avoided'''

        lsensor_status, rsensor_status = self.robot_controller.read_obstacle_sensors()

        if lsensor_status != 0 and rsensor_status != 0: ### if no obstacles
            self.robot_controller.move_forward()
            self.evacuations_v2(frame)
            self.gf_counter =+ 1
            self.gf_obstacle_counter = 0

        elif lsensor_status == 0 and rsensor_status == 0: # if double detection
            if self.gf_obstacle_counter == 1 and self.gf_counter > 50:
                self.robot_controller.hard_brake(self.maxDC)
                self.gf_obstacle_counter =+ 1
                self.gf_counter = 0
            else:
                self.robot_controller.brake()
        
        else:  # single detection
            self.evacuations_single_obstacle_v2(frame, lsensor_status)

# Create an instance of GreenObjectDetectionController
# green_object_detector = GreenObjectDetectionController(robot_controller)
