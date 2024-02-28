import cv2
import numpy as np
import settings
# from camera_controller import CameraObjectController

def region_of_interest(mask, frame):

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
        print("No se encontró región verde en la imagen.")
        return frame  # Return the original frame if no green region is found

    # cv2.imshow('ROI Verde', roi_green)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # O guardar la región verde como una nueva imagen
    # cv2.imwrite('region_verde.jpg', roi_green)

def detect_movement(image1, image2, threshold=39):
    # Convert images to grayscale
    gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

    # Calculate optical flow using Gunnar Farneback's algorithm
    flow = cv2.calcOpticalFlowFarneback(gray1, gray2, None, 0.5, 3, 15, 3, 5, 1.2, 0) # (prev, next, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags)
    # documentation: https://docs.opencv.org/3.4/dc/d6b/group__video__track.html#ga5d10ebbd59fe09c5f650289ec0ece5af

    # Calculate magnitude of the optical flow vectors
    magnitude = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)

    # print(magnitude)

    # Threshold the magnitude to identify regions with movement
    movement_mask = magnitude > threshold

    # Draw squares on the image for visualization
    image_with_squares = image2.copy()
    square_size = 10

    for y in range(0, image2.shape[0], square_size):
        for x in range(0, image2.shape[1], square_size):
            if movement_mask[y:y + square_size, x:x + square_size].any():
                cv2.rectangle(image_with_squares, (x, y), (x + square_size, y + square_size), (0, 255, 0), 2)

    return image_with_squares, movement_mask, magnitude

def calculate_displacement(image1, image2):
    '''This code assumes that the displacement is relatively small between the two images. 
    If the displacement is large or involves rotation, more advanced techniques like image registration or o
    ptical flow might be necessary, but they could also be more computationally expensive.'''

    # Convert images to grayscale
    gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

    # Compute absolute difference
    diff = cv2.absdiff(gray1, gray2)

    # cv2.imshow('frame', diff)
    # cv2.waitKey(0)

    # Thresholding
    _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

    cv2.imshow('frame', thresh)
    cv2.waitKey(0)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:

        largest_contour = max(contours, key=cv2.contourArea)


        # Create a black image of the same size as thresh
        # contour_overlay = np.zeros_like(thresh)
        contour_overlay = thresh.copy()

        # Draw the largest contour in white on the black image
        cv2.drawContours(contour_overlay, [largest_contour], -1, 50, thickness=cv2.FILLED)

        # Display the black image with the largest contour in white
        cv2.imshow('Black Image with Largest Contour', contour_overlay)
        cv2.waitKey(0)


        # Calculate the area of the largest contour
        area = cv2.contourArea(largest_contour)

        # Calculate centroid of the largest contour
        
        # M = cv2.moments(largest_contour)
        # cx = int(M['m10'] / M['m00'])
        # cy = int(M['m01'] / M['m00'])

        return area
    else:
        return 0

def detect_movement_diff(image1, image2, threshold=30):
    # Convert images to grayscale
    gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

    # Compute absolute difference between frames
    frame_diff = cv2.absdiff(gray1, gray2)

    # Threshold the difference to identify regions with movement
    _, movement_mask = cv2.threshold(frame_diff, threshold, 255, cv2.THRESH_BINARY)

    # Draw squares on the image for visualization
    # image_with_squares = image2.copy()
    # square_size = 20

    # for y in range(0, image2.shape[0], square_size):
    #     for x in range(0, image2.shape[1], square_size):
    #         if movement_mask[y:y + square_size, x:x + square_size].any():
    #             cv2.rectangle(image_with_squares, (x, y), (x + square_size, y + square_size), (0, 255, 0), 2)

    return frame_diff, movement_mask

# # Find the centroid of the biggest green contour
# self.pos_x = self.centroid_biggest_green_contour(mask, settings.area)

def green_mask(frame):

    #Convert the frame from BGR to HSV color space
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define thresholds for the green color in HSV
    lower_green = np.array([settings.channel1Min, settings.channel2Min, settings.channel3Min])
    upper_green = np.array([settings.channel1Max, settings.channel2Max, settings.channel3Max])

    # Create a mask for the green color
    mask = cv2.inRange(hsv_image, lower_green, upper_green)

    return mask


image_path = '/home/lacho/Documents/Postdoc_Pamplona_2023/Investigacion/Robots/pi_cam/imagenes/sequence/output_image2276.jpg'
image_path2 = '/home/lacho/Documents/Postdoc_Pamplona_2023/Investigacion/Robots/pi_cam/imagenes/sequence/output_image2277.jpg'
frame = cv2.imread(image_path)
frame2 = cv2.imread(image_path2)

green_mask1 = green_mask(frame)
green_mask2 = green_mask(frame2)

image = region_of_interest(green_mask1, frame)
image2 = region_of_interest(green_mask2, frame2)

# cv2.imshow('frame', frame2)
# cv2.waitKey(0)

# cv2.imshow('frame', image2)
# cv2.waitKey(0)


displacement = calculate_displacement(frame, frame2)
if displacement:
    # if displacement[0] >  or displacement[1]
    print(f"Displacement: {displacement}")
else:
    print("No significant displacement detected.")

# print(frame.shape)
# prev_frame = np.zeros((480,720))
# print(prev_frame.shape)
# # cv2.imshow('frame', frame2)
# # cv2.waitKey(0)

# # # Crop the upper portion of the frame (adjust the values to your desired crop size)
# # cropped_frame = frame.array[settings.crop_pos:, :]

# mask1 = green_mask(frame)
# mask2 = green_mask(frame2)

# # cv2.imshow('frame', mask1)
# # cv2.waitKey(0)


# roi1 = region_of_interest(mask1, frame)
# roi2 = region_of_interest(mask2, frame2)

# # cv2.imshow('Movement Detection', roi1)
# # cv2.waitKey(0)

# # cv2.imshow('Movement Detection', roi2)
# # cv2.waitKey(0)

# # frame_diff, movement_mask = detect_movement(roi1, roi2, threshold=10)
# # frame_diff, movement_mask = detect_movement_diff(roi1, roi2, threshold=0)

# image_sq, movement_mask2, magnitude = detect_movement(roi1,roi2)

# if np.max(magnitude) < 20:
#     print('You are in a jam!', np.max(magnitude))
#     # move !!!
#     pass
# else:
#     print('You are moving!')
#     pass

# # cv2.imshow('Movement Detection', image_sq)
# # cv2.waitKey(0)
