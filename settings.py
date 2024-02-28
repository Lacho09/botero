### Camera settings
gain_value = 1.16796875
gain_value2 = 1.90234375
shutter_speed = 33000 # 1ms

############################################################33
### Green detection
crop_pos = 0
connected_pixels = 15000

# ########## Cartulina
squared_exit = True

# Define thresholds for channel 1 based on histogram settings
channel1Min = 0.281 * 179
channel1Max = 0.372 * 179

# Define thresholds for channel 2 based on histogram settings
channel2Min = 0.388 * 255
channel2Max = 0.802 * 255

# Define thresholds for channel 3 based on histogram settings
channel3Min = 0.293 * 255
channel3Max = 0.649 * 255

### PID values
green_P = 20 
green_I = 0 
green_D = 10
green_maxDC = 25

# ########## Plastic 3D-printed rod
# # Define thresholds for channel 1 based on histogram settings
# channel1Min = 0.384 * 179 # Scale to HSV range (0 to 179) 
# channel1Max = 0.415 * 179 # Scale to HSV range (0 to 179)

# # Define thresholds for channel 2 based on histogram settings
# channel2Min = 0.569 * 255
# channel2Max = 1.000 * 255

# # Define thresholds for channel 3 based on histogram settings
# channel3Min = 0.384 * 255
# channel3Max = 0.665 * 255

# Define the threshold area for contours
area = 1000
#############################################################

### Line follow
sensor_threshold = 500
breaking_time = 0.2

### Green follow
lower_limit = 0.3
upper_limit = 0.7
correction_obstacle = 1
correction = 1

maximum = 20
P = 10
I = 0
D = 5
