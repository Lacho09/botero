### Camera settings
gain_value = 1.16796875
gain_value2 = 1.90234375
shutter_speed = 20000 # 1ms
iso = 1600

############################################################33
### Green detection
crop_pos = 0
connected_pixels = 15000
pixel_count_threshold = 25000

# ########## Cartulina
squared_exit = True

### RGB values
# Define thresholds for channel 1 based on histogram settings
RGBchannel1Min = 0
RGBchannel1Max = 255

# Define thresholds for channel 2 based on histogram settings
RGBchannel2Min = 18
RGBchannel2Max = 120

# Define thresholds for channel 3 based on histogram settings
RGBchannel3Min = 0
RGBchannel3Max = 40


### Iker values
# # Define thresholds for channel 1 based on histogram settings
# HSVchannel1Min = 0.252 * 179
# HSVchannel1Max = 0.348 * 179

# # Define thresholds for channel 2 based on histogram settings
# HSVchannel2Min = 0.441 * 255
# HSVchannel2Max = 1.000 * 255

# # Define thresholds for channel 3 based on histogram settings
# HSVchannel3Min = 0.108 * 255
# HSVchannel3Max = 0.581 * 255

# ### Others
# # Define thresholds for channel 1 based on histogram settings
# channel1Min = 0.286 * 179
# channel1Max = 0.391 * 179

# # Define thresholds for channel 2 based on histogram settings
# channel2Min = 0.319 * 255
# channel2Max = 0.973 * 255

# # Define thresholds for channel 3 based on histogram settings
# channel3Min = 0.160 * 255
# channel3Max = 0.436 * 255

# # ### Old values
# # Define thresholds for channel 1 based on histogram settings
# channel1Min = 0.281 * 179
# channel1Max = 0.372 * 179

# # Define thresholds for channel 2 based on histogram settings
# channel2Min = 0.388 * 255
# channel2Max = 0.802 * 255

# # Define thresholds for channel 3 based on histogram settings
# channel3Min = 0.293 * 255
# channel3Max = 0.649 * 255

### PID values
green_P = 20 
green_I = 0 
green_D = 10
green_maxDC = 25

# ### PID values
# green_P = 30 
# green_I = 0 
# green_D = 30
# green_maxDC = 40

### PID values
# green_P = 30 
# green_I = 0 
# green_D = 20
# green_maxDC = 55

### Jam detection
diff_thres = 1000000

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
sensor_threshold = 200
breaking_time = 0.2

# ### PID values
# line_P = 0.03
# line_I = 0
# line_D = 1
# line_maxDC = 30

### PID values
line_P = 0.03
line_I = 0
line_D = 1
line_maxDC = 25

### Bifurcation imbalance maxDC = 25
bif_right = 3000
bif_left = 1000

# Steps after line detection to ignore obstacle detection
lf_ignore_obstacle = 800