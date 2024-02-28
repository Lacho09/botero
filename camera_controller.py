import settings
import picamera
import picamera.array
import time
import cv2

class CameraObjectController:

    def __init__(self):
        self.camera = picamera.PiCamera()
        # self.camera = camera
        self.stream = picamera.array.PiRGBArray(self.camera)
        self.shutter_speed = settings.shutter_speed
        self.gain_value = settings.gain_value
        self.gain_value2 = settings.gain_value2
        pass

    def adjust_settings(self):

        # Set camera exposure mode (options: 'auto', 'off', 'night', 'nightpreview', 'backlight', 'spotlight', 'sports', 'snow', 'beach', 'verylong', 'fixedfps', 'antishake', 'fireworks')
        self.camera.exposure_mode = 'off'

        # # Set camera ISO sensitivity (values: 0, 100, 200, 320, 400, 500, 640, 800)
        self.camera.iso = 1600  # Adjust this value based on lighting conditions

        self.camera.shutter_speed = 20000 #microsegs

        # # Set camera shutter speed (units: microseconds, e.g., 10000 = 1/10000s)
        # camera.shutter_speed = 0  # Set to 0 to use auto shutter speed

        # # Set camera white balance mode (options: 'auto', 'off', 'sunlight', 'cloudy', 'shade', 'tungsten', 'fluorescent', 'incandescent', 'flash', 'horizon')
        self.camera.awb_mode = 'off'


        # # Set the camera gain to the specified values
        self.camera.awb_gains = (self.gain_value, self.gain_value2)

        # # Wait for the camera to adjust to the new gain settings
        time.sleep(2)

        # # Capture a photo (you can also capture a video)
        # camera.capture('photo_with_gain_' + str(gain_value) + '.jpg')

        self.stream = picamera.array.PiRGBArray(self.camera)

        # return self.camera, stream

    def show(frame):
        # # Show the original image using cv2.imshow
        cv2.imshow('frame', frame)
        cv2.waitKey(0)


    def save_image(self, ide=0):
    
        # Save the image
        output_path = "/home/pi/W/pi_cam/pi_cam_upgraded/output_image"+f'{ide}'+".jpg"  # Replace with the desired output path
        cv2.imwrite(output_path,self.stream.array)
        
        # # Clear the stream in preparation for the next frame
        # self.stream.truncate(0)

        # # Break the loop when 'q' key is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()