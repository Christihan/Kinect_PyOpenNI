from openni import *
import numpy as np
import cv2
 
# Initialise OpenNI
context = Context()
context.init()

depth = DepthGenerator()
depth.create(context)
depth.set_resolution_preset(RES_VGA)
depth.fps = 30

# Start Kinect
    

context.start_generating_all()   
#while True: 


while True:
    context.wait_any_update_all()
# Create a depth generator to access the depth stream
    

# Create array from the raw depth map string
    frame = np.fromstring(depth.get_raw_depth_map_8(), "uint8").reshape(480, 640)
 
# Render in OpenCV
    cv2.imshow("image", frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
       	break
