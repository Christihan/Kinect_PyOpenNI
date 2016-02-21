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


image = ImageGenerator()
image.create(context)
image.set_resolution_preset(DefResolution.RES_VGA)
image.fps = 30

# Start Kinect
context.start_generating_all()   
#while True: 


while True:
    #context.wait_any_update_all()
    context.wait_and_update_all()
    #context.wait_one_update_all(image)
    #context.wait_one_update_all(depth)
# Create a depth generator to access the depth stream

# Create array from the raw depth map string
    frame = np.fromstring(depth.get_raw_depth_map_8(), "uint8").reshape(480, 640)
    frame2 = np.fromstring(image.get_raw_image_map(), "uint8").reshape(480, 640,3)   
    frame2 = cv2.cvtColor(frame2,cv2.COLOR_BGR2RGB)
# Render in OpenCV
    cv2.imshow("Depth", frame)
    cv2.imshow("Color", frame2)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
       	break
