from openni import *
import numpy as np
import cv2
 
# Initialise OpenNI
context = Context()
context.init()

image = ImageGenerator()
image.create(context)
image.set_resolution_preset(DefResolution.RES_VGA)
image.fps = 30
context.start_generating_all()

#while True: 


while True:
    context.wait_any_update_all()
# Create a depth generator to access the depth stream
    

# Create array from the raw depth map string
    frame = np.fromstring(image.get_raw_image_map(), "uint8").reshape(480, 640,3)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
# Render in OpenCV
    cv2.imshow("image", frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
       	break
