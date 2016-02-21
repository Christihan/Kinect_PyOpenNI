#!/usr/bin/python
# Choregraphe simplified export in Python.
from naoqi import ALProxy

# Pose to use to calibrate the user
pose_to_use = 'Psi'

import cv2
from cv import *
from openni import *
import numpy as np
import math
import time
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

n = 0

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("equal")
zdir = (1, 1, 1)

ax.set_xlim3d(0, 1.5)
ax.set_ylim3d(0, 1.5)
ax.set_zlim3d(0, 1.5)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.view_init(-90,-90)
plt.ion()
plt.draw()

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

naoip = "127.0.0.1" 
port = 56174
posture = ALProxy("ALRobotPosture", naoip, port)
motion = ALProxy("ALMotion", naoip, port)

ctx = Context()
ctx.init()

# Create the user generator
user = UserGenerator()
user.create(ctx)

# Obtain the skeleton & pose detection capabilities
skel_cap = user.skeleton_cap
pose_cap = user.pose_detection_cap

# Declare the callbacks
def new_user(src, id):
    print "1/4 User {} detected. Looking for pose..." .format(id)
    pose_cap.start_detection(pose_to_use, id)

def pose_detected(src, pose, id):
    print "2/4 Detected pose {} on user {}. Requesting calibration..." .format(pose,id)
    pose_cap.stop_detection(id)
    skel_cap.request_calibration(id, True)

def calibration_start(src, id):
    print "3/4 Calibration started for user {}." .format(id)

def calibration_complete(src, id, status):
    if status == CALIBRATION_STATUS_OK:
        print "4/4 User {} calibrated successfully! Starting to track." .format(id)
        skel_cap.start_tracking(id)
    else:
        print "ERR User {} failed to calibrate. Restarting process." .format(id)
        new_user(user, id)

def lost_user(src, id):
    print "--- User {} lost." .format(id)

# Register them
user.register_user_cb(new_user, lost_user)
pose_cap.register_pose_detected_cb(pose_detected)
skel_cap.register_c_start_cb(calibration_start)
skel_cap.register_c_complete_cb(calibration_complete)

# Set the profile
skel_cap.set_profile(SKEL_PROFILE_ALL)

# Start generating
ctx.start_generating_all()
print "0/4 Starting to detect users. Press Ctrl-C to exit."

################################################################################################
################################################################################################
def detect_skeleton(): 
    global neck, head, rshoul, relbow, rhand, lhand, lelbow, lshoul, torso, lhip, rhip, lknee, rknee, lfoot, rfoot
    m=0
    neck = np.arange(3)
    head = np.arange(3)
    lshoul = np.arange(3)
    lelbow = np.arange(3)
    lhand = np.arange(3)
    rshoul = np.arange(3)
    relbow = np.arange(3)
    rhand = np.arange(3)
    torso = np.arange(3)
    lhip = np.arange(3)
    rhip = np.arange(3)
    rknee = np.arange(3)
    rfoot = np.arange(3)
    lknee = np.arange(3)
    lfoot = np.arange(3) 
# Extract head position of each tracked user
    for id in user.users:
        if skel_cap.is_tracking(id):
	    m=1
	    neck1 = skel_cap.get_joint_position(id, SKEL_NECK)
	    head1 = skel_cap.get_joint_position(id, SKEL_HEAD)
	    rshoul1=skel_cap.get_joint_position(id, SKEL_RIGHT_SHOULDER)
	    relbow1 = skel_cap.get_joint_position(id, SKEL_RIGHT_ELBOW)
	    rhand1 = skel_cap.get_joint_position(id, SKEL_RIGHT_HAND)
            lhand1 = skel_cap.get_joint_position(id, SKEL_LEFT_HAND)
	    lelbow1 = skel_cap.get_joint_position(id, SKEL_LEFT_ELBOW)
	    lshoul1=skel_cap.get_joint_position(id, SKEL_LEFT_SHOULDER)
	    torso1 = skel_cap.get_joint_position(id, SKEL_TORSO)
	    lhip1 = skel_cap.get_joint_position(id, SKEL_LEFT_HIP)
	    rhip1= skel_cap.get_joint_position(id, SKEL_RIGHT_HIP)
	    lknee1=skel_cap.get_joint_position(id, SKEL_LEFT_KNEE)
	    rknee1=skel_cap.get_joint_position(id, SKEL_RIGHT_KNEE)
	    lfoot1=skel_cap.get_joint_position(id, SKEL_LEFT_FOOT)
	    rfoot1=skel_cap.get_joint_position(id, SKEL_RIGHT_FOOT)
	    
	    neck[0] = (0.000002*(neck1.point[0]/3.5 + 320)**3-0.0019*(neck1.point[0]/3.5 + 320)**2 + 1.2803*(neck1.point[0]/3.5 + 320) +33.701)
	    neck[1] = ((-0.001*(-neck1.point[1]/3.5 + 267.9943)**2+1.1816*(-neck1.point[1]/3.5 + 267.9943) - 1.072))
	    neck[2] = neck1.point[2]/4
	    head = (head1.point[0] - neck1.point[0])/4, -(head1.point[1] - neck1.point[1])/4, -(head1.point[2] - neck1.point[2])/4 
	    rshoul = (rshoul1.point[0] - neck1.point[0])/4, -(rshoul1.point[1] - neck1.point[1])/4, -(rshoul1.point[2] - neck1.point[2])/4 
	    relbow = (relbow1.point[0] - neck1.point[0])/4, -(relbow1.point[1] - neck1.point[1])/4, -(relbow1.point[2] - neck1.point[2])/4
	    rhand = (rhand1.point[0]-neck1.point[0])/4, -(rhand1.point[1]-neck1.point[1])/4, -(rhand1.point[2]-neck1.point[2])/4	
	    lhand = (lhand1.point[0]-neck1.point[0])/4, -(lhand1.point[1]-neck1.point[1])/4, -(lhand1.point[2]-neck1.point[2])/4
	    lelbow = (lelbow1.point[0] - neck1.point[0])/4, -(lelbow1.point[1] - neck1.point[1])/4, -(lelbow1.point[2] - neck1.point[2])/4
	    lshoul = (lshoul1.point[0] - neck1.point[0])/4, -(lshoul1.point[1] - neck1.point[1])/4, -(lshoul1.point[2] - neck1.point[2])/4
	    torso = (torso1.point[0] - neck1.point[0])/4, -(torso1.point[1] - neck1.point[1])/4, -(torso1.point[2] - neck1.point[2])/4
	    lhip = (lhip1.point[0] - neck1.point[0])/4, -(lhip1.point[1] - neck1.point[1])/4, -(lhip1.point[2] - neck1.point[2])/4
	    rhip = (rhip1.point[0] - neck1.point[0])/4, -(rhip1.point[1] - neck1.point[1])/4, -(rhip1.point[2] - neck1.point[2])/4
	    lknee = (lknee1.point[0] - neck1.point[0])/4, -(lknee1.point[1] - neck1.point[1])/4, -(lknee1.point[2] - neck1.point[2])/4
	    rknee = (rknee1.point[0] - neck1.point[0])/4, -(rknee1.point[1] - neck1.point[1])/4, -(rknee1.point[2] - neck1.point[2])/4
	    lfoot = (lfoot1.point[0] - neck1.point[0])/4, -(lfoot1.point[1] - neck1.point[1])/4, -(lfoot1.point[2] - neck1.point[2])/4
	    rfoot = (rfoot1.point[0] - neck1.point[0])/4, -(rfoot1.point[1] - neck1.point[1])/4, -(rfoot1.point[2] - neck1.point[2])/4
	    
    return m, neck, head, rshoul, relbow, rhand, lhand, lelbow, lshoul, torso, lhip, rhip, lknee, rknee, lfoot, rfoot

################################################################################################
################################################################################################
def show_skeleton(m, neck1, head1, rshoul1, relbow1, rhand1, lhand1, lelbow1, lshoul1, torso1, lhip1, rhip1, lknee1, rknee1, lfoot1, rfoot1):	    
    #Genero la imagen de profundidad
    depth = DepthGenerator()
    depth.create(ctx)
    depth.set_resolution_preset(RES_VGA)
    depth.fps = 30
    newImg = np.fromstring(depth.get_raw_depth_map_8(), "uint8").reshape(480, 640)
    newImg = cv2.cvtColor(newImg,cv2.COLOR_GRAY2BGR)
    #detecto los joints
	    	
    if (m == 1):	    	 	
    	cv2.circle(newImg, (int(neck[0]),int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(head[0]) + int(neck[0]),int(head[1]) + int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(rshoul[0])+int(neck[0]),int(rshoul[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(relbow[0])+int(neck[0]),int(relbow[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(rhand[0])+int(neck[0]),int(rhand[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(lshoul[0])+int(neck[0]),int(lshoul[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(lelbow[0])+int(neck[0]),int(lelbow[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(lhand[0])+int(neck[0]),int(lhand[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(torso[0])+int(neck[0]),int(torso[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(rhip[0])+int(neck[0]),int(rhip[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(lhip[0])+int(neck[0]),int(lhip[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(rknee[0])+int(neck[0]),int(rknee[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(lknee[0])+int(neck[0]),int(lknee[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(rfoot[0])+int(neck[0]),int(rfoot[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.circle(newImg, (int(lfoot[0])+int(neck[0]),int(lfoot[1])+int(neck[1])),5,(255, 255, 255),-1)
    	cv2.line(newImg,(int(neck[0]),int(neck[1])),(int(head[0]) + int(neck[0]),int(head[1]) + int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(neck[0]),int(neck[1])),(int(lshoul[0])+int(neck[0]),int(lshoul[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(neck[0]),int(neck[1])),(int(rshoul[0])+int(neck[0]),int(rshoul[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(rshoul[0])+int(neck[0]),int(rshoul[1])+int(neck[1])),(int(torso[0])+int(neck[0]),int(torso[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(rshoul[0])+int(neck[0]),int(rshoul[1])+int(neck[1])),(int(relbow[0])+int(neck[0]),int(relbow[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(relbow[0])+int(neck[0]),int(relbow[1])+int(neck[1])),(int(rhand[0])+int(neck[0]),int(rhand[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(lshoul[0])+int(neck[0]),int(lshoul[1])+int(neck[1])),(int(lelbow[0])+int(neck[0]),int(lelbow[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(lelbow[0])+int(neck[0]),int(lelbow[1])+int(neck[1])),(int(lhand[0])+int(neck[0]),int(lhand[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(lshoul[0])+int(neck[0]),int(lshoul[1])+int(neck[1])),(int(torso[0])+int(neck[0]),int(torso[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(torso[0])+int(neck[0]),int(torso[1])+int(neck[1])),(int(rhip[0])+int(neck[0]),int(rhip[1])+int(neck[1])),(0,255,0),2)	
    	cv2.line(newImg,(int(torso[0])+int(neck[0]),int(torso[1])+int(neck[1])),(int(lhip[0])+int(neck[0]),int(lhip[1])+int(neck[1])),(0,255,0),2)	
    	cv2.line(newImg,(int(rhip[0])+int(neck[0]),int(rhip[1])+int(neck[1])),(int(rknee[0])+int(neck[0]),int(rknee[1])+int(neck[1])),(0,255,0),2)	
    	cv2.line(newImg,(int(lhip[0])+int(neck[0]),int(lhip[1])+int(neck[1])),(int(lknee[0])+int(neck[0]),int(lknee[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(rknee[0])+int(neck[0]),int(rknee[1])+int(neck[1])),(int(rfoot[0])+int(neck[0]),int(rfoot[1])+int(neck[1])),(0,255,0),2)
    	cv2.line(newImg,(int(lknee[0])+int(neck[0]),int(lknee[1])+int(neck[1])),(int(lfoot[0])+int(neck[0]),int(lfoot[1])+int(neck[1])),(0,255,0),2)
   	cv2.line(newImg,(int(lhip[0])+int(neck[0]),int(lhip[1])+int(neck[1])),(int(rhip[0])+int(neck[0]),int(rhip[1])+int(neck[1])),(0,255,0),2)
    
    newImg = cv2.flip (newImg, 1)
    cv2.imshow("Main", newImg)

################################################################################################
################################################################################################
def equations_lhand_x(xp,anglelbowxz):
    alpha = math.atan2(xp[1],xp[0])
    beta = math.atan2(xp[2],math.sqrt(xp[0]**2+xp[1]**2))
    yp = [-math.sin(beta)*math.cos(alpha), -math.sin(beta)*math.sin(alpha), math.cos(beta)]
    zp = [xp[1]*yp[2]-xp[2]*yp[1], -xp[0]*yp[2]+xp[2]*yp[0], xp[0]*yp[1]-xp[1]*yp[0]]
    theta = 180*math.pi/180 
    yp_r_x = yp[0]*(math.cos(theta-anglelbowxz)+(xp[0]**2)*(1-math.cos(theta-anglelbowxz)))+yp[1]*(xp[0]*xp[1]*(1-math.cos(theta-anglelbowxz))-xp[2]*math.sin(theta-anglelbowxz))+yp[2]*(xp[0]*xp[2]*(1-math.cos(theta-anglelbowxz))-xp[1]*math.sin(theta-anglelbowxz))
    yp_r_y = yp[0]*(xp[1]*xp[0]*(1-math.cos(theta-anglelbowxz))-xp[2]*math.sin(theta-anglelbowxz))+yp[1]*(math.cos(theta-anglelbowxz)+(xp[1]**2)*(1-math.cos(theta-anglelbowxz)))+yp[2]*(xp[2]*xp[1]*(1-math.cos(theta-anglelbowxz))-xp[0]*math.sin(theta-anglelbowxz))
    yp_r_z = yp[0]*(xp[2]*xp[0]*(1-math.cos(theta-anglelbowxz))-xp[1]*math.sin(theta-anglelbowxz))+yp[1]*(xp[2]*xp[1]*(1-math.cos(theta-anglelbowxz))-xp[0]*math.sin(theta-anglelbowxz))+yp[2]*(math.cos(theta-anglelbowxz)+(xp[2]**2)*(1-math.cos(theta-anglelbowxz)))
    yp_r = [yp_r_x,yp_r_y,yp_r_z]
    zp_r = [xp[1]*yp_r[2]-xp[2]*yp_r[1], -xp[0]*yp_r[2]+xp[2]*yp_r[0], xp[0]*yp_r[1]-xp[1]*yp_r[0]]
    return yp_r,zp_r

################################################################################################
################################################################################################
def equations_rhand_x(xp,angrelbowxz):
    beta = math.atan2(xp[2],math.sqrt(xp[0]**2+xp[1]**2))
    yp = [-math.sin(beta)*math.cos(alpha), -math.sin(beta)*math.sin(alpha), math.cos(beta)]
    zp = [xp[1]*yp[2]-xp[2]*yp[1], -xp[0]*yp[2]+xp[2]*yp[0], xp[0]*yp[1]-xp[1]*yp[0]]
    theta = 180*math.pi/180 
    yp_r_x = yp[0]*(math.cos(theta-angrelbowxz)+(xp[0]**2)*(1-math.cos(theta-angrelbowxz)))+yp[1]*(xp[0]*xp[1]*(1-math.cos(theta-angrelbowxz))-xp[2]*math.sin(theta-angrelbowxz))+yp[2]*(xp[0]*xp[2]*(1-math.cos(theta-angrelbowxz))-xp[1]*math.sin(theta-angrelbowxz))
    yp_r_y = yp[0]*(xp[1]*xp[0]*(1-math.cos(theta-angrelbowxz))-xp[2]*math.sin(theta-angrelbowxz))+yp[1]*(math.cos(theta-angrelbowxz)+(xp[1]**2)*(1-math.cos(theta-angrelbowxz)))+yp[2]*(xp[2]*xp[1]*(1-math.cos(theta-angrelbowxz))-xp[0]*math.sin(theta-angrelbowxz))
    yp_r_z = yp[0]*(xp[2]*xp[0]*(1-math.cos(theta-angrelbowxz))-xp[1]*math.sin(theta-angrelbowxz))+yp[1]*(xp[2]*xp[1]*(1-math.cos(theta-angrelbowxz))-xp[0]*math.sin(theta-angrelbowxz))+yp[2]*(math.cos(theta-angrelbowxz)+(xp[2]**2)*(1-math.cos(theta-angrelbowxz)))
    yp_r = [yp_r_x,yp_r_y,yp_r_z]
    zp_r = [xp[1]*yp_r[2]-xp[2]*yp_r[1], -xp[0]*yp_r[2]+xp[2]*yp_r[0], xp[0]*yp_r[1]-xp[1]*yp_r[0]]
    return yp_r,zp_r

################################################################################################
################################################################################################
def show_3d_axis(xp,yp,zp):
    ax.text(xp[0]+0.75, xp[1]+0.75, xp[2]+0.75, 'xp', zdir)
    ax.text(yp[0]+0.75, yp[1]+0.75, yp[2]+0.75,'yp', zdir)
    ax.text(zp[0]+0.75, zp[1]+0.75, zp[2]+0.75, 'zp', zdir)
    ax.text(1 + 0.75, 0.75, 0.75, 'x', zdir)
    ax.text(0.75, 1+0.75, 0.75,'y', zdir)
    ax.text(0.75, 0.75, 1+0.75, 'z', zdir)

    a = Arrow3D([0.75,xp[0]+0.75],[0.75, xp[1]+0.75],[0.75,xp[2]+0.75], mutation_scale=30, lw=1, arrowstyle="-|>", color="k")
    b = Arrow3D([0.75,yp[0]+0.75],[0.75,yp[1]+0.75],[0.75,yp[2]+0.75], mutation_scale=30, lw=1, arrowstyle="-|>", color="k")
    c = Arrow3D([0.75,zp[0]+0.75],[0.75,zp[1]+0.75],[0.75,zp[2]+0.75], mutation_scale=30, lw=1, arrowstyle="-|>", color="k")
    d = Arrow3D([0.75,1+0.75],[0.75, 0.75],[0.75,0.75], mutation_scale=30, lw=1, arrowstyle="-|>", color="k")
    e = Arrow3D([0.75,0.75],[0.75, 1+0.75],[0.75,0.75], mutation_scale=30, lw=1, arrowstyle="-|>", color="k")
    f = Arrow3D([0.75,0.75],[0.75, 0.75],[0.75,1+0.75], mutation_scale=30, lw=1, arrowstyle="-|>", color="k")

    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)
    ax.add_artist(d)
    ax.add_artist(e)
    ax.add_artist(f)
	
    plt.draw()
    plt.pause(0.0000000000001) 
    plt.cla()	

################################################################################################
################################################################################################
def Operations_larm(lshoul,lelbow,lhand):
    vect_lhandlelbow = [lhand[0]-lelbow[0], lhand[1]-lelbow[1], lhand[2]-lelbow[2]]
    abs_vect_lhandlelbow = math.sqrt(vect_lhandlelbow[0]**2+vect_lhandlelbow[1]**2+vect_lhandlelbow[2]**2)
    vect_lhandlelbow = [vect_lhandlelbow[0]/abs_vect_lhandlelbow,vect_lhandlelbow[1]/abs_vect_lhandlelbow,vect_lhandlelbow[2]/abs_vect_lhandlelbow]
    abs_vect_lelbow = math.sqrt((lelbow[0]-lshoul[0])**2+(lelbow[1]-lshoul[1])**2+((lelbow[2]-lshoul[2])**2))
    anglelbowxy = -math.asin(abs((lelbow[2]-lshoul[2]))/abs_vect_lelbow)
    anglelbowyz = -math.asin(abs((lelbow[0]-lshoul[0]))/abs_vect_lelbow)
    anglelbowxz = math.asin(abs((lelbow[1]-lshoul[1]))/abs_vect_lelbow)
    if  (lelbow[2] >= lshoul[2]):
        anglelbowxy = math.asin(abs((lelbow[2]-lshoul[2]))/abs_vect_lelbow)
  
    if  (lelbow[0] >= lshoul[0]):
        anglelbowyz = math.asin(abs((lelbow[0]-lshoul[0]))/abs_vect_lelbow)
    		    
    if (lelbow[1] <= lshoul[1]): 
        anglelbowxz = -math.asin(abs((lelbow[1]-lshoul[1]))/abs_vect_lelbow)
    xp = [(lelbow[0]-lshoul[0])/abs_vect_lelbow,(lelbow[1]-lshoul[1])/abs_vect_lelbow, (lelbow[2]-lshoul[2])/abs_vect_lelbow]    
    yp_r,zp_r = equations_lhand_x(xp,anglelbowxz)
    if vect_lhandlelbow[1] > yp_r[1]:
        anglhand_xpyp = -abs(zp_r[0]*vect_lhandlelbow[0]+zp_r[1]*vect_lhandlelbow[1]+zp_r[2]*vect_lhandlelbow[2])
    else:
	anglhand_xpyp = abs(zp_r[0]*vect_lhandlelbow[0]+zp_r[1]*vect_lhandlelbow[1]+zp_r[2]*vect_lhandlelbow[2])
    f1 = vect_lhandlelbow[1]*zp_r[2]-vect_lhandlelbow[2]*zp_r[1]
    f2 = vect_lhandlelbow[0]*zp_r[2]-vect_lhandlelbow[2]*zp_r[0]
    f3 = vect_lhandlelbow[0]*zp_r[1]-vect_lhandlelbow[1]*zp_r[0]
    proy_xpyp = [(-(zp_r[1]*f3+zp_r[2]*f2)),(zp_r[0]*f3-zp_r[2]*f1),(zp_r[0]*(zp_r[1]*f3+zp_r[2]*f2)-zp_r[1]*(zp_r[0]*f3-zp_r[2]*f1))/zp_r[2]]
    abs_proy_xpyp = math.sqrt(proy_xpyp[0]**2+proy_xpyp[1]**2+proy_xpyp[2]**2)
    proy_xpyp = [proy_xpyp[0]/abs_proy_xpyp,proy_xpyp[1]/abs_proy_xpyp ,proy_xpyp[2]/abs_proy_xpyp ]
    anglhand_xp = math.acos(abs(xp[0]*proy_xpyp[0]+xp[1]*proy_xpyp[1]+xp[2]*proy_xpyp[2]))
    return xp, yp_r, zp_r, anglelbowxy, anglelbowyz, anglelbowxz, anglhand_xpyp, anglhand_xp
    
################################################################################################
################################################################################################

def Operations_rarm(rshoul,relbow,rhand):
    vect_rhandrelbow = [rhand[0]-relbow[0], rhand[1]-relbow[1], rhand[2]-relbow[2]]
    abs_vect_rhandrelbow = math.sqrt(vect_rhandrelbow[0]**2+vect_rhandrelbow[1]**2+vect_rhandrelbow[2]**2)
    vect_rhandrelbow = [vect_rhandrelbow[0]/abs_vect_rhandrelbow,vect_rhandrelbow[1]/abs_vect_rhandrelbow,vect_rhandrelbow[2]/abs_vect_rhandrelbow]
    abs_vect_relbow = math.sqrt((relbow[0]-rshoul[0])**2+(relbow[1]-rshoul[1])**2+((relbow[2]-rshoul[2])**2))
    angrelbowxy = -math.asin(abs((relbow[2]-rshoul[2]))/abs_vect_relbow)
    angrelbowyz = -math.asin(abs((relbow[0]-rshoul[0]))/abs_vect_relbow)
    angrelbowxz = math.asin(abs((relbow[1]-rshoul[1]))/abs_vect_relbow)
    if  (relbow[2] >= rshoul[2]):
        angrelbowxy = math.asin(abs((relbow[2]-rshoul[2]))/abs_vect_relbow)
  
    if  (relbow[0] >= rshoul[0]):
        angrelbowyz = math.asin(abs((relbow[0]-rshoul[0]))/abs_vect_relbow)
    		    
    if (relbow[1] <= rshoul[1]): 
        angrelbowxz = -math.asin(abs((relbow[1]-rshoul[1]))/abs_vect_relbow)
    xp = [(relbow[0]-rshoul[0])/abs_vect_relbow,(relbow[1]-rshoul[1])/abs_vect_relbow, (relbow[2]-rshoul[2])/abs_vect_relbow]    
    yp_r,zp_r = equations_rhand_x(xp,angrelbowxz)
    if vect_rhandrelbow[1] > yp_r[1]:
        angrhand_xpyp = -abs(zp_r[0]*vect_rhandrelbow[0]+zp_r[1]*vect_rhandrelbow[1]+zp_r[2]*vect_rhandrelbow[2])
    else:
	angrhand_xpyp = abs(zp_r[0]*vect_rhandrelbow[0]+zp_r[1]*vect_rhandrelbow[1]+zp_r[2]*vect_rhandrelbow[2])
    f1 = vect_rhandrelbow[1]*zp_r[2]-vect_rhandrelbow[2]*zp_r[1]
    f2 = vect_rhandrelbow[0]*zp_r[2]-vect_rhandrelbow[2]*zp_r[0]
    f3 = vect_rhandrelbow[0]*zp_r[1]-vect_rhandrelbow[1]*zp_r[0]
    proy_xpyp = [(-(zp_r[1]*f3+zp_r[2]*f2)),(zp_r[0]*f3-zp_r[2]*f1),(zp_r[0]*(zp_r[1]*f3+zp_r[2]*f2)-zp_r[1]*(zp_r[0]*f3-zp_r[2]*f1))/zp_r[2]]
    abs_proy_xpyp = math.sqrt(proy_xpyp[0]**2+proy_xpyp[1]**2+proy_xpyp[2]**2)
    proy_xpyp = [proy_xpyp[0]/abs_proy_xpyp,proy_xpyp[1]/abs_proy_xpyp ,proy_xpyp[2]/abs_proy_xpyp ]
    angrhand_xp = math.acos(abs(xp[0]*proy_xpyp[0]+xp[1]*proy_xpyp[1]+xp[2]*proy_xpyp[2]))
    return xp, yp_r, zp_r, angrelbowxy, angrelbowyz, angrelbowxz, angrhand_xpyp, angrhand_xp

################################################################################################
################################################################################################
def NAO(m, neck,lshoul,lelbow,lhand):
    if (m == 1):    
        names = list()
        times = list()
        keys = list()
		
	l_xp, l_yp_r, l_zp_r, anglelbowxy, anglelbowyz, anglelbowxz, anglhand_xpyp, anglhand_xp = Operations_larm(lshoul,lelbow,lhand)
	r_xp, r_yp_r, r_zp_r, angrelbowxy, angrelbowyz, angrelbowxz, angrhand_xpyp, angrhand_xp = Operations_rarm(rshoul,relbow,rhand)
        	 
        names.append("LElbowRoll")
        times.append([0.72])
        keys.append([-angrhand_xp])
	 
        names.append("LElbowYaw")
        times.append([0.72])
        keys.append([-angrhand_xpyp])
	
        names.append("LHand")
        times.append([0.72])
        keys.append([1])
	
	names.append("LShoulderPitch")
	times.append([0.72])
	keys.append([angrelbowxz])

	names.append("LShoulderRoll") 
	times.append([0.72])
	keys.append([0.84351922587*angrelbowyz])

	names.append("LWristYaw")
	times.append([0.72])
	keys.append([-1.82346])

	names.append("RShoulderPitch")
        times.append([0.72])
	keys.append([anglelbowxz])
	
        names.append("RElbowYaw")
	times.append([0.72])
        keys.append([anglhand_xpyp])
	
	names.append("RHand")
	times.append([0.72])
	keys.append([1])

	names.append("RElbowRoll")
	times.append([0.72])
	keys.append([anglhand_xp]) 

	names.append("RShoulderRoll")
	times.append([0.72])
	keys.append([0.84351922587*anglelbowyz])
	    
	names.append("RWristYaw")
	times.append([0.72])
	keys.append([1.82346])

	motion.angleInterpolation(names, keys, times, True) 

################################################################################################
################################################################################################
while True:
    # Update to next frame
    ctx.wait_and_update_all()
    
    m, fneck, fhead, frshoul, frelbow, frhand, flhand, flelbow, flshoul, ftorso, flhip, frhip, flknee, frknee, flfoot, frfoot = detect_skeleton()
    show_skeleton(m, fneck, fhead, frshoul, frelbow, frhand, flhand, flelbow, flshoul, ftorso, flhip, frhip, flknee, frknee, flfoot, frfoot)
    if n ==5:
    	NAO(m, fneck,flshoul,flelbow,flhand)
	n = 0
    n = n + 1
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
       	break


