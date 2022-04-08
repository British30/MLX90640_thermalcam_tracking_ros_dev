##### Code developed by Rui Brites M00725721, for final project of Robotics Engineering, Middlesex University London.

##### The code goal is to track human positions using MLX90640 sensor, and Opencv to compute the image.

#### This is the code that will be developed to implement in the ROS comunnity, by developing a package to do human tracking.

####In the same folder you can find example codes to test the sensor and get a visual output as a older version of this code that extracts contorns.

####ATENCION!!!	Must activate 12c bus and change 12C max bus speed to 1000000.
###In rasperry edit config.txt file and put "dtparam_i2c_arm=on,i2c_arm_baudrate=10000000"
#######################################################################################################



######### SETUP ########
import rospy
from geometry_msgs.msg import Twist
import roslib

import time,board,busio  #Import libraries for timing, and connections
import numpy as np       #Import Numpy for data manipulation
import adafruit_mlx90640 #Import sensor library
import cv2 
import math              #Import Opencv for computer vision


font = cv2.FONT_HERSHEY_SIMPLEX                                 #Define font to write in image
i2c = busio.I2C(board.SCL, board.SDA, frequency=1000000)        #Initialize 12c bus and set baud rate
mlx = adafruit_mlx90640.MLX90640(i2c)                           #Initialize sensor
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ  #Define sensor refresh rate
mlx_shape = (24,32)                                             #Define sensor raw output matrix size
OldMax = 40                                                     #Define max temperature value captured by sensor
OldMin = 15                                                     #Define min temperature value captured by sensor
NewMax = 255                                                    #Define grayscale pixel value range
NewMin = 0                                                      #Define grayscale pixel value range
frame = np.zeros((24*32,))                                      #Initialize matrix with correct image format
t_array = []                                                    #Initialize timing list to check frame rate
ratio_lst =  []                                                 #Initialize aspect ratio list to implement rolling average in the contorn detection and selection                          
l_speed = 0.1                                                   #Define linear speed 0.1 m/s
a_speed = 0.01                                                  #Define angular speed 0.01 rad/s
rospy.init_node('move')                                         #Initialize node in ros
p = rospy.Publisher('cmd_vel', Twist)                           #Define publisher and mesage time
def twistpub(l,a):                                              #Build wist message function to move linear and angular
 twist = Twist()
 twist.linear.x = l;                   # our forward speed
 twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
 twist.angular.x = 0; twist.angular.y = 0;   #          or these!
 twist.angular.z = a;
 return twist                        # rotation
img_center_col = 320                                            #Center column of interpolated image
def col_deviation_to_degree(hd):                                #Convert deviation of the human shape center to the image center to degrees
   hd_degree = 110*hd/640                                       #knowing that the camera field of view is 110 degrees
   return hd_degree

######### MAIN LOOP ########

while True:                                                     #Main loop that runs every frame.
     t1 = time.monotonic()                                      #Get time passed since last frame.
     try:
      mlx.getFrame(frame)                                       #Get frame from sensor.
      OldRange = (OldMax - OldMin)                              #Method to convert temperature values to gray scale 255 range.
      NewRange = (NewMax - NewMin)  
      Newframe = (((frame - OldMin) * NewRange) / OldRange) + NewMin 
      framearr = np.reshape(Newframe, (24, 32))                 #Reshape list to matrix
      frameformat = framearr.astype(np.uint8)                   #Convert array to 8 bit format accepted by opencv
      near_img = cv2.resize(frameformat,None, fx = 20, fy = 20, interpolation = cv2.INTER_CUBIC ) #Increase image size by 20 times using cubic interpolation.
      ret, thresh = cv2.threshold(near_img, 115, 255, 0)        #Treshold image to get the regions that are more hot.
      contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Extract external contourns of hot regions.
      rgb = cv2.cvtColor(near_img, cv2.COLOR_GRAY2RGB)          #Convert grayscale image to rgb to draw points and data using colors.
      for cnt in contours:                                      #Loop trought all contourns detected.
       leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])            #Find the left bounding point
       rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])           #Find the right bounding point.
       topmost = tuple(cnt[cnt[:,:,1].argmin()][0])             #Find the top bounding point.
       bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])          #Find the bottom bounding point.
       leftupcorner = (leftmost[0], topmost[1])                 #Find the left up corner of the bounding rectangle.
       rightdowncorner = (rightmost[0], bottommost[1])          #Find the right down corner of the bounding rectangle.
       centercol = int((rightdowncorner[0]-leftupcorner[0])/2 + leftupcorner[0]) #Find the column of the contourn center point.
       centerrow = int((rightdowncorner[1]-leftupcorner[1])/2 + leftupcorner[1]) #Find the row of the contourn center point.
       width = rightdowncorner[0]-leftupcorner[0]               #Find the width of the bounding box.
       height = rightdowncorner[1]-leftupcorner[1]              #Find the height of the bounding box.
       ratio = float(width/height)                              #Calculate aspect ratio of the contorn bounding box.
       area = int(width*height)                                 #Calculate the area of the bounding box.
       
       if ratio > 0.2 and ratio < 0.9 and area > 6000:          #Condition to determine if contorn can be a human shape using aspect ratio and area.
         cnt_track = []                                         #Initialize empty list to store the contorns of possible human shapes
         center_track=()                                        #Initialize empty tuple to store the center point of a possible human detection
         area_track = int()                                     #Create variable to store area of the possible human bounding box
         leftupcorner_track = (leftupcorner[0],leftupcorner[1]) #Store the left up corner of possible human bounding box in a new tuple.
         rightdowncorner_track = (rightdowncorner[0],rightdowncorner[1]) #Store the right down corner of possible human bounding box in a new tuple.
         cnt_track.append(cnt)                                  #Append possible human contorn list to new list initialized previously.
         center_track = (centercol, centerrow)                  #Store center of possible human bounding box as new tuple.
         area_track = area                                      #Store possible human bounding box area as new variable.
         horizontal_deviation =img_center_col-center_track[0]   #Calculate column deviation from image center
         horizontal_deviation_deg = col_deviation_to_degree(horizontal_deviation*-1) #Convert to degrees


         ratio_lst.insert(0, ratio)                             #Insert possible human bounding box aspect ratio in the first index of the rolling average list.
         if len(ratio_lst) > 200:                               #Limit rolling average list aize and keep deleting last index.
          ratio_lst.pop()
         
        
      if np.average(ratio_lst) > 0.3 and np.average(ratio_lst) < 0.8:  #condition to check if rolling average value is within human size aspect ratio
           cv2.putText(rgb,str(area_track),(center_track[0]+10, center_track[1]+10), font, 0.5,(25,25,255),2,cv2.LINE_AA)#Put area text in the center of bounding box
           cv2.circle(rgb,(center_track), 10, (255,255,255), -1)       #Draw circle in bounding box center.
           cv2.circle(rgb,(leftupcorner_track), 10, (0,255,255), -1)   #Draw circle in left up corner of bounding box.
           cv2.circle(rgb,(rightdowncorner_track), 10, (255,0,255), -1) #Draw circle in right down corner of bounding box.
           cv2.drawContours(rgb, cnt_track, -1, (0,255,0), 2)           #draw contorns
           relative_deviation_rad = math.radians(horizontal_deviation_deg) #Conver angle deviations in degrees to radians
           t0 = rospy.Time.now().to_sec()                                  #Get current time
           current_angle = 0                                               #Initialize current angle a 0, angle to anchieve
           while(current_angle < relative_deviation_rad):                  #Move until deviation is 0
             p.publish(twistpub(0, a_speed))                               #publish twist message
             t1 = rospy.Time.now().to_sec()                                #Get current time
             current_angle = a_speed*(t1-t0)                               #Get current anlge acording to time passedand speed
           p.publish(twistpub(0, 0))                                       #Stop moving when deviation is 0


      t_array.append(time.monotonic()-t1)                               #Append time passed of each frame to list
      cv2.putText(rgb,str(int(len(t_array)/np.sum(t_array))),(20,30), font, 1,(255,255,255),2,cv2.LINE_AA) #Draw average of the frame rate in top left corner
      cv2.imshow("THERMAL IMAGE", rgb)                                  #Show final image with data
      cv2.waitKey(1)                                                    #Waitkey needed to plot image..
 
      
     except ValueError:                                                #
        
        continue
