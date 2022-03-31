import time,board,busio
import numpy as np
import adafruit_mlx90640
import matplotlib.pyplot as plt
import cv2
from math import sqrt
font = cv2.FONT_HERSHEY_SIMPLEX
i2c = busio.I2C(board.SCL, board.SDA, frequency=1000000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
mlx_shape = (24,32)
OldMax = 38
OldMin = 25
NewMax = 255
NewMin = 0
frame = np.zeros((24*32,))
t_array = []

while True:
     t1 = time.monotonic()
     try:
      mlx.getFrame(frame)
      OldRange = (OldMax - OldMin)  
      NewRange = (NewMax - NewMin)  
      Newframe = (((frame - OldMin) * NewRange) / OldRange) + NewMin
      framearr = np.reshape(Newframe, (24, 32))
      frameformat = framearr.astype(np.uint8)
      near_img = cv2.resize(frameformat,None, fx = 20, fy = 20, interpolation = cv2.INTER_CUBIC )
      ret, thresh = cv2.threshold(near_img, 100, 255, 0)
      contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      rgb = cv2.cvtColor(near_img, cv2.COLOR_GRAY2RGB)
      for cnt in contours:
       leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
       rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
       topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
       bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
       leftupcorner = (leftmost[0], topmost[1])
       rightdowncorner = (rightmost[0], bottommost[1])
       centercol = int((rightdowncorner[0]-leftupcorner[0])/2 + leftupcorner[0])
       centerrow = int((rightdowncorner[1]-leftupcorner[1])/2 + leftupcorner[1])
       hypo = sqrt((rightdowncorner[0]-leftupcorner[0])**2 + (rightdowncorner[1]-leftupcorner[1])**2)
       width = rightdowncorner[0]-leftupcorner[0]
       height = rightdowncorner[1]-leftupcorner[1]
       ratio = float(width/height)
       if hypo > 50 and hypo < 700 and ratio > 0.1 and ratio < 0.75:
        cv2.putText(rgb,str(int(hypo)),(centercol+10,centerrow-10), font, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(rgb,str(ratio),(centercol+10,centerrow-40), font, 1,(25,25,255),2,cv2.LINE_AA)
        cv2.circle(rgb,(centercol,centerrow), 10, (255,255,255), -1)
        cv2.circle(rgb,(leftupcorner), 10, (0,255,255), -1)
        cv2.circle(rgb,(rightdowncorner), 10, (255,0,255), -1)
        cv2.circle(rgb,(leftmost), 10, (0,0,255), -1)
        cv2.circle(rgb,(rightmost), 10, (0,0,255), -1)
        cv2.circle(rgb,(topmost), 10, (0,0,255), -1)
        cv2.circle(rgb,(bottommost), 10, (0,0,255), -1)
       else:
           pass
      t_array.append(time.monotonic()-t1)
      cv2.putText(rgb,str(int(len(t_array)/np.sum(t_array))),(20,30), font, 1,(255,255,255),2,cv2.LINE_AA)
      cv2.drawContours(rgb, contours, -1, (0,255,0), 2)
      #print('Sample rate: {0:2.1f}fps'.format(len(t_array)/np.sum(t_array)))
      cv2.imshow("THERMAL IMAGE", thresh)  
      cv2.waitKey(10)
 
      
     except ValueError:
        
        continue
