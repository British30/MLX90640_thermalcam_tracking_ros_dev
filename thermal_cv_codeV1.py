import time,board,busio
import numpy as np
import adafruit_mlx90640
import matplotlib.pyplot as plt
import cv2

i2c = busio.I2C(board.SCL, board.SDA, frequency=1000000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ
mlx_shape = (24,32)
colormap = plt.get_cmap('plasma')
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
      ret, thresh = cv2.threshold(near_img, 50, 255, 0)
      contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      cv2.drawContours(near_img, contours, -1, (0,255,0), 3)
      t_array.append(time.monotonic()-t1)
      print('Sample rate: {0:2.1f}fps'.format(len(t_array)/np.sum(t_array)))
      cv2.imshow("THERMAL IMAGE", near_img)  
      cv2.waitKey(10)
 
      
     except ValueError:
        
        continue
