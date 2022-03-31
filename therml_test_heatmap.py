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
NewMax = 1
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
      heatmap = (colormap(framearr) * 2**16).astype(np.uint16)[:,:,:3]
      heatmap = cv2.cvtColor(heatmap, cv2.COLOR_RGB2BGR)
      near_img = cv2.resize(heatmap,None, fx = 30, fy = 30, interpolation = cv2.INTER_CUBIC )
      t_array.append(time.monotonic()-t1)
      print('Sample rate: {0:2.1f}fps'.format(len(t_array)/np.sum(t_array)))
      cv2.imshow("THERMAL IMAGE", near_img)  
      cv2.waitKey(10)
     except ValueError:
        continue   
