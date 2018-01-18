# -*- coding: utf-8 -*-
from __future__ import division
from collections import deque
from matplotlib import pyplot as plt
import numpy as np
import cv2
import time

blueLower=np.array([100,40,40])
blueUpper=np.array([140,255,255])

FLASH_MIN_PTS = 50
FLASH_MAX_PTS = 200

timeStamp = time.time()
sm = 0
frameTime = 0
sign = 0

frameQ = deque(maxlen = 8)
cap = cv2.VideoCapture(1) #连接到硬件摄像头

def flash(diffFrame):
    global sm, ts, sign
    print (sm)
    if sm == 0 and cv2.countNonZero(diffFrame) > FLASH_MAX_PTS:
        sm = 1
    elif sm == 1 and cv2.countNonZero(diffFrame) < FLASH_MIN_PTS:
        sm = 2
    elif sm == 2 and cv2.countNonZero(diffFrame) > FLASH_MAX_PTS:
        sm = 0
        sign += 1

while True:
    ret, frame = cap.read() # 捕获图像
    frameQ.append(frame)
    while len(frameQ) != 8:
        frameQ.append(frame)
    diff = cv2.absdiff(frameQ[-1], frameQ[-2])
    cv2.imshow("Diff", diff)
    hsv = cv2.cvtColor(diff, cv2.COLOR_BGR2HSV) # 转换颜色空间
    blue = cv2.inRange(hsv, blueLower, blueUpper) # 根据阈值构建掩膜
    blurred = cv2.GaussianBlur(blue, (1, 1), 0) # 高斯模糊
    flash(blurred)
    #print(sign)
    '''
    sum = 0
    for i in range(blurred):
        sum += i
        print(timt.time(), sum)
    '''
    cnts=cv2.findContours(blurred.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2] # 寻找轮廓
    center = None
    if (len(cnts)>0):
        try:
            c=max(cnts,key=cv2.contourArea) # 取面积最大的轮廓
            M=cv2.moments(c) # 计算轮廓的矩
            center=(int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"])) # 计算质心坐标
            cx=center[0]
            cy=center[1]
            cv2.circle(frame,(cx,cy),3,(255,0,0),2)
            timeStamp = time.time()
        except:
            pass
    
    circles=cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT,1,100,param1=100,param2=35,minRadius=5,maxRadius=200)
    if circles is not None: # 至少有一个圆找到了
        findMethod=2
        circles=np.round(circles[0,:]).astype("int") # 转换所有坐标和半径为整形
        for (cx,cy,rad) in circles: # 遍历所有的圆
            cv2.circle(frame,(cx,cy),rad,(0,255,0),2)
    
    cv2.imshow("Capture", frame)
    cv2.imshow("Blue", blue)
    cv2.imshow("Blurred", blurred)
    '''
    if (len(frameQ) == 8):
        for i in range(8):
            plt.subplot(4, 2, i + 1)
            plt.imshow(frameQ[i])
        plt.show()
    '''

    key = cv2.waitKey(1) & 0xFF # 等待按键
    if (key == 27): # 按下[Esc]键退出
        break

cap.release() # 释放摄像头
cv2.destroyAllWindows() # 关闭开启的一切窗口
