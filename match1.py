import numpy as np
import cv2
from matplotlib import pyplot as plt
matchesMask = list(range(4))
matches = list(range(4))
results = list(range(4))

queryImage = cv2.imread('./data/test4.jpg',0)
trainingImage1 = cv2.imread('./data/1.png',0)
trainingImage2 = cv2.imread('./data/2.png',0)
trainingImage3 = cv2.imread('./data/3.png',0)
trainingImage4 = cv2.imread('./data/4.png',0)

# 只使用SIFT 或 SURF 检测角点
sift = cv2.xfeatures2d.SIFT_create()
# sift = cv2.xfeatures2d.SURF_create(float(4000))
kp, des = sift.detectAndCompute(queryImage,None)
kp1, des1 = sift.detectAndCompute(trainingImage1,None)
kp2, des2 = sift.detectAndCompute(trainingImage2,None)
kp3, des3 = sift.detectAndCompute(trainingImage3,None)
kp4, des4 = sift.dete   ctAndCompute(trainingImage4,None)


# 设置FLANN匹配器参数
# algorithm设置可参考https://docs.opencv.org/3.1.0/dc/d8c/namespacecvflann.html
indexParams = dict(algorithm=0, trees=5)
searchParams = dict(checks=50)
# 定义FLANN匹配器
flann = cv2.FlannBasedMatcher(indexParams,searchParams)
# 使用 KNN 算法实现匹配
matches[0] = flann.knnMatch(des,des1,k=2)
matches[1] = flann.knnMatch(des,des2,k=2)
matches[2] = flann.knnMatch(des,des3,k=2)
matches[3] = flann.knnMatch(des,des4,k=2)
# 根据matches生成相同长度的matchesMask列表，列表元素为[0,0]
for i in range(0,4):
    matchesMask[i] = [[0,0] for i in range(len(matches[i]))]

# 去除错误匹配
for i in range(0,4):
    for j,(m,n) in enumerate(matches[i]):
        if m.distance < 0.7*n.distance:
            matchesMask[i][j] = [1,0]
    results[i] = matchesMask[i].count([1, 0])
print(results.index(max(results)))





