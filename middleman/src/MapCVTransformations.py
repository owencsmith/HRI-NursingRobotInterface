import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread("HospitalMap.png")
kernel11 = np.ones((11,11), np.uint8)
kernel3 = np.ones((3,3), np.uint8)
# image_center = tuple(np.array(img.shape[1::-1]) / 2)
# rot_mat = cv2.getRotationMatrix2D(image_center, .5, 1.0)
# img = cv2.warpAffine(img, rot_mat, img.shape[1::-1], flags=cv2.INTER_LINEAR)

# gradient = cv2.morphologyEx(img, cv2.MORPH_GRADIENT, kernel11)
# de1 = cv2.blur(img, (3,3))
de1 = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel3)
de1 = cv2.erode(de1, kernel3, iterations=1)
de1 = cv2.blur(de1, (5,5))
de1 = cv2.bilateralFilter(de1,9,75,75)
ret, de1 = cv2.threshold(de1, 127, 255, cv2.THRESH_BINARY)
de1 = cv2.bilateralFilter(de1,9,75,75)
de1 = cv2.blur(de1, (3,3))
ret, de1 = cv2.threshold(de1, 127, 255, cv2.THRESH_BINARY)
de1 = cv2.blur(de1, (3,3))
de1 = cv2.erode(de1, kernel3, iterations=1)
de1 = cv2.dilate(de1, kernel3, iterations=1)
ret, de1 = cv2.threshold(de1, 127, 255, cv2.THRESH_BINARY)
de1 = cv2.dilate(de1, kernel3, iterations=1)

# plt.imsave('HospitalMapCleaned2.png', np.array(de1), cmap='gray')

# cv2.imshow("open1", d_e_1)
# cv2.imshow("open2", d_e_2)
# cv2.imshow("gradient", gradient)
cv2.imshow("de1", de1)
cv2.waitKey(5000)



