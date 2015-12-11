import cv2
from matplotlib import pyplot as plt
import numpy as np


image = cv2.imread('../images/000.jpg')
# cv2.imshow('Image', image)
# cv2.waitKey(0)

img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# cv2.imshow('Gray Image', img)
# cv2.waitKey(0)
# img = cv2.GaussianBlur(img, (9, 9), 0)
_, th1 = cv2.threshold(img, 115, 255, cv2.THRESH_BINARY)
# cv2.imshow('Threshold Image', th1)
# cv2.waitKey(0)
edges = cv2.Laplacian(th1, cv2.CV_8UC1)
# cv2.imshow('Edge Image', edges)
# cv2.waitKey(0)
lines = cv2.HoughLines(edges, 1, np.pi/180., 70)
for i in range(0, len(lines[0])):
    rho = lines[0][i][0]
    theta = lines[0][i][1]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(round(x0 + 1000*(-b)))
    y1 = int(round(y0 + 1000*a))
    x2 = int(round(x0 - 1000*(-b)))
    y2 = int(round(y0 - 1000*a))
    cv2.line(edges, (x1, y1), (x2, y2), (0, 0, 255), 2)
cv2.imshow('lines', edges)
cv2.waitKey(0)

# contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# contours = sorted(contours, key=cv2.contourArea, reverse=True)
# cv2.drawContours(edges, contours, 0, (0, 255, 0))
# cv2.imshow('Contour Image', edges)
# cv2.waitKey(0)
# table_cnt = None
# for contour in contours:
#     peri = cv2.arcLength(contour, True)
#     approx = cv2.approxPolyDP(contour, 0.02*peri, True)
#     if len(approx) == 4:  # looking for a rectangle
#         table_cnt = approx
#         cv2.drawContours(img, [table_cnt], -1, (0, 255, 0), 3)
#         cv2.imshow('Contour Image', img)
#         cv2.waitKey(0)
#         break

#
# plt.subplot(131)
# plt.imshow(img, cmap='gray')
# plt.title('Original Image'), plt.xticks([]), plt.yticks([])
# plt.subplot(132)
# plt.imshow(th1, cmap='gray')
# plt.title('Thresholded Image'), plt.xticks([]), plt.yticks([])
# plt.subplot(133)
# plt.imshow(edges, cmap='gray')
# plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
# plt.show()
