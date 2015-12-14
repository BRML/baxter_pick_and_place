import cv2
import numpy as np


image = cv2.imread('../images/001.jpg', cv2.IMREAD_COLOR)
# cv2.imshow('Image', image)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# cv2.imshow('hsv', hsv)
lower_white = np.array([38, 20, 125])
upper_white = np.array([48, 41, 250])
mask = cv2.inRange(hsv, lower_white, upper_white)
# res = cv2.bitwise_and(image, image, mask=mask)
cv2.imshow('mask', mask)
# cv2.imshow('white', res)
# kernel = np.ones((5, 5), np.uint8)
# closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=10)
# cv2.imshow('dilation', closing)
# resc = cv2.bitwise_and(image, image, mask=closing)
# cv2.imshow('white2', resc)

points = cv2.findNonZero(mask)
x, y, w, h = cv2.boundingRect(points)
img = image.copy()
cv2.rectangle(img, (x, y), (x + w, y + h), np.array([0, 255, 0]), 2)
cv2.imshow('rectangle', img)

cropped = image[y:y + h, x:x + w]
cv2.imshow('cropped', cropped)
print cropped.shape

# img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# # cv2.imshow('Gray Image', img)
# # img = cv2.GaussianBlur(img, (9, 9), 0)
# _, th1 = cv2.threshold(img, 115, 255, cv2.THRESH_BINARY)
# # cv2.imshow('Threshold Image', th1)
# _, contours, _ = cv2.findContours(th1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
# print len(contours)
# for contour in contours:
#     cv2.drawContours(th1, [contour], 0, (255, 0, 0), 3)
#     # if 5000 < cv2.contourArea(contour) < 10000:
#     #     (x, y, w, h) = cv2.boundingRect(contour)
#     #     cv2.rectangle(th1, (x, y), (x + w, y + h), 0, -1)
# cv2.imshow('contours', th1)
# edges = cv2.Laplacian(th1, cv2.CV_8UC1)
# cv2.imshow('Edge Image', edges)
# lines = cv2.HoughLines(edges, 1, np.pi/180., 70)
# print len(lines)
# for i in range(0, len(lines)):
#     rho = lines[i][0][0]
#     theta = lines[i][0][1]
#     a = np.cos(theta)
#     b = np.sin(theta)
#     x0 = a*rho
#     y0 = b*rho
#     x1 = int(round(x0 + 1000*(-b)))
#     y1 = int(round(y0 + 1000*a))
#     x2 = int(round(x0 - 1000*(-b)))
#     y2 = int(round(y0 - 1000*a))
#     # print np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
#     cv2.line(edges, (x1, y1), (x2, y2), (0, 0, 255), 2)
# cv2.imshow('lines', edges)

# contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# contours = sorted(contours, key=cv2.contourArea, reverse=True)
# cv2.drawContours(edges, contours, 0, (0, 255, 0))
# cv2.imshow('Contour Image', edges)
# table_cnt = None
# for contour in contours:
#     peri = cv2.arcLength(contour, True)
#     approx = cv2.approxPolyDP(contour, 0.02*peri, True)
#     if len(approx) == 4:  # looking for a rectangle
#         table_cnt = approx
#         cv2.drawContours(img, [table_cnt], -1, (0, 255, 0), 3)
#         cv2.imshow('Contour Image', img)
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
cv2.waitKey(0)
cv2.destroyAllWindows()
