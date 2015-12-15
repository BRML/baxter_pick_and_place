import cv2
import matplotlib.pyplot as plt
import numpy as np


print cv2.__version__


def _segment_table(img, lower_hsv=np.array([38, 20, 125]),
                   upper_hsv=np.array([48, 41, 250]), offset=20,
                   verbose=False):
    """ Segment table in input image based on color information.
    :param img: the image to work on
    :param lower_hsv: lower bound of HSV values to segment
    :param upper_hsv: upper bound of HSV values to segment
    :param offset: offset for ROI
    :param verbose: show intermediate images or not
    :return: region of interest (x, y, w, h)
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    if verbose:
        cv2.imshow('Mask', mask)
        table = cv2.bitwise_and(img, img, mask=mask)
        cv2.imshow('Table', table)
        kernel = np.ones((5, 5), np.uint8)
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=9)
        cv2.imshow('Closing', closing)
        table2 = cv2.bitwise_and(img, img, mask=closing)
        cv2.imshow('Table 2', table2)

    points = cv2.findNonZero(mask)
    x, y, w, h = cv2.boundingRect(points)
    if verbose:
        image = img.copy()
        cv2.rectangle(image, (x, y), (x + w, y + h), np.array([0, 255, 0]), 2)
        cv2.rectangle(image, (x + offset, y + offset),
                      (x + w - offset, y + h - offset),
                      np.array([255, 0, 0]), 2)
        cv2.imshow('Rectangle', image)
    # mask = np.zeros(img.shape[:2], np.uint8)
    # mask[y + offset:y + h - offset, x + offset:x + w - offset] = 1
    # res = mask[:, :, np.newaxis]*img
    # res[mask == 0] = 255
    # return res
    x += offset
    y += offset
    w -= 2*offset
    h -= 2*offset
    return img[y:y + h, x:x + h], (x, y)


def get_neighbors((row, column), neighborhood=8):
    assert neighborhood == 4 or neighborhood == 8, 'Neighborhood needs to be 4 or 8!'
    neighbors = [(row - 1, column),
                 (row + 1, column),
                 (row, column - 1),
                 (row, column + 1)]
    if neighborhood == 8:
        neighbors.append((row - 1, column - 1))
        neighbors.append((row - 1, column + 1))
        neighbors.append((row + 1, column - 1))
        neighbors.append((row + 1, column + 1))
    return neighbors


def blob_detector(label_image):
    # Fill the image with blobs of connected pixels
    # 0  - background
    # 1  - unlabelled foreground
    # 2+ - labelled foreground
    label_image[label_image == 255] = 1
    label_count = 2

    blobs = list()

    for row in range(label_image.shape[0]):
        for column in range(label_image.shape[1]):
            if label_image[row, column] != 1:
                continue
            # check if one of the neighbors already has a label
            neighbors = get_neighbors((row, column), neighborhood=8)
            label = 1
            for n in neighbors:
                r, c = n
                try:
                    lbl = label_image[r, c]
                except IndexError:
                    print 'border pixel'
                    continue
                if lbl == 0:
                    continue
                if lbl == 1:
                    continue
                label = lbl
                break
            if label == 1:
                label = label_count
                label_count += 1
            label_image[row, column] = label
            print '%i %i: %i' % (row, column, label_image[row, column])
            print 'label count: %i' % label_count

    plt.imshow(label_image)
    plt.colorbar()
    plt.show()
    return blobs


image = cv2.imread('../images/001.jpg', cv2.IMREAD_COLOR)
# cv2.imshow('Image', image)

cropped, xy_cropped = _segment_table(image, verbose=False)
cv2.imshow('cropped', cropped)

# mask = np.zeros(cropped.shape[:2], np.uint8)
# bgdModel = np.zeros((1, 65), np.float64)
# fgdModel = np.zeros((1, 65), np.float64)
# rect = (20, 20, cropped.shape[0] - 20, cropped.shape[1] - 20)
# # this modifies mask
# cv2.grabCut(cropped, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
# # If mask==2 or mask== 1, mask2 get 0, other wise it gets 1 as 'uint8' type.
# mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
# # adding additional dimension for rgb to the mask, by default it gets 1
# # multiply it with input image to get the segmented image
# img_cut = cropped*mask2[:,:,np.newaxis]
# cv2.imshow('fg', img_cut)

# detector = cv2.SimpleBlobDetector()
# keypoints = detector.detect(cropped)
# im_with_keypoints = cv2.drawKeypoints(cropped, keypoints, color=(0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# cv2.imshow('keypoints', im_with_keypoints)

# gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
# kernel = np.ones((3, 3), np.uint8)
# _, th = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)
# cv2.imshow('threshold', th)
# opening = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)
# cv2.imshow('opening', opening)
# # black region shows sure background
# sure_bg = cv2.dilate(opening, kernel, iterations=1)
# cv2.imshow('sure bg', sure_bg)
# # white region shows sure foreground
# sure_fg = cv2.erode(opening, kernel, iterations=1)
# # dist_transform = cv2.distanceTransform(opening, cv2.DIST_LABEL_CCOMP, 5)
# # cv2.imshow('dist transform', dist_transform)
# # _, sure_fg = cv2.threshold(dist_transform, 0.7*dist_transform.max(), 255, 0)
# # sure_fg = np.uint8(sure_fg)
# cv2.imshow('sure fg', sure_fg)
# unknown = cv2.subtract(sure_bg, sure_fg)
# cv2.imshow('unknown', unknown)
#
# markers, ncc = label(sure_fg)
# markers *= 254./ncc
# markers += 1
# # plt.imshow(markers, cmap='jet')
# # plt.colorbar()
# # plt.show()
# # cv2.imshow('markers', markers)
# markers[unknown == 255] = 0
# # plt.imshow(markers, cmap='jet')
# # plt.colorbar()
# # plt.show()
# # cv2.imshow('markers 2', markers)
#
# watershed = cv2.watershed(cropped, markers)
# print watershed
# cv2.imshow('watershed', watershed)
# # plt.imshow(watershed, cmap='jet')
# # plt.colorbar()
# # plt.show()

# _, markers = cv2.connectedComponents(sure_fg)
# markers += 1
# markers[unknown == 255] = 0
# cv2.imshow('markers', markers)


shifted = cv2.pyrMeanShiftFiltering(cropped, 21, 21)
cv2.imshow('shifted', shifted)
gray = cv2.cvtColor(shifted, cv2.COLOR_BGR2GRAY)
# cv2.imshow('Gray Image', img)
# img = cv2.GaussianBlur(img, (9, 9), 0)
# _, th1 = cv2.threshold(gray, 115, 255, cv2.THRESH_BINARY)
th1 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
cv2.imshow('Threshold Image', th1)
kernel = np.ones((3, 3), np.uint8)
opening = cv2.morphologyEx(th1, cv2.MORPH_OPEN, kernel, iterations=1)
cv2.imshow('opening', opening)
kernel = np.ones((5, 5), np.uint8)
closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel, iterations=3)
cv2.imshow('closing', closing)

label_image = closing.copy()
blobs = blob_detector(label_image)
print blobs
label_image[label_image == 1] = 255
cv2.imshow('labelled', label_image)

# filled = gray.copy()

# for y in range(filled.shape[0]):
#     for x in range(filled.shape[1]):
#         if filled[y, x] != 1:
#             continue
#         rect = cv2.floodFill(filled, mask, (y, x), label_count)
#         blob = []
#         for i in range(rect[1], rect[1] + rect[3]):
#             for j in range(rect[0], rect[0] + rect[2]):
#                 if filled[i, j] != label_count:
#                     continue
#                 blob.append((i, j))
#         blobs.append(blob)
#         label_count += 1
# cv2.imshow('filled', filled)
# print blobs

# contours, _ = cv2.findContours(th1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
# cv2.drawContours(image, contours, 0, (255, 0, 0), 3)
# cv2.imshow('contours', image)
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
