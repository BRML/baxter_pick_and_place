import cv2
import numpy as np


def flood_fill(canny):
    h, w = canny.shape
    white = 255
    black = 0

    # set border pixels to white
    canny[0, :] = white
    canny[-1, :] = white
    canny[:, 0] = white
    canny[:, -1] = white

    # prime to-do list
    to_do = [(2, 2), (2, h - 3), (w - 3, h - 3), (w - 3, 2)]

    # test pixels
    while len(to_do) > 0:
        x, y = to_do.pop()
        if canny[y, x] == black:
            canny[y, x] = white
            to_do.append((x, y - 1))
            to_do.append((x, y + 1))
            to_do.append((x - 1, y))
            to_do.append((x + 1, y))

    return canny


def get_neighbors((row, column), neighborhood=8, imsize=(800, 1280)):
    """ Return a list of neighboring pixels to an input pixel for a given
    neighborhood relation (four- or eight-neighborhood).
    :param row: row index to compute neighbors for
    :param column: column index to compute neighbors for
    :param neighborhood: four- or eight-neighborhood to consider
    :return: indices of neighboring pixels
    """
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

    # neighbors = list()
    # if row > 0:
    #     neighbors.append((row - 1, column))
    # if row < imsize[0] - 1:
    #     neighbors.append((row + 1, column))
    # if column > 0:
    #     neighbors.append((row, column - 1))
    # if column < imsize[1] - 1:
    #     neighbors.append((row, column + 1))
    #
    # if neighborhood == 8:
    #     if row > 0:
    #         if column > 0:
    #             neighbors.append((row - 1, column - 1))
    #         if column < imsize[1] - 1:
    #             neighbors.append((row - 1, column + 1))
    #     if row < imsize[0] - 1:
    #         if column > 0:
    #             neighbors.append((row + 1, column - 1))
    #         if column < imsize[1] - 1:
    #             neighbors.append((row + 1, column + 1))
    return neighbors


def blob_detector(label_image):
    """ Detect and return connected components in an image.
    :param label_image: (binary) image to label, 0 indicating background, 1
    indicating unlabelled foreground; will be modified, then 2+ indicating
    labelled foreground
    :return: list of connected components (lists of pixels)
    """
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
                # pixel is background or already labelled
                continue
            # pixel is unlabelled foreground
            blob = [(row, column)]
            new_neighbors = True
            # find all neighbors that are unlabelled and add them to blob
            while new_neighbors:
                new_neighbors = False
                for r, c in blob:
                    neighbors = get_neighbors((r, c), neighborhood=4)
                    for y, x in neighbors:
                        if (y, x) not in blob:
                            if label_image[y, x] == 1:
                                blob.append((y, x))
                                new_neighbors = True
            for r, c in blob:
                try:
                    label_image[r, c] = label_count
                except IndexError:
                    print 'border pixel'
            label_count += 1
            blobs.append(blob)
    # print 'label count: %i' % label_count
    return blobs


# load image as gray scale
image = cv2.imread('../data/top_view.jpg', cv2.IMREAD_GRAYSCALE)
# cv2.imshow('Image', image)

# do Canny edge extraction and widen edges
# c_low = 50
# c_high = 270
# canny = cv2.Canny(image, c_low, c_high, apertureSize=3)
# kernel = np.ones((3, 3), np.uint8)
# canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=1)
# cv2.imshow('Canny', canny)
#
# # flood fill image to extract bounded regions
# flooded = flood_fill(canny)
# cv2.imshow('Flooded', flooded)
# cv2.imwrite('../data/flooded.jpg', flooded)

flooded = cv2.imread('../data/flooded.jpg', cv2.IMREAD_GRAYSCALE)
contours, _ = cv2.findContours(flooded, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
for c in contours:
    if 10000 < cv2.contourArea(c) < 100000:
        print cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(rect)
        print box
        box = np.int0(box)
        cv2.drawContours(img, [box], 0, (0, 255, 0), 2)
        cv2.circle(img, (int(rect[0][0]), int(rect[0][1])), 3, (0, 0, 255), 3)
cv2.imshow('Contours', img)
# do blob detection
# label_image = 255-flooded
# blobs = blob_detector(label_image)
# cv2.imshow('labelled', label_image)
# cv2.imwrite('../data/labelled.jpg', label_image)
# np.savez('../data/blobs.npz', blobs=blobs)
#
# label_image = cv2.imread('../data/labelled.jpg', cv2.IMREAD_GRAYSCALE)
#
# rectangled = image.copy()
# rectangled = cv2.cvtColor(rectangled, cv2.COLOR_GRAY2BGR)
# centers = list()
# print 'blob\tarea\tcenter'
# for i in range(len(blobs)):
#     mask = np.zeros(label_image.shape[:2], np.uint8)
#     mask[label_image == i + 2] = 1
#     points = cv2.findNonZero(mask)
#     try:
#         x, y, w, h = cv2.boundingRect(points)
#         centers.append((x + w/2, y + h/2))
#         print '%i\t\t%i\t\t(%i, %i)' % (i, len(points), centers[i][0], centers[i][1])
#         if len(points) > 20000:
#             cv2.rectangle(rectangled, (x, y), (x + w, y + h), np.array([0, 255, 0]), 2)
#             cv2.circle(rectangled, centers[i], 3, np.array([0, 255, 0]), 2)
#             cv2.line(rectangled, (x, y), (x + w, y + h), np.array([0, 255, 0]))
#             cv2.line(rectangled, (x + w, y), (x, y + h), np.array([0, 255, 0]))
#         elif len(points) > 100:
#             cv2.rectangle(rectangled, (x, y), (x + w, y + h), np.array([255, 0, 0]), 2)
#         else:
#             cv2.rectangle(rectangled, (x, y), (x + w, y + h), np.array([0, 0, 255]), 2)
#     except cv2.error:
#         pass
# cv2.imshow('rectangles', rectangled)

cv2.waitKey(0)
cv2.destroyAllWindows()
