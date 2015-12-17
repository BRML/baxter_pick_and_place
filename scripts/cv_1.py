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
                    neighbors = get_neighbors((r, c), neighborhood=8)
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


image = cv2.imread('../images/001.jpg', cv2.IMREAD_COLOR)
# cv2.imshow('Image', image)

cropped, xy_cropped = _segment_table(image, verbose=False)
cv2.imshow('cropped', cropped)

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
cv2.imshow('labelled', label_image)
rectangled = cropped.copy()
for i in range(len(blobs)):
    mask = np.zeros(label_image.shape[:2], np.uint8)
    mask[label_image == i + 2] = 1
    points = cv2.findNonZero(mask)
    x, y, w, h = cv2.boundingRect(points)
    cv2.rectangle(rectangled, (x, y), (x + w, y + h), np.array([0, 255, 0]), 2)
cv2.imshow('rectangles', rectangled)
# TODO: filter out candidates that obviously are to small
# TODO: compute center of candidates
# TODO: compute Baxter coordinates of center of candidates

cv2.waitKey(0)
cv2.destroyAllWindows()
