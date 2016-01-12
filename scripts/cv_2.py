import cv2
import matplotlib.pyplot as plt
import numpy as np


print cv2.__version__


def _get_features(image, descriptor='brief'):
    # if descriptor == 'sift':
    #     sift = cv2.SIFT()
    #     kp, des = sift.detectAndCompute(image, None)
    # elif descriptor == 'surf':
    #     surf = cv2.SURF(400)
    #     kp, des = surf.detectAndCompute(image, None)
    if descriptor == 'brief':
        star = cv2.FeatureDetector_create("STAR")
        kp = star.detect(image, None)
        brief = cv2.DescriptorExtractor_create("BRIEF")
        kp, des = brief.compute(image, kp)
    elif descriptor == 'orb':
        orb = cv2.ORB()
        kp, des = orb.detectAndCompute(image, None)
    else:
        raise AttributeError("Descriptor '{}' not implemented!".format(descriptor))
    return kp, des


def _draw_matches(img1, kp1, img2, kp2, matches):
    """ Visualize keypoint matches between images. Stolen from
    http://stackoverflow.com/questions/11114349/how-to-visualize-descriptor-matching-using-opencv-module-in-python
    :param img1:
    :param kp1:
    :param img2:
    :param kp2:
    :param matches:
    :return:
    """
    h1, w1 = img1.shape[:2]
    d1 = 1 if len(img1.shape) == 2 else img1.shape[2]
    h2, w2 = img2.shape[:2]
    d2 = 1 if len(img2.shape) == 2 else img2.shape[2]
    view = np.zeros((max(h1, h2), w1 + w2, max(d1, d2)), dtype=np.uint8)
    view[:h1, :w1, :] = img1 if view[:h1, :w1, :].shape == img1.shape else img1[:, :, np.newaxis]
    view[:h2, w1:, :] = img2 if view[:h2, w1:, :].shape == img2.shape else img2[:, :, np.newaxis]
    for m in matches:
        color = tuple([np.random.randint(0, 255) for _ in xrange(3)])
        cv2.line(view,
                 (int(kp1[m.queryIdx].pt[0]), int(kp1[m.queryIdx].pt[1])),
                 (int(kp2[m.trainIdx].pt[0] + w1), int(kp2[m.trainIdx].pt[1])),
                 color=color, thickness=1)
    return view


img_scene = cv2.imread('../images/003.jpg', cv2.IMREAD_COLOR)
img_scene = cv2.resize(img_scene, (640, 480))
gray_scene = cv2.cvtColor(img_scene, cv2.COLOR_BGR2GRAY)
# cv2.imshow('Scene', img_scene)
img_baxter = cv2.imread('../images/007.jpg', cv2.IMREAD_COLOR)
img_baxter = cv2.resize(img_baxter, (640, 480))
gray_baxter = cv2.cvtColor(img_baxter, cv2.COLOR_BGR2GRAY)
# cv2.imshow('Baxter', img_baxter)

descriptor = 'brief'
kp_scene, des_scene = _get_features(gray_scene, descriptor)
kp_baxter, des_baxter = _get_features(gray_baxter, descriptor)

# img = cv2.drawKeypoints(img_scene, kp_scene, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# cv2.imshow('Scene', img)
# img = cv2.drawKeypoints(img_baxter, kp_baxter, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# cv2.imshow('Baxter', img)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des_scene, des_baxter)
matches = sorted(matches, key=lambda x: x.distance)
print len(matches)
img = _draw_matches(img_scene, kp_scene, img_baxter, kp_baxter, matches)
cv2.imshow('Matches', img)
cv2.imwrite('../images/matches003007.jpg', img)

cv2.waitKey(0)
cv2.destroyAllWindows()
