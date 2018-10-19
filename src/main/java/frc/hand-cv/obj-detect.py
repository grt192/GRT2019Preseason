import cv2
import numpy as np
import math

center_x = 0
center_y = 0
old_x = 0
old_y = 0

cap = cv2.VideoCapture(0)
while(cap.isOpened()):

    ret, img = cap.read()
    cv2.rectangle(img, (400,400), (200,200), (0,255,0),0)
    crop_img = img[200:400, 200:400]
    grey = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    value = (35, 35)
    blurred = cv2.GaussianBlur(grey, value, 0)

    _, thresh1 = cv2.threshold(blurred, 127, 255,
                               cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    cv2.imshow('Thresholded', thresh1)

    image, contours, hierarchy = cv2.findContours(thresh1.copy(), \
           cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cnt = max(contours, key = lambda x: cv2.contourArea(x))
    x, y, w, h = cv2.boundingRect(cnt)
    cv2.rectangle(crop_img, (x, y), (x+w, y+h), (0, 0, 255), 0)
    M = cv2.moments(cnt)

    center_x = int(M['m10']/M['m00'])
    center_y = int(M['m01']/M['m00'])

    diff_x = old_x - center_x
    diff_y = old_y - center_y

    if mag < 0.2:
        mag = 0
        horiz = "none"
        vert = "none"
    else:
        # opposite because of camera
        horiz = "right" if (diff_x > 0) else "left"
        vert = "up" if (diff_y > 0) else "down"
        mag = round(((diff_x ** 2) + (diff_y ** 2) ** (0.5)), 2)

    if diff_x == 0 or diff_y == 0:
        angle = 0
    else:
        angle = np.arctan(diff_y / diff_x)

    angle = np.arctan(diff_y / diff_x)
    cv2.circle(crop_img, (center_x, center_y), 5, (255, 255, 255), -1)

    hull = cv2.convexHull(cnt)
    drawing = np.zeros(crop_img.shape,np.uint8)
    cv2.drawContours(drawing, [cnt], 0, (0, 255, 0), 0)
    cv2.drawContours(drawing, [hull], 0,(0, 0, 255), 0)
    hull = cv2.convexHull(cnt, returnPoints=False)

    defects = cv2.convexityDefects(cnt, hull)
    count_defects = 0
    cv2.drawContours(thresh1, contours, -1, (0, 255, 0), 3)

    cv2.imshow('Hand Gestures', img)
    all_img = np.hstack((drawing, crop_img))
    cv2.imshow('Contours', all_img)

    old_x = center_x
    old_y = center_y

    cv2.waitKey(5)
