#
# test_find_points - test cv.MatchTemplate using real images collected from the stage
#
# TVI
# 02 / 2025
#
#this method will likely work best with template images the same size as the points, thus the template
# image might have to be reset depending on elevation changes

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import os

# debug path crap (thank you Aiden)
print("Current Working Directory:", os.getcwd())

def find_points(input_image, template_image):

    #input image and check if it has been read
    img = cv.imread(input_image)
    if type(img) == None:
        print("Input image could not be read")
        exit

    #convert to grayscale for matching
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    #input template image as grayscale image
    temp = cv.imread(template_image, cv.IMREAD_GRAYSCALE)
    if type(temp) == None:
        print("Input image could not be read")
        exit
        #there has to be a better way to do this

    #determine sizes of images, check to see that template is smaller and warn
    img_w, img_h = img_gray.shape[::-1]
    temp_w, temp_h = temp.shape[::-1]
    if img_w <= temp_w or img_h <= temp_h:
        print("Template image exceeds input image dimensions, unexpected behavior may result!")

    #match template function
    recognize = cv.matchTemplate(img_gray, temp, cv.TM_CCOEFF_NORMED)
    threshold = 0.8
    locations = np.where(recognize >= threshold)

    points = zip(*locations[::-1])

    #temporary code to draw rectangles around points
    for pt in points:
        cv.rectangle(img, pt, (pt[0] + temp_w, pt[1] + temp_h), (0,0,255), 2)

    cv.imwrite('matched.png',img)

    return points

    #would be really nice to have a class devoted to camera functions so that you can just have programs utilizing them
    #have something similar to image capture thread, just constantly watching the button

#temp code to just display this image
points = find_points('constellation1.png', 'crop.png')

plt.imshow('matched.png')
plt.show()

