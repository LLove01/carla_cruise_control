import cv2
import numpy as np


def canny(image):
    # canny method for edge detection -> derivateves in all directions (getting gradient)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # gaussian blur - reducing noise
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # canny method for edge detection -> derivateves in all directions (getting gradient)
    canny = cv2.Canny(blur, 50, 150)
    return canny


def region_of_interest(image, label):
    height = image.shape[0]
    if label == 'FW':
        # array of polygons
        polygons = np.array(
            [[(100, height - 20), (700, height - 20), (370, 275)]])
    elif label == 'R':
        polygons = np.array(
            [[(100, height - 20), (700, height - 20), (700, int(height/2)), (100, int(height/2))]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    # bitwise & of each homologous pixel in both arrays -> showing only the region of interest
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_image


def get_lanes(image, label):
    # takes rgb image as argument - 3 channels
    lane_image = np.copy(image)
    # canny filter
    canny_image = canny(lane_image)
    # region of interest
    cropped_image = region_of_interest(canny_image, label)
    # detecting straight lines (image, theta, row, threshold, placeholder)
    # theta, row ... resolution of hough space - size of bins
    # threshold ... number of intersections needed to detect a line
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180,
                            100, np.array([]), minLineLength=40, maxLineGap=5)
    line_image = display_lines(lane_image, lines)
    overlay_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
    return lines, overlay_image
