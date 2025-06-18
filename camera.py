import cv2
import numpy as np

def detect_white_circles_generator(cap, circle_sensitivity=30, circle_min_radius=5, circle_max_radius=100):
    if not cap.isOpened():
        raise ValueError("Cannot open camera.")
    while True:
        ret, img = cap.read()
        if not ret:
            yield []
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh_circ = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        blurred_circ = cv2.medianBlur(thresh_circ, 5)
        circles = cv2.HoughCircles(
            blurred_circ,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=50,
            param2=circle_sensitivity,
            minRadius=circle_min_radius,
            maxRadius=circle_max_radius
        )
        circle_list = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                circle_list.append((i[0], i[1], i[2]))
        yield circle_list
