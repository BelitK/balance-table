import cv2
import numpy as np

def detect_shapes_generator(cap, circle_sensitivity=30, rect_area_threshold=500, circle_min_radius=5, circle_max_radius=100, black_threshold=100):
    if not cap.isOpened():
        raise ValueError("Cannot open camera.")
    while True:
        ret, img = cap.read()
        if not ret:
            yield [], [], False
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh_rect = cv2.threshold(gray, black_threshold, 255, cv2.THRESH_BINARY_INV)
        blurred_rect = cv2.GaussianBlur(thresh_rect, (5, 5), 0)
        contours, _ = cv2.findContours(blurred_rect, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rectangles = []
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)
            if len(approx) == 4 and area > rect_area_threshold and cv2.isContourConvex(approx):
                rectangles.append(approx)
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
        has_circle_in_rectangle = False
        for rect in rectangles:
            for (x, y, r) in circle_list:
                if cv2.pointPolygonTest(rect, (x, y), False) >= 0:
                    has_circle_in_rectangle = True
                    break
            if has_circle_in_rectangle:
                break
        yield rectangles, circle_list, has_circle_in_rectangle
