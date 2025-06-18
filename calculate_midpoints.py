import cv2
import numpy as np
from camera import detect_shapes_generator

def calculate_midpoints(rectangles, circles):
    rect_centers = []
    for rect in rectangles:
        # rect is a 4x1x2 array, get mean of points
        pts = rect.reshape(-1, 2)
        center = np.mean(pts, axis=0)
        rect_centers.append(tuple(center))
    circle_centers = [(x, y) for (x, y, r) in circles]
    return rect_centers, circle_centers

def calculate_distances(rect_centers, circle_centers):
    distances = []
    for rc in rect_centers:
        for cc in circle_centers:
            dist = np.linalg.norm(np.array(rc) - np.array(cc))
            distances.append((rc, cc, dist))
    return distances

def filter_overlapping_circles(circles, min_dist=5):
    # Keep only the largest circle for each center within min_dist
    if not circles:
        return []
    circles = sorted(circles, key=lambda c: c[2], reverse=True)  # largest first
    filtered = []
    for c in circles:
        if all(np.linalg.norm(np.array((c[0], c[1])) - np.array((f[0], f[1]))) > min_dist for f in filtered):
            filtered.append(c)
    return filtered

def main():
    cap = cv2.VideoCapture(0)
    # Increase rect_area_threshold and black_threshold to detect dark rectangles
    gen = detect_shapes_generator(cap, rect_area_threshold=2000, black_threshold=100)
    while True:
        rectangles, circles, _ = next(gen)
        ret, frame = cap.read()
        if not ret:
            break
        # Filter overlapping circles
        circles = filter_overlapping_circles(circles, min_dist=20)
        # Draw rectangles
        for rect in rectangles:
            cv2.drawContours(frame, [rect], 0, (255, 0, 0), 2)
        # Draw circles
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
        # Draw centers
        rect_centers, circle_centers = calculate_midpoints(rectangles, circles)
        for center in rect_centers:
            cv2.circle(frame, (int(center[0]), int(center[1])), 5, (255, 255, 0), -1)
        for center in circle_centers:
            cv2.circle(frame, (int(center[0]), int(center[1])), 5, (0, 255, 255), -1)
        # Optionally, draw lines and show distances
        distances = calculate_distances(rect_centers, circle_centers)
        for rc, cc, dist in distances:
            cv2.line(frame, (int(rc[0]), int(rc[1])), (int(cc[0]), int(cc[1])), (128, 128, 128), 1)
            midx = int((rc[0] + cc[0]) / 2)
            midy = int((rc[1] + cc[1]) / 2)
            cv2.putText(frame, f"{dist:.1f}", (midx, midy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.imshow('Midpoints and Distances', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
