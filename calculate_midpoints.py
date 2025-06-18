import cv2
import numpy as np
from camera import detect_white_circles_generator

def get_outermost_circle(circles, cam_center):
    if not circles:
        return None
    max_dist = -1
    outermost = None
    for (x, y, r) in circles:
        dist = np.linalg.norm(np.array([x, y]) - np.array(cam_center))
        if dist > max_dist:
            max_dist = dist
            outermost = (x, y, r)
    return outermost

def get_ball_position():
    cap = cv2.VideoCapture(0)
    gen = detect_white_circles_generator(cap)
    ret, frame = cap.read()
    if not ret:
        cap.release()
        return None
    h, w = frame.shape[:2]
    cam_center = (w // 2, h // 2)
    circle_list = next(gen)
    ball = get_outermost_circle(circle_list, cam_center)
    cap.release()
    return ball

def get_relative_position(x, y, cam_center):
    cx, cy = cam_center
    rel_x = x - cx
    rel_y = y - cy
    # Quadrant logic: (0,0) is center, axes split into 4
    if rel_x >= 0 and rel_y < 0:
        quadrant = 'Top-Right'
    elif rel_x < 0 and rel_y < 0:
        quadrant = 'Top-Left'
    elif rel_x < 0 and rel_y >= 0:
        quadrant = 'Bottom-Left'
    else:
        quadrant = 'Bottom-Right'
    return rel_x, rel_y, quadrant

def get_motor_commands(rel_x, rel_y, threshold=20):
    cmd_x = 'STOP'
    cmd_y = 'STOP'
    if rel_x > threshold:
        cmd_x = 'RIGHT'
    elif rel_x < -threshold:
        cmd_x = 'LEFT'
    if rel_y > threshold:
        cmd_y = 'DOWN'
    elif rel_y < -threshold:
        cmd_y = 'UP'
    return cmd_x, cmd_y

def main():
    cap = cv2.VideoCapture(0)
    gen = detect_white_circles_generator(cap)
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    cam_center = [w // 2, h // 2]
    def set_center(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cam_center[0] = x
            cam_center[1] = y
    cv2.namedWindow('White Circle to Camera Center')
    cv2.setMouseCallback('White Circle to Camera Center', set_center)
    while True:
        circle_list = next(gen)
        ret, frame = cap.read()
        if not ret:
            break
        h, w = frame.shape[:2]
        # Only keep the outermost circle
        circle_list = get_outermost_circle(circle_list, cam_center)
        for (x, y, r) in circle_list:
            x = int(x)
            y = int(y)
            r = int(r)
            rel_x, rel_y, quadrant = get_relative_position(x, y, cam_center)
            cmd_x, cmd_y = get_motor_commands(rel_x, rel_y)
            cv2.circle(frame, (x, y), r, (255, 255, 255), 2)  # white
            cv2.circle(frame, (x, y), 2, (200, 200, 200), 3)  # light gray
            cv2.line(frame, (x, y), tuple(cam_center), (220, 220, 220), 2)  # very light gray
            dist = np.linalg.norm(np.array([x, y]) - np.array(cam_center))
            cv2.putText(frame, f"{dist:.1f}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.putText(frame, f"({rel_x},{rel_y}) {quadrant}", (x+10, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.putText(frame, f"Motor X: {cmd_x}  Y: {cmd_y}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            print(f"Ball: ({rel_x},{rel_y}) Quadrant: {quadrant} | Motor X: {cmd_x}  Y: {cmd_y}")
        cv2.circle(frame, tuple(cam_center), 5, (255, 255, 255), -1)
        # Draw axes
        cv2.line(frame, (cam_center[0], 0), (cam_center[0], h), (180, 180, 180), 1)
        cv2.line(frame, (0, cam_center[1]), (w, cam_center[1]), (180, 180, 180), 1)
        cv2.imshow('White Circle to Camera Center', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
