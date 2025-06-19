import cv2
import numpy as np
import serial
from simple_pid import PID
from camera import detect_white_circles_generator
from calculate_midpoints import get_outermost_circle
from collections import deque

arduino = serial.Serial('COM3', 9600, timeout=1)
cap = cv2.VideoCapture(1)
cap.set(3, 640)
cap.set(4, 480)
table_width_cm = 18
table_height_cm = 25
pixels_per_cm_x = 640 / table_width_cm
pixels_per_cm_y = 480 / table_height_cm
center_x, center_y = 640 // 2, 480 // 2
default_pwm = 75
target_x, target_y = center_x, center_y
pid_x = PID(0.2, 0.01, 15.5, setpoint=0)
pid_y = PID(0.2, 0.01, 15.5, setpoint=0)
pid_x.output_limits = (-40, 40)
pid_y.output_limits = (-40, 40)
alpha = 0.9
max_pwm_step = 3
previous_pwm_a = default_pwm
previous_pwm_b = default_pwm
previous_pwm_c = default_pwm
previous_pwm_d = default_pwm
gen = detect_white_circles_generator(cap)
pwm_window_size = 5
pwm_a_history = deque([default_pwm]*pwm_window_size, maxlen=pwm_window_size)
pwm_b_history = deque([default_pwm]*pwm_window_size, maxlen=pwm_window_size)
pwm_c_history = deque([default_pwm]*pwm_window_size, maxlen=pwm_window_size)
pwm_d_history = deque([default_pwm]*pwm_window_size, maxlen=pwm_window_size)

def send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d):
    command = f"A:{pwm_a},B:{pwm_b},C:{pwm_c},D:{pwm_d}\n"
    arduino.write(command.encode())

def select_target(event, x, y, flags, param):
    global target_x, target_y
    if event == cv2.EVENT_LBUTTONDOWN:
        target_x, target_y = x, y
        print(f"Yeni hedef seçildi: ({target_x / pixels_per_cm_x:.2f} cm, {target_y / pixels_per_cm_y:.2f} cm)")

def smooth_pwm(previous, target, step=max_pwm_step):
    if target > previous:
        return min(previous + step, target)
    elif target < previous:
        return max(previous - step, target)
    else:
        return target

cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", select_target)

try:
    while True:
        circle_list = next(gen)
        ret, frame = cap.read()
        if not ret:
            break
        cam_center = (center_x, center_y)
        circle_list = get_outermost_circle(circle_list, cam_center)
        if circle_list:
            (x, y, r) = circle_list[0]
            current_x, current_y = int(x), int(y)
            delta_x = (target_x - current_x) / pixels_per_cm_x
            delta_y = (target_y - current_y) / pixels_per_cm_y
            pwm_delta_x = pid_x(delta_x)
            pwm_delta_y = pid_y(delta_y)
            target_pwm_a = int(default_pwm + pwm_delta_y + pwm_delta_x)
            target_pwm_b = int(default_pwm + pwm_delta_y - pwm_delta_x)
            target_pwm_c = int(default_pwm - pwm_delta_y + pwm_delta_x)
            target_pwm_d = int(default_pwm - pwm_delta_y - pwm_delta_x)
            target_pwm_a = max(75, min(255, target_pwm_a))
            target_pwm_b = max(75, min(255, target_pwm_b))
            target_pwm_c = max(75, min(255, target_pwm_c))
            target_pwm_d = max(75, min(255, target_pwm_d))
            pwm_a = smooth_pwm(previous_pwm_a, target_pwm_a)
            pwm_b = smooth_pwm(previous_pwm_b, target_pwm_b)
            pwm_c = smooth_pwm(previous_pwm_c, target_pwm_c)
            pwm_d = smooth_pwm(previous_pwm_d, target_pwm_d)
            pwm_a = int(alpha * pwm_a + (1 - alpha) * previous_pwm_a)
            pwm_b = int(alpha * pwm_b + (1 - alpha) * previous_pwm_b)
            pwm_c = int(alpha * pwm_c + (1 - alpha) * previous_pwm_c)
            pwm_d = int(alpha * pwm_d + (1 - alpha) * previous_pwm_d)
            pwm_a = max(75, min(255, pwm_a))
            pwm_b = max(75, min(255, pwm_b))
            pwm_c = max(75, min(255, pwm_c))
            pwm_d = max(75, min(255, pwm_d))
            pwm_a_history.append(pwm_a)
            pwm_b_history.append(pwm_b)
            pwm_c_history.append(pwm_c)
            pwm_d_history.append(pwm_d)
            pwm_a = int(sum(pwm_a_history) / pwm_window_size)
            pwm_b = int(sum(pwm_b_history) / pwm_window_size)
            pwm_c = int(sum(pwm_c_history) / pwm_window_size)
            pwm_d = int(sum(pwm_d_history) / pwm_window_size)
            previous_pwm_a, previous_pwm_b = pwm_a, pwm_b
            previous_pwm_c, previous_pwm_d = pwm_c, pwm_d
            send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d)
            cv2.circle(frame, (target_x, target_y), 5, (0, 255, 0), -1)
            cv2.circle(frame, (current_x, current_y), int(r), (255, 0, 0), 2)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Klavye ile durduruldu.")
finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
