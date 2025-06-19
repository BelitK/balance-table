import cv2
import numpy as np
import serial
from simple_pid import PID
import time

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
pid_x.output_limits = (-120, 120)
pid_y.output_limits = (-120, 120)

MIN_CONTOUR_AREA = 4000

previous_pwm_a = default_pwm
previous_pwm_b = default_pwm
previous_pwm_c = default_pwm
previous_pwm_d = default_pwm

LOWER_H = 0
UPPER_H = 180
LOWER_S = 0
UPPER_S = 30
LOWER_V = 200
UPPER_V = 255

deadzone_cm = 0.5
lost_ball_max_frames = 15
lost_ball_counter = 0

def send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d):
    command = f"A:{pwm_a},B:{pwm_b},C:{pwm_c},D:{pwm_d}\n"
    arduino.write(command.encode())
    time.sleep(0.01)

def select_target(event, x, y, flags, param):
    global target_x, target_y
    if event == cv2.EVENT_LBUTTONDOWN:
        target_x, target_y = x, y
        print(f"Yeni hedef seçildi: ({target_x / pixels_per_cm_x:.2f} cm, {target_y / pixels_per_cm_y:.2f} cm)")

cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", select_target)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (LOWER_H, LOWER_S, LOWER_V), (UPPER_H, UPPER_S, UPPER_V))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_found = False
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                current_x, current_y = int(x), int(y)
                delta_x = (target_x - current_x) / pixels_per_cm_x
                delta_y = (target_y - current_y) / pixels_per_cm_y
                if abs(delta_x) < deadzone_cm:
                    delta_x = 0
                if abs(delta_y) < deadzone_cm:
                    delta_y = 0
                distance = np.hypot(delta_x, delta_y)
                min_alpha = 0.6
                max_alpha = 0.95
                max_distance = 5.0
                alpha = min_alpha + (max_alpha - min_alpha) * min(distance / max_distance, 1.0)
                pwm_delta_x = pid_x(delta_x)
                pwm_delta_y = pid_y(delta_y)
                target_pwm_a = default_pwm + pwm_delta_y + pwm_delta_x
                target_pwm_b = default_pwm + pwm_delta_y - pwm_delta_x
                target_pwm_c = default_pwm - pwm_delta_y + pwm_delta_x
                target_pwm_d = default_pwm - pwm_delta_y - pwm_delta_x
                pwm_a = alpha * previous_pwm_a + (1 - alpha) * target_pwm_a
                pwm_b = alpha * previous_pwm_b + (1 - alpha) * target_pwm_b
                pwm_c = alpha * previous_pwm_c + (1 - alpha) * target_pwm_c
                pwm_d = alpha * previous_pwm_d + (1 - alpha) * target_pwm_d
                pwm_a = int(max(75, min(255, pwm_a)))
                pwm_b = int(max(75, min(255, pwm_b)))
                pwm_c = int(max(75, min(255, pwm_c)))
                pwm_d = int(max(75, min(255, pwm_d)))
                previous_pwm_a, previous_pwm_b = pwm_a, pwm_b
                previous_pwm_c, previous_pwm_d = pwm_c, pwm_d
                send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d)
                cv2.circle(frame, (target_x, target_y), 5, (0, 255, 0), -1)
                cv2.circle(frame, (current_x, current_y), int(radius), (255, 0, 0), 2)
                ball_found = True
                lost_ball_counter = 0
        if not ball_found:
            lost_ball_counter += 1
            if lost_ball_counter >= lost_ball_max_frames:
                previous_pwm_a = previous_pwm_b = previous_pwm_c = previous_pwm_d = default_pwm
                send_motor_commands(default_pwm, default_pwm, default_pwm, default_pwm)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Klavye ile durduruldu.")
finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()