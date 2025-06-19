# Motor mapping: A=left-bottom, B=left-up, C=right-up, D=right-bottom
import cv2
import numpy as np
import serial
from simple_pid import PID
import time
import threading

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

def read_arduino_serial():
    while True:
        try:
            if arduino.in_waiting:
                line = arduino.readline().decode(errors='ignore').strip()
                if line:
                    print(f"[ARDUINO] {line}")
        except Exception:
            break

serial_thread = threading.Thread(target=read_arduino_serial, daemon=True)
serial_thread.start()

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
                dx = current_x - center_x
                dy = current_y - center_y
                print(f"Ball: ({current_x},{current_y})  dx: {dx}  dy: {dy}")
                sector_pwm = 180
                rest_pwm = 75
                if abs(dx) < 10 and abs(dy) < 10:
                    pwm_a = pwm_b = pwm_c = pwm_d = rest_pwm
                    print("Sector: Center")
                elif dx < 0 and dy > 0:
                    pwm_a = sector_pwm
                    pwm_b = rest_pwm
                    pwm_c = rest_pwm
                    pwm_d = rest_pwm
                    print("Sector: Left-Bottom (A)")
                elif dx < 0 and dy < 0:
                    pwm_b = sector_pwm
                    pwm_a = rest_pwm
                    pwm_c = rest_pwm
                    pwm_d = rest_pwm
                    print("Sector: Left-Up (B)")
                elif dx > 0 and dy < 0:
                    pwm_c = sector_pwm
                    pwm_a = rest_pwm
                    pwm_b = rest_pwm
                    pwm_d = rest_pwm
                    print("Sector: Right-Up (C)")
                elif dx > 0 and dy > 0:
                    pwm_d = sector_pwm
                    pwm_a = rest_pwm
                    pwm_b = rest_pwm
                    pwm_c = rest_pwm
                    print("Sector: Right-Bottom (D)")
                elif dx < 0:
                    pwm_a = pwm_b = sector_pwm
                    pwm_c = pwm_d = rest_pwm
                    print("Sector: Left")
                elif dx > 0:
                    pwm_c = pwm_d = sector_pwm
                    pwm_a = pwm_b = rest_pwm
                    print("Sector: Right")
                elif dy < 0:
                    pwm_b = pwm_c = sector_pwm
                    pwm_a = pwm_d = rest_pwm
                    print("Sector: Up")
                elif dy > 0:
                    pwm_a = pwm_d = sector_pwm
                    pwm_b = pwm_c = rest_pwm
                    print("Sector: Down")
                print(f"PWM: A={pwm_a} B={pwm_b} C={pwm_c} D={pwm_d}")
                send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d)
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.circle(frame, (current_x, current_y), int(radius), (255, 0, 0), 2)
                ball_found = True
                lost_ball_counter = 0
        else:
            print("Ball not found")
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