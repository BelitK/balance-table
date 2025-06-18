import cv2
import numpy as np
import serial
import time
from simple_pid import PID
from camera import detect_white_circles_generator

detect_cam = 0  # Change to 1 if needed
arduino = serial.Serial('COM3', 9600, timeout=1)
cap = cv2.VideoCapture(detect_cam)
cap.set(3, 640)
cap.set(4, 480)

# Masa boyutları (cm)
table_width_cm = 18
table_height_cm = 25

# Piksel/cm dönüşüm oranları
pixels_per_cm_x = 640 / table_width_cm  # 35.56 px/cm
pixels_per_cm_y = 480 / table_height_cm  # 19.2 px/cm

# PWM başlangıç değeri
default_pwm = 75

# Görüntü merkezi koordinatları (px)
center_x, center_y = 640 // 2, 480 // 2

# Hedef konum başlangıcı (px)
target_x, target_y = center_x, center_y

# PID kontrolör tanımları
pid_x = PID(0.6, 0.1, 0.15, setpoint=0)
pid_y = PID(0.6, 0.1, 0.15, setpoint=0)
pid_x.output_limits = (-50, 50)
pid_y.output_limits = (-50, 50)

alpha = 0.9

# Önceki PWM değerleri (filtreleme için)
previous_pwm_a = default_pwm
previous_pwm_b = default_pwm
previous_pwm_c = default_pwm
previous_pwm_d = default_pwm

gen = detect_white_circles_generator(cap)

# Arduino'ya PWM komutlarını gönder
def send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d):
    command = f"A:{pwm_a},B:{pwm_b},C:{pwm_c},D:{pwm_d}\n"
    arduino.write(command.encode())
    time.sleep(0.01)  # Küçük gecikme, tampon taşmasını önlemek için

# Hedef seçimini değiştirme
def select_target(event, x, y, flags, param):
    global target_x, target_y
    if event == cv2.EVENT_LBUTTONDOWN:
        target_x, target_y = x, y
        print(f"Yeni hedef seçildi: ({target_x / pixels_per_cm_x:.2f} cm, {target_y / pixels_per_cm_y:.2f} cm)")

cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", select_target)

try:
    while True:
        circle_list = next(gen)
        ret, frame = cap.read()
        if not ret:
            break
        if circle_list:
            (x, y, r) = max(circle_list, key=lambda c: c[2])
            current_x, current_y = int(x), int(y)

            # Hedefle top konumu arasındaki fark (cm cinsinden)
            delta_x = (target_x - current_x) / pixels_per_cm_x
            delta_y = (target_y - current_y) / pixels_per_cm_y

            # PID kontrol çıktısı
            pwm_delta_x = pid_x(delta_x)
            pwm_delta_y = pid_y(delta_y)

            # Solenoid PWM hesaplama
            pwm_a = int(default_pwm + pwm_delta_y + pwm_delta_x)
            pwm_b = int(default_pwm + pwm_delta_y - pwm_delta_x)
            pwm_c = int(default_pwm - pwm_delta_y + pwm_delta_x)
            pwm_d = int(default_pwm - pwm_delta_y - pwm_delta_x)

            # PWM sınırlarını uygula
            pwm_a = max(0, min(255, pwm_a))
            pwm_b = max(0, min(255, pwm_b))
            pwm_c = max(0, min(255, pwm_c))
            pwm_d = max(0, min(255, pwm_d))

            # Düşük geçiren filtre uygulama
            pwm_a = int(alpha * pwm_a + (1 - alpha) * previous_pwm_a)
            pwm_b = int(alpha * pwm_b + (1 - alpha) * previous_pwm_b)
            pwm_c = int(alpha * pwm_c + (1 - alpha) * previous_pwm_c)
            pwm_d = int(alpha * pwm_d + (1 - alpha) * previous_pwm_d)

            previous_pwm_a, previous_pwm_b, previous_pwm_c, previous_pwm_d = pwm_a, pwm_b, pwm_c, pwm_d

            # Arduino'ya komut gönder
            send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d)

            # Top ve hedef konumunu çerçeveye çiz
            cv2.circle(frame, (target_x, target_y), 5, (0, 255, 0), -1)
            cv2.circle(frame, (current_x, current_y), int(r), (255, 0, 0), 2)
            cv2.putText(frame, f"Ball: ({current_x},{current_y})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(frame, f"Target: ({target_x},{target_y})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(frame, f"PWM A:{pwm_a} B:{pwm_b} C:{pwm_c} D:{pwm_d}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Klavye ile durduruldu.")
finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
