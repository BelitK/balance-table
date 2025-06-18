import numpy as np
import serial
from simple_pid import PID

# Arduino seri port bağlantısı
arduino = serial.Serial('COM3', 9600, timeout=1)

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

# Arduino'ya PWM komutlarını gönder
def send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d):
    command = f"A:{pwm_a},B:{pwm_b},C:{pwm_c},D:{pwm_d}\n"
    arduino.write(command.encode())

# Harici bir kaynaktan top pozisyonunu alma (örnek fonksiyon)
# Bu fonksiyonu, gerçek top pozisyonunu döndüren bir işlem veya modül ile değiştirin
def get_ball_position():
    # Yer tutucu: top yoksa None, tespit edildiyse (x, y, r) döndür
    return None

try:
    while True:
        ball = get_ball_position()
        if ball:
            x, y, r = ball
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
            pwm_a = max(75, min(255, pwm_a))
            pwm_b = max(75, min(255, pwm_b))
            pwm_c = max(75, min(255, pwm_c))
            pwm_d = max(75, min(255, pwm_d))

            # Düşük geçiren filtre uygulama
            pwm_a = int(alpha * pwm_a + (1 - alpha) * previous_pwm_a)
            pwm_b = int(alpha * pwm_b + (1 - alpha) * previous_pwm_b)
            pwm_c = int(alpha * pwm_c + (1 - alpha) * previous_pwm_c)
            pwm_d = int(alpha * pwm_d + (1 - alpha) * previous_pwm_d)

            previous_pwm_a, previous_pwm_b, previous_pwm_c, previous_pwm_d = pwm_a, pwm_b, pwm_c, pwm_d

            # Arduino'ya komut gönder
            send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d)
except KeyboardInterrupt:
    print("Klavye ile durduruldu.")
finally:
    arduino.close()
