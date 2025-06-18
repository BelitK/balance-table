// Motor pin tanımları
const int in1A = 13;  // Motor A yön pini
const int ENA = 11;   // Motor A PWM hız kontrol pini
const int in1B = 8;   // Motor B yön pini
const int ENB = 9;    // Motor B PWM hız kontrol pini
const int in1C = 7;   // Motor C yön pini
const int ENC = 6;    // Motor C PWM hız kontrol pini
const int in1D = 2;   // Motor D yön pini
const int END = 3;    // Motor D PWM hız kontrol pini

// Başlangıç PWM değerleri
int motorSpeedA = 75;  // Motor A başlangıç hızı (0-255 arası)
int motorSpeedB = 75;  // Motor B başlangıç hızı
int motorSpeedC = 105; // Motor C başlangıç hızı
int motorSpeedD = 75;  // Motor D başlangıç hızı

const int speedStep = 6;    // Hız değişim miktarı
const int maxSpeed = 255;   // Maksimum hız
const int minSpeed = 0;     // Minimum hız

void setup() {
  // Motor pinlerini çıkış olarak ayarla
  pinMode(in1A, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(in1C, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(in1D, OUTPUT);
  pinMode(END, OUTPUT);

  // Seri haberleşmeyi başlat
  Serial.begin(9600);

  // Motor başlangıç yönleri ve hızlarını ayarla
  digitalWrite(in1A, HIGH);
  analogWrite(ENA, motorSpeedA);
  digitalWrite(in1B, HIGH);
  analogWrite(ENB, motorSpeedB);
  digitalWrite(in1C, HIGH);
  analogWrite(ENC, motorSpeedC);
  digitalWrite(in1D, HIGH);
  analogWrite(END, motorSpeedD);

  // Başlangıç PWM değerlerini ekrana yazdır
  Serial.println("Başlangıç PWM değerleri:");
  Serial.print("Motor A: "); Serial.println(motorSpeedA);
  Serial.print("Motor B: "); Serial.println(motorSpeedB);
  Serial.print("Motor C: "); Serial.println(motorSpeedC);
  Serial.print("Motor D: "); Serial.println(motorSpeedD);
}

void loop() {
  // Seri porttan veri gelirse işle
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Gelen veriyi oku
    // Tüm alanların varlığını kontrol et
    int idxA = input.indexOf("A:");
    int idxB = input.indexOf(",B:");
    int idxC = input.indexOf(",C:");
    int idxD = input.indexOf(",D:");
    if (idxA == 0 && idxB > 0 && idxC > 0 && idxD > 0) {
      // Gelen veriyi ayrıştır
      int pwmA = input.substring(idxA + 2, idxB).toInt();
      int pwmB = input.substring(idxB + 3, idxC).toInt();
      int pwmC = input.substring(idxC + 3, idxD).toInt();
      int pwmD = input.substring(idxD + 3).toInt();

      // PWM değerlerini sınırla
      pwmA = constrain(pwmA, minSpeed, maxSpeed);
      pwmB = constrain(pwmB, minSpeed, maxSpeed);
      pwmC = constrain(pwmC, minSpeed, maxSpeed);
      pwmD = constrain(pwmD, minSpeed, maxSpeed);

      // Motorlara PWM değerlerini gönder
      analogWrite(ENA, pwmA);
      analogWrite(ENB, pwmB);
      analogWrite(ENC, pwmC);
      analogWrite(END, pwmD);

      // PWM değerlerini seri monitöre yazdır
      Serial.print("Motor A PWM: "); Serial.print(pwmA);
      Serial.print(", Motor B PWM: "); Serial.print(pwmB);
      Serial.print(", Motor C PWM: "); Serial.print(pwmC);
      Serial.print(", Motor D PWM: "); Serial.println(pwmD);
    } else {
      // Hatalı komut gelirse buffer'ı temizle
      Serial.println("Hatalı komut alındı, buffer temizleniyor.");
      while (Serial.available() > 0) Serial.read();
    }
  }
}