// IKA Motor Controller - Arduino Uno
// Pi'den JSON komut alir: {"l":0.12,"r":-0.05}
// l, r: m/s biriminde sol/sag hiz hedefleri.
// USB-Serial timeout: TIMEOUT_MS asilirsa motorlar durur.

#include <ArduinoJson.h>

// --- Pin Tanimlari ---
const int LEFT_PWM  = 5;
const int LEFT_IN1  = 6;
const int LEFT_IN2  = 7;

const int RIGHT_PWM = 10;
const int RIGHT_IN1 = 8;
const int RIGHT_IN2 = 9;

// --- Kalibrasyon Parametreleri ---
// Gercek arac olcumleriyle guncellenecek.
const float MAX_SPEED_MPS  = 0.30f;
const int   MAX_PWM        = 200;
const int   MIN_PWM        = 60;
const unsigned long TIMEOUT_MS = 500UL;

unsigned long lastCmdTime = 0;

void stopMotors() {
  analogWrite(LEFT_PWM,  0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}

void setMotor(int pwmPin, int in1, int in2, float speed_mps) {
  int dir = (speed_mps >= 0) ? 1 : -1;
  float absSpeed = fabs(speed_mps);
  int pwm = 0;

  if (absSpeed > 0.001f) {
    pwm = (int)(MIN_PWM + (absSpeed / MAX_SPEED_MPS) * (MAX_PWM - MIN_PWM));
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  }

  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir > 0 ? LOW  : HIGH);
  analogWrite(pwmPin, pwm);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();
}

void loop() {
  // Watchdog
  if (millis() - lastCmdTime > TIMEOUT_MS) {
    stopMotors();
  }

  // Seri okuma
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    StaticJsonDocument<96> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (!err) {
      float v_left  = doc["l"] | 0.0f;
      float v_right = doc["r"] | 0.0f;
      setMotor(LEFT_PWM,  LEFT_IN1,  LEFT_IN2,  v_left);
      setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, v_right);
      lastCmdTime = millis();
    }
  }
}
