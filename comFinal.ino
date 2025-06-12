
#define IN1 26          // L298N 控制腳
#define IN2 27
#define ENA 25          // PWM 輸出腳位
#define ENCODER_PIN 34  // 編碼器 A 相（C1）

volatile long encoderCount = 0;

// PID 參數
float Kp = 0.6, Ki = 0.03, Kd = 0.15; 
float integral = 0;
float previous_error = 0;

void IRAM_ATTR readEncoder() {
  encoderCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), readEncoder, RISING);

  Serial.println("PID啟動");
}

void analogMotorWrite(int pwmVal, bool direction) {
  pwmVal = constrain(pwmVal, 0, 255);
  if (direction) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, pwmVal);
}

void loop() {
  static int target_error = 0;

  if (Serial.available()) {
    target_error = Serial.parseInt();
  }

  float error = target_error;


  if (abs(error) < 20) {
    analogMotorWrite(0, true);
    integral = 0;
    Serial.println("偏差:0,輸出:0");
    return;
  }

  integral += error;
  integral = constrain(integral, -300, 300);

  if (abs(error) < 5) {
    integral = 0;
  }

  float derivative = error - previous_error;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  output = constrain(output, -255, 255);

  if (abs(output) > 0 && abs(output) < 50) {
    output = (output > 0) ? 50 : -50;
  }

  Serial.print("偏差:");
  Serial.print(target_error);
  Serial.print(",輸出:");
  Serial.println(output);

  if (output > 0) {
    analogMotorWrite(output, true);
  } else if (output < 0) {
    analogMotorWrite(-output, false);
  } else {
    analogMotorWrite(0, true);
  }
}
