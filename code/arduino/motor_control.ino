#define MOTOR1_PWM 9  // Front-left wheel
#define MOTOR2_PWM 10 // Front-right wheel
#define MOTOR3_PWM 11 // Rear-left wheel
#define MOTOR4_PWM 12 // Rear-right wheel

void setup() {
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Example: Move forward at 50% speed
  analogWrite(MOTOR1_PWM, 128); // 50% PWM
  analogWrite(MOTOR2_PWM, 128);
  analogWrite(MOTOR3_PWM, 128);
  analogWrite(MOTOR4_PWM, 128);
  
  if (Serial.available()) {
    int speed = Serial.parseInt();
    analogWrite(MOTOR1_PWM, speed);
    analogWrite(MOTOR2_PWM, speed);
    analogWrite(MOTOR3_PWM, speed);
    analogWrite(MOTOR4_PWM, speed);
  }
  delay(100);
}
