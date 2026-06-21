// =========================
// ESP32 + L298N (Core 3.x)
// Encoder skipped
// =========================

// Motor A
const int IN1 = 26;
const int IN2 = 25;
const int ENA = 32;

// Motor B
const int IN3 = 14;
const int IN4 = 27;
const int ENB = 33;

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
const int ENCODER1_A = 23;
const int ENCODER1_B = 22;
const int ENCODER2_A = 19;
const int ENCODER2_B = 18;
void IRAM_ATTR encoder1ISR();
void IRAM_ATTR encoder2ISR();



// PWM settings
const int pwmFreq = 20000;
const int pwmResolution = 8;   // 0..255

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER1_A, INPUT_PULLUP); pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP); pinMode(ENCODER2_B, INPUT_PULLUP);
  
  attachInterrupt(
    digitalPinToInterrupt(ENCODER1_A),
    encoder1ISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENCODER2_A),
    encoder2ISR,
    RISING
  );
//SETUP PINS
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // ESP32 Core 3.x PWM attach
  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);

  stopAll();
  Serial.println("Starting motor test...");
}

void loop() {
  static long lastCount1 = 0;
  static long lastCount2 = 0;

  noInterrupts();
  long count1 = encoder1Count;
  long count2 = encoder2Count;
  interrupts();

  if (count1 != lastCount1) {
    Serial.print("enc1: ");
    Serial.println(count1);
    lastCount1 = count1;
  }
  if (count2 != lastCount2) {
    Serial.print("enc2: ");
    Serial.println(count2);
    lastCount2 = count2;
  }
  const long l1 = 200;
  const long t1 = 135;

  //goForward(l1,170);
  turnRight(t1,170);
  delay(5000);
  turnLeft(t1, 170);
  delay(5000);
  goForward(l1,200);
  // turnRight(t1,200);
  // goForward(l1,200);
  // turnRight(t1,200);
  // goForward(l1,200);
  // turnRight(t1,200);

  stopAll();
  delay(2000);

}
void goForward(long target, int basePWM){
  noInterrupts();
  encoder1Count = 0;
  encoder2Count = 0;
  interrupts();

  const int Kp = 1.5;          // try 1..5
  const int maxAdjust = 80;  // limit correction
  const int deadband = 1;

  while (true) {
    noInterrupts();
    long c1 = encoder1Count;
    long c2 = encoder2Count;
    interrupts();

    if ( (c1 + c2)/2 >= target ) {
      stopAll();
      break;
    }

    long err = c1-c2;
    if(labs(err)< deadband){
      err = 0;
    }
    int adjust = (int)constrain(Kp*err,-maxAdjust, maxAdjust);
    // if c1 > c2 (err positive) => slow A / speed B
    int pwmA = constrain(basePWM - adjust, 0, 255);
    int pwmB = constrain(basePWM + adjust, 0, 255);

    setMotorA(pwmA);
    setMotorB(pwmB);

    delay(5);
  }
}
void turnRight(long target, int pwm) {
  noInterrupts();
  encoder1Count = 0;
  encoder2Count = 0;
  interrupts();

  const int Kp = 2;          // keeps turn symmetric
  const int maxAdjust = 60;

  // start in-place right turn
  while (true) {
    noInterrupts();
    long c1 = labs(encoder1Count);
    long c2 = labs(encoder2Count);
    interrupts();

    if ((c1 + c2)/2 >= target) break;

    long err = c1 - c2; // keep both sides rotating same amount
    int adjust = (int)constrain(Kp * err, -maxAdjust, maxAdjust);

    int pwmA = constrain(pwm - adjust, 0, 255);
    int pwmB = constrain(pwm + adjust, 0, 255);

    setMotorA(+pwmA);
    setMotorB(-pwmB);

    delay(5);
  }

  stopAll();

}
void turnLeft(long target, int pwm) {
  noInterrupts();
  encoder1Count = 0;
  encoder2Count = 0;
  interrupts();

  const int Kp = 2;          // keeps turn symmetric
  const int maxAdjust = 60;

  // start in-place right turn
  while (true) {
    noInterrupts();
    long c1 = labs(encoder1Count);
    long c2 = labs(encoder2Count);
    interrupts();

    if ((c1 + c2)/2 >= target) break;

    long err = c1 - c2; // keep both sides rotating same amount
    int adjust = (int)constrain(Kp * err, -maxAdjust, maxAdjust);

    int pwmA = constrain(pwm - adjust, 0, 255);
    int pwmB = constrain(pwm + adjust, 0, 255);

    setMotorA(-pwmA);
    setMotorB(+pwmB);

    delay(5);
  }

  stopAll();

}
// power: -255..255  (sign = direction, magnitude = speed)
void setMotorA(int power) {
  power = constrain(power, -255, 255);
  int pwm = abs(power);

  if (power > 0) {            // forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (power < 0) {     // backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {                    // stop (coast)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  ledcWrite(ENA, pwm);
}

void setMotorB(int power) {
  power = constrain(power, -255, 255);
  int pwm = abs(power);

  if (power > 0) {            // forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (power < 0) {     // backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {                    // stop (coast)
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  ledcWrite(ENB, pwm);
}

void stopAll() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void IRAM_ATTR encoder1ISR() {
  int b = digitalRead(ENCODER1_B);
  if (b == HIGH) {
    encoder1Count++;   // forward
  } else {
    encoder1Count--;   // backward
  }
}
void IRAM_ATTR encoder2ISR() {
  int b = digitalRead(ENCODER2_B);
  if (b == HIGH) {
    encoder2Count++;   // forward
  } else {
    encoder2Count--;   // backward
  }
}

