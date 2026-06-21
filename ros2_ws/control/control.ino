// ==========================================
// ESP32 + L298N (Core 3.x) + Encoders
// ROS2 serial protocol compatible:
//
// RX (from Jetson/PC):  "V <wl_rad_s> <wr_rad_s>\n"
// TX (to Jetson/PC):    "C <left_total_counts> <right_total_counts>\n"
//
// Notes:
// - Counts are absolute running totals (can be negative).
// - Wheel commands are angular velocity in rad/s.
// - This sketch does a simple closed-loop speed control using encoders.
// ==========================================

#include <Arduino.h>
#include <math.h>

// ---------- Pins ----------
// Motor A (Left)
const int IN1 = 18;
const int IN2 = 8;
const int ENA = 17;

// Motor B (Right)
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 16;

// Encoders
const int ENCODER1_A = 5; // Left A
const int ENCODER1_B = 4; // Left B
const int ENCODER2_A = 47; // Right A
const int ENCODER2_B = 21; // Right B

//Set deadbands
const int PWM_DEADBAND_LEFT  = 150;
const int PWM_DEADBAND_RIGHT = 150;
const double CMD_ZERO_EPS = 0.05;


// ---------- Encoder counts (absolute totals) ----------
// volatile -> these value can be changed inside interupt
volatile int64_t encoder1Count = 0;
volatile int64_t encoder2Count = 0;

void IRAM_ATTR encoder1ISR();
void IRAM_ATTR encoder2ISR();

// ---------- PWM ----------
const int pwmFreq = 20000;
const int pwmResolution = 8;   // 0..255

// ---------- Robot / encoder parameters ----------
double counts_per_rev = 449;    // MUST match ros2_control param counts_per_rev
bool invert_left  = true;         // match ros2_control param invert_left
bool invert_right = false;         // match ros2_control param invert_right
// maximum angular speed
double max_rad_s  = 20.0;          // match ros2_control param max_rad_s

// ---------- Control tuning (simple PI on speed) ----------
// Convert velocity error (rad/s) -> PWM command.
// Start with only P, then add a tiny I if needed.
double Kp = 18.0;   // try 8..30
double Ki = 0;   // try 0..80 (set 0 if you want P-only)

//Integrator error storage
double i_left = 0.0;
double i_right = 0.0;

// ---------- State ----------
double cmd_wl = 0.0; // commanded rad/s (left)
double cmd_wr = 0.0; // commanded rad/s (right)

uint32_t last_cmd_ms = 0;
// No command more than 250ms -> stop
const uint32_t CMD_TIMEOUT_MS = 250; // stop motors if no command

// For velocity measurement
int64_t last_c1 = 0, last_c2 = 0;
uint32_t last_vel_ms = 0;

// ---------- Serial line buffer ----------
String rx_line;

// ----------       Helpers ----------
//Clamp double
static inline double clampd(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
// Clamp integers
static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// power: -255..255  (sign = direction, magnitude = speed)
void setMotorA(int power) {
  power = clampi(power, -255, 255);
  int pwm = abs(power);

  if (power > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (power < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  ledcWrite(ENA, pwm);
}

void setMotorB(int power) {
  power = clampi(power, -255, 255);
  int pwm = abs(power);

  if (power > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (power < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  ledcWrite(ENB, pwm);
}

void stopAll() {
  setMotorA(0);
  setMotorB(0);
  i_left = 0.0;
  i_right = 0.0;
}

// Parse: "V wl wr" from JETSON NANO 
bool parseVelocityCommand(const String& line, double &wl, double &wr) {
  // REJECT INVALID COMMAND
  if (line.length() < 2) return false;
  if (line[0] != 'V') return false;

  // Split by spaces
  // Expected: V <wl> <wr>
  int sp1 = line.indexOf(' ');
  if (sp1 < 0) return false;
  int sp2 = line.indexOf(' ', sp1 + 1);
  if (sp2 < 0) return false;

  String s1 = line.substring(sp1 + 1, sp2);
  String s2 = line.substring(sp2 + 1);

  wl = s1.toFloat();
  wr = s2.toFloat();
  return true;
}

int applyDeadband(int pwm, double cmd_w, int deadband) {
  if (fabs(cmd_w) < CMD_ZERO_EPS) return 0;

  int sign = (pwm >= 0) ? 1 : -1;
  int mag = abs(pwm);

  if (mag > 0 && mag < deadband) {
    mag = deadband;
  }

  return sign * mag;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  // Preallocate memory for input string
  rx_line.reserve(128);

  // Encoder pins
  pinMode(ENCODER1_A, INPUT_PULLUP); pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP); pinMode(ENCODER2_B, INPUT_PULLUP);
  //monitor pins for interruption -> call encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2ISR, CHANGE);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // PWM attach (Core 3.x style)
  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);

  stopAll();

  noInterrupts();
  last_c1 = encoder1Count;
  last_c2 = encoder2Count;
  interrupts();

// Store current time
  last_cmd_ms = millis();
  last_vel_ms = millis();
}

// ---------- Main loop ----------
void loop() {
  // 1) Read serial non-blocking, line by line
  while (Serial.available() > 0) {
    //read each character
    char ch = (char)Serial.read();
    //Ignore carriage return
    if (ch == '\r') continue;

    if (ch == '\n') {
      // Got a full line
      double wl, wr;
      if (parseVelocityCommand(rx_line, wl, wr)) {
        // Clamp and store command (rad/s)
        wl = clampd(wl, -max_rad_s, max_rad_s);
        wr = clampd(wr, -max_rad_s, max_rad_s);

        // Apply invert flags the same way your ROS2 HW interface does BEFORE sending.
        // (If you already flip in ROS2, keep these false.)
        if (invert_left)  wl = -wl;
        if (invert_right) wr = -wr;

        cmd_wl = wl;
        cmd_wr = wr;
        last_cmd_ms = millis();
      }
      else
      {
        Serial.print("PARSE PACKET ERROR");
      }
      rx_line = "";
    } else {
      // If rx_line recieved not complete and < 120 -> add to rxline
      if (rx_line.length() < 120) rx_line += ch;
    }
  }

  // 2) Safety timeout: stop if commands stop arriving
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    cmd_wl = 0.0;
    cmd_wr = 0.0;
  }

  // 3) Control/update at ~100 Hz (every 10 ms)
  const uint32_t now = millis();
  if (now - last_vel_ms >= 10) {
    const double dt = (now - last_vel_ms) * 0.001;
    last_vel_ms = now;

    // Snapshot counts
    int64_t c1, c2;
    //STOP ALL ENCODER temporarily
    noInterrupts();
    c1 = encoder1Count;
    c2 = encoder2Count;
    interrupts();

    // Compute wheel angular velocities from counts delta
    // Calculate counts change from last updates
    const int64_t dc1 = c1 - last_c1;
    const int64_t dc2 = c2 - last_c2;
    last_c1 = c1;
    last_c2 = c2;

    //Convert counts to radian
    const double rad_per_count = (2.0 * M_PI) / (counts_per_rev > 1.0 ? counts_per_rev : 1.0);
    //Convert to angular velocity
    const double w1_meas = (double)dc1 * rad_per_count / (dt > 1e-6 ? dt : 1e-6);
    const double w2_meas = (double)dc2 * rad_per_count / (dt > 1e-6 ? dt : 1e-6);

    // PI control on wheel speed (rad/s)

    //calculate left wheel speed
    double eL = cmd_wl - w1_meas;
    //calculate right wheel speed
    double eR = cmd_wr - w2_meas;

    i_left  += eL * dt;
    i_right += eR * dt;

    // Anti-windup clamp
    i_left  = clampd(i_left,  -5.0, 5.0);
    i_right = clampd(i_right, -5.0, 5.0);

    // Control output in "PWM units"
    double uL = Kp * eL + Ki * i_left;
    double uR = Kp * eR + Ki * i_right;

    // Add a small feedforward based on desired speed (helps reduce steady error)
    // Map rad/s -> PWM roughly. Tune FF_GAIN so that max_rad_s ~ 255.
    const double FF_GAIN = 255.0 / (max_rad_s > 1e-6 ? max_rad_s : 1.0);
    uL += cmd_wl * FF_GAIN;
    uR += cmd_wr * FF_GAIN;

    int pwmL_ = (int)round(clampd(uL, -255.0, 255.0));
    int pwmR_ = (int)round(clampd(uR, -255.0, 255.0));

    int pwmL = applyDeadband(pwmL, cmd_wl, PWM_DEADBAND_LEFT);
    int pwmR = applyDeadband(pwmR, cmd_wr, PWM_DEADBAND_RIGHT);


    
    setMotorA(pwmL);
    setMotorB(pwmR);

    // 4) Send counts back to ROS2 at ~50 Hz (every 20 ms)
    static uint32_t last_pub_ms = 0;
    if (now - last_pub_ms >= 20) {
      last_pub_ms = now;

      // IMPORTANT: ROS2 expects: "C left right\n" absolute totals
      // Also, your ROS2 side may invert again; ideally keep inversion only on ONE side.
      int64_t outL = c1;
      int64_t outR = c2;
      if (invert_left)  outL = -outL;
      if (invert_right) outR = -outR;

      Serial.print("C ");
      Serial.print((long long)outL);
      Serial.print(" ");
      Serial.print((long long)outR);
      Serial.print("\n");
    }
  }
}

// ---------- Encoder ISRs ----------
// void IRAM_ATTR encoder1ISR() {
//   int b = digitalRead(ENCODER1_B);
//   if (b == HIGH) encoder1Count++;
//   else          encoder1Count--;
// }

// void IRAM_ATTR encoder2ISR() {
//   int b = digitalRead(ENCODER2_B);
//   if (b == HIGH) encoder2Count++;
//   else          encoder2Count--;
// }

void IRAM_ATTR encoder1ISR() {
  bool a = digitalRead(ENCODER1_A);
  bool b = digitalRead(ENCODER1_B);
  if (a == b) encoder1Count++;
  else        encoder1Count--;
}

void IRAM_ATTR encoder2ISR() {
  bool a = digitalRead(ENCODER2_A);
  bool b = digitalRead(ENCODER2_B);
  if (a == b) encoder2Count++;
  else        encoder2Count--;
}
