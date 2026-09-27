// BoatLab motor bridge - Jetson (USB serial) -> Arduino -> two motor controllers + payload actuators.
// TEMPLATE: your motor controller model is not documented yet. Pick OUTPUT_MODE below, set the pins,
// and bench test with the propellers OUT of the water before any lake test.
//
// Serial protocol (115200 baud, one command per line):
//   M,<left>,<right>   left/right in [-1000, 1000]   (+ = forward thrust; left = port motor)
//   S                  stop (neutral)
//   A,DEPLOY | A,LAUNCH | A,RECOVER   fire a payload mechanism, replies "ACK,<NAME>"
// Safety: motors go neutral if no M command arrives for 500 ms (the Python runner sends 20 Hz).

#include <Servo.h>

#define OUTPUT_MODE_RC_PWM 1      // 1: RC-style ESC / Sabertooth R/C input (1000-2000 us, 1500 neutral)
                                  // 0: DIR + PWM driver (e.g. Cytron MD30C): one direction pin + one PWM pin per motor
const int LEFT_PIN = 9, RIGHT_PIN = 10;           // RC mode signal pins
const int LEFT_DIR = 7, LEFT_PWM = 5, RIGHT_DIR = 8, RIGHT_PWM = 6;   // DIR+PWM mode pins
const int PULSE_NEUTRAL = 1500, PULSE_RANGE = 400; // RC mode: +/-400 us around neutral (tune to your ESC)
const bool INVERT_LEFT = false, INVERT_RIGHT = false;
const unsigned long WATCHDOG_MS = 500;

// payload mechanisms (servo positions) - adjust to your hardware
const int DEPLOY_PIN = 3, LAUNCH_PIN = 4, RECOVER_PIN = 11;
Servo leftEsc, rightEsc, deployServo, launchServo, recoverServo;

unsigned long lastCmd = 0;
String line;

void writeMotor(bool left, int value) {           // value in [-1000, 1000]
  value = constrain(value, -1000, 1000);
  if ((left && INVERT_LEFT) || (!left && INVERT_RIGHT)) value = -value;
#if OUTPUT_MODE_RC_PWM
  int us = PULSE_NEUTRAL + (long)value * PULSE_RANGE / 1000;
  (left ? leftEsc : rightEsc).writeMicroseconds(us);
#else
  digitalWrite(left ? LEFT_DIR : RIGHT_DIR, value >= 0 ? HIGH : LOW);
  analogWrite(left ? LEFT_PWM : RIGHT_PWM, map(abs(value), 0, 1000, 0, 255));
#endif
}

void neutral() { writeMotor(true, 0); writeMotor(false, 0); }

void fire(const String& name) {
  if (name == "DEPLOY")  { deployServo.write(120);  delay(700); deployServo.write(20); }
  else if (name == "LAUNCH")  { launchServo.write(150);  delay(400); launchServo.write(30); }
  else if (name == "RECOVER") { recoverServo.write(140); delay(1200); recoverServo.write(40); }
  else { Serial.println("ERR,UNKNOWN_ACTUATOR"); return; }
  Serial.print("ACK,"); Serial.println(name);
}

void handle(const String& s) {
  if (s.startsWith("M,")) {
    int c1 = s.indexOf(',', 2);
    if (c1 < 0) return;
    writeMotor(true, s.substring(2, c1).toInt());
    writeMotor(false, s.substring(c1 + 1).toInt());
    lastCmd = millis();
  } else if (s == "S") {
    neutral();
  } else if (s.startsWith("A,")) {
    neutral();                                    // never fire a mechanism while thrusting
    fire(s.substring(2));
    lastCmd = millis();
  }
}

void setup() {
  Serial.begin(115200);
#if OUTPUT_MODE_RC_PWM
  leftEsc.attach(LEFT_PIN); rightEsc.attach(RIGHT_PIN);
#else
  pinMode(LEFT_DIR, OUTPUT); pinMode(RIGHT_DIR, OUTPUT); pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
#endif
  deployServo.attach(DEPLOY_PIN); launchServo.attach(LAUNCH_PIN); recoverServo.attach(RECOVER_PIN);
  deployServo.write(20); launchServo.write(30); recoverServo.write(40);
  neutral();
  delay(2000);                                    // many ESCs need a neutral signal at power-up to arm
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { line.trim(); handle(line); line = ""; }
    else if (line.length() < 40) line += ch;
  }
  if (millis() - lastCmd > WATCHDOG_MS) neutral();
}
