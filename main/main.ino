/*
  SOLAR-POWERED WASTE COLLECTOR
  Board: Arduino Uno R3 compatible

  Sensor layout (from driver's view):
    LEFT  JSN-SR04T: TRIG D2,  ECHO D3
    CENTER HC-SR04: TRIG D4,  ECHO D7
    RIGHT JSN-SR04T: TRIG D12, ECHO D13

  Joystick: VRX A0, VRY A1, +5V and GND
  L298N: ENA D5, IN1 D8, IN2 D9  (P1 = left motors)
         ENB D6, IN3 D10, IN4 D11 (P2 = right motors)
  Conveyor: A2 -> active-HIGH control input of a SEPARATE,
            appropriately rated motor switch / driver.
            Do NOT connect the conveyor motor to the Arduino pin.

  Serial Monitor: 9600 baud
  A = AUTO mode (supervised tests only)
  J = JOYSTICK / MANUAL mode
  C = Toggle conveyor ON/OFF in MANUAL mode
  S = Latch stop (software-only)
  R = Clear stop, return to MANUAL (conveyor OFF)
  H = Help

  NOTE: Ultrasonic echoes cannot identify garbage versus pool walls.
  This is an experimental algorithm; do not operate unattended.
*/

const byte JOY_X = A0;
const byte JOY_Y = A1;

const byte ENA = 5, IN1 = 8, IN2 = 9;
const byte ENB = 6, IN3 = 10, IN4 = 11;
const byte CONVEYOR_PIN = A2;

const byte L = 0, C = 1, R = 2;
const byte trigPins[3] = {2, 4, 12};
const byte echoPins[3] = {3, 7, 13};
const float minCM[3] = {25.0f, 2.0f, 25.0f};
const float maxCM[3] = {450.0f, 400.0f, 450.0f};

const int LOW_LIMIT = 350, HIGH_LIMIT = 670;
const bool FORWARD_IS_HIGH = true;
const bool LEFT_IS_LOW = true;

const byte PWM_MANUAL = 180;
const byte PWM_SEARCH = 145;
const byte PWM_ALIGN = 165;
const byte PWM_CAPTURE = 135;

// Prototype tuning values, not a debris-identification algorithm.
const float SIDE_ACQUIRE_MIN_CM = 35.0f;
const float SIDE_ACQUIRE_MAX_CM = 85.0f;
const float CENTER_CAPTURE_MIN_CM = 10.0f;
const float CENTER_CAPTURE_MAX_CM = 35.0f;
const float SIDE_HARD_STOP_CM = 30.0f;
const float CENTER_HARD_STOP_CM = 12.0f;

const unsigned long PING_GAP_MS = 75;   // >60 ms between any two pings
const unsigned long ECHO_TIMEOUT_US = 25000UL;
const unsigned long PRINT_GAP_MS = 750;
const unsigned long CONVEYOR_RUN_ON_MS = 3000;
const byte REQUIRED_HITS = 2;

float distanceCM[3] = {-1.0f, -1.0f, -1.0f};
byte validStreak[3] = {0, 0, 0};
unsigned long lastPingMs = 0, lastPrintMs = 0;
byte nextSensor = 0;

bool autoMode = false;       // Safe startup: manual control only
bool stopped = false;       // Software stop latch
bool manualConveyor = false;
bool conveyorOn = false;
bool centerEverSeen = false;
unsigned long lastCenterSeenMs = 0;
bool searching = false;
unsigned long searchStartMs = 0;
byte p1Pwm = 0, p2Pwm = 0;
const char* movement = "MANUAL STOP";

void setMotors(byte leftPwm, byte rightPwm) {
  // Each channel always runs FORWARD or is disabled.
  // Low on all inputs when disabled.
  p1Pwm = leftPwm;
  p2Pwm = rightPwm;
  digitalWrite(IN1, leftPwm ? HIGH : LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, rightPwm ? HIGH : LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftPwm);
  analogWrite(ENB, rightPwm);
}

void setConveyor(bool on) {
  conveyorOn = on;
  // Requires an active-HIGH driver input. Check YOUR driver/module.
  digitalWrite(CONVEYOR_PIN, on ? HIGH : LOW);
}

void stopEverything() {
  setMotors(0, 0);
  setConveyor(false);
}

// Return -1 when there is no echo or it is outside this sensor's range.
float readSensor(byte i) {
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(4);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPins[i], LOW);

  unsigned long us = pulseIn(echoPins[i], HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1.0f;
  float cm = us * 0.0343f / 2.0f;
  if (cm < minCM[i] || cm > maxCM[i]) return -1.0f;
  return cm;
}

void scanOneSensor() {
  unsigned long now = millis();
  if (now - lastPingMs < PING_GAP_MS) return;
  lastPingMs = now;
  byte i = nextSensor;
  distanceCM[i] = readSensor(i);
  if (distanceCM[i] >= 0) {
    if (validStreak[i] < 3) validStreak[i]++;
  } else {
    validStreak[i] = 0;
  }
  nextSensor = (nextSensor + 1) % 3;
}

bool valid(byte i) {
  return distanceCM[i] >= 0 && validStreak[i] >= REQUIRED_HITS;
}

bool inRange(byte i, float lo, float hi) {
  return valid(i) && distanceCM[i] >= lo && distanceCM[i] <= hi;
}

void printHelp() {
  Serial.println(F("Commands: A=Auto, J=Manual, C=Conveyor (manual),"));
  Serial.println(F("          S=Stop latch, R=Reset to manual, H=Help"));
}

void handleSerial() {
  while (Serial.available()) {
    char command = Serial.read();
    switch (command) {
      case 'S': case 's':
        stopped = true;
        autoMode = false;
        manualConveyor = false;
        searching = false;
        stopEverything();
        Serial.println(F("STOP LATCHED - use R to reset"));
        break;
      case 'R': case 'r':
        stopped = false;
        autoMode = false;
        manualConveyor = false;
        centerEverSeen = false;
        searching = false;
        stopEverything();
        Serial.println(F("Reset to MANUAL; center joystick"));
        break;
      case 'J': case 'j':
        if (!stopped) {
          autoMode = false;
          manualConveyor = false;
          searching = false;
          stopEverything();
          Serial.println(F("MANUAL mode"));
        }
        break;
      case 'A': case 'a':
        if (!stopped) {
          autoMode = true;
          manualConveyor = false;
          centerEverSeen = false;
          searching = false;
          stopEverything();
          Serial.println(F("AUTO enabled - supervise closely"));
        }
        break;
      case 'C': case 'c':
        if (!stopped && !autoMode) {
          manualConveyor = !manualConveyor;
          Serial.println(manualConveyor ? F("Conveyor MANUAL ON") : F("Conveyor MANUAL OFF"));
        }
        break;
      case 'H': case 'h': printHelp(); break;
      default: break;  // Ignore CR, LF and other characters.
    }
  }
}

void updateManual(int x, int y) {
  searching = false;
  setConveyor(manualConveyor);
  bool forward = FORWARD_IS_HIGH ? (y > HIGH_LIMIT) : (y < LOW_LIMIT);
  bool back = FORWARD_IS_HIGH ? (y < LOW_LIMIT) : (y > HIGH_LIMIT);
  bool left = LEFT_IS_LOW ? (x < LOW_LIMIT) : (x > HIGH_LIMIT);
  bool right = LEFT_IS_LOW ? (x > HIGH_LIMIT) : (x < LOW_LIMIT);

  if (back) { setMotors(0, 0); movement = "BACK DISABLED"; }
  else if (forward) { setMotors(PWM_MANUAL, PWM_MANUAL); movement = "FORWARD"; }
  else if (left) { setMotors(0, PWM_MANUAL); movement = "LEFT (P2)"; }
  else if (right) { setMotors(PWM_MANUAL, 0); movement = "RIGHT (P1)"; }
  else { setMotors(0, 0); movement = "MANUAL STOP"; }
}

void updateAuto() {
  unsigned long now = millis();
  bool left = inRange(L, SIDE_ACQUIRE_MIN_CM, SIDE_ACQUIRE_MAX_CM);
  bool right = inRange(R, SIDE_ACQUIRE_MIN_CM, SIDE_ACQUIRE_MAX_CM);
  bool center = inRange(C, CENTER_CAPTURE_MIN_CM, CENTER_CAPTURE_MAX_CM);

  // Conservative close-distance stop. Echoes could be debris OR pool walls.
  bool tooClose = (valid(L) && distanceCM[L] < SIDE_HARD_STOP_CM) ||
                  (valid(R) && distanceCM[R] < SIDE_HARD_STOP_CM) ||
                  (valid(C) && distanceCM[C] < CENTER_HARD_STOP_CM);
  if (tooClose) {
    searching = false;
    stopEverything();
    movement = "TOO CLOSE - HALT";
    return;
  }

  // Detect the object close to the conveyor's center line.
  if (center) {
    centerEverSeen = true;
    lastCenterSeenMs = now;
  }
  bool keepConveyor = centerEverSeen &&
                      (now - lastCenterSeenMs <= CONVEYOR_RUN_ON_MS);
  setConveyor(keepConveyor);

  if (center) {
    searching = false;
    // Gentle trim while moving the object toward the intake.
    if (left && !right) { setMotors(80, PWM_CAPTURE); movement = "CAPTURE LEFT"; }
    else if (right && !left) { setMotors(PWM_CAPTURE, 80); movement = "CAPTURE RIGHT"; }
    else { setMotors(PWM_CAPTURE, PWM_CAPTURE); movement = "CAPTURE CENTER"; }
    return;
  }
  if (left && right) {
    // Two echoes, unknown target: avoid assuming it is a piece of trash.
    searching = false;
    setMotors(0, 0);
    movement = "AMBIGUOUS - HALT";
    return;
  }
  if (left) {
    searching = false;
    setMotors(0, PWM_ALIGN); // P2 right propeller -> turn left
    movement = "ALIGN LEFT";
    return;
  }
  if (right) {
    searching = false;
    setMotors(PWM_ALIGN, 0); // P1 left propeller -> turn right
    movement = "ALIGN RIGHT";
    return;
  }

  // No candidate objects. Only search if at least one measurement provides
  // evidence of a non-close surface. No echo is NOT a clear-path signal.
  bool hasClearReturn = (valid(L) && distanceCM[L] > SIDE_ACQUIRE_MAX_CM) ||
                        (valid(R) && distanceCM[R] > SIDE_ACQUIRE_MAX_CM) ||
                        (valid(C) && distanceCM[C] > CENTER_CAPTURE_MAX_CM);
  if (!hasClearReturn) {
    searching = false;
    setMotors(0, 0);
    movement = "NO SAFE RANGE - HALT";
    return;
  }

  if (!searching) { searching = true; searchStartMs = now; }
  unsigned long phase = (now - searchStartMs) % 3800UL;
  if (phase < 1100UL) {
    setMotors(PWM_SEARCH, PWM_SEARCH);
    movement = "SEARCH FORWARD";
  } else if (phase < 1900UL) {
    setMotors(0, PWM_SEARCH);
    movement = "SEARCH SWEEP LEFT";
  } else if (phase < 3000UL) {
    setMotors(PWM_SEARCH, PWM_SEARCH);
    movement = "SEARCH FORWARD";
  } else {
    setMotors(PWM_SEARCH, 0);
    movement = "SEARCH SWEEP RIGHT";
  }
}

void printRange(const __FlashStringHelper* name, byte i) {
  Serial.print(name);
  if (valid(i)) { Serial.print(distanceCM[i], 1); Serial.print(F("cm")); }
  else Serial.print(F("NO ECHO"));
}

void statusReport(int x, int y) {
  Serial.print(autoMode ? F("AUTO ") : F("MANUAL "));
  if (stopped) Serial.print(F("STOP-LATCH "));
  printRange(F("L="), L); Serial.print(F(" | "));
  printRange(F("C="), C); Serial.print(F(" | "));
  printRange(F("R="), R); Serial.println();
  Serial.print(F("X=")); Serial.print(x);
  Serial.print(F(" Y=")); Serial.print(y);
  Serial.print(F(" | ")); Serial.print(movement);
  Serial.print(F(" | P1=")); Serial.print(p1Pwm);
  Serial.print(F(" P2=")); Serial.print(p2Pwm);
  Serial.print(F(" CONV=")); Serial.println(conveyorOn ? F("ON") : F("OFF"));
}

void setup() {
  // Outputs default to safe OFF (also use an external gate pulldown).
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(CONVEYOR_PIN, OUTPUT);
  stopEverything();
  for (byte i=0; i<3; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }
  Serial.begin(9600);
  Serial.println(F("3-SENSOR WASTE COLLECTOR - MANUAL STARTUP"));
  printHelp();
}

void loop() {
  handleSerial();
  scanOneSensor();
  int x = analogRead(JOY_X);
  int y = analogRead(JOY_Y);
  if (stopped) {
    stopEverything();
    movement = "STOP LATCHED";
  } else if (autoMode) {
    updateAuto();
  } else {
    updateManual(x, y);
  }
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_GAP_MS) {
    lastPrintMs = now;
    statusReport(x, y);
  }
}
