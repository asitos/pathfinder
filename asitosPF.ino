// ─────────────────────────────────────────────────────────────────────────────
// Pathfinder LFR — asitosPF
// Hardware : Arduino Nano + TB6612FNG + CD74HC4067 + 2×8-ch IR arrays (V 65°)
// Algorithm: Weighted-position PID with junction passthrough and lost-line recovery
// ─────────────────────────────────────────────────────────────────────────────

// ── PIN MAP ──────────────────────────────────────────────────────────────────

// TB6612FNG
#define PWMA 5  // Left motor speed  (PWM, Timer0)
#define AIN1 4  // Left motor dir A
#define AIN2 3  // Left motor dir B
#define PWMB 6  // Right motor speed (PWM, Timer0)
#define BIN1 7  // Right motor dir A
#define BIN2 8  // Right motor dir B
#define STBY 9  // Driver standby — HIGH = active

// CD74HC4067 MUX
#define MUX_S0 10
#define MUX_S1 11
#define MUX_S2 12
#define MUX_S3 13
#define MUX_SIG A0  // Analog signal in

// ── SENSOR CONFIG ────────────────────────────────────────────────────────────

#define NUM_SENSORS 16

// Lateral weights in tenths-of-mm from robot centerline.
// Forward-pointing Arrowhead (^) V-array:
//   Center sensors (C7, C8) = Apex (20cm ahead of drive axle)
//   Wing tip sensors (C0, C15) = Swept-back tips (15cm ahead of drive axle, 5cm back from apex)
//   Inner angle = 65°–70°
// C0  = Left outermost | C7  = Left innermost (Apex)
// C8  = Right innermost (Apex)| C15 = Right outermost
const int WEIGHT[NUM_SENSORS] = {
  -188, -161, -134, -107, -81, -54, -27, -5,  // C0–C7  left wing (swept back from apex)
  5, 27, 54, 81, 107, 134, 161, 188           // C8–C15 right wing (swept back from apex)
};

// ── CALIBRATION STORAGE ───────────────────────────────────────────────────────
int calMin[NUM_SENSORS];
int calMax[NUM_SENSORS];

// ── PID & SPEED TUNING (Tunable live via Serial Monitor) ─────────────────────
// Controlled cruise speed prevents overshooting short links in zigzags/loops.
// Dynamic base speed bleeds to 0 on sharp turns to execute pure in-place pivots.
float Kp = 0.70;  // Proportional gain (command: p <val>)
float Ki = 0.0;   // Zero integral: avoids phase lag / overshoot (command: i <val>)
float Kd = 1.8;   // Damping derivative: stops oscillation (command: d <val>)

int baseSpeed = 220;  // Cruise speed on straights (command: s <val>)
int maxSpeed = 255;   // Maximum wheel forward speed PWM
int minSpeed = -200;  // Maximum counter-rotation reverse PWM for tank turns
int turnSpeed = 220;  // Atomic recovery spin PWM (command: t <val>)

// ── JUNCTION DETECTION ───────────────────────────────────────────────────────
// Only trigger on full perpendicular cross lines (not diagonal acute angle apexes)
#define JUNCTION_THRESHOLD 12

// ── MOTOR SPEED BALANCE / TRIM ───────────────────────────────────────────────
// Adjust to equalize wheel speeds if one motor is physically faster/slower.
// Tunable live via Serial: "l 0.95" or "r 1.0"
float leftScale = 1.00f;
float rightScale = 1.00f;

// ── STATE ────────────────────────────────────────────────────────────────────
float lastError = 0;
float integral = 0;
int lastPos = 0;         // last valid weighted position (Pololu-style memory)
int latchedTurnDir = 0;  // +1 = Right, -1 = Left (latched to prevent left-right shaking)
bool lineWasLeft = false;

// ─────────────────────────────────────────────────────────────────────────────
// LIVE SERIAL COMMAND PARSER
// Format: letter followed by number (e.g. "p 0.8", "d 2.0", "s 105", "t 190")
// Send '?' or 'h' to print current configuration.
// ─────────────────────────────────────────────────────────────────────────────
void checkSerialCommands() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  if (cmd == ' ' || cmd == '\r' || cmd == '\n') return;
  float val = Serial.parseFloat();
  switch (cmd) {
    case 'p':
    case 'P': Kp = val; break;
    case 'd':
    case 'D': Kd = val; break;
    case 'i':
    case 'I': Ki = val; break;
    case 's':
    case 'S': baseSpeed = (int)val; break;
    case 't':
    case 'T': turnSpeed = (int)val; break;
    case 'l':
    case 'L': leftScale = val; break;
    case 'r':
    case 'R': rightScale = val; break;
    case '?':
    case 'h':
    case 'H': break;
    default: return;
  }
  Serial.print(F("CFG: Kp="));
  Serial.print(Kp);
  Serial.print(F(" Kd="));
  Serial.print(Kd);
  Serial.print(F(" Ki="));
  Serial.print(Ki, 4);
  Serial.print(F(" Base="));
  Serial.print(baseSpeed);
  Serial.print(F(" Turn="));
  Serial.print(turnSpeed);
  Serial.print(F(" LScale="));
  Serial.print(leftScale);
  Serial.print(F(" RScale="));
  Serial.println(rightScale);
}

// ─────────────────────────────────────────────────────────────────────────────
// MUX: select channel, short settle, read analog
// ─────────────────────────────────────────────────────────────────────────────
int muxRead(uint8_t ch) {
  digitalWrite(MUX_S0, ch & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
  delayMicroseconds(10);
  return analogRead(MUX_SIG);
}

// ─────────────────────────────────────────────────────────────────────────────
// CALIBRATION
// Robot wiggles left-right for 4 s; user slides it over the line & floor.
// Stores per-sensor min (white) and max (black) raw ADC values.
// ─────────────────────────────────────────────────────────────────────────────
void calibrate() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 1023;
    calMax[i] = 0;
  }

  // Auto-sweep with wide arc and higher speed (PWM 135) for full V-wing coverage & high torque
  const int SWEEP_SPEED = 135;

  // Step 1: Turn Left (450ms)
  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  unsigned long t = millis();
  while (millis() - t < 450) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = muxRead(i);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
  }

  // Deadband pause: eliminates reverse back-EMF spike that triggers Brownout Reset
  setMotors(0, 0);
  delay(40);

  // Step 2: Wide Turn Right (900ms)
  setMotors(SWEEP_SPEED, -SWEEP_SPEED);
  t = millis();
  while (millis() - t < 900) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = muxRead(i);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
  }

  setMotors(0, 0);
  delay(40);

  // Step 3: Wide Turn Left (900ms)
  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  t = millis();
  while (millis() - t < 900) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = muxRead(i);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
  }

  setMotors(0, 0);
  delay(40);

  // Step 4: Wide Turn Right (900ms)
  setMotors(SWEEP_SPEED, -SWEEP_SPEED);
  t = millis();
  while (millis() - t < 900) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = muxRead(i);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
  }

  setMotors(0, 0);
  delay(40);

  // Step 5: Return to Center (450ms)
  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  t = millis();
  while (millis() - t < 450) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = muxRead(i);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
  }

  setMotors(0, 0);

  // Fallback guard: if any sensor missed the line, use verified defaults
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (calMax[i] - calMin[i] < 100) {
      calMin[i] = 200;
      calMax[i] = 880;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// READ + PROCESS SENSORS
// Returns: number of sensors currently ON (seeing black line)
// Writes normalised 0–1000 value and boolean onLine[] array.
// ─────────────────────────────────────────────────────────────────────────────
int sensorNorm[NUM_SENSORS];
bool sensorOn[NUM_SENSORS];

int readSensors() {
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = muxRead(i);
    // Standard track: Black line on White surface
    // White floor (~calMin) → 0, Black line (~calMax) → 1000
    sensorNorm[i] = constrain(map(raw, calMin[i], calMax[i], 0, 1000), 0, 1000);
    sensorOn[i] = (sensorNorm[i] > 400);  // 400 margin prevents line dropouts on shiny tape
    if (sensorOn[i]) count++;
  }
  return count;
}

// ─────────────────────────────────────────────────────────────────────────────
// WEIGHTED POSITION
// Returns tenths-of-mm from centre. Positive = line is to the right.
// Pololu QTR memory: if line is lost, retains extreme edge instead of false center.
// ─────────────────────────────────────────────────────────────────────────────
int computePosition() {
  long wsum = 0;
  int total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorOn[i]) {
      wsum += (long)WEIGHT[i] * sensorNorm[i];
      total += sensorNorm[i];
    }
  }
  if (total == 0) {
    return (lastPos >= 0) ? WEIGHT[NUM_SENSORS - 1] : WEIGHT[0];
  }
  lastPos = (int)(wsum / total);
  return lastPos;
}

// ─────────────────────────────────────────────────────────────────────────────
// MOTOR DRIVER
// Positive = forward, Negative = reverse, 0 = coast.
// Applies speed scaling factors to equalize physical motor differences.
// ─────────────────────────────────────────────────────────────────────────────
void setMotors(int left, int right) {
  left = (int)(left * leftScale);
  right = (int)(right * rightScale);

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Left (Channel A)
  if (left > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (left < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, abs(left));

  // Right (Channel B)
  if (right > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (right < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, abs(right));
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  // Motor driver
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  setMotors(0, 0);

  // MUX
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  Serial.begin(115200);
  delay(100);

  // ── PHASE 1: 5-Second Countdown to Calibration ───────────────────────────
  Serial.println(F("\n>>> POWER ON <<<"));
  Serial.println(F("Place robot on line. Calibration begins in 5 seconds..."));
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.print(F("... "));
    // Motor click/blip each second as audible countdown
    setMotors(60, 60);
    delay(25);
    setMotors(0, 0);
    delay(975);
  }
  Serial.println(F("\n>>> AUTO-CALIBRATING (Sweeping across line)... <<<"));
  calibrate();
  Serial.println(F(">>> Calibration complete! <<<"));

  // ── PHASE 2: 4-Second Countdown to Race Launch ────────────────────────────
  Serial.println(F("Place robot on START line. Racing starts in 4 seconds..."));
  for (int i = 4; i > 0; i--) {
    Serial.print(i);
    Serial.print(F("... "));
    // Motor click/blip
    setMotors(70, 70);
    delay(30);
    setMotors(0, 0);
    delay(970);
  }

  Serial.println(F("\n>>> GO! <<<"));
  Serial.println(F("Live Serial commands: p<val>, d<val>, i<val>, s<baseSpeed>, t<turnSpeed>, l<leftScale>, r<rightScale>, ?"));
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  checkSerialCommands();

  int active = readSensors();

  // ── JUNCTION / CROSSING DETECTION ─────────────────────────────────────────
  // A true perpendicular crossing activates BOTH the far-left and far-right wings.
  // Diagonal acute angles or 90° corners only light up one side.
  bool leftWingActive = (sensorOn[0] || sensorOn[1] || sensorOn[2]);
  bool rightWingActive = (sensorOn[13] || sensorOn[14] || sensorOn[15]);
  if (active >= JUNCTION_THRESHOLD && leftWingActive && rightWingActive) {
    integral = 0;
    lastError = 0;
    setMotors(baseSpeed, baseSpeed);
    return;
  }

  // ── ATOMIC LOST-LINE RECOVERY (ESP32-STYLE WITH 20CM NOSE PULL-BACK) ─────
  if (active == 0) {
    // Step 1: Brief straight-back pulse (45ms)
    // Pulls 20cm nose back ~2cm toward vertex so in-place spin stays on tape
    setMotors(-160, -160);
    delay(45);

    // Step 2: Atomic recovery spin (like ESP32 while loop) in direction of last seen line
    int spinLeft = (latchedTurnDir >= 0) ? turnSpeed : -turnSpeed;
    int spinRight = (latchedTurnDir >= 0) ? -turnSpeed : turnSpeed;
    setMotors(spinLeft, spinRight);

    // Tight spin loop until line is re-acquired (zero state-machine jitter)
    while (readSensors() == 0) {
      // spinning in place
    }

    // Step 3: Corner alignment lock
    // Continue in-place spin until line swings toward center (|pos| <= 55)
    // Ensures heading is aligned with the new path before driving forward
    unsigned long alignStart = millis();
    while (millis() - alignStart < 150) {
      readSensors();
      int p = computePosition();
      if (abs(p) <= 55) break;
      setMotors(spinLeft, spinRight);
    }

    // Re-acquired & aligned! Re-arm PID without derivative kick
    lastError = (float)lastPos;
    integral = 0;
    return;
  }

  int pos = computePosition();

  // Latch turn direction: remember which side line was on
  if (pos > 15) latchedTurnDir = 1;         // Line to right
  else if (pos < -15) latchedTurnDir = -1;  // Line to left

  float error = (float)pos;

  // Smooth 1st-order filter eliminates photodiode edge-jitter on angled lines
  float smoothError = 0.75f * error + 0.25f * lastError;

  integral += smoothError;
  integral = constrain(integral, -4000.0f, 4000.0f);  // anti-windup

  float derivative = smoothError - lastError;
  lastError = smoothError;

  float absErr = abs(smoothError);

  // Progressive P-gain: smooth on center, sharp torque boost on tight bends
  float pTerm = Kp * smoothError;
  if (absErr > 25.0f) {
    float excess = absErr - 25.0f;
    pTerm += (smoothError > 0 ? 1.0f : -1.0f) * (excess * 0.70f);
  }

  float correction = pTerm + Ki * integral + Kd * derivative;

  // DYNAMIC BASE SPEED:
  // On straights (absErr < 20): full cruise speed (baseSpeed).
  // On sharp turns: forward speed smoothly bleeds to 0 as error grows.
  // When error > 75: base speed drops to 0 so wheels execute a pure in-place
  // pivot (+180 outer, -180 inner) without driving forward off the corner.
  int currentBase = baseSpeed;
  if (absErr > 20.0f) {
    currentBase = baseSpeed - (int)((absErr - 20.0f) * 1.5f);
    if (currentBase < 0) currentBase = 0;
  }

  int leftSpeed = (int)(currentBase + correction);
  int rightSpeed = (int)(currentBase - correction);

  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  setMotors(leftSpeed, rightSpeed);
}
