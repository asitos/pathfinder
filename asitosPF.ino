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
// Derived from V-shape geometry: 65° inner angle, 32.5° half-angle, 5mm pitch.
// C0  = Left outermost | C7  = Left innermost
// C8  = Right innermost| C15 = Right outermost
// Adjust sign if your boards are physically mirrored.
const int WEIGHT[NUM_SENSORS] = {
  -188, -161, -134, -107, -81, -54, -27, -5,  // C0–C7  left wing
  5, 27, 54, 81, 107, 134, 161, 188           // C8–C15 right wing
};

// ── CALIBRATION STORAGE ───────────────────────────────────────────────────────
int calMin[NUM_SENSORS];
int calMax[NUM_SENSORS];

// ── PID TUNING (MIXED TRACK: ZIG-ZAG + LOOPS + SERPENTINE) ──────────────────
// Track segments (START→END):
//   1. Tight zig-zag column (~110–130°, short straights)
//   2. Figure-8 crossing (junction passthrough)
//   3. Rounded rectangle loop (moderate curves)
//   4. Wide serpentine U-turns (slow sweeping 180°)
// Kp/Kd balanced for zig-zag authority WITHOUT serpentine oscillation.
float Kp = 0.50;  // Enough pivot authority for 110° apexes; won't resonance-oscillate on wide sweeps
float Ki = 0.0;   // Zero integral — no lag on rapid zig-zag direction reversals
float Kd = 1.2;   // Damps apex exit overshoot; settles on serpentine long straights

// ── SPEED LIMITS ─────────────────────────────────────────────────────────────
#define BASE_SPEED 185  // Conservative cruise; short zig-zag links leave no room for momentum bleed
#define MAX_SPEED 255   // Outer wheel drive; sufficient for all turn types
#define MIN_SPEED -255  // Controlled inner-wheel reverse; handles 110° zig-zag without over-spin
#define TURN_SPEED 255  // Recovery pivot at junction exits and lost-line at zig-zag apexes

// ── JUNCTION DETECTION ───────────────────────────────────────────────────────
// If this many sensors or more are ON simultaneously → junction / loop entry.
#define JUNCTION_THRESHOLD 7

// ── STATE ────────────────────────────────────────────────────────────────────
float lastError = 0;
float integral = 0;
int lastPos = 0;  // last valid weighted position
bool lineWasLeft = false;

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
    // Inverted track: White line on Black surface
    // White line (~calMin) → 1000, Black floor (~calMax) → 0
    sensorNorm[i] = constrain(map(raw, calMax[i], calMin[i], 0, 1000), 0, 1000);
    sensorOn[i] = (sensorNorm[i] > 500);
    if (sensorOn[i]) count++;
  }
  return count;
}

// ─────────────────────────────────────────────────────────────────────────────
// WEIGHTED POSITION
// Returns tenths-of-mm from centre. Positive = line is to the right.
// Returns 0 if no sensors are on (caller handles lost-line separately).
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
  if (total == 0) return 0;  // caller checks count==0 before calling
  return (int)(wsum / total);
}

// ─────────────────────────────────────────────────────────────────────────────
// MOTOR DRIVER
// Positive = forward, Negative = reverse, 0 = active short brake.
// ─────────────────────────────────────────────────────────────────────────────
void setMotors(int left, int right) {
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
    // Active Short Brake (holding torque)
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
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
    // Active Short Brake (holding torque)
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
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
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  int active = readSensors();

  // ── JUNCTION / LOOP ENTRY ─────────────────────────────────────────────────
  // Many sensors active at once → wide cross or loop.
  // Strategy: go straight at reduced speed, reset integrator.
  if (active >= JUNCTION_THRESHOLD) {
    integral = 0;
    lastError = 0;
    setMotors(BASE_SPEED, BASE_SPEED);
    return;
  }

  // ── LOST LINE ─────────────────────────────────────────────────────────────
  if (active == 0) {
    // Spin toward the side the line was last seen on.
    if (lastPos > 0)
      setMotors(TURN_SPEED, -TURN_SPEED);  // spin right
    else
      setMotors(-TURN_SPEED, TURN_SPEED);  // spin left
    return;
  }

  // ── NORMAL TRACKING — PID ─────────────────────────────────────────────────
  int pos = computePosition();
  lastPos = pos;

  float error = (float)pos;

  integral += error;
  integral = constrain(integral, -8000.0f, 8000.0f);  // anti-windup

  float derivative = error - lastError;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = (int)(BASE_SPEED + correction);
  int rightSpeed = (int)(BASE_SPEED - correction);

  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);

  // ── SERIAL DEBUG (comment out before competition — adds ~2ms latency) ─────
  // Serial.print(pos); Serial.print('\t');
  // Serial.print(leftSpeed); Serial.print('\t');
  // Serial.println(rightSpeed);
}
