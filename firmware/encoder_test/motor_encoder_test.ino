/*
 * motor_encoder_test.ino — combined motor drive + encoder counting
 *
 * Runs the motors through a fixed test sequence while continuously counting
 * encoder pulses, and prints both together.
 *
 * WHY THIS TEST EXISTS
 * Hand-turning the wheels proves the encoders are wired correctly. It does NOT
 * prove they still work while the motors are running. Motor windings and their
 * power leads generate electrical noise that couples readily into nearby
 * signal wires, and encoder lines are a classic victim. This test catches that
 * before any odometry work depends on clean counts.
 *
 * Board:  ESP32 Dev Module (arduino-esp32 core 2.0.17)
 * Serial: 115200 baud
 *
 * ---------------------------------------------------------------------------
 * WIRING
 * ---------------------------------------------------------------------------
 *  TB6612FNG          ESP32            Encoders (LM393)      ESP32
 *    VCC      ......  3V3                VCC   ............  3V3  (NOT 5V)
 *    GND      ......  GND                GND   ............  GND
 *    PWMA     ......  GPIO 25            DO (left)  .......  GPIO 32
 *    AIN1     ......  GPIO 26            DO (right) .......  GPIO 4
 *    AIN2     ......  GPIO 27            AO         .......  unused
 *    PWMB     ......  GPIO 14
 *    BIN1     ......  GPIO 12          Battery + -> VM
 *    BIN2     ......  GPIO 13          Battery - -> GND (common)
 *    STBY     ......  GPIO 33
 *    AO1/AO2  ......  Motor A
 *    BO1/BO2  ......  Motor B
 *
 *  Encoders use INPUT_PULLUP on GPIO 32 and 4. GPIO 34-39 do NOT have internal
 *  pull-ups, which is why the encoders were moved off 34/35 -- an
 *  open-collector DO output never returns HIGH without a pull-up, so nothing
 *  is ever counted even though the module's own LED toggles correctly.
 *
 *  Encoder VCC is 3V3, not 5V: the DO output sits at the supply voltage, and
 *  5V would exceed the ESP32's 3.3V pin tolerance.
 *
 * ---------------------------------------------------------------------------
 * RUN IT WITH THE WHEELS OFF THE GROUND.
 * Upload with the LiPo DISCONNECTED (USB power only), then connect the battery.
 * ---------------------------------------------------------------------------
 */

// ---- Motor pins (PIN_ prefix avoids clashes with ESP32 core macros) --------
const int PIN_PWMA = 25, PIN_AIN1 = 26, PIN_AIN2 = 27;   // Motor A -> AO1/AO2
const int PIN_PWMB = 14, PIN_BIN1 = 12, PIN_BIN2 = 13;   // Motor B -> BO1/BO2
const int PIN_STBY = 33;

const int CH_A = 0, CH_B = 1;                 // LEDC PWM channels
const int PWM_FREQ = 1000, PWM_RES = 8;       // 1 kHz, 0-255
const int SPEED = 150;                        // modest, not full tilt

// ---- Encoder pins ----------------------------------------------------------
const int PIN_ENC_LEFT  = 32;
const int PIN_ENC_RIGHT = 4;

const int SLOTS_PER_REV  = 20;   // COUNT YOUR ACTUAL DISC
const int EDGES_PER_SLOT = 2;    // CHANGE interrupt -> 2 edges per slot

// ---- Encoder state ---------------------------------------------------------
volatile unsigned long leftCount  = 0;
volatile unsigned long rightCount = 0;

void IRAM_ATTR leftISR()  { leftCount++;  }
void IRAM_ATTR rightISR() { rightCount++; }

// ---- Reporting state -------------------------------------------------------
unsigned long lastReportMs  = 0;
unsigned long lastLeftSnap  = 0;
unsigned long lastRightSnap = 0;
const unsigned long REPORT_INTERVAL_MS = 250;

const char *phaseLabel = "idle";

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor outputs
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  ledcSetup(CH_A, PWM_FREQ, PWM_RES);  ledcAttachPin(PIN_PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);  ledcAttachPin(PIN_PWMB, CH_B);

  digitalWrite(PIN_STBY, HIGH);        // TB6612 is disabled while STBY is LOW

  // Encoder inputs -- INPUT_PULLUP is required for open-collector DO outputs.
  pinMode(PIN_ENC_LEFT,  INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT),  leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), rightISR, CHANGE);

  stopMotors();

  Serial.println();
  Serial.println("=== Motor + Encoder Test ===");
  Serial.println("WHEELS OFF THE GROUND.");
  Serial.print("Encoders: L=GPIO "); Serial.print(PIN_ENC_LEFT);
  Serial.print("  R=GPIO ");         Serial.println(PIN_ENC_RIGHT);
  Serial.print("Edges per revolution: ");
  Serial.println(SLOTS_PER_REV * EDGES_PER_SLOT);
  Serial.println("--------------------------------------------------");
  lastReportMs = millis();
}

// ---- Motor helpers ---------------------------------------------------------

void motorA(int dir, int spd) {        // dir: 1 fwd, -1 rev, 0 stop
  digitalWrite(PIN_AIN1, dir > 0);
  digitalWrite(PIN_AIN2, dir < 0);
  ledcWrite(CH_A, dir == 0 ? 0 : spd);
}

void motorB(int dir, int spd) {
  digitalWrite(PIN_BIN1, dir > 0);
  digitalWrite(PIN_BIN2, dir < 0);
  ledcWrite(CH_B, dir == 0 ? 0 : spd);
}

void stopMotors() {
  motorA(0, 0);
  motorB(0, 0);
}

// ---- Encoder reporting -----------------------------------------------------
// Reports continuously DURING each phase, not just at the end, so
// noise-induced miscounting is visible while the motors are actually running.

void reportWhileRunning(unsigned long durationMs) {
  const unsigned long start = millis();
  while (millis() - start < durationMs) {
    const unsigned long now = millis();
    if (now - lastReportMs >= REPORT_INTERVAL_MS) {

      noInterrupts();
      const unsigned long l = leftCount;
      const unsigned long r = rightCount;
      interrupts();

      const unsigned long dtMs   = now - lastReportMs;
      const unsigned long dLeft  = l - lastLeftSnap;
      const unsigned long dRight = r - lastRightSnap;

      const float edgesPerRev = (float)SLOTS_PER_REV * (float)EDGES_PER_SLOT;
      const float dtMin       = (dtMs / 1000.0f) / 60.0f;
      const float lRPM = dtMin > 0 ? (dLeft  / edgesPerRev) / dtMin : 0.0f;
      const float rRPM = dtMin > 0 ? (dRight / edgesPerRev) / dtMin : 0.0f;

      Serial.print("[");          Serial.print(phaseLabel);
      Serial.print("]  L=");      Serial.print(l);
      Serial.print(" (+");        Serial.print(dLeft);
      Serial.print(", ");         Serial.print(lRPM, 1);
      Serial.print(" RPM)   R="); Serial.print(r);
      Serial.print(" (+");        Serial.print(dRight);
      Serial.print(", ");         Serial.print(rRPM, 1);
      Serial.println(" RPM)");

      lastReportMs  = now;
      lastLeftSnap  = l;
      lastRightSnap = r;
    }
  }
}

// ---- Test sequence ---------------------------------------------------------

void loop() {
  phaseLabel = "A fwd";
  motorA(1, SPEED);   reportWhileRunning(2000);
  stopMotors();
  phaseLabel = "stop";  reportWhileRunning(1000);

  phaseLabel = "A rev";
  motorA(-1, SPEED);  reportWhileRunning(2000);
  stopMotors();
  phaseLabel = "stop";  reportWhileRunning(1000);

  phaseLabel = "B fwd";
  motorB(1, SPEED);   reportWhileRunning(2000);
  stopMotors();
  phaseLabel = "stop";  reportWhileRunning(1000);

  phaseLabel = "B rev";
  motorB(-1, SPEED);  reportWhileRunning(2000);
  stopMotors();
  phaseLabel = "stop";  reportWhileRunning(1000);

  phaseLabel = "both fwd";
  motorA(1, SPEED); motorB(1, SPEED);  reportWhileRunning(3000);
  stopMotors();
  phaseLabel = "stop";  reportWhileRunning(2000);
}

/*
 * ---------------------------------------------------------------------------
 * WHAT GOOD LOOKS LIKE
 * ---------------------------------------------------------------------------
 *  [A fwd]   L delta climbs steadily, R delta stays ~0
 *  [A rev]   L delta climbs AGAIN -- a single-channel encoder cannot sense
 *            direction, so the count rises in reverse too. Expected.
 *  [B fwd]   R delta climbs, L delta ~0
 *  [both]    both deltas climb, at similar rates if the motors are matched
 *  [stop]    both deltas return to 0
 *
 *  RPM should be roughly steady within a phase rather than jumping around.
 *
 * ---------------------------------------------------------------------------
 * WARNING SIGNS
 * ---------------------------------------------------------------------------
 *  Counts climbing during [stop]
 *      -> electrical noise being counted as pulses. Fixes in order: route
 *         encoder wires away from motor power leads; keep them short; ensure
 *         encoder and motor grounds share a solid common connection; if it
 *         persists, a 0.1 uF capacitor from each DO line to GND damps it.
 *
 *  The wrong side counts (driving A moves the R counter)
 *      -> the two DO wires are swapped at the ESP32.
 *
 *  Erratic only while motors run, clean when hand-turned
 *      -> classic motor noise coupling. Same fixes as above.
 *
 *  RPM differs noticeably between motors at the same PWM
 *      -> normal for cheap gearmotors. Record it; closed-loop speed control
 *         using these encoders is what corrects it later. (This difference is
 *         also why MIN_PWM had to be raised to 90 in cmd_vel_bridge.)
 *
 * ---------------------------------------------------------------------------
 * RECORD THIS NUMBER
 * ---------------------------------------------------------------------------
 * With the motors OFF, turn one wheel exactly one full revolution by hand and
 * note how much the count rises (expect SLOTS_PER_REV * EDGES_PER_SLOT, i.e.
 * ~40 for a 20-slot disc). That edges-per-revolution figure, together with the
 * wheel diameter (0.045 m), is what converts pulses into distance travelled
 * for odometry. Record it in docs/hardware.md.
 * ---------------------------------------------------------------------------
 */
