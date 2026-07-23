/*
 * cmd_vel_bridge.ino — ESP32 side of the Pi <-> ESP32 motor link
 *
 * Receives velocity commands from the Raspberry Pi over USB serial and drives
 * the two motors through the TB6612FNG. Replaces the fixed test sequence: the
 * ESP32 no longer decides what to do, it obeys commands from ROS 2.
 *
 * Board:  ESP32 Dev Module (arduino-esp32 core 2.0.17)
 * Serial: 115200 baud
 *
 * ---------------------------------------------------------------------------
 * PROTOCOL  (Pi -> ESP32), one command per line:
 *
 *     V <linear_m_s> <angular_rad_s>\n
 *
 *   "V 0.20 0.00"   drive forward at 0.2 m/s
 *   "V 0.00 0.50"   rotate left in place
 *   "V -0.15 0.00"  reverse
 *   "V 0.00 0.00"   stop
 *
 * Mirrors the planned ultrasonic uplink format ("U <d0> ... <d4>") so the link
 * speaks one consistent line-based protocol in both directions.
 *
 * ---------------------------------------------------------------------------
 * SAFETY — COMMAND WATCHDOG
 * If no valid command arrives for CMD_TIMEOUT_MS the motors stop. Without it,
 * the robot keeps driving at its last commanded speed if the Pi crashes, the
 * ROS node dies, or the USB cable is knocked out. Not optional on a machine
 * that moves under its own power.
 * ---------------------------------------------------------------------------
 *
 * WIRING
 *   TB6612FNG        ESP32              Encoders (not used by this sketch)
 *     VCC   .......  3V3                  left  DO -> GPIO 32
 *     GND   .......  GND                  right DO -> GPIO 4
 *     PWMA  .......  GPIO 25
 *     AIN1  .......  GPIO 26            Battery + -> VM
 *     AIN2  .......  GPIO 27            Battery - -> GND (common)
 *     PWMB  .......  GPIO 14
 *     BIN1  .......  GPIO 12
 *     BIN2  .......  GPIO 13
 *     STBY  .......  GPIO 33
 *     AO1/AO2 -> Motor A (treated as LEFT)
 *     BO1/BO2 -> Motor B (treated as RIGHT)
 */

// ---- Pin map (PIN_ prefix avoids clashes with ESP32 core macros) -----------
const int PIN_PWMA = 25, PIN_AIN1 = 26, PIN_AIN2 = 27;   // Motor A -> AO1/AO2
const int PIN_PWMB = 14, PIN_BIN1 = 12, PIN_BIN2 = 13;   // Motor B -> BO1/BO2
const int PIN_STBY = 33;

const int CH_A = 0, CH_B = 1;                 // LEDC PWM channels
const int PWM_FREQ = 1000, PWM_RES = 8;       // 1 kHz, 0-255

// ---- Robot geometry / calibration ------------------------------------------
//
// MEASURED VALUES (update if the chassis changes):
//
//   Wheel base ......... ~13 cm  = 0.13 m
//   Wheel diameter ..... ~4.5 cm = 0.045 m
//   Measured speed ..... ~108 RPM at PWM 150, wheels OFF the ground
//
// Derivation of MAX_WHEEL_SPEED (speed at full PWM 255):
//   RPM at 255      = 108 * (255 / 150)      = ~184 RPM
//   rev per second  = 184 / 60               = ~3.06 rev/s
//   circumference   = pi * 0.045             = ~0.141 m
//   speed           = 3.06 * 0.141           = ~0.43 m/s
//
// NOTE: 108 RPM was a NO-LOAD measurement (wheels in the air). With the
// robot's weight on the wheels the real figure will be lower -- likely
// 0.30-0.35 m/s. If commanded speeds come out consistently faster than the
// robot actually travels, reduce MAX_WHEEL_SPEED to match reality.

const float WHEEL_BASE_M    = 0.13f;    // distance between wheel centres, m
const float WHEEL_DIAM_M    = 0.045f;   // wheel diameter, m (reference)
const float MAX_WHEEL_SPEED = 0.43f;    // m/s at full PWM -- see note above

// Below a certain duty cycle the motors buzz but do not turn (static
// friction). Any non-zero command is lifted to at least this value.
const int MIN_PWM = 90;

// ---- Watchdog ---------------------------------------------------------------
const unsigned long CMD_TIMEOUT_MS = 500;

// ---- State ------------------------------------------------------------------
unsigned long lastCmdMs = 0;
String rxBuffer = "";
bool stoppedByWatchdog = false;

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  ledcSetup(CH_A, PWM_FREQ, PWM_RES);  ledcAttachPin(PIN_PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);  ledcAttachPin(PIN_PWMB, CH_B);

  digitalWrite(PIN_STBY, HIGH);        // TB6612 is disabled while STBY is LOW

  stopMotors();
  rxBuffer.reserve(64);
  lastCmdMs = millis();

  Serial.println();
  Serial.println("# cmd_vel_bridge ready");
  Serial.print  ("# wheel base "); Serial.print(WHEEL_BASE_M, 3);
  Serial.print  (" m, max wheel speed "); Serial.print(MAX_WHEEL_SPEED, 2);
  Serial.println(" m/s");
  Serial.println("# send:  V <linear m/s> <angular rad/s>");
  Serial.println("# e.g.   V 0.2 0.0");
}

void loop() {
  // --- Read complete lines from the Pi (or the Serial Monitor) -------------
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuffer.length() > 0) {
        handleLine(rxBuffer);
        rxBuffer = "";
      }
    } else if (rxBuffer.length() < 63) {
      rxBuffer += c;
    }
  }

  // --- Watchdog: stop if commands have gone quiet ---------------------------
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    if (!stoppedByWatchdog) {
      stopMotors();
      stoppedByWatchdog = true;
      Serial.println("# watchdog: no command, motors stopped");
    }
  }
}

// Parse "V <linear> <angular>"
void handleLine(const String &line) {
  if (line.length() < 1) return;
  if (line.charAt(0) != 'V' && line.charAt(0) != 'v') return;

  int firstSpace = line.indexOf(' ');
  if (firstSpace < 0) return;
  int secondSpace = line.indexOf(' ', firstSpace + 1);
  if (secondSpace < 0) return;

  float linear  = line.substring(firstSpace + 1, secondSpace).toFloat();
  float angular = line.substring(secondSpace + 1).toFloat();

  applyTwist(linear, angular);

  lastCmdMs = millis();
  stoppedByWatchdog = false;

  Serial.print("# ok  lin="); Serial.print(linear, 3);
  Serial.print("  ang=");     Serial.println(angular, 3);
}

// Differential-drive kinematics: split a body twist into two wheel speeds.
void applyTwist(float linear, float angular) {
  float left  = linear - (angular * WHEEL_BASE_M / 2.0f);
  float right = linear + (angular * WHEEL_BASE_M / 2.0f);

  // Motor A is treated as LEFT, motor B as RIGHT.
  // If the robot turns the wrong way, swap these two lines.
  setMotorA(left);
  setMotorB(right);
}

int speedToPwm(float speedMs) {
  float mag = fabs(speedMs);
  if (mag < 0.001f) return 0;
  int pwm = (int)((mag / MAX_WHEEL_SPEED) * 255.0f);
  if (pwm > 255)     pwm = 255;
  if (pwm < MIN_PWM) pwm = MIN_PWM;
  return pwm;
}

void setMotorA(float speedMs) {
  int pwm = speedToPwm(speedMs);
  digitalWrite(PIN_AIN1, speedMs > 0);
  digitalWrite(PIN_AIN2, speedMs < 0);
  ledcWrite(CH_A, pwm);
}

void setMotorB(float speedMs) {
  int pwm = speedToPwm(speedMs);
  digitalWrite(PIN_BIN1, speedMs > 0);
  digitalWrite(PIN_BIN2, speedMs < 0);
  ledcWrite(CH_B, pwm);
}

void stopMotors() {
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); ledcWrite(CH_A, 0);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); ledcWrite(CH_B, 0);
}

/*
 * ---------------------------------------------------------------------------
 * BENCH TEST WITHOUT THE PI
 * ---------------------------------------------------------------------------
 * 1. Wheels OFF the ground. Upload with the LiPo disconnected, then connect
 *    the battery to VM/GND.
 * 2. Open Serial Monitor at 115200.
 * 3. IMPORTANT: set the line-ending dropdown (bottom of the Serial Monitor) to
 *    "Newline". Without a line terminator the ESP32 never sees a complete
 *    command and nothing happens.
 * 4. Type commands and press Enter:
 *        V 0.2 0.0     both wheels forward
 *        V 0.0 0.5     spin in place (wheels counter-rotate)
 *        V -0.15 0.0   both wheels reverse
 *        V 0.0 0.0     stop
 *
 * Each typed command gives a SHORT BURST then a stop, printing
 * "# watchdog: no command, motors stopped". That is correct -- the watchdog
 * halts 500 ms after the last command. The Pi streams continuously, so real
 * motion is smooth.
 *
 * WHAT TO CHECK
 *   * "# ok lin=... ang=..." echoes back -> the line parsed. Nothing at all
 *     means the line ending is not set to Newline.
 *   * On "V 0.2 0.0", do BOTH wheels turn the same physical direction? On a
 *     differential drive the motors are mirrored, so one is commonly wired
 *     backwards. If one spins the wrong way, SWAP THAT MOTOR'S TWO OUTPUT
 *     WIRES at the driver (BO1 <-> BO2). Fix it in hardware so "forward"
 *     genuinely means forward once ROS 2 is driving.
 *   * On "V 0.0 0.5" the wheels should counter-rotate. If they both turn the
 *     same way, the left/right assignment in applyTwist is swapped.
 *
 * IF NOTHING MOVES AT ALL
 *   * STBY not HIGH (most common cause).
 *   * Battery ground and ESP32 ground not truly common.
 *   * Battery not connected to VM.
 *
 * ---------------------------------------------------------------------------
 * CALIBRATION CHECK (do this once the robot is on the floor)
 * ---------------------------------------------------------------------------
 * With a tape measure, command a known speed for a known time and compare:
 *   "V 0.2 0.0" streamed for 5 s should cover ~1.0 m.
 * Travelling noticeably SHORT means MAX_WHEEL_SPEED is set too high -- the
 * no-load 108 RPM figure overestimates loaded speed. Lower it until commanded
 * and actual distances agree.
 *
 * Similarly for rotation: "V 0.0 0.5" for ~12.6 s should be one full turn
 * (2*pi / 0.5). If the robot over- or under-rotates, WHEEL_BASE_M is off --
 * re-measure between the wheel centres.
 * ---------------------------------------------------------------------------
 */
