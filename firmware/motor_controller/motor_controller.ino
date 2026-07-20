// RMA robot – ESP32 motor firmware v0.1
// Protocol: "M <left> <right>\n", values -255..255. Stops if silent >500ms.

// --- Pin assignments (TB6612) — we lock these in now, wire to match later ---
#define PWMA 25   // left motor speed
#define AIN1 26   // left motor direction
#define AIN2 27
#define PWMB 14   // right motor speed
#define BIN1 12
#define BIN2 13
// STBY on the TB6612 goes to 3.3V directly (always enabled)

#define MAX_DUTY 170          // ~67% of 255 — protects 6V motors on 7.4V battery
#define TIMEOUT_MS 500

unsigned long lastCmd = 0;

void setMotor(int pwmPin, int in1, int in2, int val) {
  val = constrain(val, -MAX_DUTY, MAX_DUTY);
  digitalWrite(in1, val >= 0 ? HIGH : LOW);
  digitalWrite(in2, val >= 0 ? LOW  : HIGH);
  analogWrite(pwmPin, abs(val));
}

void stopMotors() {
  setMotor(PWMA, AIN1, AIN2, 0);
  setMotor(PWMB, BIN1, BIN2, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  stopMotors();
  Serial.println("Motor firmware v0.1 ready");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("M")) {
      int l, r;
      if (sscanf(line.c_str(), "M %d %d", &l, &r) == 2) {
        setMotor(PWMA, AIN1, AIN2, l);
        setMotor(PWMB, BIN1, BIN2, r);
        lastCmd = millis();
        Serial.printf("OK L=%d R=%d\n", l, r);
      }
    }
  }
  if (millis() - lastCmd > TIMEOUT_MS) stopMotors();
}
