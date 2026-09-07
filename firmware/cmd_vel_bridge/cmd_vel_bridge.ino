/*
 * tfmini_test.ino — verify the TFmini-S is wired and talking, before touching
 * the main firmware.
 *
 * Reads distance from the TFmini-S over UART2 and prints it once a second.
 * Wave your hand in front of the sensor: the number should change.
 *
 * Board:  ESP32 Dev Module (arduino-esp32 core 2.0.17)
 * Serial Monitor: 115200 baud
 *
 * ---------------------------------------------------------------------------
 * WIRING
 * ---------------------------------------------------------------------------
 *   TFmini Red   (5V)  -> ESP32 5V / VIN
 *   TFmini Black (GND) -> common GND
 *   TFmini Green (TX)  -> ESP32 GPIO 16   (this is ESP32 UART2 RX)
 *   TFmini White (RX)  -> ESP32 GPIO 17   (this is ESP32 UART2 TX)
 *
 * NOTE THE CROSS: sensor TX -> ESP32 RX (16), sensor RX -> ESP32 TX (17).
 * If you get no data, swapping 16 and 17 is the first thing to try.
 * Verify wire colours against the sensor label -- batches vary.
 * ---------------------------------------------------------------------------
 *
 * The TFmini-S defaults to 115200 baud UART and streams 9-byte frames:
 *   0x59 0x59  DistL DistH  StrengthL StrengthH  TempL TempH  Checksum
 * Distance (cm) = DistL + (DistH << 8).
 */

const int TFMINI_RX = 5;    // <- sensor TX (green)
const int TFMINI_TX = 18;   // -> sensor RX (white)

void setup() {
  Serial.begin(115200);                 // USB serial, to the Serial Monitor
  delay(500);

  // UART2 for the sensor.
  Serial2.begin(115200, SERIAL_8N1, TFMINI_RX, TFMINI_TX);

  Serial.println();
  Serial.println("=== TFmini-S read test ===");
  Serial.println("Distance should change as you move a target in front of it.");
  Serial.println("If it stays at 0 or prints nothing, check wiring / swap 16<->17.");
  Serial.println("-------------------------------------------------------------");
}

unsigned long lastPrint = 0;
int lastDistance = -1;

void loop() {
  // Look for a valid frame: two 0x59 header bytes, then read the rest.
  if (Serial2.available() >= 9) {
    if (Serial2.read() == 0x59) {
      if (Serial2.read() == 0x59) {
        uint8_t buf[7];
        for (int i = 0; i < 7; i++) buf[i] = Serial2.read();

        // Checksum: low byte of the sum of the first 8 bytes.
        uint16_t sum = 0x59 + 0x59;
        for (int i = 0; i < 6; i++) sum += buf[i];
        if ((sum & 0xFF) == buf[6]) {
          int distance = buf[0] + (buf[1] << 8);       // cm
          int strength = buf[2] + (buf[3] << 8);       // signal strength
          lastDistance = distance;

          // Print at most once every 250 ms so the monitor is readable.
          if (millis() - lastPrint > 250) {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.print(" cm   (strength ");
            Serial.print(strength);
            Serial.println(")");
            lastPrint = millis();
          }
        }
      }
    }
  }

  // If nothing has been read for a while, say so.
  if (millis() - lastPrint > 2000) {
    Serial.println("... no valid data (check wiring / TX-RX cross / baud)");
    lastPrint = millis();
  }
}

/*
 * ---------------------------------------------------------------------------
 * WHAT GOOD LOOKS LIKE
 *   "Distance: 47 cm (strength 1200)" and the number tracks your hand.
 *
 * TROUBLESHOOTING
 *   Prints "no valid data" forever
 *     -> TX/RX swapped: try wiring sensor TX to 17 and RX to 16, or just swap
 *        TFMINI_RX and TFMINI_TX in this sketch and re-upload.
 *     -> No power: check red on 5V, black on common GND.
 *   Distance reads 0 or jumps wildly
 *     -> Very close targets (<10 cm) or highly reflective/absorptive surfaces.
 *        Test against a wall ~50 cm away first.
 *   Strength very low (<100)
 *     -> Weak return; surface too dark, angled, or out of range. Normal at the
 *        edges; the /scan bridge will treat these as invalid later.
 * ---------------------------------------------------------------------------
 */
