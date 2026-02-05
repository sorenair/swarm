// ESP32 + HC-SR04 + LED control + L298N motor control
// Notes:
// - Power HC-SR04 at 5 V. Shift ECHO down to 3.3 V with a divider.
// - L298N: ENA is PWM for speed, IN1/IN2 set direction.
// - Commands over USB serial (115200):
//     LED 1            -> LED on
//     LED 0            -> LED off
//     MOTOR 0          -> stop
//     MOTOR <0-255> FWD|REV

// ---------- Pins (edit if your wiring differs) ----------
#define TRIG_PIN   5
#define ECHO_PIN   18
#define LED_PIN    23

// L298N, channel A
#define ENA_PIN    33   // PWM
#define IN1_PIN    26
#define IN2_PIN    27

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  motorStop();
}

// ---------- Helpers ----------
void motorStop() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 0);
}

void motorDrive(uint8_t duty, bool forward) {
  // duty: 0..255
  digitalWrite(IN1_PIN, forward ? HIGH : LOW);
  digitalWrite(IN2_PIN, forward ? LOW  : HIGH);
  analogWrite(ENA_PIN, duty);
}

// Measure distance in cm. Returns 0 if no echo.
float measure_cm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long us = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout
  if (us <= 0) return 0.0f;
  return (us / 2.0f) * 0.0343f; // cm
}

void handleCommand(const String& line) {
  if (line.startsWith("LED")) {
    int val = line.endsWith("1") ? 1 : 0;
    digitalWrite(LED_PIN, val ? HIGH : LOW);
    Serial.print("{\"ack\":\"LED\",\"state\":");
    Serial.print(val);
    Serial.println("}");
    return;
  }

  if (line.startsWith("MOTOR")) {
    int spd = 0; char dir[8] = {0};
    int n = sscanf(line.c_str(), "MOTOR %d %7s", &spd, dir);
    spd = constrain(spd, 0, 255);

    if (n == 1 || spd == 0) {
      motorStop();
      Serial.println("{\"ack\":\"MOTOR\",\"state\":\"STOP\",\"spd\":0}");
      return;
    }

    if (n == 2) {
      bool fwd = (strcmp(dir, "FWD") == 0);
      bool rev = (strcmp(dir, "REV") == 0);
      if (fwd || rev) {
        motorDrive((uint8_t)spd, fwd);
        Serial.print("{\"ack\":\"MOTOR\",\"state\":\"");
        Serial.print(fwd ? "FWD" : "REV");
        Serial.print("\",\"spd\":");
        Serial.print(spd);
        Serial.println("}");
      }
    }
  }
}

// ---------- Main loop ----------
void loop() {
  // 1) Measurement and output, 1 Hz
  float cm = measure_cm();
  if (cm <= 0.0f) {
    Serial.println("{\"ok\":false}");
  } else {
    float inches = cm / 2.54f;
    Serial.print("{\"ok\":true,\"cm\":");
    Serial.print(cm, 2);
    Serial.print(",\"in\":");
    Serial.print(inches, 2);
    Serial.println("}");
  }

  // 2) Short command window (~50 ms)
  unsigned long t0 = millis();
  while (millis() - t0 < 50) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) handleCommand(line);
    }
    delay(1);
  }

  // 3) Pace to ~1 Hz total
  delay(950);
}
