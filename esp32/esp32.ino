#include <Adafruit_MAX31865.h>

// --- Cycle state machine types must be declared before Arduino auto-prototypes ---
enum CycleState : uint8_t {
  CYCLE_IDLE = 0,
  CYCLE_WASHING = 1,
  CYCLE_PAUSED = 2,
  CYCLE_COMPLETE = 3,
  CYCLE_FAULT = 4
};

static const char* cycleStateToStr(CycleState s);

/////////////////////////////////////////////////////////////////////////////////
// TEMPERATURE SENSOR (PT100 & MAX31865 via SPI)
/////////////////////////////////////////////////////////////////////////////////

#define MAX_CS   5
#define MAX_MOSI 23
#define MAX_MISO 19
#define MAX_CLK  18

Adafruit_MAX31865 sensor(MAX_CS, MAX_MOSI, MAX_MISO, MAX_CLK);
#define RREF 4301.0
#define RNOMINAL 1000.0

/////////////////////////////////////////////////////////////////////////////////
// FLOW SENSOR (YF-B2)
/////////////////////////////////////////////////////////////////////////////////

#define FLOW 32
volatile uint32_t pulseCount = 0;
unsigned long lastTick = 0;

void IRAM_ATTR pulseISR()
{
  pulseCount++;
}

float pulsesToLpm(uint32_t pulses, float dtSeconds)
{
  // Adjust this after you do a 1-L bucket test
  const float K = 450.0f; // pulses per liter
  return (pulses / K) / (dtSeconds / 60.0f);
}

/////////////////////////////////////////////////////////////////////////////////
// TUBIDITY (DFRobot SEN0554 UART)
/////////////////////////////////////////////////////////////////////////////////

// Sensor wiring: VCC=5V, GND, TX(blue)->ESP32 RX2 (GPIO16 via level shift), RX(white)<-ESP32 TX2 (GPIO17)
#define TURB_RX 16   // ESP32 receives here (from sensor TX)
#define TURB_TX 17   // ESP32 transmits here (to sensor RX)
HardwareSerial Turb(2);  // UART2

// Vendor command (5 bytes), per DFRobot example
const uint8_t TURB_CMD[5] = { 0x18, 0x05, 0x00, 0x01, 0x0D };

// Read turbidity percentage (0..255). Returns true on success.
bool readTurbidity(uint8_t &pct, uint32_t timeout_ms = 50)
{
  while (Turb.available()) Turb.read();        // flush any stale bytes
  Turb.write(TURB_CMD, sizeof(TURB_CMD));      // send request

  uint8_t buf[5] = {0};
  uint32_t t0 = millis();
  int got = 0;
  // collect exactly 5 bytes with timeout
  while ((millis() - t0) < timeout_ms)
  {
    while (Turb.available() && got < 5)
    {
      buf[got++] = (uint8_t)Turb.read();
    }
    
    if (got >= 5)
    {
      break;
    }
    
    delay(1);
  }
  
  if (got < 5)
  {
    return false;
  }

  // DFRobot uses buf[3] as the value to print in decimal
  pct = buf[3];
  return true;
}

// Simple NTU estimate from 0..255 percentage.
// The vendor curve is nonlinear and saturating; this is just a usable rough map.
// Tweak once you collect your own reference points.
float ntuEstimateFromPct(uint8_t pct)
{
  // piecewise: faster rise early, then flatten
  if (pct <= 200)
  {
    return pct * 10.0f;               // ~0..2000 NTU
  }
  
  else
  {
    return 2000.0f + (pct - 200) * 15.0f;  // ~2000..(≈2000+55*15≈2825 NTU)
  }
}

/////////////////////////////////////////////////////////////////////////////////
// FLOAT SWITCH (LEVEL)
/////////////////////////////////////////////////////////////////////////////////

#define LEVEL_SW 33  // GPIO for float switch input (one lead here, other to GND)

/////////////////////////////////////////////////////////////////////////////////
// LID INTERLOCK
/////////////////////////////////////////////////////////////////////////////////

#define LID_LOCK 21 // GPIO for lid interlock switch (one lead here, other to GND)

/////////////////////////////////////////////////////////////////////////////////
// HEATER
/////////////////////////////////////////////////////////////////////////////////

#define HEATER_LED 4
const float OVER_F = 120.0f;
float setTempF = 95.0f;
uint8_t secOver = 0;
bool heaterStop = false;
bool heaterOn = false;
bool heaterEn = false;
//bool heaterLevel = false;

// bang-bang automation
bool bangBang(float temp_f)
{
  if (isnan(temp_f))  // failsafe in case of invalid temp value
  {
    return false;
  }

  return (temp_f < setTempF);
}

void setHeaterLED(float temp_f, bool requestOn)
{
  if (isnan(temp_f))  
  {
    heaterOn = false;
    digitalWrite(HEATER_LED, LOW);
    return;
  }

  if (temp_f >= OVER_F)
  {
    if (secOver < 255)
    {
      secOver++;
    }
  }

  else
  {
    secOver = 0;
    heaterStop = false;
  }

  // If temp too high for 20 sec or more, shut off heater
  if (secOver >= 20)
  {
    heaterStop = true;
  }


  // Final output
  if (heaterStop)
  {
    heaterOn = false;
  }

  else
  {
    heaterOn = requestOn;
  }

  digitalWrite(HEATER_LED, heaterOn ? HIGH : LOW);
  
  // FOR TESTING: use float switch to control heater
  bool levelRaw = digitalRead(33);   // HIGH = open, LOW = closed
  // Assuming NC contact wired so "closed" = level OK.
  bool levelOk = (levelRaw == LOW);

  if (!levelOk)
  {
    digitalWrite(HEATER_LED, LOW);
  }
}

void handleHeaterCommand(const String &line)
{
  // Backwards-compatible:
  //   "HEATER 1" / "HEATER 0"  -> enable/disable
  // Extended:
  //   "HEATER SET <tempF>"     -> update setpoint
  //   "HEATER RESET"           -> clear overtemp latch (heaterStop/secOver)
  if (!line.startsWith("HEATER")) return;

  float tF = 0.0f;
  if (sscanf(line.c_str(), "HEATER SET %f", &tF) == 1)
  {
    setTempF = tF;
    Serial.print("{\"ack\":\"HEATER\",\"setTempF\":");
    Serial.print(setTempF, 1);
    Serial.println("}");
    return;
  }

  if (line.equalsIgnoreCase("HEATER RESET"))
  {
    secOver = 0;
    heaterStop = false;
    Serial.println("{\"ack\":\"HEATER\",\"reset\":true}");
    return;
  }

  int v = line.endsWith("1") ? 1 : 0;
  heaterEn = (v == 1);
  Serial.print("{\"ack\":\"HEATER\",\"req\":");
  Serial.print(heaterEn ? "true" : "false");
  Serial.println("}");
}

/////////////////////////////////////////////////////////////////////////////////
// PUMP / MOTOR
/////////////////////////////////////////////////////////////////////////////////

#define ENA 25  // PWM
#define IN1 26
#define IN2 27

void motorStop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void motorDrive(uint8_t duty, bool forward)
{
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW : HIGH);
  analogWrite(ENA, duty);
}

void handleMotorCommand(const String &line)
{
  if (line.startsWith("MOTOR"))
  {
    int spd = 0; char dir[8] = {0};
    int n = sscanf(line.c_str(), "MOTOR %d %7s", &spd, dir);
    spd = constrain(spd, 0, 255);

    if (n == 1 || spd == 0)
    {
      motorStop();
      Serial.println("{\"ack\":\"MOTOR\",\"state\":\"STOP\",\"spd\":0}");
      return;
    }

    if (n == 2)
    {
      bool fwd = (strcmp(dir, "FWD") == 0);
      bool rev = (strcmp(dir, "REV") == 0);
      if (fwd || rev)
      {
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

/////////////////////////////////////////////////////////////////////////////////
// CYCLE (WASH PROGRAM STATE MACHINE)
/////////////////////////////////////////////////////////////////////////////////

CycleState cycleState = CYCLE_IDLE;
int32_t cycleRemainingS = 0;

// Defaults for wash behavior (tune for your machine)
uint8_t cycleMotorDuty = 100;     // 0..255
bool cycleMotorFwd = true;

static const char* cycleStateToStr(CycleState s)
{
  switch (s)
  {
    case CYCLE_IDLE:     return "IDLE";
    case CYCLE_WASHING:  return "WASHING";
    case CYCLE_PAUSED:   return "PAUSED";
    case CYCLE_COMPLETE: return "COMPLETE";
    case CYCLE_FAULT:    return "FAULT";
    default:             return "IDLE";
  }
}

void cycleStopAllOutputs()
{
  heaterEn = false;
  motorStop();
}

void cycleApplyOutputsForState()
{
  if (cycleState == CYCLE_WASHING)
  {
    // Heater remains subject to lidClosed + bang-bang + overtemp shutoff in loop()
    heaterEn = true;
    //motorDrive(cycleMotorDuty, cycleMotorFwd);
  }
  else if (cycleState == CYCLE_PAUSED)
  {
    heaterEn = false;
    motorStop();
  }
  else
  {
    cycleStopAllOutputs();
  }
}

void handleCycleCommand(const String &line)
{
  // "CYCLE START <seconds> <tempF>"
  // "CYCLE PAUSE"
  // "CYCLE RESUME"
  // "CYCLE STOP"
  if (!line.startsWith("CYCLE")) return;

  if (line.startsWith("CYCLE START"))
  {
    int secs = 0;
    float tF = setTempF;
    int n = sscanf(line.c_str(), "CYCLE START %d %f", &secs, &tF);
    if (n >= 1)
    {
      cycleRemainingS = max(0, secs);
      if (n >= 2) setTempF = tF;

      cycleState = (cycleRemainingS > 0) ? CYCLE_WASHING : CYCLE_COMPLETE;
      cycleApplyOutputsForState();

      Serial.print("{\"ack\":\"CYCLE\",\"cmd\":\"START\",\"state\":\"");
      Serial.print(cycleStateToStr(cycleState));
      Serial.print("\",\"remainingS\":");
      Serial.print(cycleRemainingS);
      Serial.print(",\"setTempF\":");
      Serial.print(setTempF, 1);
      Serial.println("}");
    }
    return;
  }

  if (line.equalsIgnoreCase("CYCLE PAUSE"))
  {
    if (cycleState == CYCLE_WASHING)
    {
      cycleState = CYCLE_PAUSED;
      cycleApplyOutputsForState();
    }
    Serial.print("{\"ack\":\"CYCLE\",\"cmd\":\"PAUSE\",\"state\":\"");
    Serial.print(cycleStateToStr(cycleState));
    Serial.println("\"}");
    return;
  }

  if (line.equalsIgnoreCase("CYCLE RESUME"))
  {
    if (cycleState == CYCLE_PAUSED)
    {
      cycleState = (cycleRemainingS > 0) ? CYCLE_WASHING : CYCLE_COMPLETE;
      cycleApplyOutputsForState();
    }
    Serial.print("{\"ack\":\"CYCLE\",\"cmd\":\"RESUME\",\"state\":\"");
    Serial.print(cycleStateToStr(cycleState));
    Serial.println("\"}");
    return;
  }

  if (line.equalsIgnoreCase("CYCLE STOP"))
  {
    cycleState = CYCLE_IDLE;
    cycleRemainingS = 0;
    cycleApplyOutputsForState();

    Serial.println("{\"ack\":\"CYCLE\",\"cmd\":\"STOP\",\"state\":\"IDLE\"}");
    return;
  }

  // Unknown CYCLE subcommand
  Serial.println("{\"ack\":\"CYCLE\",\"err\":\"UNKNOWN\"}");
}

/////////////////////////////////////////////////////////////////////////////////
// SETUP AND INITIALIZATION
/////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);

  // PT100
  sensor.begin(MAX31865_3WIRE);

  // FLOW
  pinMode(FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW), pulseISR, RISING);
  lastTick = millis();

  // TURBIDITY (9600 8N1 as per DFRobot sample)
  Turb.begin(9600, SERIAL_8N1, TURB_RX, TURB_TX);

  // HEATER
  pinMode(HEATER_LED, OUTPUT);
  digitalWrite(HEATER_LED, LOW);  // Starts off
  heaterEn = false;
  heaterStop = false;
  secOver = 0;

  // MOTOR
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  motorStop();

  // FLOAT SWITCH
  pinMode(LEVEL_SW, INPUT_PULLUP);  // enable internal pull-up

  // LID INTERLOCK
  pinMode(LID_LOCK, INPUT_PULLUP);
}

/////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
/////////////////////////////////////////////////////////////////////////////////
void loop()
{
  unsigned long now = millis();
  if (now - lastTick >= 1000) // Once per second
  {
    // FLOW
    detachInterrupt(digitalPinToInterrupt(FLOW));
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW), pulseISR, RISING);
    float dt = (now - lastTick) / 1000.0f;
    lastTick = now;
    float flowLpm = pulsesToLpm(pulses, dt);

    // TEMPERATURE
    float tempC = sensor.temperature(RNOMINAL, RREF);
    bool okT = !isnan(tempC);
    float tempF = okT ? (tempC * 9.0f / 5.0f + 32.0f) : NAN;

    // TURBIDITY
    uint8_t turbPct = 0;
    bool okTur = readTurbidity(turbPct);
    float ntu = okTur ? ntuEstimateFromPct(turbPct) : NAN;
    float turbPctScaled = (turbPct / 243.0f) * 100.0f;

    // FLOAT SWITCH
    bool levelRaw = digitalRead(LEVEL_SW);   // HIGH = open, LOW = closed
    // Assuming NC contact wired so "closed" = level OK.
    bool levelOk = (levelRaw == LOW);

    // LID INTERLOCK
    bool lidClosed = (digitalRead(LID_LOCK) == LOW);

    // CYCLE countdown
    if (cycleState == CYCLE_WASHING)
    {
      if (cycleRemainingS > 0) cycleRemainingS--;
      if (cycleRemainingS <= 0)
      {
        cycleRemainingS = 0;
        cycleState = CYCLE_COMPLETE;
        cycleApplyOutputsForState();
      }
    }
    
    // HEATER
    bool demandHeat = false;
    if (heaterEn && lidClosed)
    {
      demandHeat = bangBang(tempF);
    }

    else
    {
      demandHeat = false;
    }

    setHeaterLED(tempF, demandHeat);
    
    // JSON OUT
    Serial.print("{\"ok\":true");
    
    Serial.print(",\"flowLpm\":"); Serial.print(flowLpm, 2);
    if (okT)
    {
      Serial.print(",\"C\":"); Serial.print(tempC, 2);
      Serial.print(",\"F\":"); Serial.print(tempF, 2);
    }
    
    if (okTur)
    {
      Serial.print(",\"% Turbidity\":"); Serial.print(turbPctScaled, 1);
      Serial.print(",\"NTU\":"); Serial.print(ntu, 1);
    }
    else
    {
      Serial.print(",\"turbPct\":null,\"ntu\":null");
    }

    // Float switch status
    Serial.print(",\"levelOk\":");
    Serial.print(levelOk ? "true" : "false");

    // Heater Status
    Serial.print(",\"heaterEnable\":");
    Serial.print(heaterEn ? "true" : "false");

    Serial.print(",\"setTempF\":");
    Serial.print(setTempF, 1);

    Serial.print(",\"heaterDemand\":");
    Serial.print(demandHeat ? "true" : "false");
    
    Serial.print(",\"heaterOn\":");
    Serial.print(heaterOn ? "true" : "false");
    
    Serial.print(",\"overTemp\":");
    Serial.print((okT && !isnan(tempF) && tempF >= OVER_F) ? "true" : "false");

    Serial.print(",\"secOver\":");
    Serial.print(secOver);

    Serial.print(",\"heaterStop\":");
    Serial.print(heaterStop ? "true" : "false");

    // Lid Interlock Status
    Serial.print(",\"lidClosed\":");
    Serial.print(lidClosed ? "true" : "false");

    // Cycle state (RPi UI uses these fields)
    Serial.print(",\"cycleState\":\"");
    Serial.print(cycleStateToStr(cycleState));
    Serial.print("\"");

    Serial.print(",\"cycleRemainingS\":");
    Serial.print(cycleRemainingS);

    Serial.print(",\"cycleTempSetF\":");
    Serial.print(setTempF, 1);

    Serial.println("}");
  }

  // Rx Command Handling from RPi
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    if (line.equalsIgnoreCase("PING"))
    {
      Serial.println("{\"ack\":\"PING\"}");
    }
    
    else if (line.startsWith("MOTOR"))
    {
      handleMotorCommand(line);   // calls motor handler
    }
    
    else if (line.startsWith("HEATER"))
    {
      handleHeaterCommand(line);  // calls heater handler
    }

    else if (line.startsWith("CYCLE"))
    {
      handleCycleCommand(line);
    }
  }

  delay(950);  // maintain 1 Hz update rate
}