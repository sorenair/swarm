#include <Adafruit_MAX31865.h>

// --- Cycle state machine types must be declared before Arduino auto-prototypes ---
enum CycleState : uint8_t {
  CYCLE_IDLE = 0,
  CYCLE_PREHEATING = 1,
  CYCLE_WASHING = 2,
  CYCLE_PAUSED = 3,
  CYCLE_COMPLETE = 4,
  CYCLE_FAULT = 5
};

static const char* cycleStateToStr(CycleState s);
uint16_t computeFaultFlags(bool okTemp, float tempF, bool levelOk, bool lidClosed);
const char* primaryFaultCodeFromFlags(uint16_t flags);
void printFaultCodeArray(uint16_t flags);

enum FaultFlags : uint16_t {
  FAULT_TEMP_SENSOR      = 1 << 0,
  FAULT_OVERTEMP         = 1 << 1,
  FAULT_LOW_LEVEL        = 1 << 2,
  FAULT_LID_OPEN         = 1 << 3,
  FAULT_FLOW_LOW         = 1 << 4,
  FAULT_HEATER_SHUTDOWN  = 1 << 5
};

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

// Low flow detection
const float FLOW_LOW_LPM = 2.0f;     // warning threshold
const uint8_t FLOW_LOW_LIMIT_S = 20; // non-blocking fault after 20 continuous seconds

bool flowLow = false;     // live warning condition
bool flowFault = false;   // non-blocking latched flow fault
uint8_t secLowFlow = 0;   // consecutive seconds below threshold
CycleState cycleState = CYCLE_IDLE;
bool blockingFaultActive = false;

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

void updateFlowStatus(float flowLpm)
{
  // Only monitor low flow during an active wash
  if (cycleState != CYCLE_WASHING)
  {
    flowLow = false;
    secLowFlow = 0;
    flowFault = false;
    return;
  }

  if (flowLpm < FLOW_LOW_LPM)
  {
    flowLow = true;

    if (secLowFlow < 255)
    {
      secLowFlow++;
    }

    if (secLowFlow >= FLOW_LOW_LIMIT_S)
    {
      flowFault = true;
    }
  }
  else
  {
    flowLow = false;
    secLowFlow = 0;
    flowFault = false;
  }
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
// STATUS LEDs
/////////////////////////////////////////////////////////////////////////////////

#define GREEN_LED 13 // GPIO for green LED switch (one lead here, other to GND)
#define YELLOW_LED 12 // GPIO for yellow LED switch (one lead here, other to GND)
#define RED_LED 14 // GPIO for red LED switch (one lead here, other to GND)

/////////////////////////////////////////////////////////////////////////////////
// HEATER
/////////////////////////////////////////////////////////////////////////////////

#define HEATER_LED 4
const float OVER_F = 120.0f;
const uint8_t OVER_TEMP_LIMIT_S = 20;
float setTempF = 95.0f;
uint8_t secOver = 0;  
bool heaterStop = false;
bool heaterOn = false;
bool heaterEn = false;
bool heaterDemandLatched = false;
bool currentTempValid = false;
float currentTempF = NAN;
//bool heaterLevel = false;

// bang-bang automation
bool bangBang(float temp_f)
{
  if (isnan(temp_f))  // failsafe in case of invalid temp value
  {
    heaterDemandLatched = false;
    return false;
  }

  const float lowerSetpointF = setTempF - 3.0f;
  const float upperSetpointF = setTempF + 3.0f;

  if (temp_f <= lowerSetpointF)
  {
    heaterDemandLatched = true;
  }
  else if (temp_f >= upperSetpointF)
  {
    heaterDemandLatched = false;
  }

  return heaterDemandLatched;
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
  if (secOver >= OVER_TEMP_LIMIT_S)
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

int32_t cycleRemainingS = 0;

// Defaults for wash behavior (tune for your machine)
uint8_t cycleMotorDuty = 255;     // 0..255
bool cycleMotorFwd = true;

static const char* cycleStateToStr(CycleState s)
{
  switch (s)
  {
    case CYCLE_IDLE:     return "IDLE";
    case CYCLE_PREHEATING: return "PREHEATING";
    case CYCLE_WASHING:  return "WASHING";
    case CYCLE_PAUSED:   return "PAUSED";
    case CYCLE_COMPLETE: return "COMPLETE";
    case CYCLE_FAULT:    return "FAULT";
    default:             return "IDLE";
  }
}

float cycleLowerSetpointF()
{
  return setTempF - 3.0f;
}

float cycleUpperSetpointF()
{
  return setTempF + 3.0f;
}

bool shouldPreheat(float tempF)
{
  if (isnan(tempF))
  {
    return true;
  }
  return tempF < cycleUpperSetpointF();
}

bool shouldReportLidFault()
{
  return cycleState == CYCLE_WASHING || cycleState == CYCLE_PAUSED || cycleState == CYCLE_FAULT;
}

uint16_t computeFaultFlags(bool okTemp, float tempF, bool levelOk, bool lidClosed)
{
  uint16_t flags = 0;

  if (!okTemp || isnan(tempF))
  {
    flags |= FAULT_TEMP_SENSOR;
  }

  if (okTemp && !isnan(tempF) && tempF >= OVER_F)
  {
    flags |= FAULT_OVERTEMP;
  }

  if (!levelOk)
  {
    flags |= FAULT_LOW_LEVEL;
  }

  if (!lidClosed && shouldReportLidFault())
  {
    flags |= FAULT_LID_OPEN;
  }

  if (flowFault)
  {
    flags |= FAULT_FLOW_LOW;
  }

  if (heaterStop)
  {
    flags |= FAULT_HEATER_SHUTDOWN;
  }

  return flags;
}

const char* primaryFaultCodeFromFlags(uint16_t flags)
{
  if (flags & FAULT_HEATER_SHUTDOWN) return "HEATER_SHUTDOWN";
  if (flags & FAULT_FLOW_LOW) return "FLOW_LOW";
  if (flags & FAULT_OVERTEMP) return "OVERTEMP";
  if (flags & FAULT_LOW_LEVEL) return "LOW_LEVEL";
  if (flags & FAULT_LID_OPEN) return "LID_OPEN";
  if (flags & FAULT_TEMP_SENSOR) return "TEMP_SENSOR";
  return "NONE";
}

void printFaultCodeArray(uint16_t flags)
{
  bool first = true;

  Serial.print("[");

  if (flags & FAULT_HEATER_SHUTDOWN)
  {
    Serial.print("\"HEATER_SHUTDOWN\"");
    first = false;
  }
  if (flags & FAULT_FLOW_LOW)
  {
    if (!first) Serial.print(",");
    Serial.print("\"FLOW_LOW\"");
    first = false;
  }
  if (flags & FAULT_OVERTEMP)
  {
    if (!first) Serial.print(",");
    Serial.print("\"OVERTEMP\"");
    first = false;
  }
  if (flags & FAULT_LOW_LEVEL)
  {
    if (!first) Serial.print(",");
    Serial.print("\"LOW_LEVEL\"");
    first = false;
  }
  if (flags & FAULT_LID_OPEN)
  {
    if (!first) Serial.print(",");
    Serial.print("\"LID_OPEN\"");
    first = false;
  }
  if (flags & FAULT_TEMP_SENSOR)
  {
    if (!first) Serial.print(",");
    Serial.print("\"TEMP_SENSOR\"");
  }

  Serial.print("]");
}

void cycleStopAllOutputs()
{
  heaterEn = false;
  motorStop();
}

void cycleApplyOutputsForState()
{
  if (cycleState == CYCLE_PREHEATING)
  {
    //heaterEn = true;
    //motorDrive(cycleMotorDuty, cycleMotorFwd);
    heaterEn = false;
    motorStop();
  }
  else if (cycleState == CYCLE_WASHING)
  {
    // Heater remains subject to lidClosed + bang-bang + overtemp shutoff in loop()
    //heaterEn = true;
    //motorDrive(cycleMotorDuty, cycleMotorFwd);
    heaterEn = false;
    motorStop();
  }
  else if (cycleState == CYCLE_PAUSED || cycleState == CYCLE_COMPLETE || cycleState == CYCLE_FAULT)
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
      heaterDemandLatched = false;
      blockingFaultActive = false;

      if (cycleRemainingS <= 0)
      {
        cycleState = CYCLE_COMPLETE;
      }
      else if (shouldPreheat(currentTempValid ? currentTempF : NAN))
      {
        cycleState = CYCLE_PREHEATING;
      }
      else
      {
        cycleState = CYCLE_WASHING;
      }
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
    if (cycleState == CYCLE_WASHING || cycleState == CYCLE_PREHEATING)
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
      if (cycleRemainingS <= 0)
      {
        cycleState = CYCLE_COMPLETE;
      }
      else if (shouldPreheat(currentTempValid ? currentTempF : NAN))
      {
        cycleState = CYCLE_PREHEATING;
      }
      else
      {
        cycleState = CYCLE_WASHING;
      }
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
    heaterDemandLatched = false;
    blockingFaultActive = false;
    cycleApplyOutputsForState();

    Serial.println("{\"ack\":\"CYCLE\",\"cmd\":\"STOP\",\"state\":\"IDLE\"}");
    return;
  }

  // Unknown CYCLE subcommand
  Serial.println("{\"ack\":\"CYCLE\",\"err\":\"UNKNOWN\"}");
}

/////////////////////////////////////////////////////////////////////////////////
// FAULT HANDLER
/////////////////////////////////////////////////////////////////////////////////

bool hasActiveFault(bool okTemp, float tempF, bool levelOk, bool lidClosed)
{
  uint16_t flags = computeFaultFlags(okTemp, tempF, levelOk, lidClosed);

  // Always honor latched heater shutdown
  if (flags & FAULT_HEATER_SHUTDOWN) return true;

  // Only safety-critical process faults should block machine operation.
  if (cycleState == CYCLE_WASHING || cycleState == CYCLE_PREHEATING || cycleState == CYCLE_FAULT)
  {
    uint16_t blockingMask =
      FAULT_TEMP_SENSOR |
      FAULT_OVERTEMP |
      FAULT_LOW_LEVEL |
      FAULT_LID_OPEN;

    return (flags & blockingMask) != 0;
  }

  return false;
}

void faultHandler(bool okTemp, float tempF, bool levelOk, bool lidClosed)
{
  bool fault = hasActiveFault(okTemp, tempF, levelOk, lidClosed);
  blockingFaultActive = fault;

  // If a blocking fault occurs during an active process, pause into FAULT state
  if (fault && (cycleState == CYCLE_WASHING || cycleState == CYCLE_PREHEATING))
  {
    cycleState = CYCLE_FAULT;
    cycleApplyOutputsForState();
    return;
  }

  // If fault clears while in FAULT, move to PAUSED
  if (!fault && cycleState == CYCLE_FAULT)
  {
    cycleState = CYCLE_PAUSED;
    cycleApplyOutputsForState();
    return;
  }
}

/////////////////////////////////////////////////////////////////////////////////
// LED HANDLER
/////////////////////////////////////////////////////////////////////////////////

void updateStatusLEDs()
{
  static unsigned long lastBlink = 0;
  static bool blinkState = false;

  // Default all LEDs off
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);

  if (blockingFaultActive)
  {
    digitalWrite(RED_LED, HIGH);
  }
  else if (cycleState == CYCLE_IDLE || cycleState == CYCLE_COMPLETE)
  {
    digitalWrite(GREEN_LED, HIGH);
  }
  else if (cycleState == CYCLE_PREHEATING || cycleState == CYCLE_WASHING)
  {
    digitalWrite(YELLOW_LED, HIGH);
  }
  else if (cycleState == CYCLE_PAUSED)
  {
    unsigned long now = millis();

    if (now - lastBlink >= 500)
    {
      lastBlink = now;
      blinkState = !blinkState;
    }

    digitalWrite(YELLOW_LED, blinkState ? HIGH : LOW);
  }
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

  // Turbidity support is retained above but inactive while the sensor is not installed.

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

  // STATUS LEDs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
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
    updateFlowStatus(flowLpm);

    // TEMPERATURE
    float tempC = sensor.temperature(RNOMINAL, RREF);
    bool okT = !isnan(tempC);
    float tempF = okT ? (tempC * 9.0f / 5.0f + 32.0f) : NAN;
    currentTempValid = okT;
    currentTempF = tempF;

    // FLOAT SWITCH
    bool levelRaw = digitalRead(LEVEL_SW);   // HIGH = open, LOW = closed
    // Assuming NC contact wired so "closed" = level OK.
    bool levelOk = (levelRaw == LOW);

    // LID INTERLOCK
    bool lidClosed = (digitalRead(LID_LOCK) == LOW);

    // CYCLE state progression
    if (cycleState == CYCLE_PREHEATING && okT && !isnan(tempF) && tempF >= setTempF)
    {
      heaterDemandLatched = false;
      cycleState = CYCLE_WASHING;
      cycleApplyOutputsForState();
    }

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

    //faultHandler call
    faultHandler(okT, tempF, levelOk, lidClosed);
    
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
    uint16_t faultFlags = computeFaultFlags(okT, tempF, levelOk, lidClosed);
    const char* faultCode = primaryFaultCodeFromFlags(faultFlags);
    
    // JSON OUT
    Serial.print("{\"ok\":true");
    
    Serial.print(",\"flowLpm\":"); Serial.print(flowLpm, 2);
    Serial.print(",\"flowLow\":");
    Serial.print(flowLow ? "true" : "false");

    Serial.print(",\"secLowFlow\":");
    Serial.print(secLowFlow);

    Serial.print(",\"flowFault\":");
    Serial.print(flowFault ? "true" : "false");
    
    if (okT)
    {
      Serial.print(",\"C\":"); Serial.print(tempC, 2);
      Serial.print(",\"F\":"); Serial.print(tempF, 2);
    }
    
    // Float switch status
    Serial.print(",\"levelOk\":");
    Serial.print(levelOk ? "true" : "false");

    // Heater Status
    Serial.print(",\"heaterEnable\":");
    Serial.print(heaterEn ? "true" : "false");

    Serial.print(",\"setTempF\":");
    Serial.print(setTempF, 1);

    Serial.print(",\"heatLowerSetpointF\":");
    Serial.print(cycleLowerSetpointF(), 1);

    Serial.print(",\"heatUpperSetpointF\":");
    Serial.print(cycleUpperSetpointF(), 1);

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

    Serial.print(",\"overTempThresholdF\":");
    Serial.print(OVER_F, 1);

    Serial.print(",\"overTempLimitS\":");
    Serial.print(OVER_TEMP_LIMIT_S);

    Serial.print(",\"flowLowThresholdLpm\":");
    Serial.print(FLOW_LOW_LPM, 2);

    Serial.print(",\"flowLowLimitS\":");
    Serial.print(FLOW_LOW_LIMIT_S);

    // Lid Interlock Status
    Serial.print(",\"lidClosed\":");
    Serial.print(lidClosed ? "true" : "false");

    Serial.print(",\"faultActive\":");
    Serial.print((faultFlags != 0) ? "true" : "false");

    Serial.print(",\"faultFlags\":");
    Serial.print(faultFlags);

    Serial.print(",\"faultCode\":\"");
    Serial.print(faultCode);
    Serial.print("\"");

    Serial.print(",\"activeFaults\":");
    printFaultCodeArray(faultFlags);

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

  updateStatusLEDs();

  delay(950);  // maintain 1 Hz update rate
}
