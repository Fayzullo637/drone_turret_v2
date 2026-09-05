/*
 * drone_turret_firmware.ino
 * Production Arduino Uno Firmware for AI Interceptor Turret with Net Launcher
 * 
 * Hardware Pinout:
 *   - Pan Servo (Azimuth):        Digital Pin 9  (Timer 1 PWM)
 *   - Tilt Servo (Elevation):     Digital Pin 10 (Timer 1 PWM)
 *   - Pneumatic Solenoid Trigger: Digital Pin 8  (Active HIGH)
 *   - Status / Heartbeat LED:     Digital Pin 13 (Builtin LED)
 * 
 * Features:
 *   - Non-blocking circular buffer serial command parser (115200 baud).
 *   - Commands:
 *       "P<pan>,T<tilt>\n" -> Moves servos to absolute angles [0, 180]. Responds "OK P:<pan> T:<tilt>\n"
 *       "<pan>,<tilt>\n"   -> Legacy comma format. Responds "OK P:<pan> T:<tilt>\n"
 *       "FIRE\n"           -> Activates pneumatic launch solenoid on Pin 8 with 200ms auto-cutoff. Responds "FIRED\n"
 *       "HOME\n"           -> Commands neutral position (90, 90). Responds "OK P:90 T:90\n"
 *       "PING\n"           -> Health check. Responds "PONG\n"
 *   - Internal Slew Rate Smoothing (caps max speed to 1.5 deg per 20ms update tick).
 *   - Hardware Watchdog Failsafe (parks servos at 90, 90 on 1500ms timeout).
 *   - Solenoid Auto-Cutoff (pin 8 sets LOW after 200ms non-blockingly).
 */

#include <Servo.h>

// --- Pin Assignments ---
const uint8_t PIN_PAN_SERVO      = 9;
const uint8_t PIN_TILT_SERVO     = 10;
const uint8_t PIN_SOLENOID_FIRE  = 8;
const uint8_t PIN_STATUS_LED     = 13;

// --- Constants & Limits ---
const uint32_t SERIAL_BAUD_RATE  = 115200;
const float HOME_PAN_DEG         = 90.0f;
const float HOME_TILT_DEG        = 90.0f;
const float MIN_SERVO_DEG        = 0.0f;
const float MAX_SERVO_DEG        = 180.0f;
const float MAX_SLEW_DEG_PER_TICK= 1.5f;   // Max slew rate limit per 20ms (~75 deg/s)
const uint32_t TICK_INTERVAL_MS  = 20;     // 50 Hz control loop
const uint32_t WATCHDOG_TIMEOUT_MS = 1500; // 1.5s failsafe timeout
const uint32_t SOLENOID_PULSE_MS   = 200;  // 200ms firing pulse width

// --- Servo Objects ---
Servo panServo;
Servo tiltServo;

// --- Target & Current Angular State ---
float targetPanDeg   = HOME_PAN_DEG;
float targetTiltDeg  = HOME_TILT_DEG;
float currentPanDeg  = HOME_PAN_DEG;
float currentTiltDeg = HOME_TILT_DEG;

// --- Timing & Failsafe State ---
uint32_t lastCommandTimeMs = 0;
uint32_t lastTickTimeMs    = 0;
uint32_t fireStartTimeMs   = 0;
bool     isFiringActive    = false;
bool     isWatchdogActive  = false;

// --- Serial Ring Buffer ---
const uint8_t RX_BUFFER_SIZE = 64;
char rxBuffer[RX_BUFFER_SIZE];
uint8_t rxBufferHead = 0;

void setup() {
  // Initialize Serial
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 1000); // Allow USB connection

  // Initialize GPIO Pins
  pinMode(PIN_SOLENOID_FIRE, OUTPUT);
  digitalWrite(PIN_SOLENOID_FIRE, LOW);

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);

  // Attach Servos
  panServo.attach(PIN_PAN_SERVO, 544, 2400);   // Standard 544-2400us PWM pulse width
  tiltServo.attach(PIN_TILT_SERVO, 544, 2400);

  // Home Servos
  panServo.write((int)HOME_PAN_DEG);
  tiltServo.write((int)HOME_TILT_DEG);

  lastCommandTimeMs = millis();
  lastTickTimeMs    = millis();

  // Startup indication
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(50);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(50);
  }
  digitalWrite(PIN_STATUS_LED, HIGH);

  Serial.println("DRONE_TURRET_FIRMWARE_V2_READY");
}

void loop() {
  uint32_t nowMs = millis();

  // 1. Process incoming Serial bytes (non-blocking)
  readSerialCommands();

  // 2. Control Tick: Slew-rate smoothing & servo update at 50Hz (every 20ms)
  if (nowMs - lastTickTimeMs >= TICK_INTERVAL_MS) {
    lastTickTimeMs = nowMs;
    updateServoPositions();
  }

  // 3. Solenoid Firing Auto-Cutoff
  if (isFiringActive && (nowMs - fireStartTimeMs >= SOLENOID_PULSE_MS)) {
    digitalWrite(PIN_SOLENOID_FIRE, LOW);
    isFiringActive = false;
  }

  // 4. Watchdog Failsafe
  if (nowMs - lastCommandTimeMs > WATCHDOG_TIMEOUT_MS) {
    if (!isWatchdogActive) {
      isWatchdogActive = true;
      targetPanDeg = HOME_PAN_DEG;
      targetTiltDeg = HOME_TILT_DEG;
      digitalWrite(PIN_STATUS_LED, LOW); // LED off indicates failsafe
    }
  } else {
    if (isWatchdogActive) {
      isWatchdogActive = false;
      digitalWrite(PIN_STATUS_LED, HIGH);
    }
  }
}

// --- Non-blocking Serial Command Reader & Parser ---
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue; // Ignore carriage return
    }

    if (c == '\n') {
      // Process complete command line
      if (rxBufferHead > 0) {
        rxBuffer[rxBufferHead] = '\0';
        parseCommand(rxBuffer);
        rxBufferHead = 0;
      }
    } else {
      if (rxBufferHead < RX_BUFFER_SIZE - 1) {
        rxBuffer[rxBufferHead++] = c;
      } else {
        // Buffer overflow: discard and reset
        rxBufferHead = 0;
      }
    }
  }
}

void parseCommand(const char* cmd) {
  lastCommandTimeMs = millis();

  // 1. PING Command
  if (strcmp(cmd, "PING") == 0) {
    Serial.println("PONG");
    return;
  }

  // 2. FIRE Command
  if (strcmp(cmd, "FIRE") == 0) {
    digitalWrite(PIN_SOLENOID_FIRE, HIGH);
    isFiringActive = true;
    fireStartTimeMs = millis();
    Serial.println("FIRED");
    return;
  }

  // 3. HOME Command
  if (strcmp(cmd, "HOME") == 0) {
    targetPanDeg = HOME_PAN_DEG;
    targetTiltDeg = HOME_TILT_DEG;
    Serial.println("OK P:90 T:90");
    return;
  }

  // 4. Standard Format: "P<pan>,T<tilt>"
  if (cmd[0] == 'P' || cmd[0] == 'p') {
    const char* tPtr = strchr(cmd, 'T');
    if (!tPtr) tPtr = strchr(cmd, 't');

    if (tPtr != NULL) {
      float panVal = atof(cmd + 1);
      float tiltVal = atof(tPtr + 1);

      panVal = constrain(panVal, MIN_SERVO_DEG, MAX_SERVO_DEG);
      tiltVal = constrain(tiltVal, MIN_SERVO_DEG, MAX_SERVO_DEG);

      targetPanDeg = panVal;
      targetTiltDeg = tiltVal;

      Serial.print("OK P:");
      Serial.print((int)targetPanDeg);
      Serial.print(" T:");
      Serial.println((int)targetTiltDeg);
      return;
    }
  }

  // 5. Legacy Format: "<pan>,<tilt>"
  const char* commaPtr = strchr(cmd, ',');
  if (commaPtr != NULL) {
    float panVal = atof(cmd);
    float tiltVal = atof(commaPtr + 1);

    panVal = constrain(panVal, MIN_SERVO_DEG, MAX_SERVO_DEG);
    tiltVal = constrain(tiltVal, MIN_SERVO_DEG, MAX_SERVO_DEG);

    targetPanDeg = panVal;
    targetTiltDeg = tiltVal;

    Serial.print("OK P:");
    Serial.print((int)targetPanDeg);
    Serial.print(" T:");
    Serial.println((int)targetTiltDeg);
    return;
  }

  // Unknown command
  Serial.print("ERR_UNKNOWN_CMD: ");
  Serial.println(cmd);
}

// --- Slew-Rate Smoothing & Hardware Servo Update ---
void updateServoPositions() {
  // Smooth Pan
  float panError = targetPanDeg - currentPanDeg;
  if (fabs(panError) > 0.05f) {
    if (panError > MAX_SLEW_DEG_PER_TICK) {
      currentPanDeg += MAX_SLEW_DEG_PER_TICK;
    } else if (panError < -MAX_SLEW_DEG_PER_TICK) {
      currentPanDeg -= MAX_SLEW_DEG_PER_TICK;
    } else {
      currentPanDeg = targetPanDeg;
    }
    panServo.write((int)round(currentPanDeg));
  }

  // Smooth Tilt
  float tiltError = targetTiltDeg - currentTiltDeg;
  if (fabs(tiltError) > 0.05f) {
    if (tiltError > MAX_SLEW_DEG_PER_TICK) {
      currentTiltDeg += MAX_SLEW_DEG_PER_TICK;
    } else if (tiltError < -MAX_SLEW_DEG_PER_TICK) {
      currentTiltDeg -= MAX_SLEW_DEG_PER_TICK;
    } else {
      currentTiltDeg = targetTiltDeg;
    }
    tiltServo.write((int)round(currentTiltDeg));
  }
}

