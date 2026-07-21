//SUMMER 2026 ROVER PROJECT: ESP32 DRIVE CONTROLLER
//Role: Real-time motor control co-processor. Receives high-level drive commands
//from the Raspberry Pi 4B over UART2, translates them into PWM/DIR signals for
//two Cytron MDD10A drivers (Left side = Driver #1, Right side = Driver #2), and
//executes speed tests/programmed paths locally so timing isn't at the mercy of
//USB/serial jitter from the Pi.
//
//Requires Arduino-ESP32 core 3.x (uses the unified analogWrite() PWM API).
//If on core 2.x, swap analogWrite() calls for ledcSetup/ledcAttachPin/ledcWrite.

//WIRING (confirmed/datasheet-verified, from Wiring & Hardware doc)
//MDD10A #1 (LEFT):  DIR1->GPIO32  PWM1->GPIO33 (Left-Front)
//                   DIR2->GPIO25  PWM2->GPIO26 (Left-Rear)
//MDD10A #2 (RIGHT): DIR1->GPIO27  PWM1->GPIO14 (Right-Front)
//                   DIR2->GPIO18  PWM2->GPIO19 (Right-Rear)
//Pi 4B <-> ESP32 UART2: GPIO16(RX2)<-Pi TX, GPIO17(TX2)->Pi RX
//GND shared between ESP32 and both MDD10As (required for logic reference).

//SERIAL COMMAND PROTOCOL (from Pi, over UART2, newline-terminated ASCII)
//M,<left>,<right>   Direct differential speeds, each -255..255
//D,<speed>,<turn>   Speed + turn mixing, each -255..255
//S                  Soft stop (ramps to 0)
//B                  Hard brake (immediate coast-to-zero)
//T                  Run speed test sweep (non-blocking)
//P                  Run programmed test path (non-blocking)
//X                  Abort any running test/path immediately
//
//Telemetry is written back out the same UART2 line as "ACK,..." or "LOG,..."
//messages, and mirrored to USB serial (Serial) for local debugging.
//
//NOTE ON MANUAL CONTROL: the Pi-side teleop script (drive/pi4b/pi_teleop_control.py)
//maps WASD+N/M/B keys into these same D/B commands before sending them here --
//this firmware never parses WASD directly, it only ever sees the final computed
//D,<speed>,<turn> or B command. See that script for the full control scheme,
//including the reversed-steering convention used for s+a / s+d.

#include <Arduino.h>

//FORWARD DECLARATIONS
//logLine() is defined further down (near the UART2 setup), but SpeedTest and
//PathRunner call it from inside their methods, which are defined earlier in
//this file. Arduino's auto-prototype generator doesn't reliably catch calls
//made from inside class bodies, so this is declared explicitly here instead.
void logLine(const char *msg);

//PIN DEFINITIONS
//MDD10A #1: LEFT CONTROL
const int LF_DIR = 32;  //Left-Front direction
const int LF_PWM = 33;  //Left-Front speed
const int LR_DIR = 25;  //Left-Rear direction
const int LR_PWM = 26;  //Left-Rear speed

//MDD10A #2: RIGHT CONTROL
const int RF_DIR = 27;  //Right-Front direction
const int RF_PWM = 14;  //Right-Front speed
const int RR_DIR = 18;  //Right-Rear direction
const int RR_PWM = 19;  //Right-Rear speed

//UART2 to Raspberry Pi 4B
const int PI_RX_PIN = 16;   //ESP32 RX2 <- Pi TX
const int PI_TX_PIN = 17;   //ESP32 TX2 -> Pi RX
const long PI_BAUD = 115200;

//PWM Channel Configuration (analogWrite on core 3.x auto-manages LEDC channels)
const int PWM_RESOLUTION_BITS = 8;  //0-255 duty range
const int PWM_FREQ_HZ = 20000;  //20kHz, above audible range, avoids motor whine

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
const int MAX_SPEED = 255;
const int SOFT_STOP_STEP = 15;        // duty reduced per soft-stop tick
const unsigned long SOFT_STOP_INTERVAL_MS = 20;

//MOTOR CONTROL STRUCTS
struct MotorChannel {
  int dirPin;
  int pwmPin;
  bool invert;  //Set invert to true if motor is mounted backwards or needs to be reversed

  void begin() {
    pinMode(dirPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
  }

  //MOTOR SPEED CONTROL
  /*
  FULL REVERSE: -255
  STOP: 0
  FULL FORWARD: +255
  */
  void setSpeed(int speed) {
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    //If invert is true, reverse the direction to negate physical orientation.
    if (invert) speed = -speed;
    bool forward = speed >= 0;
    //DIR=LOW is FORWARD per the verified MDD10A truth table (Wiring & Pinouts doc):
    //PWM HIGH + DIR LOW = FORWARD, PWM HIGH + DIR HIGH = BACKWARD.
    digitalWrite(dirPin, forward ? LOW : HIGH);
    analogWrite(pwmPin, abs(speed));
  }
};

//DECLARE MOTOR CHANNELS
MotorChannel leftFront  = {LF_DIR, LF_PWM, false};
MotorChannel leftRear   = {LR_DIR, LR_PWM, false};
MotorChannel rightFront = {RF_DIR, RF_PWM, false};
MotorChannel rightRear  = {RR_DIR, RR_PWM, false};

//DRIVE SYSTEM: DIFFERENTIAL CONTROLS
class DriveSystem {
 public:
  void begin() {
    leftFront.begin();
    leftRear.begin();
    rightFront.begin();
    rightRear.begin();
  }

  //Set left and right motor speeds directly within range [-255,255]
  void setLeft(int speed) {
    currentLeft = constrain(speed, -MAX_SPEED, MAX_SPEED);
    leftFront.setSpeed(currentLeft);
    leftRear.setSpeed(currentLeft);
  }

  void setRight(int speed) {
    currentRight = constrain(speed, -MAX_SPEED, MAX_SPEED);
    rightFront.setSpeed(currentRight);
    rightRear.setSpeed(currentRight);
  }

  //Set left and right motor speeds together to achieve differential turning
  void setDifferential(int left, int right) {
    setLeft(left);
    setRight(right);
  }

  //Mix speed and turn values to compute left/right motor speeds
  void driveMix(int speed, int turn) {
    int left = constrain(speed + turn, -MAX_SPEED, MAX_SPEED);
    int right = constrain(speed - turn, -MAX_SPEED, MAX_SPEED);
    setDifferential(left, right);
  }

  //Reverse both motors at the same magnitude (for backing up)
  void reverse(int magnitude) {
    magnitude = constrain(magnitude, 0, MAX_SPEED);
    setDifferential(-magnitude, -magnitude);
  }

  //Immediately stop both motors (coast to zero)
  //Likely to be used on manual testing to evaluate ADAS system response to sudden stop events.
  void hardBrake() {
    setDifferential(0, 0);
    softStopActive = false;
  }

  //Gradually ramp both motors to zero over time (soft stop)
  void softStop() {
    softStopActive = true;
    lastSoftStopTick = millis();
  }

  //Abort any soft stop in progress and immediately stop both motors
  void abortAll() {
    softStopActive = false;
    setDifferential(0, 0);
  }

  //Update function to be called in the main loop to handle soft stop ramping
  void update() {
    if (!softStopActive) return;
    unsigned long now = millis();
    if (now - lastSoftStopTick < SOFT_STOP_INTERVAL_MS) return;
    lastSoftStopTick = now;

    currentLeft = stepTowardZero(currentLeft);
    currentRight = stepTowardZero(currentRight);
    leftFront.setSpeed(currentLeft);
    leftRear.setSpeed(currentLeft);
    rightFront.setSpeed(currentRight);
    rightRear.setSpeed(currentRight);

    if (currentLeft == 0 && currentRight == 0) {
      softStopActive = false;
    }
  }

  int getLeft() const { return currentLeft; }
  int getRight() const { return currentRight; }

 private:
  int currentLeft = 0;
  int currentRight = 0;
  bool softStopActive = false;
  unsigned long lastSoftStopTick = 0;

  static int stepTowardZero(int value) {
    if (value > 0) return max(0, value - SOFT_STOP_STEP);
    if (value < 0) return min(0, value + SOFT_STOP_STEP);
    return 0;
  }
};

DriveSystem drive;

//MANUAL SPEED TEST
class SpeedTest {
 public:
  void start() {
    active = true;
    direction = 1;
    currentDuty = 0;
    lastStepTime = millis();
    logLine("LOG,SPEED_TEST_START");
  }

  void abort() {
    if (active) {
      active = false;
      drive.hardBrake();
      logLine("LOG,SPEED_TEST_ABORTED");
    }
  }

  bool isActive() const { return active; }

  void update() {
    if (!active) return;
    unsigned long now = millis();
    if (now - lastStepTime < stepIntervalMs) return;
    lastStepTime = now;

    drive.setDifferential(currentDuty, currentDuty);

    char buf[64];
    snprintf(buf, sizeof(buf), "LOG,SPEED_TEST,t=%lu,duty=%d", now, currentDuty);
    logLine(buf);

    currentDuty += stepSize * direction;

    if (currentDuty >= MAX_SPEED) {
      currentDuty = MAX_SPEED;
      direction = -1;
    } else if (currentDuty <= 0 && direction == -1) {
      currentDuty = 0;
      finish();
    }
  }

 private:
  bool active = false;
  int currentDuty = 0;
  int direction = 1;
  int stepSize = 15;
  unsigned long stepIntervalMs = 250;
  unsigned long lastStepTime = 0;

  void finish() {
    active = false;
    drive.hardBrake();
    logLine("LOG,SPEED_TEST_COMPLETE");
  }
};

SpeedTest speedTest;

//PROGRAMMED PATH EXECUTION
//Used for testing autonomous driving routines and ADAS responses to known paths.
struct PathStep {
  int leftSpeed;
  int rightSpeed;
  unsigned long durationMs;
};

PathStep pathSteps[] = {
  {150, 150, 1500},
  {120, -120, 700},
  {150, 150, 1500},
  {0, 0, 500},
};
const int PATH_STEP_COUNT = sizeof(pathSteps) / sizeof(pathSteps[0]);

class PathRunner {
 public:
  void start() {
    active = true;
    stepIndex = 0;
    stepStartTime = millis();
    applyStep(stepIndex);
    logLine("LOG,PATH_START");
  }

  void abort() {
    if (active) {
      active = false;
      drive.hardBrake();
      logLine("LOG,PATH_ABORTED");
    }
  }

  bool isActive() const { return active; }

  void update() {
    if (!active) return;
    unsigned long now = millis();
    if (now - stepStartTime < pathSteps[stepIndex].durationMs) return;

    stepIndex++;
    if (stepIndex >= PATH_STEP_COUNT) {
      active = false;
      drive.hardBrake();
      logLine("LOG,PATH_COMPLETE");
      return;
    }
    stepStartTime = now;
    applyStep(stepIndex);
  }

 private:
  bool active = false;
  int stepIndex = 0;
  unsigned long stepStartTime = 0;

  void applyStep(int idx) {
    drive.setDifferential(pathSteps[idx].leftSpeed, pathSteps[idx].rightSpeed);
    char buf[64];
    snprintf(buf, sizeof(buf), "LOG,PATH_STEP,%d,L=%d,R=%d", idx,
             pathSteps[idx].leftSpeed, pathSteps[idx].rightSpeed);
    logLine(buf);
  }
};

PathRunner pathRunner;

//PI 4 UART2 SERIAL
HardwareSerial PiSerial(2); //UART2

void logLine(const char *msg) {
  PiSerial.println(msg);
  Serial.println(msg);
}

String rxBuffer = "";

void handleCommand(const String &line) {
  if (line.length() == 0) return;

  char cmd = line.charAt(0);

  switch (cmd) {
    case 'M': {
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      if (firstComma < 0 || secondComma < 0) {
        logLine("ERR,BAD_M_FORMAT");
        return;
      }
      int left = line.substring(firstComma + 1, secondComma).toInt();
      int right = line.substring(secondComma + 1).toInt();
      speedTest.abort();
      pathRunner.abort();
      drive.setDifferential(left, right);
      logLine("ACK,M");
      break;
    }
    case 'D': {
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      if (firstComma < 0 || secondComma < 0) {
        logLine("ERR,BAD_D_FORMAT");
        return;
      }
      int spd = line.substring(firstComma + 1, secondComma).toInt();
      int turn = line.substring(secondComma + 1).toInt();
      speedTest.abort();
      pathRunner.abort();
      drive.driveMix(spd, turn);
      logLine("ACK,D");
      break;
    }
    case 'S':
      drive.softStop();
      logLine("ACK,S");
      break;
    case 'B':
      speedTest.abort();
      pathRunner.abort();
      drive.hardBrake();
      logLine("ACK,B");
      break;
    case 'T':
      pathRunner.abort();
      speedTest.start();
      logLine("ACK,T");
      break;
    case 'P':
      speedTest.abort();
      pathRunner.start();
      logLine("ACK,P");
      break;
    case 'X':
      speedTest.abort();
      pathRunner.abort();
      drive.abortAll();
      logLine("ACK,X");
      break;
    default:
      logLine("ERR,UNKNOWN_CMD");
      break;
  }
}

void pollSerialInput(Stream &port) {
  while (port.available()) {
    char c = port.read();
    if (c == '\n') {
      rxBuffer.trim();
      handleCommand(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
      if (rxBuffer.length() > 64) {
        rxBuffer = "";
      }
    }
  }
}

//SETUP LOOP
//Used to initialize serial ports, PWM, and motor control pins.
void setup() {
  Serial.begin(115200);
  PiSerial.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);

  analogWriteResolution(PWM_RESOLUTION_BITS);
  analogWriteFrequency(PWM_FREQ_HZ);

  drive.begin();
  drive.hardBrake();

  logLine("LOG,ESP32_DRIVE_READY");
}

void loop() {
  pollSerialInput(PiSerial);
  pollSerialInput(Serial);

  drive.update();
  speedTest.update();
  pathRunner.update();
}