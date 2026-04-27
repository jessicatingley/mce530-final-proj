// Disk commands:
// 'h' = start
// 'q' = stop
// 'x' = exit
// '+N\n' = N revolutions CW
// '-N\n' = N revolutions CCW

// DC motor commands:
// 't<val> = sample interval (s)
// 'd<val> = run duration (s)
// 's1' = start (only applies to dc motor, not disk)
// 's0' = stop
// 'p<val>' = set Kp
// 'i<val>' = set Ki
// 'o1' = open loop mode

// TODO: how to adjust speed of stepper?
// TODO: Adjust pins
// Disk pins
const int stepPin = 11;
const int dirPin = 9;
const int optSensorPin = 5;

// DC motor pins
const int dcPin = 12;
const int IN1_Pin = 7;
const int IN2_Pin = 8;

// Disk state definitions
const byte DISK_HOME = 0;
const byte DISK_HOMING = 1;
const byte DISK_RUNNING = 2;
const byte DISK_EXITING = 3;

// dir
const byte CW = 1;
const byte CCW = 0;

byte diskState, diskNextState;

// Disk entry flags
int EntryHoming = 0;
int EntryRunning = 0;
int EntryExiting = 0;

// Disk runtime vars
bool diskStarted = false;
int totalRev = 0;
int localRev = 0;
bool inNotch = false;
unsigned long moveStartTime = 0;

// Disk command flags
bool cmdReady = false;
int cmdCount = 0;
byte cmdDir = CW;
bool cmdStop = false;
bool cmdStart = false;
bool cmdExit = false;

// Serial parsing
char serialBuf[8];
int bufIndex = 0;


void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(optSensorPin, INPUT);
  Serial.begin(38400);
  diskNextState = DISK_HOME;
}


void loop() {
  readSerial();
  DiskSequence();
}


void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    // No need to buffer single char commands
    if (c == 's') { cmdStart = true; bufIndex = 0; continue; }
    if (c == 'e') { cmdExit = true; bufIndex = 0; continue; }
    if (c == 'f') { cmdStop = true; bufIndex = 0; continue; }

    // Put direction at first index
    if (c == '+' || c == '-') {
      bufIndex = 0;
      serialBuf[bufIndex++] = c;
    } else if (isDigit(c) && bufIndex > 1) {
      // Store numeric values after direction
      if (bufIndex < 7) {
        serialBuf[bufIndex++] = c;
      }
    } else if (c == '\n' && bufIndex > 1) {
      // At endline, convert buffer into variables to be used
      serialBuf[bufIndex] = '\0';
      int val = atoi(serialBuf);
      if (val != 0) {
        cmdDir = (val > 0) ? CW : CCW;
        cmdCount = abs(val);
        cmdReady = true;
      }
      bufIndex = 0;
    }
  }
}


void DiskSequence(void) {
  diskState = diskNextState;

  switch (diskState) {

    case DISK_HOME:
      // TEST: Check for buttons pressed
      if (cmdStart) {
        // If start pressed, do setup
        cmdStart = false;
        diskStarted = true;
        EntryHoming = 0;
        diskNextState = DISK_HOMING;
      } else if (cmdExit) {
        // If exit pressed, stop executing
        cmdExit = false;
        EntryExiting = 0;
        diskNextState = DISK_EXITING;
      } else if (cmdReady && diskStarted) {
        // Command entered (only valid after 'start' pressed)
        cmdReady = false;
        EntryRunning = 0;
        diskNextState = DISK_RUNNING;
      }
      cmdStop = false;
      break;

     case DISK_HOMING:
      // ENTRY
      if (EntryHoming == 0) {
        digitalWrite(dirPin, CW);
        if (digitalRead(optSensorPin)) {
          analogWrite(stepPin, 128);
        }
        EntryHoming = 1;
      }

      // EXIT
      if (!digitalRead(optSensorPin)) {
        analogWrite(stepPin, 0);
        EntryHoming = 0;
        diskNextState = DISK_HOME;
      }
      break;

    case DISK_RUNNING:
      // ENTRY
      if (EntryRunning == 0) {
        localRev = 0;
        inNotch = true;
        digitalWrite(dirPin, cmdDir);
        analogWrite(stepPin, 128);
        moveStartTime = millis();
        EntryRunning = 1;
      }

      // ACTION: check whether another command was sent
      if (cmdReady) {
        analogWrite(stepPin, 0);
        EntryRunning = 0;
        cmdReady = false;
        diskNextState = DISK_RUNNING;
        break;
      }

      // TEST
      if (!digitalRead(optSensorPin)) {
        inNotch = true;
        moveStartTime = millis();
        localRev++;
        totalRev += (cmdDir == CW) ? 1 : -1;
        Serial.println(totalRev);

        // EXIT
        if (localRev >= cmdCount) {
          analogWrite(stepPin, 0);
          EntryRunning = 0;
          diskNextState = DISK_HOME;
        }
      }

        // EXIT: stop or exit has been pressed
        if(cmdStop || cmdExit) {
          analogWrite(stepPin, 0);
          EntryRunning = 0;
          diskNextState = cmdExit ? DISK_EXITING : DISK_HOME;
          cmdStop = cmdExit = false;
          if (diskNextState == DISK_EXITING) EntryExiting = 0;
        }
        break;

      case DISK_EXITING:
        if (EntryExiting == 0) {
          analogWrite(stepPin, 0);
          diskStarted      = false;
          EntryExiting = 1;
          exit(0);
        }
        break;
  }
}
