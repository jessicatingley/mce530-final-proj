// TODO: Adjust these
const int stepPin = 11;
const int dirPin = 9;
const int optSensorPin = 5;

// State definitions
const byte HOME = 0;
const byte HOMING = 1;
const byte RUNNING = 2;
const byte EXITING = 3;

// dir
const byte CW = 1;
const byte CCW = 0;

byte State, NextState;

// Entry flags
int EntryHoming = 0;
int EntryRunning = 0;
int EntryExiting = 0;

// Runtime vars
bool started = false;
int totalRev = 0;
int localRev = 0;
bool inNotch = false;
unsigned long moveStartTime = 0;

// Serial parsing state
bool cmdReady = false;
int cmdCount = 0;
byte cmdDir = CW;
char serialBuf[8];
int bufIndex = 0;
bool cmdStop = false;
bool cmdStart = false;
bool cmdExit = false;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(optSensorPin, INPUT);
  Serial.begin(38400);
  NextState = HOME;
}


void loop() {
  readSerial();
  Sequence();
}


void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    // no need to buffer single char commands
    if (c == 's') { cmdStart = true; bufIndex = 0; continue; }
    if (c == 'e') { cmdExit = true; bufIndex = 0; continue; }
    if (c == 'f') { cmdStop = true; bufIndex = 0; continue; }

    // put direction at first index
    if (c == '+' || c == '-') {
      bufIndex = 0;
      serialBuf[bufIndex++] = c;
    } else if (isDigit(c) && bufIndex > 1) {
      // store numeric values after direction
      if (bufIndex < 7) {
        serialBuf[bufIndex++] = c;
      }
    } else if (c == '\n' && bufIndex > 1) {
      // at endline, convert buffer into used variables
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


void Sequence(void) {
  State = NextState;

  switch (State) {

    case HOME:
      // TEST: Check for buttons pressed
      if (cmdStart) {
        // If start pressed, do setup
        cmdStart = false;
        started = true;
        EntryHoming = 0;
        NextState = HOMING;
      } else if (cmdExit) {
        // If exit pressed, stop executing
        cmdExit = false;
        EntryExiting = 0;
        NextState = EXITING;
      } else if (cmdReady && started) {
        // Command entered (only valid after 'start' pressed)
        cmdReady = false;
        EntryRunning = 0;
        NextState = RUNNING;
      }
      cmdStop = false;
      break;

     case HOMING:
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
        NextState = HOME;
      }
      break;

    case RUNNING:
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
        NextState = RUNNING;
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
          NextState = HOME;
        }
      }

        // EXIT: stop or exit has been pressed
        if(cmdStop || cmdExit) {
          analogWrite(stepPin, 0);
          EntryRunning = 0;
          NextState = cmdExit ? EXITING : HOME;
          cmdStop = cmdExit = false;
          if (NextState == EXITING) EntryExiting = 0;
        }
        break;

      case EXITING:
        if (EntryExiting == 0) {
          analogWrite(stepPin, 0);
          started      = false;
          EntryExiting = 1;
          exit(0);
        }
        break;
  }
}
