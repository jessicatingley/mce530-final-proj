// Disk commands:
// 'h' = start
// 'q' = stop
// 'x' = exit
// '+N\n' = N revolutions CW
// '-M\n' = M revolutions CCW

// DC motor commands:
// 't<val> = sample interval (s)
// 'd<val> = run duration (s)
// 's1' = start (only applies to dc motor, not disk)
// 's0' = stop
// 'p<val>' = set Kp
// 'i<val>' = set Ki
// 'o1' = open loop mode

// TODO: FreeRTOS
// TODO: how to adjust speed of stepper?
// Disk pins
const int stepPin = 3;
const int dirPin = 9;
const int optSensorPin = 5;

// DC motor pins
const int dcPin = 11;
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

// DC motor state def
const byte DC_IDLE = 0;
const byte DC_PLOTTING = 1;

byte dcState, dcNextState;

int EntryDCIdle = 0;
int EntryDCPlotting = 0;

// DC parameters
float sampleInterval_ms = 10.0;
float duration_ms = 1000.0;
float stepval_V = 3.0;
float Kp = 0.0;
float Ki = 0.0;
bool openLoop = false;

// DC PI variables
double integral       = 0.0;
double controlOutput  = 0.0;

// DC timing
unsigned long plotStartTime_ms  = 0;
unsigned long lastSampleTime_ms = 0;

// DC command flags
bool dcCmdStart = false;
bool dcCmdStop = false;
bool dcCmdT = false;
bool dcCmdD = false;
bool dcCmdV = false;
bool dcCmdP = false;
bool dcCmdI = false;
bool dcCmdO = false;
float dcCmdVal = 0.0;

const float V_MIN = 0.0;
const float V_MAX = 12.0;

// Serial parsing
String user_input;
char command;
float val;


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {
  Serial.begin(38400);
  
  // Stepper
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(optSensorPin, INPUT);

  // DC
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(dcPin, OUTPUT);
  pinMode(IN1_Pin, OUTPUT);
  pinMode(IN2_Pin, OUTPUT);
  digitalWrite(IN1_Pin, HIGH);
  digitalWrite(IN2_Pin, LOW);
  analogWrite(dcPin, 0);
  
  diskNextState = DISK_HOME;
  dcNextState = DC_IDLE;
}


void loop() {
  readSerial();
  DiskSequence();
  DCSequence();
}


void readSerial() {
  if (Serial.available()) {
    user_input = Serial.readStringUntil('\n');
    user_input.trim();
  }
  
  if (user_input.length() > 0) {
    command = user_input.charAt(0);
    val = user_input.substring(1).toFloat();

    switch(command) {
      // Disk: start
      case 'h': cmdStart = true; break;

      // Disk: exit
      case 'x': cmdExit = true; break;

      // Disk: stop
      case 'q': cmdStop = true; break;
      
      // Disk: +N = CW N rotations
      case '+':
        cmdDir = CW;
        cmdCount = abs(val);
        cmdReady = true;
        break;

      // Disk: -N = CCW N rotations  
      case '-':
        cmdDir = CCW;
        cmdCount = abs(val);
        cmdReady = true;
        break;

      // DC motor: Sample Interval
      case 't': 
        dcCmdT = true;
        dcCmdVal = val;
        break;

      // DC motor: Duration 
      case 'd':
        dcCmdD = true; 
        dcCmdVal = val; 
        break;

      // DC motor: Voltage
      case 'v': 
        dcCmdV = true; 
        dcCmdVal = val; 
        break;

      // DC motor: Kp
      case 'p':
        dcCmdP = true;
        dcCmdVal = val;
        break;

      // DC motor: Ki
      case 'i':
        dcCmdI = true; 
        dcCmdVal = val; 
        break;

      // DC motor: Open/closed loop
      case 'o':
        dcCmdO = true; 
        dcCmdVal = val; 
        break;

      // DC motor: start
      case 's':
        if (val == 1.0) dcCmdStart = true;
        else dcCmdStop = true;
        break;
    }
    user_input = "";
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
      } else if (cmdStop) {
        diskStarted = false;
        cmdStop = false;
      }
      break;

     case DISK_HOMING:
      // ENTRY
      if (EntryHoming == 0) {
        digitalWrite(dirPin, CW);
        analogWrite(stepPin, 128);
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
        analogWrite(stepPin, 250);
        moveStartTime = millis();
        EntryRunning = 1;
      }

      // INTERRUPT: check whether another command was sent
      if (cmdReady) {
        analogWrite(stepPin, 0);
        EntryRunning = 0;
        cmdReady = false;
        diskNextState = DISK_RUNNING;
        break;
      }

      // ACTION: ignore sensor for 100ms to clear notch
      if (((millis() - moveStartTime) < 100) && inNotch) break;
      inNotch = false;

      // TEST: revolution complete
      if (!digitalRead(optSensorPin)) {
        inNotch = true;
        moveStartTime = millis();
        localRev++;
        totalRev += (cmdDir == CW) ? 1 : -1;
        Serial.print("r,");
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
          // TODO: should this kill the entire program or should it set 'start' flag false?
          exit(0);
        }
        break;
  }
}


void DCSequence(void) {

  // Apply parameter updates before going into state logic
  if (dcCmdT) { 
    float t = dcCmdVal; 
    if (t < 0.001) t = 0.001; 
    sampleInterval_ms = t * 1000.0; 
    dcCmdT = false; 
  }
  if (dcCmdD) { 
    float d = dcCmdVal; 
    if (d < 0) d = 0; 
    duration_ms = d * 1000.0; 
    dcCmdD = false; 
  }
  if (dcCmdV) { 
    stepval_V = dcCmdVal; 
    dcCmdV = false; 
  }
  if (dcCmdP) { 
    Kp = dcCmdVal; 
    openLoop = false; 
    dcCmdP = false; 
  }
  if (dcCmdI) { 
    Ki = dcCmdVal; 
    openLoop = false; 
    dcCmdI = false; 
  }
  if (dcCmdO) { 
    if (dcCmdVal == 1.0) 
    openLoop = true; 
    dcCmdO = false; 
  }

  dcState = dcNextState;

  switch (dcState) {
    case DC_IDLE:
      // ENTRY
      if (EntryDCIdle == 0) {
        analogWrite(dcPin, 0);
        EntryDCIdle = 1;
        EntryDCPlotting = 0;
      }

      // TEST: Start command received
      if (dcCmdStart) {
        dcCmdStart = false;
        dcNextState = DC_PLOTTING;
        EntryDCIdle = 0;
      }

      dcCmdStop = false;
      break;

    case DC_PLOTTING:
      // ENTRY: record start times, reset integrator
      if (EntryDCPlotting == 0) {
        plotStartTime_ms = millis();
        lastSampleTime_ms = plotStartTime_ms;
        integral = 0.0;
        controlOutput = 0.0;
        EntryDCPlotting = 1;
        EntryDCIdle = 0;
      }

      // EXIT: Duration reached
      if ((millis() - plotStartTime_ms) >= (unsigned long)duration_ms) {
        analogWrite(dcPin, 0);
        EntryDCPlotting = 0;
        dcNextState = DC_IDLE;
        break;
      }

      // EXIT: stop command
      if (dcCmdStop) {
        dcCmdStop = false;
        analogWrite(dcPin, 0);
        EntryDCPlotting = 0;
        dcNextState = DC_IDLE;
        break;
      }

      // INTERRUPT: new start while already plotting (restart)
      if (dcCmdStart) {
        dcCmdStart = false;
        EntryDCPlotting = 0;
        dcNextState = DC_PLOTTING;
        break;
      }

      // ACTION: sample on interval
      if ((millis() - lastSampleTime_ms) >= (unsigned long)sampleInterval_ms) {
        lastSampleTime_ms = millis();
        
        int sensorVal = analogRead(A1);
        float voltage = fmap((float)sensorVal, 0.0, 1023.0, V_MIN, 5.0);
    
        if (openLoop) {
          // Open loop = desired is exactly applied as output
          controlOutput = stepval_V;
        } else {
          double error = stepval_V - voltage;
    
          integral += error * sampleInterval_ms;
          integral = constrain(integral, V_MIN / Ki, V_MAX / Ki);  // anti-windup
    
          controlOutput = (Kp * error) + (Ki * integral);
        }
    
        int pwmVal = (int)constrain((constrain(controlOutput, V_MIN, V_MAX) / V_MAX) * 255.0, 0, 255);
        analogWrite(dcPin, pwmVal);

        Serial.print("d,");
        Serial.print(voltage);
        Serial.print(',');
        Serial.println(pwmVal);
      }
      break;
  }
}
