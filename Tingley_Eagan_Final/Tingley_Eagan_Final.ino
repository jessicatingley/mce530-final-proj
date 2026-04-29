/**
 * Serial command reference
 * ────────────────────────
 * Disk:
 *   h        — Home disk (start)
 *   q        — Stop disk
 *   x        — Exit program
 *   l/m/f    — Set speed: low (250 Hz) / medium (500 Hz) / fast (750 Hz)
 *   +N       — Rotate N revolutions clockwise
 *   -N       — Rotate N revolutions counter-clockwise
 *
 * DC motor:
 *   t<val>   — Set sample interval (seconds)
 *   d<val>   — Set run duration (seconds)
 *   v<val>   — Set target voltage (volts)
 *   p<val>   — Set Kp (switches to closed-loop)
 *   i<val>   — Set Ki (switches to closed-loop)
 *   o1       — Enable open-loop mode
 *   s1       — Start DC motor run
 *   s0       — Stop DC motor
 *
 * TODO: FreeRTOS
 * TODO: do we need pwm.end() for speed changing?
 */

#include "pwm.h"

// Disk / stepper pins
const int stepPin = 3;
PwmOut pwm(D3);
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

// Rotation direction
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
float cmdPWMFreq = 500.0;
float cmdDuty = 50.0;
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


/**
 * Equivalent to Arduino's map() but for floating-point values.
 *
 * @param x       Input value
 * @param in_min  Lower bound of input range
 * @param in_max  Upper bound of input range
 * @param out_min Lower bound of output range
 * @param out_max Upper bound of output range
 * @return        Mapped output value (not clamped)
 */
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Start the disk 
 * @param dir     CW or CCW
 * @param freq    PWM frequency in Hz
 * @param duty    Duty cycle percent (0–100)
 */
void diskMotorRun(byte dir, float freq, float duty) {
  digitalWrite(dirPin, dir);
  pwm.begin(freq, 0.0f);
  pwm.pulse_perc(duty);
}

/** Stop the disk */
void diskMotorStop() {
  analogWrite(stepPin, 0);
}

/** Returns true when the optical sensor detects the notch (active low). */
bool diskNotchDetected() {
  return !digitalRead(optSensorPin);
}

/**
 * Drive the DC motor at given PWM value.
 * @param pwmVal  0–255
 */
void dcMotorSet(int pwmVal) {
  pwmVal = constrain(pwmVal, 0, 255);
  analogWrite(dcPin, pwmVal);
}

/** Stop the DC motor. */
void dcMotorStop() {
  analogWrite(dcPin, 0);
}

/**
 * Read the DC motor's voltage.
 * @return Voltage in volts (0–5 V mapped from A1).
 */
float dcSensorReadVolts() {
  int raw = analogRead(A1);
  return fmap((float)raw, 0.0, 1023.0, V_MIN, 5.0);
}


void setup() {
  Serial.begin(38400);
  
  // Stepper / disk
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(optSensorPin, INPUT);

  // DC motor
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(dcPin, OUTPUT);
  pinMode(IN1_Pin, OUTPUT);
  pinMode(IN2_Pin, OUTPUT);
  digitalWrite(IN1_Pin, HIGH);
  digitalWrite(IN2_Pin, LOW);
  analogWrite(dcPin, 0);
  
  // Set both state machines in their initial states
  diskNextState = DISK_HOME;
  dcNextState = DC_IDLE;
}


void loop() {
  readSerial();
  DiskSequence();
  DCSequence();
}


/**
 * Reads one line from Serial and sets the appropriate command flags.
 *
 * Commands are single-character with an optional numeric suffix (e.g. "+3", "p0.5").
 * Flags are used by DiskSequence() and DCSequence() on the same loop iteration
 * or the next, so they should not be set more than once before being cleared.
 */
void readSerial() {
  if (Serial.available()) {
    user_input = Serial.readStringUntil('\n');
    user_input.trim();
  }
  
  if (user_input.length() > 0) {
    command = user_input.charAt(0);
    val = user_input.substring(1).toFloat();

    switch(command) {

      // Disk commands
      case 'h': cmdStart = true; break;     // Home / start  
      case 'x': cmdExit = true; break;      // Exit
      case 'q': cmdStop = true; break;      // Stop
      case 'l': cmdPWMFreq = 250.0; break;  // Low speed
      case 'm': cmdPWMFreq = 500.0; break;  // Medium speed
      case 'f': cmdPWMFreq = 750.0; break;  // High speed
      
      case '+':                             // CW N rotations
        cmdDir = CW;
        cmdCount = abs(val);
        cmdReady = true;
        break;
  
      case '-':                             // CCW N rotations
        cmdDir = CCW;
        cmdCount = abs(val);
        cmdReady = true;
        break;

      // DC motor commands
      case 't': dcCmdT = true; dcCmdVal = val; break; // Sample interval (s)
      case 'd': dcCmdD = true; dcCmdVal = val; break; // Run duration (s)
      case 'v': dcCmdV = true; dcCmdVal = val; break; // Target voltage (V)
      case 'p': dcCmdP = true; dcCmdVal = val; break; // Kp
      case 'i': dcCmdI = true; dcCmdVal = val; break; // Ki
      case 'o': dcCmdO = true; dcCmdVal = val; break; // Open/closed loop

      case 's':                                       // Start/stop
        if (val == 1.0) dcCmdStart = true;
        else dcCmdStop = true;
        break;
    }

    user_input = "";
  }
}


/**
 * States:
 * 
 *  - DISK_HOME    — Idle; waits for 'h' (home), 'x' (exit), or a rotation command.
 *  - DISK_HOMING  — Spins CW until notch sensor activated, then returns to HOME.
 *  - DISK_RUNNING — Executes the current cmdDir/cmdCount move, counting revolutions
 *                 using the optical sensor. Handles mid-move command replacement.
 *  - DISK_EXITING — Stops the motor and halts execution.
 *
 * Entry flags (EntryHoming, EntryRunning, EntryExiting) gate setup code
 * inside each state. They must be reset to 0 before every transition into that state.
 */
void DiskSequence(void) {
  diskState = diskNextState;

  switch (diskState) {

    case DISK_HOME:
      // TEST: Check for buttons pressed
      if (cmdStart) {
        // If start pressed, do setup and move to homing
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
        // Rotation command received (only valid after 'start' pressed)
        cmdReady = false;
        EntryRunning = 0;
        diskNextState = DISK_RUNNING;

      } else if (cmdStop) {
        // Clears started flag (start will need to be pressed again)
        diskStarted = false;
        cmdStop = false;
      }
      break;

     case DISK_HOMING:
      // ENTRY: Start spinning CW
      if (EntryHoming == 0) {
        diskMotorRun(CW, cmdPWMFreq, cmdDuty);
        EntryHoming = 1;
      }

      // EXIT: Notch detected, stop and return home
      if (diskNotchDetected()) {
        diskMotorStop();
        EntryHoming = 0;
        diskNextState = DISK_HOME;
      }
      break;

    case DISK_RUNNING:
      // ENTRY: reset local counter and begin moving
      if (EntryRunning == 0) {
        localRev = 0;
        inNotch = true;
        diskMotorRun(cmdDir, cmdPWMFreq, cmdDuty);
        
        moveStartTime = millis();
        EntryRunning = 1;
      }

      // INTERRUPT: check whether another command was sent
      if (cmdReady) {
        diskMotorStop();
        EntryRunning = 0;
        cmdReady = false;
        diskNextState = DISK_RUNNING;
        break;
      }

      // ACTION: ignore sensor for 100ms to clear notch
      if (((millis() - moveStartTime) < 100) && inNotch) break;
      inNotch = false;

      // TEST: notch detected, one revolution complete
      if (diskNotchDetected()) {
        inNotch = true;
        moveStartTime = millis(); // Reset 'in notch' timer
        localRev++;
        totalRev += (cmdDir == CW) ? 1 : -1;
        
        // Report position
        Serial.print("r,");
        Serial.println(totalRev);

        // EXIT: target revolution count reached
        if (localRev >= cmdCount) {
          diskMotorStop();
          EntryRunning = 0;
          diskNextState = DISK_HOME;
        }
      }

       // EXIT: stop or exit has been pressed
       if(cmdStop || cmdExit) {
        diskMotorStop();
        EntryRunning = 0;
        diskNextState = cmdExit ? DISK_EXITING : DISK_HOME;
        cmdStop = cmdExit = false;
        if (diskNextState == DISK_EXITING) EntryExiting = 0;
       }
       break;

      case DISK_EXITING:
        if (EntryExiting == 0) {
          diskMotorStop();
          diskStarted = false;
          EntryExiting = 1;
          // TODO: should this kill the entire program or should it set 'start' flag false?
          exit(0);
        }
        break;
  }
}


/**
 * Parameter updates (t/d/v/p/i/o commands) are applied at the start of every call
 * before state logic executes, so new values take effect on the next sample.
 *
 * States:

 *  - DC_IDLE     — Motor off; waits for 's1' to begin a run.
 *  - DC_PLOTTING — Runs the control loop at sampleInterval_ms, printing
 *                "d,<voltage>,<pwmVal>" lines until duration_ms elapses
 *                or a stop command is received.
 */
void DCSequence(void) {

  // Apply parameter updates before going into state logic
  if (dcCmdT) { 
    float t = dcCmdVal; 
    if (t < 0.001) t = 0.001;     // TODO: do we need this?
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
      // ENTRY: ensure motor is off
      if (EntryDCIdle == 0) {
        dcMotorStop();
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
        dcMotorStop();
        EntryDCPlotting = 0;
        dcNextState = DC_IDLE;
        break;
      }

      // EXIT: stop command
      if (dcCmdStop) {
        dcCmdStop = false;
        dcMotorStop();
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
        
        float voltage = dcSensorReadVolts();
    
        if (openLoop) {
          // Open loop = desired is exactly applied as output
          controlOutput = stepval_V;
        } else {
          // Closed-loop PI: compute error and accumulate integral
          double error = stepval_V - voltage;
    
          integral += error * sampleInterval_ms;
          integral = constrain(integral, V_MIN / Ki, V_MAX / Ki);  // anti-windup
    
          controlOutput = (Kp * error) + (Ki * integral);
        }
    
        // Scale control output voltage to 8-bit PWM, clamped to valid range
        int pwmVal = (int)constrain((constrain(controlOutput, V_MIN, V_MAX) / V_MAX) * 255.0, 0, 255);
        dcMotorSet(pwmVal);

        Serial.print("d,");
        Serial.print(voltage);
        Serial.print(',');
        Serial.println(pwmVal);
      }
      break;
  }
}
