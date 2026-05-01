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
 *   j<vStart>,<vEnd>,<dur>,<vStart>,<vEnd>,<dur>,<vStart>,<vEnd>,<dur>
 *            — Load 3 trajectory segments, then sent s1 to execute
 *   s1       — Start DC motor run
 *   s0       — Stop DC motor
 *
 * TODO: FreeRTOS
 * TODO: do we need pwm.end() for speed changing?
 */

#include "pwm.h"
#include <Arduino_FreeRTOS.h>

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

// Disk command flags
volatile bool cmdReady = false;
volatile int cmdCount = 0;
volatile byte cmdDir = CW;
volatile float cmdPWMFreq = 500.0;
volatile float cmdDuty = 50.0;
volatile bool cmdStop = false;
volatile bool cmdStart = false;
volatile bool cmdExit = false;

// DC motor state def
const byte DC_IDLE = 0;
const byte DC_PLOTTING = 1;
const byte DC_TRAJECTORY = 2;

// DC command flags
volatile bool dcCmdStart = false;
volatile bool dcCmdStop = false;
volatile bool dcCmdT = false;
volatile bool dcCmdD = false;
volatile bool dcCmdV = false;
volatile bool dcCmdP = false;
volatile bool dcCmdI = false;
volatile bool dcCmdO = false;
volatile bool dcCmdTraj = false;
volatile float dcCmdVal = 0.0;

// DC Segment struct for trajectory
struct Segment {
  float startV;
  float endV;
  float duration;
};
const int MAX_SEGMENTS = 3;
volatile Segment segments[MAX_SEGMENTS];
volatile int numSegments = 0;

const float V_MIN = 0.0;
const float V_MAX = 12.0;

TaskHandle_t serial_task_handle;
TaskHandle_t disk_task_handle;
TaskHandle_t dc_task_handle;


void serial_thread_func(void *pvParameters);
void disk_thread_func(void *pvParameters);
void dc_thread_func(void *pvParameters);

float nextFloat(String &data, int &idx);
void parseTrajectory(String data);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
void diskMotorRun(byte dir, float freq, float duty);
void diskMotorStop();
bool diskNotchDetected();
void dcMotorSet(int pwmVal);
void dcMotorStop();
float dcSensorReadVolts();


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

  /* Serial parser — highest priority so commands are never dropped */
  if (xTaskCreate(serial_thread_func, "Serial", 512 / 4, nullptr, 3,
                  &serial_task_handle)
      != pdPASS) {
    Serial.println("Failed to create serial task");
    return;
  }

  /* Disk State Machine */
  if (xTaskCreate(disk_thread_func, "Disk", 512 / 4, nullptr, 1,
                  &disk_task_handle)
      != pdPASS) {
    Serial.println("Failed to create disk task");
    return;
  }

  /* DC State Machine */
  if (xTaskCreate(dc_thread_func, "DC", 512 / 4, nullptr, 1,
                  &dc_task_handle)
      != pdPASS) {
    Serial.println("Failed to create DC task");
    return;
  }

  vTaskStartScheduler();
}


void loop() {
}


/**
 * Reads one line from Serial and sets the appropriate command flags.
 *
 * Commands are single-character with an optional numeric suffix (e.g. "+3", "p0.5").
 * Flags are used by DiskSequence() and DCSequence() on the same loop iteration
 * or the next, so they should not be set more than once before being cleared.
 */
void serial_thread_func(void *pvParameters) {
  (void)pvParameters;

  String user_input = "";

  for (;;) {
    if (Serial.available()) {
      user_input = Serial.readStringUntil('\n');
      user_input.trim();
    }

    if (user_input.length() > 0) {
      char command = user_input.charAt(0);
      float val = 0.0f;

      switch (command) {

        // Disk commands
        case 'h': cmdStart = true; break;  // Home / start
        case 'x': cmdExit = true; break;   // Exit
        case 'q': cmdStop = true; break;   // Stop
        case 'l':                          // Low speed
          cmdPWMFreq = 250.0;

          // Read rest of input
          val = strtof(user_input.substring(1).c_str(), NULL);
          cmdDir = (val > 0) ? CW : CCW;
          cmdCount = fabsf(val);
          cmdReady = true;
          break;
        case 'm':  // Medium speed
          cmdPWMFreq = 500.0;

          // Read rest of input
          val = strtof(user_input.substring(1).c_str(), NULL);
          cmdDir = (val > 0) ? CW : CCW;
          cmdCount = fabsf(val);
          cmdReady = true;
          break;
        case 'f':  // High speed
          cmdPWMFreq = 750.0;

          // Read rest of input
          val = strtof(user_input.substring(1).c_str(), NULL);
          cmdDir = (val > 0) ? CW : CCW;
          cmdCount = fabsf(val);
          cmdReady = true;
          break;


        // DC motor commands
        case 't':
          dcCmdT = true;
          dcCmdVal = user_input.substring(1).toFloat();
          break;  // Sample interval (s)
        case 'd':
          dcCmdD = true;
          dcCmdVal = user_input.substring(1).toFloat();
          break;  // Run duration (s)
        case 'v':
          dcCmdV = true;
          dcCmdVal = user_input.substring(1).toFloat();
          break;  // Target voltage (V)
        case 'p':
          dcCmdP = true;
          dcCmdVal = user_input.substring(1).toFloat();
          break;  // Kp
        case 'i':
          dcCmdI = true;
          dcCmdVal = user_input.substring(1).toFloat();
          break;  // Ki
        case 'o':
          dcCmdO = true;
          dcCmdVal = user_input.substring(1).toFloat();
          break;  // Open/closed loop

        case 's':  // Start/stop
          if (user_input.substring(1).toFloat() == 1.0) dcCmdStart = true;
          else dcCmdStop = true;
          break;

        case 'j':
          dcCmdTraj = true;
          parseTrajectory(user_input.substring(1));
          break;
      }

      user_input = "";
    }

    vTaskDelay(pdMS_TO_TICKS(10));
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
void disk_thread_func(void *pvParameters) {
  (void)pvParameters;

  byte diskNextState = DISK_HOME;

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

  /* Local copies snapshotted when cmdReady fires */
  byte loc_cmdDir = CW;
  int loc_cmdCount = 0;
  float loc_cmdFreq = 500.0;


  for (;;) {
    byte diskState = diskNextState;

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
          loc_cmdDir = cmdDir;
          loc_cmdCount = cmdCount;
          loc_cmdFreq = cmdPWMFreq;
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
          diskMotorRun(CW, loc_cmdFreq, cmdDuty);
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
          diskMotorRun(loc_cmdDir, loc_cmdFreq, cmdDuty);

          moveStartTime = millis();
          EntryRunning = 1;
        }

        // INTERRUPT: check whether another command was sent
        if (cmdReady) {
          loc_cmdDir = cmdDir;
          loc_cmdCount = cmdCount;
          loc_cmdFreq = cmdPWMFreq;
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
          moveStartTime = millis();  // Reset 'in notch' timer
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
        if (cmdStop || cmdExit) {
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
          vTaskSuspend(dc_task_handle);
          vTaskSuspend(NULL);
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
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
void dc_thread_func(void *pvParameters) {
  (void)pvParameters;

  byte dcNextState = DC_IDLE;

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
  double integral = 0.0;
  double controlOutput = 0.0;
  int currentSegment = 0;
  int loc_numSeg = 0;
  Segment loc_segments[MAX_SEGMENTS];

  // DC timing
  unsigned long segStartTime_ms = 0;
  unsigned long plotStartTime_ms = 0;
  unsigned long lastSampleTime_ms = 0;

  for (;;) {
    // Apply parameter updates before going into state logic
    if (dcCmdT) {
      float t = dcCmdVal;
      if (t < 0.001) t = 0.001;  // TODO: do we need this?
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
    /* Snapshot trajectory segments into local storage */
    if (dcCmdTraj) {
      loc_numSeg = numSegments;
      for (int s = 0; s < loc_numSeg; s++) {
        loc_segments[s].startV = segments[s].startV;
        loc_segments[s].endV = segments[s].endV;
        loc_segments[s].duration = segments[s].duration;
      }
      dcCmdTraj = false;
    }

    byte dcState = dcNextState;

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
          EntryDCIdle = 0;

          // If trajectory segments loaded, go to trajectory mode
          if (loc_numSeg > 0) {
            currentSegment = 0;
            dcNextState = DC_TRAJECTORY;
          } else {
            dcNextState = DC_PLOTTING;
          }
        }

        dcCmdStop = false;
        break;


      case DC_TRAJECTORY:
        // ENTRY: start first segment
        if (EntryDCPlotting == 0) {
          segStartTime_ms = millis();
          lastSampleTime_ms = segStartTime_ms;
          integral = 0.0;
          controlOutput = 0.0;
          EntryDCPlotting = 1;
          EntryDCIdle = 0;
        }

        // EXIT: stop command sent
        if (dcCmdStop) {
          dcCmdStop = false;
          loc_numSeg = 0;
          dcMotorStop();
          EntryDCPlotting = 0;
          dcNextState = DC_IDLE;
          break;
        }

        // ACTION: sample on interval
        if ((millis() - lastSampleTime_ms) >= (unsigned long)sampleInterval_ms) {
          lastSampleTime_ms = millis();

          Segment &seg = loc_segments[currentSegment];
          float elapsed = (float)(millis() - segStartTime_ms);

          // Linear interpolation within segment
          float alpha = constrain(elapsed / (seg.duration * 1000.0f), 0.0f, 1.0f);
          float targetV = seg.startV + alpha * (seg.endV - seg.startV);

          float voltage = dcSensorReadVolts();

          if (openLoop) {
            // Open loop = desired is exactly applied as output
            controlOutput = targetV;
          } else {
            // Closed-loop PI: compute error and accumulate integral
            double error = targetV - voltage;

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
          Serial.print(pwmVal);
          Serial.print(',');
          Serial.println(targetV);
        }

        // ADVANCE: segment duration elapsed, move to next
        if ((millis() - segStartTime_ms) >= (unsigned long)(loc_segments[currentSegment].duration * 1000.0f)) {
          currentSegment++;

          // EXIT: All segments done
          if (currentSegment >= loc_numSeg) {
            dcMotorStop();
            loc_numSeg = 0;
            EntryDCPlotting = 0;
            dcNextState = DC_IDLE;
          } else {
            // Start next segment, reset integrator and timer
            segStartTime_ms = millis();
            integral = 0.0;
          }
        }
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

        // EXIT: stop command sent
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

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Parse segment trajectories
void parseTrajectory(String data) {
  numSegments = 0;
  int idx = 0;

  while (data.length() > 0 && numSegments < MAX_SEGMENTS) {
    float startV = nextFloat(data, idx);
    float endV = nextFloat(data, idx);
    float duration = nextFloat(data, idx);

    segments[numSegments].startV = startV;
    segments[numSegments].endV = endV;
    segments[numSegments].duration = duration;
    numSegments++;
  }
}


float nextFloat(String &data, int &idx) {
  int commaPos = data.indexOf(',');
  float val;

  if (commaPos == -1) {
    // Last token
    val = data.toFloat();
    data = "";
  } else {
    val = data.substring(0, commaPos).toFloat();
    data = data.substring(commaPos + 1);
  }
  return val;
}


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
  pwm.end();
  digitalWrite(dirPin, dir);
  pwm.begin(freq, 0.0f);
  pwm.pulse_perc(duty);
}

/** Stop the disk */
void diskMotorStop() {
  pwm.end();
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
