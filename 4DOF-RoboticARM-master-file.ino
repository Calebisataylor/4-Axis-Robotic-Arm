// MASTER ESP32 -- 4-axis cable arm + gripper 
// BLUETOOTH EDITION (Bluepad32 / Nimbus Controller)
// Features: 3-Point Synchronized Ping-Pong Cycling, R2 Claw, A/Y Homing

#include <FastAccelStepper.h>
#include <Bluepad32.h>

// ---------- Stepper pins ----------
#define STEP_BOT 27
#define DIR_BOT  14
#define STEP_MID 25
#define DIR_MID  26
#define STEP_TOP 32
#define DIR_TOP  33
#define STEP_YAW 13   
#define DIR_YAW  4    

// ---------- Serial link to claw ESP32 ----------
#define LINK_TX 17

ControllerPtr myController = nullptr;

// ---------- Motion tuning ----------
const uint32_t MAX_SPEED     = 12500;  
const uint32_t MAX_SPEED_YAW = 8000;   
const uint32_t ACCEL         = 20000;
const int      DEADZONE      = 40;      // Lowered for Bluetooth gamepads
const float    JOY_SENSITIVITY = 0.04f; // Scales gamepad input to degree delta

// ---------- Transmission ----------
const float MOTOR_STEPS_PER_REV = 200.0;
const float MICROSTEP           = 16.0;

const float GEARBOX[4]     = { 30.0, 20.0, 20.0, 30.0 };
const float SPOOL_DRIVE[4] = { 36.0, 36.0, 36.0, 1.0 };
const float SPOOL_JOINT[4] = { 40.0, 40.0, 40.0, 1.0 };
float STEPS_PER_DEG[4] = { 0, 0, 0, 0 };

int STEP_SIGN[4] = { -1, -1, -1, +1 };
const float JOINT_RANGE = 90.0;   

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* M[4] = { nullptr, nullptr, nullptr, nullptr };
const int STEP_PIN[4] = { STEP_BOT, STEP_MID, STEP_TOP, STEP_YAW };
const int DIR_PIN[4]  = { DIR_BOT,  DIR_MID,  DIR_TOP,  DIR_YAW  };

// --- State Variables ---
bool  homed = false;
bool  returningHome = false;
float qCur[4] = { 0, 0, 0, 0 };

// --- Claw Variables ---
bool clawClosed = false;
bool r2Prev = false; // To track trigger pulls
unsigned long lastClawUpdate = 0;
const unsigned long CLAW_HEARTBEAT = 50;

// --- Waypoint / Cycling Variables ---
float pos1[4] = {0, 0, 0, 0};
float pos2[4] = {0, 0, 0, 0};
float pos3[4] = {0, 0, 0, 0};
bool  hasPos1 = false;
bool  hasPos2 = false;
bool  hasPos3 = false;

bool  isCycling = false;
int   cycleTarget = 1; 
int   cycleDir = 1;    // 1 = moving forward (1->2->3), -1 = moving backward (3->2->1)

// --- Button State Trackers ---
bool  aPrev = false;
bool  yPrev = false;
bool  dLeftPrev = false;
bool  dUpPrev = false;
bool  dRightPrev = false;

// --- Telemetry Timer ---
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200; 

// =========================================================================
// BLUEPAD32 CONTROLLER CALLBACKS
// =========================================================================
void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    myController = ctl;
    Serial.println("\n--- CONTROLLER CONNECTED! ---");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    myController = nullptr;
    Serial.println("\n--- CONTROLLER DISCONNECTED! ---");
  }
}

// =========================================================================
// MATH & KINEMATICS
// =========================================================================
void computeStepsPerDeg() {
  for (int j = 0; j < 4; j++)
    STEPS_PER_DEG[j] = (MOTOR_STEPS_PER_REV * MICROSTEP) * GEARBOX[j] *
                       (SPOOL_JOINT[j] / SPOOL_DRIVE[j]) / 360.0;
}

long  degToSteps(int j, float q) { return lround((float)STEP_SIGN[j] * q * STEPS_PER_DEG[j]); }
float stepsToDeg(int j, long s)  { return ((float)s / STEPS_PER_DEG[j]) * (float)STEP_SIGN[j]; }

void readAngles() {
  for (int j = 0; j < 4; j++)
    qCur[j] = M[j] ? stepsToDeg(j, M[j]->getCurrentPosition()) : 0;
}

void applyBounds(const float qDesired[3], float qSafe[3]) {
  qSafe[0] = constrain(qDesired[0], -JOINT_RANGE, JOINT_RANGE);
  qSafe[1] = constrain(qDesired[1], qSafe[0] - JOINT_RANGE, qSafe[0] + JOINT_RANGE);
  qSafe[2] = constrain(qDesired[2], qSafe[1] - JOINT_RANGE, qSafe[1] + JOINT_RANGE);
}

// =========================================================================
// SYNCHRONIZED CYCLING (Adjusts speeds so all joints finish at the same time)
// =========================================================================
void moveToSync(float targetDeg[4]) {
  long targetSteps[4];
  long currentSteps[4];
  long deltaSteps[4];
  float times[4];
  float maxTime = 0.0;

  // 1. Calculate how far each motor needs to go and how long it would take at max speed
  for (int j = 0; j < 4; j++) {
    if (M[j]) {
      targetSteps[j] = degToSteps(j, targetDeg[j]);
      currentSteps[j] = M[j]->getCurrentPosition();
      deltaSteps[j] = abs(targetSteps[j] - currentSteps[j]);
      
      uint32_t jointMaxSpeed = (j == 3) ? MAX_SPEED_YAW : MAX_SPEED;
      times[j] = (float)deltaSteps[j] / (float)jointMaxSpeed;
      if (times[j] > maxTime) {
        maxTime = times[j]; 
      }
    }
  }

  // 2. Adjust speeds proportionally and command the move
  for (int j = 0; j < 4; j++) {
    if (M[j]) {
      uint32_t syncSpeed = 10; 
      if (maxTime > 0.001) {
        syncSpeed = (uint32_t)((float)deltaSteps[j] / maxTime);
      }
      if (syncSpeed < 10) syncSpeed = 10; // Prevent full stall
      
      M[j]->setSpeedInHz(syncSpeed); 
      M[j]->moveTo(targetSteps[j]);
    }
  }
}

// =========================================================================
// MANUAL UPDATES & HOMING
// =========================================================================
void manualUpdate(int s[4]) {
  float delta[4] = { 0, 0, 0, 0 };
  for (int j = 0; j < 4; j++)
    if (abs(s[j]) > DEADZONE) delta[j] = (float)s[j] * JOY_SENSITIVITY;

  float qDesired[3] = { qCur[0] + delta[0], qCur[1] + delta[1], qCur[2] + delta[2] };
  float qTarget[3];
  applyBounds(qDesired, qTarget);
  
  for (int j = 0; j < 3; j++) {
    if (M[j]) {
      M[j]->setSpeedInHz(MAX_SPEED); // Restore max speed from cycle adjustments
      M[j]->moveTo(degToSteps(j, qTarget[j]));
    }
  }

  float yawTarget = qCur[3] + delta[3];
  if (M[3]) {
    M[3]->setSpeedInHz(MAX_SPEED_YAW); // Restore max speed
    M[3]->moveTo(degToSteps(3, yawTarget));
  }
}

void doSetHome() {
  Serial.println("\n--- HOMING SET ---");
  returningHome = false;
  isCycling = false;
  for (int j = 0; j < 4; j++) if (M[j]) M[j]->setCurrentPosition(0);
  homed = true;
}

void doReturnHome() {
  Serial.println("\n--- RETURNING TO HOME ---");
  returningHome = true;
  isCycling = false;
  for (int j = 0; j < 4; j++) {
    if (M[j]) {
      M[j]->setSpeedInHz(j == 3 ? MAX_SPEED_YAW : MAX_SPEED); // Restore max speed
      M[j]->moveTo(0);   
    }
  }
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, LINK_TX); 
  
  BP32.setup(&onConnectedController, &onDisconnectedController);
  
  computeStepsPerDeg();
  engine.init();
  for (int j = 0; j < 4; j++) {
    M[j] = engine.stepperConnectToPin(STEP_PIN[j]);
    M[j]->setDirectionPin(DIR_PIN[j]);
    M[j]->setSpeedInHz(j == 3 ? MAX_SPEED_YAW : MAX_SPEED);
    M[j]->setAcceleration(ACCEL);
  }
  Serial.println("System Ready. Waiting for Bluetooth Controller...");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  BP32.update(); // Read Bluetooth data
  readAngles();

  // Print Telemetry
  if (homed && (millis() - lastPrintTime >= PRINT_INTERVAL)) {
    lastPrintTime = millis();
    Serial.printf("J1: %.2f | J2: %.2f | J3: %.2f | YAW: %.2f\n", qCur[0], qCur[1], qCur[2], qCur[3]);
  }

  // Transmit Claw state to Slave ESP32 every 50ms
  if (millis() - lastClawUpdate >= CLAW_HEARTBEAT) {
    lastClawUpdate = millis();
    Serial2.write(clawClosed ? 'C' : 'O');
  }

  // --- CONTROLLER LOGIC ---
  if (myController && myController->isConnected()) {
    
    // 1. Homing Buttons
    bool btnA = myController->a();
    bool btnY = myController->y();
    if (btnA && !aPrev) doSetHome();
    if (btnY && !yPrev) doReturnHome();
    aPrev = btnA;
    yPrev = btnY;

    // 2. Right Trigger Claw Toggle (Nimbus Trigger > 500 or R2 Button)
    bool r2Pressed = (myController->brake() > 500) || myController->r2(); 
    if (r2Pressed && !r2Prev) {
      clawClosed = !clawClosed;
      Serial.print("\n--- CLAW ");
      Serial.print(clawClosed ? "CLOSED" : "OPEN");
      Serial.println(" ---");
    }
    r2Prev = r2Pressed;

    // 3. Waypoint Logic (D-Pad)
    bool dLeft  = (myController->dpad() == DPAD_LEFT);
    bool dUp    = (myController->dpad() == DPAD_UP);
    bool dRight = (myController->dpad() == DPAD_RIGHT);
    bool dDown  = (myController->dpad() == DPAD_DOWN);

    if (homed) {
      // Save Positions
      if (dLeft && !dLeftPrev) {
        for(int j=0; j<4; j++) pos1[j] = qCur[j];
        hasPos1 = true;
        Serial.println("Position 1 Saved!");
      }
      if (dUp && !dUpPrev) {
        for(int j=0; j<4; j++) pos2[j] = qCur[j];
        hasPos2 = true;
        Serial.println("Position 2 Saved!");
      }
      if (dRight && !dRightPrev) {
        for(int j=0; j<4; j++) pos3[j] = qCur[j];
        hasPos3 = true;
        Serial.println("Position 3 Saved!");
      }

      // Cycle Logic (Hold Down)
      if (dDown && hasPos1 && hasPos2 && hasPos3) {
        if (!isCycling) {
          isCycling = true;
          cycleTarget = 1; 
          cycleDir = 1;
          moveToSync(pos1); 
          Serial.println("Started Synchronized Cycling!");
        }

        bool stillMoving = false;
        for (int j=0; j<4; j++) {
          if (M[j] && M[j]->isRunning()) stillMoving = true;
        }

        // Ping-Pong Sequence
        if (!stillMoving) {
          if (cycleTarget == 1) {
            cycleTarget = 2;
            cycleDir = 1;
          } else if (cycleTarget == 2) {
            if (cycleDir == 1) cycleTarget = 3;
            else cycleTarget = 1;
          } else if (cycleTarget == 3) {
            cycleTarget = 2;
            cycleDir = -1;
          }
          
          float* nextPos = (cycleTarget == 1) ? pos1 : ((cycleTarget == 2) ? pos2 : pos3);
          moveToSync(nextPos);
        }
      } else {
        // Halt if D-Pad released
        if (isCycling) {
          isCycling = false;
          for (int j=0; j<4; j++) {
            if (M[j]) M[j]->stopMove(); 
          }
          Serial.println("Cycling Stopped.");
        }
      }
    }
    dLeftPrev = dLeft;
    dUpPrev = dUp;
    dRightPrev = dRight;

    // 4. Manual Stick Movement
    if (homed && !isCycling) {
      int s[4] = {
        myController->axisX(),   // JOG0 Shoulder
        myController->axisRX(),  // JOG1 Elbow
        myController->axisRY(),  // JOG2 Wrist
        myController->axisY()    // JOG3 Base Yaw
      };

      if (returningHome) {
        bool userInput   = (abs(s[0]) > DEADZONE) || (abs(s[1]) > DEADZONE) ||
                           (abs(s[2]) > DEADZONE) || (abs(s[3]) > DEADZONE);
        bool stillMoving = (M[0] && M[0]->isRunning()) || (M[1] && M[1]->isRunning()) ||
                           (M[2] && M[2]->isRunning()) || (M[3] && M[3]->isRunning());
        if (userInput || !stillMoving) returningHome = false;
      }

      if (!returningHome) manualUpdate(s);
    }
  }
}