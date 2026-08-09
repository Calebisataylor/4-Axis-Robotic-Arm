/*  SERVO ESP32 -- Claw slave, per-finger control over the serial link
 *  -----------------------------------------------------------------
 *  Receives one byte every 50ms from the master ESP32 on Serial2:
 *
 *    NEW (per-finger):  0x80 | mask   -- bit0=F0 bottom, bit1=F1 right,
 *                                        bit2=F2 left,  bit3=F3 top  (1=closed)
 *    LEGACY:            'C' = close all,  'O' = open all
 *
 *  Both are supported, so the joystick jog sketch (which sends 'C'/'O') still
 *  works unchanged. Same open/closed angles, speeds, and boot-to-open behavior.
 *
 *  Library: ESP32Servo
 *
 *  WIRING:
 *    F0 blue  bottom -> D23
 *    F1 green right  -> D22
 *    F2 yellow left  -> D21
 *    F3 orange top   -> D19
 *    Link from master:  RX2 (GPIO16) <- master TX2 (GPIO17),  GND <-> GND
 *
 *  POWER: servos on a SEPARATE 5 V supply; all grounds share one rail.
 */

#include <ESP32Servo.h>

#define LINK_RX 16   // <- master TX2 (GPIO 17). RX only.

// -- Pins -------------------------------------------------------------
//                        F0   F1   F2   F3
const int FINGER_PIN[4] = { 23,  22,  21,  19 };  // blue / green / yellow / orange

// -- Calibration (degrees) -- unchanged -------------------------------
//                  fin4          fin1           F0blue   F1green   F2yellow   F3orange
int OPEN_POS[4]   = { 55,  0,  0, 55 };
int CLOSED_POS[4] = {  0, 55, 55,  0 };

// -- Tuning -- unchanged ----------------------------------------------
const int MOVE_INTERVAL = 5;     // ms between steps (was 15 -> ~3x faster fingers)
const int STEP_GRIP     = 8;     // deg/step -- closing (2x speed)
const int STEP_NORMAL   = 5;     // deg/step -- reopening

// -- State ------------------------------------------------------------
Servo finger[4];
int pos[4];
uint8_t gripMask = 0x00;          // bit i set => finger i closed (boot: all open)
unsigned long lastMove = 0;

int stepToward(int cur, int tgt, int step) {
  if (cur < tgt)      { cur += step; if (cur > tgt) cur = tgt; }
  else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
  return cur;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, LINK_RX, -1);  // RX only on GPIO 16

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 4; i++) {
    finger[i].setPeriodHertz(50);
    finger[i].attach(FINGER_PIN[i], 544, 2400);
    pos[i] = OPEN_POS[i];          // boot to open -- no snap
    finger[i].write(pos[i]);
  }

  Serial.println(F("\nClaw slave ready (per-finger + legacy C/O)."));
}

void loop() {
  // Latest command wins (master heartbeats every 50 ms).
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    if      (b & 0x80)  gripMask = b & 0x0F;    // per-finger frame
    else if (b == 'C')  gripMask = 0x0F;        // legacy close all
    else if (b == 'O')  gripMask = 0x00;        // legacy open all
  }

  // Smooth ramp each finger toward its own target (same speeds as before).
  if (millis() - lastMove >= MOVE_INTERVAL) {
    lastMove = millis();
    for (int i = 0; i < 4; i++) {
      bool closeThis = gripMask & (1 << i);
      int  tgt  = closeThis ? CLOSED_POS[i] : OPEN_POS[i];
      int  step = closeThis ? STEP_GRIP     : STEP_NORMAL;
      pos[i] = stepToward(pos[i], tgt, step);
      finger[i].write(pos[i]);
    }
  }
}
