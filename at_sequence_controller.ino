/*
 * [AT00 / AT01 sync and speed tuning + AT02 absolute coordinate integration]
 *
 * 1. AT00 & AT01
 * - Stage 1: Move R-axis in 'v' direction, then set R = 0 when sensor 38 is detected.
 * - Stage 2: Move R-axis in 'n' direction until the final R position is 800.
 * - Stage 3: Move in '6' direction, Z-up + X-right, then set Z = 0 and X = 0
 *            when each axis reaches its sensor.
 * - Wait: Pause for 1 second before driving the Y-axis.
 * - Stage 4: Drive Y-axis in R-key direction for 2 seconds, then set Y = 0.
 * - Stage 5: Grip sequence: hold 1s, release 1s, hold 1s, release 1s.
 *
 * 2. Z-axis
 * - Z = 0 is the top position.
 * - Moving downward in the 'B' direction decreases the Z position value.
 *
 * 3. AT02
 * - Target absolute coordinate: X = -2500, Z = -8000.
 * - AT02 speed mapping: minSpd = 750, maxSpd = 1350.
 */

// --- Global state ---
bool isRunning = false;
char currentMode = 'S';
bool stopZRight = false, stopZLeft = false, stopX = false;
long targetSteps = 0, currentStep = 0;
unsigned long lastStepTime = 0;
int diagonalCounter = 0;

int roHomingCounter = 0;
int roManualCounter = 0;

// Standard 4-axis absolute position tracking.
long xCurrentPosition = 0;
long zCurrentPosition = 0;
long rCurrentPosition = 0;
long yCurrentPosition = 0;

// AT02 absolute coordinate control.
long at02_targetX_abs = -2500;
long at02_targetZ_abs = -8000;
long at02_needStepsX = 0;
long at02_needStepsZ = 0;
long at02_movedX = 0;
long at02_movedZ = 0;
int at02_dirX = HIGH;
int at02_dirZ = LOW;

// Automation sequence control.
int homingStage = 0;
unsigned long homingTimer1 = 0;

String inputString = "";

// --- Pin definitions ---
const int zStepR = 23; const int zDirR = 22;
const int zStepL = 25; const int zDirL = 24;
const int xStep  = 27; const int xDir  = 26;
const int rStep  = 29; const int rDir  = 28;
const int yStep  = 31; const int yDir  = 30;
const int grip1  = 32; const int grip2  = 33;

// Sensor pin mapping.
const int SEN_1_X_LT = 34;   const int SEN_2_X_RT = 35;
const int SEN_3_ZL_TOP = 36; const int SEN_4_RO_RT = 37;
const int SEN_5_RO_LT = 38;  const int SEN_6_Z_BTM = 39;
const int SEN_7_ZR_TOP = 40; const int SEN_8_Y_OUT = 41;
const int SEN_9_Y_IN = 42;

const int Z_UP_DIR_R = LOW; const int Z_UP_DIR_L = LOW;

// --- Automation sequence stage transition ---
void startHomingStage(int stage) {
  homingStage = stage;
  diagonalCounter = 0;
  roHomingCounter = 0;
  roManualCounter = 0;
  stopZRight = false; stopZLeft = false; stopX = false;
  currentStep = 0;
  lastStepTime = micros();

  // --- AT00 / AT01 sequence ---
  if (stage == 1) {
    currentMode = 'O';
    targetSteps = 999999;
    digitalWrite(rDir, LOW);
    Serial.println(F(">> AT00/AT01 Stage 1: R-Axis 'v' direction (2/3 Slow Speed)..."));
  }
  else if (stage == 2) {
    currentMode = 'O';
    targetSteps = 999999;
    digitalWrite(rDir, HIGH);
    Serial.println(F(">> AT00/AT01 Stage 2: R-Axis 'n' direction to Target 800 (2/3 Slow Speed)..."));
  }
  else if (stage == 3) {
    currentMode = 'O';
    targetSteps = 999999;
    digitalWrite(zDirR, Z_UP_DIR_R); digitalWrite(zDirL, Z_UP_DIR_L);
    digitalWrite(xDir, LOW);
    Serial.println(F(">> AT00/AT01 Stage 3: Moving in '6' direction (Z-Up + X-Right Accelerated)..."));
  }
  else if (stage == 35) {
    currentMode = 'O';
    homingTimer1 = millis();
    Serial.println(F(">> AT00/AT01 Stage 3.5: Waiting for 1 Second before Y-Axis Move..."));
  }
  else if (stage == 4) {
    currentMode = 'O';
    digitalWrite(yDir, LOW);
    homingTimer1 = millis();
    Serial.println(F(">> AT00/AT01 Stage 4: Timer Gated Y-Axis Active for 2 Seconds..."));
  }
  else if (stage == 5) {
    currentMode = 'O';
    homingTimer1 = millis();
    digitalWrite(grip1, HIGH); digitalWrite(grip2, HIGH);
    Serial.println(F(">> AT00/AT01 Stage 5: Grip C/D Sequence Active (4 Seconds)..."));
  }

  // --- AT02 absolute coordinate residual tracking sequence ---
  else if (stage == 20) {
    currentMode = 'E';

    at02_needStepsX = at02_targetX_abs - xCurrentPosition;
    at02_needStepsZ = at02_targetZ_abs - zCurrentPosition;

    at02_movedX = 0;
    at02_movedZ = 0;

    if (at02_needStepsX == 0 && at02_needStepsZ == 0) {
      Serial.println(F(">> AT02: Already at target position. No movement needed."));
      isRunning = false;
      homingStage = 0;
      currentMode = 'S';
      return;
    }

    if (at02_needStepsX < 0) {
      at02_dirX = HIGH;
      at02_needStepsX = -at02_needStepsX;
    } else {
      at02_dirX = LOW;
    }

    if (at02_needStepsZ < 0) {
      at02_dirZ = !Z_UP_DIR_R;
      at02_needStepsZ = -at02_needStepsZ;
    } else {
      at02_dirZ = Z_UP_DIR_R;
    }

    digitalWrite(xDir, at02_dirX);
    digitalWrite(zDirR, at02_dirZ); digitalWrite(zDirL, at02_dirZ);

    targetSteps = (at02_needStepsX > at02_needStepsZ) ? at02_needStepsX : at02_needStepsZ;

    Serial.print(F(">> AT02 Active -> Actual Required Steps [ X: ")); Serial.print(at02_needStepsX);
    Serial.print(F(" | Z: ")); Serial.print(at02_needStepsZ);
    Serial.println(F(" ] Gated System Ready."));
  }
}

// --- Safety monitoring and multi-stage homing sequence control ---
void monitorSafety() {
  if (!isRunning) return;
  if (currentMode == 'S') { isRunning = false; homingStage = 0; return; }

  if (currentMode == 'O' || currentMode == 'E') {
    if (homingStage == 1) {
      if (digitalRead(SEN_5_RO_LT) == LOW) { rCurrentPosition = 0; startHomingStage(2); }
    }
    else if (homingStage == 2) {
      if (rCurrentPosition <= -800) { rCurrentPosition = 800; startHomingStage(3); }
    }
    else if (homingStage == 3) {
      if (digitalRead(SEN_3_ZL_TOP) == LOW) stopZLeft = true;
      if (digitalRead(SEN_7_ZR_TOP) == LOW) stopZRight = true;
      if (digitalRead(SEN_2_X_RT) == LOW) stopX = true;

      if (stopZLeft && stopZRight && stopX) {
        zCurrentPosition = 0;
        xCurrentPosition = 0;
        startHomingStage(35);
      }
    }
    else if (homingStage == 20) {
      if (at02_dirX == HIGH && digitalRead(SEN_1_X_LT) == LOW)  { stopX = true; }
      if (at02_dirX == LOW  && digitalRead(SEN_2_X_RT) == LOW)  { stopX = true; }
      if (at02_dirZ == !Z_UP_DIR_R && digitalRead(SEN_6_Z_BTM) == LOW) { stopZLeft = true; stopZRight = true; }
      if (at02_dirZ == Z_UP_DIR_R && (digitalRead(SEN_3_ZL_TOP) == LOW || digitalRead(SEN_7_ZR_TOP) == LOW)) {
        stopZLeft = true; stopZRight = true;
      }
    }
    return;
  }

  // --- Manual mode safety sensor logic ---
  if (currentMode == 'U' || currentMode == '4' || currentMode == '6') {
    if (digitalRead(SEN_3_ZL_TOP) == LOW) stopZLeft = true;
    if (digitalRead(SEN_7_ZR_TOP) == LOW) stopZRight = true;
  }
  if ((currentMode == 'L' || currentMode == '4') && digitalRead(SEN_1_X_LT) == LOW) stopX = true;
  if ((currentMode == 'R' || currentMode == '6') && digitalRead(SEN_2_X_RT) == LOW) stopX = true;
  if (currentMode == 'D' && digitalRead(SEN_6_Z_BTM) == LOW) isRunning = false;
  if (currentMode == 'v' && digitalRead(SEN_5_RO_LT) == LOW) isRunning = false;
  if (currentMode == 'n' && digitalRead(SEN_4_RO_RT) == LOW) isRunning = false;
  if (currentMode == 'Y' && digitalRead(SEN_8_Y_OUT) == LOW) isRunning = false;
  if (currentMode == 'r' && digitalRead(SEN_9_Y_IN) == LOW)  isRunning = false;
  if ((currentMode == '4' || currentMode == '6') && stopZLeft && stopZRight && stopX) isRunning = false;
}

// --- Motion execution ---
void executeStep() {
  if (currentMode == 'E' && homingStage == 20) {
    if (at02_movedZ < at02_needStepsZ) {
      if (!stopZRight && !stopZLeft) {
        digitalWrite(zStepR, LOW); digitalWrite(zStepL, LOW); delayMicroseconds(1);
        digitalWrite(zStepR, HIGH); digitalWrite(zStepL, HIGH);
        if (at02_dirZ == !Z_UP_DIR_R) zCurrentPosition--;
        else zCurrentPosition++;
        at02_movedZ++;
      }
    }
    if (at02_movedX < at02_needStepsX) {
      if (!stopX) {
        digitalWrite(xStep, LOW); delayMicroseconds(1); digitalWrite(xStep, HIGH);
        if (at02_dirX == HIGH) xCurrentPosition--;
        else xCurrentPosition++;
        at02_movedX++;
      }
    }
    return;
  }

  // Z-axis.
  if (currentMode == 'U' || currentMode == '4' || currentMode == '6' || currentMode == 'D' || (currentMode == 'O' && homingStage == 3)) {
    if (currentMode == 'D') {
      digitalWrite(zStepR, LOW); digitalWrite(zStepL, LOW); delayMicroseconds(1);
      digitalWrite(zStepR, HIGH); digitalWrite(zStepL, HIGH);
      zCurrentPosition--;
    } else {
      if (!stopZRight) { digitalWrite(zStepR, LOW); delayMicroseconds(1); digitalWrite(zStepR, HIGH); }
      if (!stopZLeft)  { digitalWrite(zStepL, LOW); delayMicroseconds(1); digitalWrite(zStepL, HIGH); }
      zCurrentPosition++;
    }
  }

  // X-axis.
  if (currentMode == 'L' || currentMode == 'R' || currentMode == '4' || currentMode == '6' || (currentMode == 'O' && homingStage == 3)) {
    if (!stopX) {
      if (currentMode == '4' || currentMode == '6' || (currentMode == 'O' && homingStage == 3)) {
        diagonalCounter++;
        if (diagonalCounter >= 8) {
          digitalWrite(xStep, LOW); delayMicroseconds(1); digitalWrite(xStep, HIGH);
          diagonalCounter = 0;
        }
      } else {
        digitalWrite(xStep, LOW); delayMicroseconds(1); digitalWrite(xStep, HIGH);
      }
      if (currentMode == 'L' || currentMode == '4') xCurrentPosition--;
      if (currentMode == 'R' || currentMode == '6' || (currentMode == 'O' && homingStage == 3)) xCurrentPosition++;
    }
  }

  // R-axis.
  if (currentMode == 'v' || currentMode == 'n' || (currentMode == 'O' && (homingStage == 1 || homingStage == 2))) {
    bool pulseTriggered = false;
    if (currentMode == 'O') {
      if (homingStage == 1) {
        roHomingCounter++;
        if (roHomingCounter >= 3) {
          digitalWrite(rStep, LOW); delayMicroseconds(1); digitalWrite(rStep, HIGH);
          roHomingCounter = 0; pulseTriggered = true;
        }
      } else if (homingStage == 2) {
        digitalWrite(rStep, LOW); delayMicroseconds(1); digitalWrite(rStep, HIGH); pulseTriggered = true;
      }
    }
    else {
      roManualCounter++;
      if (roManualCounter == 1 || roManualCounter == 2) {
        digitalWrite(rStep, LOW); delayMicroseconds(1); digitalWrite(rStep, HIGH); pulseTriggered = true;
      }
      if (roManualCounter >= 3) { roManualCounter = 0; }
    }
    if (pulseTriggered) {
      if (currentMode == 'v' || (currentMode == 'O' && homingStage == 1)) rCurrentPosition++;
      if (currentMode == 'n' || (currentMode == 'O' && homingStage == 2)) rCurrentPosition--;
    }
  }

  // Y-axis.
  if (currentMode == 'Y' || currentMode == 'r' || (currentMode == 'O' && homingStage == 4)) {
    digitalWrite(yStep, LOW); delayMicroseconds(1); digitalWrite(yStep, HIGH);
    if (currentMode == 'Y') yCurrentPosition++;
    if (currentMode == 'r' || (currentMode == 'O' && homingStage == 4)) yCurrentPosition--;
  }
}

// --- Speed calculation ---
int calculateInterval() {
  int minSpd = 300, maxSpd = 600;
  if (currentMode == 'U' || currentMode == 'D' || currentMode == '4' || currentMode == '6') { minSpd = 120; maxSpd = 400; }

  if (currentMode == 'O') {
    if (homingStage == 1 || homingStage == 2) {
      minSpd = 450; maxSpd = 900;
    } else {
      minSpd = 200; maxSpd = 360;
    }
  }
  else if (currentMode == 'L' || currentMode == 'R') {
    minSpd = 200; maxSpd = 360;
  }

  if (currentMode == 'E') { minSpd = 750; maxSpd = 1350; }

  const int ramp = 500;
  if (currentStep < ramp) return map(currentStep, 0, ramp, maxSpd, minSpd);
  else if (currentStep > targetSteps - ramp && targetSteps > ramp) return map(currentStep, targetSteps - ramp, targetSteps, minSpd, maxSpd);
  else return minSpd;
}

// General shortcut movement.
void startMove(long steps, char mode) {
  homingStage = 0;
  targetSteps = steps; currentStep = 0; currentMode = mode; isRunning = true;
  lastStepTime = micros(); diagonalCounter = 0; roHomingCounter = 0; roManualCounter = 0;
  stopZRight = false; stopZLeft = false; stopX = false;

  if (mode == 'U' || mode == '4' || mode == '6') { digitalWrite(zDirR, Z_UP_DIR_R); digitalWrite(zDirL, Z_UP_DIR_L); }
  if (mode == 'D') { digitalWrite(zDirR, !Z_UP_DIR_R); digitalWrite(zDirL, !Z_UP_DIR_L); }
  if (mode == 'L' || mode == '4') { digitalWrite(xDir, HIGH); }
  if (mode == 'R' || mode == '6') { digitalWrite(xDir, LOW); }
  if (mode == 'v') { digitalWrite(rDir, LOW); }
  if (mode == 'n') { digitalWrite(rDir, HIGH); }
  if (mode == 'Y') { digitalWrite(yDir, HIGH); }
  if (mode == 'r') { digitalWrite(yDir, LOW); }
}

void setup() {
  pinMode(zStepR, OUTPUT); pinMode(zDirR, OUTPUT);
  pinMode(zStepL, OUTPUT); pinMode(zDirL, OUTPUT);
  pinMode(xStep,  OUTPUT); pinMode(xDir,  OUTPUT);
  pinMode(rStep,  OUTPUT); pinMode(rDir,  OUTPUT);
  pinMode(yStep,  OUTPUT); pinMode(yDir,  OUTPUT);
  pinMode(grip1,  OUTPUT); pinMode(grip2,  OUTPUT);
  for (int i = 34; i <= 42; i++) pinMode(i, INPUT_PULLUP);
  Serial.begin(115200);
  inputString.reserve(10);
  Serial.println(F("System Online. R-Axis Set to 800 / AT02 Z Set to 8000."));
}

void loop() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();

    if (inChar == 's' || inChar == 'S' || inChar == 'g' || inChar == 'G') {
      isRunning = false; currentMode = 'S'; homingStage = 0; inputString = "";
      Serial.println(F("!!! EMERGENCY STOP !!!"));
    }
    else if (inChar == 'c' || inChar == 'C') { digitalWrite(grip1, HIGH); digitalWrite(grip2, HIGH); }
    else if (inChar == 'd' || inChar == 'D') { digitalWrite(grip1, LOW);  digitalWrite(grip2, LOW);  }

    // AT commands start only after receiving A/a.
    // This keeps a standalone T/t available for manual Z-up movement.
    else if (inChar == 'A' || inChar == 'a') {
      inputString = "";
      inputString += inChar;
    }
    else if (
      inputString.length() > 0 &&
      (inChar == 'T' || inChar == 't' || inChar == '0' || inChar == '1' || inChar == '2')
    ) {
      inputString += inChar;

      if (inputString.equalsIgnoreCase("AT00") || inputString.equalsIgnoreCase("AT01")) {
        inputString = ""; isRunning = true; startHomingStage(1);
      }
      else if (inputString.equalsIgnoreCase("AT02")) {
        inputString = ""; isRunning = true; startHomingStage(20);
      }

      if (inputString.length() > 5) inputString = "";
    }

    else {
      inputString = "";
      switch (inChar) {
        case '4': startMove(300000, '4'); break;
        case '6': startMove(300000, '6'); break;
        case 't': case 'T': startMove(200000, 'U'); break;
        case 'b': case 'B': startMove(200000, 'D'); break;
        case 'f': case 'F': startMove(60000, 'L');  break;
        case 'h': case 'H': startMove(60000, 'R');  break;
        case 'v': case 'V': startMove(40000, 'v');  break;
        case 'n': case 'N': startMove(40000, 'n');  break;
        case 'y': case 'Y': startMove(60000, 'Y');  break;
        case 'r': case 'R': startMove(60000, 'r');  break;
      }
    }
  }

  // --- Main physical control kernel ---
  if (isRunning) {
    if (currentMode == 'O' && homingStage == 35) {
      if (millis() - homingTimer1 >= 1000) { startHomingStage(4); }
    }
    else if (currentMode == 'O' && homingStage == 4) {
      unsigned long elapsed = millis() - homingTimer1;
      if (elapsed < 2000) {
        int interval = calculateInterval();
        if (micros() - lastStepTime >= (unsigned long)interval) { lastStepTime = micros(); executeStep(); }
      } else {
        yCurrentPosition = 0; Serial.println(F(">> Stage 4 Complete.")); startHomingStage(5);
      }
    }
    else if (currentMode == 'O' && homingStage == 5) {
      unsigned long elapsed = millis() - homingTimer1;
      if (elapsed < 1000)      { digitalWrite(grip1, HIGH); digitalWrite(grip2, HIGH); }
      else if (elapsed < 2000) { digitalWrite(grip1, LOW);  digitalWrite(grip2, LOW);  }
      else if (elapsed < 3000) { digitalWrite(grip1, HIGH); digitalWrite(grip2, HIGH); }
      else if (elapsed < 4000) { digitalWrite(grip1, LOW);  digitalWrite(grip2, LOW);  }
      else {
        isRunning = false; homingStage = 0; currentMode = 'S';
        Serial.println(F(">> AT00/AT01 Process Completed. <<"));
      }
    }
    else {
      if (currentStep < targetSteps) {
        monitorSafety();
        int interval = calculateInterval();
        if (micros() - lastStepTime >= (unsigned long)interval) {
          lastStepTime = micros();
          executeStep();
          currentStep++;
        }
      } else {
        if (currentMode == 'E' && homingStage == 20) {
          Serial.print(F(">> AT02 Absolute Success. Position Maintained. CurrX = ")); Serial.print(xCurrentPosition);
          Serial.print(F(" | CurrZ = ")); Serial.println(zCurrentPosition);
        }
        isRunning = false;
        homingStage = 0;
        currentMode = 'S';
      }
    }
  }
}
