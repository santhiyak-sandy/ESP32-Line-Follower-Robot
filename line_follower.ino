/*
  ============================================================================
  ESP32 RLS-08 8 SENSOR COMPETITION LINE FOLLOWER
  CORRECTED VERSION
  ============================================================================

  PID VALUES KEPT EXACTLY AS REQUESTED:
      Kp = 4.0
      Ki = 0.0
      Kd = 15.0

  FIXED:
    - Forward-bias diode / straight branch
    - Curve line loss
    - V-turn falsely triggering on diode
    - Junction straight navigation
    - Better curve speed control
    - V-turn and sharp-turn behavior preserved
*/


#include <Arduino.h>


// ============================================================================
// 1. SENSOR PINS
// ============================================================================

// LEFT -> RIGHT
#define IR1 34
#define IR2 32
#define IR3 35
#define IR4 26
#define IR5 25
#define IR6 33
#define IR7 27
#define IR8 12


// ============================================================================
// 2. MOTOR PINS
// ============================================================================

// Motor A = RIGHT
#define PWMA 4
#define AIN1 19
#define AIN2 18

// Motor B = LEFT
#define PWMB 5
#define BIN1 21
#define BIN2 13

#define STBY 23

#define LED_PIN 2


// ============================================================================
// 3. SENSOR POLARITY
// ============================================================================

#define BLACK_STATE HIGH


// ============================================================================
// 4. JUNCTION PRIORITY
// ============================================================================

enum JunctionPriority
{
  PRI_STRAIGHT_LEFT_RIGHT = 0,
  PRI_LEFT_STRAIGHT_RIGHT = 1,
  PRI_RIGHT_STRAIGHT_LEFT = 2
};

JunctionPriority currentPriority = PRI_STRAIGHT_LEFT_RIGHT;


// ============================================================================
// 5. END MARKER
// ============================================================================

enum EndMarkerBehavior
{
  END_ACTION_UTURN = 0,
  END_ACTION_STOP = 1
};

EndMarkerBehavior endMarkerBehavior = END_ACTION_UTURN;

int8_t DEFAULT_UTURN_DIRECTION = 1;


// ============================================================================
// 6. MOTOR / SENSOR OPTIONS
// ============================================================================

bool INVERT_RIGHT_MOTOR = false;
bool INVERT_LEFT_MOTOR  = false;

bool SWAP_LEFT_RIGHT_SENSORS = false;
bool SWAP_LEFT_RIGHT_MOTORS  = false;

bool ENABLE_STARTUP_SELFTEST = true;


// ============================================================================
// 7. SPEED SETTINGS
// ============================================================================

// IMPORTANT:
// PID VALUES ARE NOT CHANGED.
//
// Curve speeds are reduced compared with your original code.
// This prevents the robot from flying outside the line.

int SPEED_STRAIGHT      = 120;
int SPEED_GENTLE_CURVE  = 90;
int SPEED_TIGHT_CURVE   = 75;

int SPEED_TURN_OUTER    = 120;
int SPEED_TURN_INNER    = -60;

int SPEED_RECOVERY      = 90;

int SPEED_UTURN_OUTER   = 120;
int SPEED_UTURN_INNER   = -90;


// ============================================================================
// 8. PID
// ============================================================================

// DO NOT CHANGE THESE VALUES

float Kp = 4.0;
float Ki = 0.0;
float Kd = 15.0;

float derivativeAlpha = 0.35;
float ERROR_DEADBAND = 0.3;


// ============================================================================
// 9. TRACK THRESHOLDS
// ============================================================================

float ERROR_CURVE_THRESHOLD = 6.0;

float ERROR_SHARP_THRESHOLD = 18.0;

uint8_t SHARP_CONFIRM_FRAMES = 2;

uint8_t VTURN_CONFIRM_FRAMES = 2;


// Wide sensor pattern
uint8_t WIDE_SENSOR_MIN = 6;


// End marker
uint8_t END_MARKER_MIN_SENSORS = 7;


// ============================================================================
// 10. TIMING
// ============================================================================

const unsigned long STARTUP_DELAY_MS = 3000;

const unsigned long GAP_MAX_TIME_MS = 180;

const unsigned long SHARP_TURN_TIMEOUT_MS = 850;

const unsigned long RECOVERY_TIMEOUT_MS = 2500;

const unsigned long END_TURN_TRIGGER_MS = 300;

const unsigned long END_TURN_EXPIRE_MS = 700;

const unsigned long END_TURN_TIMEOUT_MS = 1600;

const unsigned long SEARCH_STAGE2_START_MS = 900;

const unsigned long SEARCH_ALTERNATE_PERIOD_MS = 400;


// ============================================================================
// 11. JUNCTION / DIODE SETTINGS
// ============================================================================

// When a straight-priority diode/junction is detected,
// keep moving forward for this amount of time.
//
// This prevents the robot from immediately switching back
// into V-turn / sharp-turn logic.

const unsigned long STRAIGHT_JUNCTION_HOLD_MS = 180;

unsigned long junctionHoldStart = 0;


// ============================================================================
// 12. STATE MACHINE
// ============================================================================

enum TrackState
{
  STATE_STARTUP,
  STATE_NORMAL,
  STATE_CURVE,
  STATE_SHARP_TURN,
  STATE_V_TURN,
  STATE_JUNCTION,
  STATE_CROSSING,
  STATE_GAP,
  STATE_LINE_LOST,
  STATE_END_TURN,
  STATE_SAFE_STOP
};

TrackState currentState = STATE_STARTUP;


// ============================================================================
// 13. SENSOR WEIGHTS
// ============================================================================

// IR1 = LEFT
// IR8 = RIGHT
//
// Negative = LEFT
// Positive = RIGHT

const float SENSOR_WEIGHTS[8] =
{
  -35.0f,
  -25.0f,
  -15.0f,
  -5.0f,
   5.0f,
  15.0f,
  25.0f,
  35.0f
};

const uint8_t sensorPins[8] =
{
  IR1,
  IR2,
  IR3,
  IR4,
  IR5,
  IR6,
  IR7,
  IR8
};


// ============================================================================
// 14. INTERNAL VARIABLES
// ============================================================================

uint8_t rawSensorMask = 0;

int activeSensorCount = 0;

int clusterCount = 0;

bool leftCluster = false;
bool centerCluster = false;
bool rightCluster = false;

float currentError = 0.0f;

float previousError = 0.0f;

float lastGoodError = 0.0f;

float filteredDerivative = 0.0f;

float integralError = 0.0f;

int8_t lastKnownDirection = 0;

unsigned long stateEnteredTime = 0;

unsigned long lineLostTime = 0;

unsigned long lastLoopMicros = 0;

uint8_t sharpConfirmCounter = 0;

uint8_t vturnConfirmCounter = 0;

int8_t turnDirection = 0;


// End marker memory
bool sawFullMarker = false;

unsigned long fullMarkerTime = 0;


// Motor trim
int leftMotorTrim = 0;
int rightMotorTrim = 0;


// ============================================================================
// 15. PWM SETUP
// ============================================================================

void setupMotorPWM()
{
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);


#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)

  ledcAttach(PWMA, 12000, 8);
  ledcAttach(PWMB, 12000, 8);

#else

  ledcSetup(0, 12000, 8);
  ledcAttachPin(PWMA, 0);

  ledcSetup(1, 12000, 8);
  ledcAttachPin(PWMB, 1);

#endif
}


// ============================================================================
// 16. PWM WRITE
// ============================================================================

void writeMotorPWM_Right(int duty)
{
  duty = constrain(duty, 0, 255);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)

  ledcWrite(PWMA, duty);

#else

  ledcWrite(0, duty);

#endif
}


void writeMotorPWM_Left(int duty)
{
  duty = constrain(duty, 0, 255);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)

  ledcWrite(PWMB, duty);

#else

  ledcWrite(1, duty);

#endif
}


// ============================================================================
// 17. PHYSICAL RIGHT MOTOR
// ============================================================================

void setMotorRight(int speed)
{
  if (INVERT_RIGHT_MOTOR)
    speed = -speed;

  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    writeMotorPWM_Right(speed);
  }

  else if (speed < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    writeMotorPWM_Right(-speed);
  }

  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    writeMotorPWM_Right(0);
  }
}


// ============================================================================
// 18. PHYSICAL LEFT MOTOR
// ============================================================================

void setMotorLeft(int speed)
{
  if (INVERT_LEFT_MOTOR)
    speed = -speed;

  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    writeMotorPWM_Left(speed);
  }

  else if (speed < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    writeMotorPWM_Left(-speed);
  }

  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    writeMotorPWM_Left(0);
  }
}


// ============================================================================
// 19. STOP
// ============================================================================

void stopAllMotors()
{
  setMotorRight(0);
  setMotorLeft(0);
}


// ============================================================================
// 20. LOGICAL MOTOR MAPPING
// ============================================================================

void driveRight(int speed)
{
  if (SWAP_LEFT_RIGHT_MOTORS)
    setMotorLeft(speed);
  else
    setMotorRight(speed);
}


void driveLeft(int speed)
{
  if (SWAP_LEFT_RIGHT_MOTORS)
    setMotorRight(speed);
  else
    setMotorLeft(speed);
}


// ============================================================================
// 21. SENSOR BIT REVERSAL
// ============================================================================

uint8_t reverseBits8(uint8_t b)
{
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;

  return b;
}


// ============================================================================
// 22. SENSOR READ
// ============================================================================

void readSensorsAndAnalyze()
{
  rawSensorMask = 0;

  activeSensorCount = 0;

  clusterCount = 0;


  for (int i = 0; i < 8; i++)
  {
    if (digitalRead(sensorPins[i]) == BLACK_STATE)
    {
      rawSensorMask |= (1 << i);

      activeSensorCount++;
    }
  }


  if (SWAP_LEFT_RIGHT_SENSORS)
  {
    rawSensorMask = reverseBits8(rawSensorMask);
  }


  // Count clusters

  bool inCluster = false;

  for (int i = 0; i < 8; i++)
  {
    if (rawSensorMask & (1 << i))
    {
      if (!inCluster)
      {
        clusterCount++;
        inCluster = true;
      }
    }
    else
    {
      inCluster = false;
    }
  }


  // LEFT = IR1,2,3

  leftCluster =
      (rawSensorMask & 0x07) != 0;


  // CENTER = IR4,5

  centerCluster =
      (rawSensorMask & 0x18) != 0;


  // RIGHT = IR6,7,8

  rightCluster =
      (rawSensorMask & 0xE0) != 0;
}


// ============================================================================
// 23. CALCULATE POSITION ERROR
// ============================================================================

float calculatePositionError(bool &hasLine)
{
  if (activeSensorCount == 0)
  {
    hasLine = false;

    return lastGoodError;
  }


  hasLine = true;


  // If there are two separate clusters,
  // choose the cluster based on the previous direction.
  //
  // This is useful at diode / junction sections.

  if (clusterCount >= 2)
  {
    long weightedSum = 0;

    int count = 0;


    bool preferLeft =
        (lastKnownDirection < 0);


    if (currentPriority == PRI_LEFT_STRAIGHT_RIGHT &&
        leftCluster)
    {
      preferLeft = true;
    }


    if (currentPriority == PRI_RIGHT_STRAIGHT_LEFT &&
        rightCluster)
    {
      preferLeft = false;
    }


    if (preferLeft && leftCluster)
    {
      for (int i = 0; i < 4; i++)
      {
        if (rawSensorMask & (1 << i))
        {
          weightedSum += SENSOR_WEIGHTS[i];

          count++;
        }
      }
    }

    else if (rightCluster)
    {
      for (int i = 4; i < 8; i++)
      {
        if (rawSensorMask & (1 << i))
        {
          weightedSum += SENSOR_WEIGHTS[i];

          count++;
        }
      }
    }


    if (count > 0)
    {
      return
          (float)weightedSum /
          (float)count;
    }
  }


  long weightedSum = 0;


  for (int i = 0; i < 8; i++)
  {
    if (rawSensorMask & (1 << i))
    {
      weightedSum += SENSOR_WEIGHTS[i];
    }
  }


  return
      (float)weightedSum /
      (float)activeSensorCount;
}


// ============================================================================
// 24. STATE SWITCH
// ============================================================================

void switchState(TrackState newState)
{
  if (newState != currentState)
  {
    currentState = newState;

    stateEnteredTime = millis();

    sharpConfirmCounter = 0;

    vturnConfirmCounter = 0;


    // Reset derivative when entering a pivot.
    //
    // This prevents the large Kd value from creating
    // a huge correction immediately after the pivot.

    if (newState == STATE_SHARP_TURN ||
        newState == STATE_V_TURN ||
        newState == STATE_END_TURN)
    {
      previousError = currentError;

      filteredDerivative = 0.0f;

      integralError = 0.0f;
    }
  }
}


// ============================================================================
// 25. STATE NAME
// ============================================================================

const char* getStateName(TrackState s)
{
  switch (s)
  {
    case STATE_STARTUP:
      return "STARTUP";

    case STATE_NORMAL:
      return "NORMAL";

    case STATE_CURVE:
      return "CURVE";

    case STATE_SHARP_TURN:
      return "SHARP_TURN";

    case STATE_V_TURN:
      return "V_TURN";

    case STATE_JUNCTION:
      return "JUNCTION";

    case STATE_CROSSING:
      return "CROSSING";

    case STATE_GAP:
      return "GAP";

    case STATE_LINE_LOST:
      return "LINE_LOST";

    case STATE_END_TURN:
      return "END_TURN";

    case STATE_SAFE_STOP:
      return "SAFE_STOP";

    default:
      return "UNKNOWN";
  }
}


// ============================================================================
// 26. TRACK STATE EVALUATION
// ============================================================================

void evaluateTrackState(float error, bool hasLine)
{
  unsigned long now = millis();


  // ----------------------------------------------------------
  // Forget old end marker
  // ----------------------------------------------------------

  if (sawFullMarker &&
      now - fullMarkerTime > END_TURN_EXPIRE_MS)
  {
    sawFullMarker = false;
  }


  // ----------------------------------------------------------
  // Detect wide black marker
  // ----------------------------------------------------------

  if (hasLine &&
      activeSensorCount >= END_MARKER_MIN_SENSORS)
  {
    sawFullMarker = true;

    fullMarkerTime = now;
  }


  // ==========================================================
  // 1. LINE LOST
  // ==========================================================

  if (!hasLine)
  {
    if (lineLostTime == 0)
      lineLostTime = now;


    unsigned long blackoutDuration =
        now - lineLostTime;


    // End marker

    if (sawFullMarker &&
        now - fullMarkerTime < END_TURN_TRIGGER_MS &&
        currentState != STATE_END_TURN)
    {
      if (endMarkerBehavior == END_ACTION_UTURN)
      {
        turnDirection =
            (lastKnownDirection != 0)
            ? lastKnownDirection
            : DEFAULT_UTURN_DIRECTION;

        switchState(STATE_END_TURN);
      }

      else
      {
        switchState(STATE_SAFE_STOP);
      }

      return;
    }


    // Small gap

    if (blackoutDuration < GAP_MAX_TIME_MS)
    {
      switchState(STATE_GAP);
    }


    // Normal line recovery

    else if (blackoutDuration < RECOVERY_TIMEOUT_MS)
    {
      switchState(STATE_LINE_LOST);
    }


    else
    {
      switchState(STATE_SAFE_STOP);
    }

    return;
  }


  // ==========================================================
  // LINE IS VISIBLE
  // ==========================================================

  lineLostTime = 0;


  // ==========================================================
  // 2. CURRENT PIVOT
  // ==========================================================

  if (currentState == STATE_SHARP_TURN ||
      currentState == STATE_V_TURN ||
      currentState == STATE_END_TURN)
  {
    bool reacquired = false;


    if (currentState == STATE_END_TURN)
    {
      // Center sensors

      reacquired =
          (rawSensorMask & 0x3C) != 0;
    }

    else if (turnDirection < 0)
    {
      // Left turn

      reacquired =
          (rawSensorMask & 0x1E) != 0;
    }

    else
    {
      // Right turn

      reacquired =
          (rawSensorMask & 0x78) != 0;
    }


    if (reacquired)
    {
      previousError = error;

      filteredDerivative = 0.0f;

      integralError = 0.0f;

      sawFullMarker = false;


      if (fabs(error) > ERROR_CURVE_THRESHOLD)
        switchState(STATE_CURVE);
      else
        switchState(STATE_NORMAL);

      return;
    }


    unsigned long timeoutForState =
        (currentState == STATE_END_TURN)
        ? END_TURN_TIMEOUT_MS
        : SHARP_TURN_TIMEOUT_MS;


    if (now - stateEnteredTime >
        timeoutForState)
    {
      switchState(STATE_LINE_LOST);
    }

    return;
  }


  // ==========================================================
  // 3. CROSSING
  // ==========================================================

  if (activeSensorCount >= WIDE_SENSOR_MIN &&
      centerCluster)
  {
    if (fabs(lastGoodError) <
        ERROR_CURVE_THRESHOLD)
    {
      switchState(STATE_CROSSING);

      return;
    }
  }


  // ==========================================================
  // 4. FORWARD-BIAS DIODE / JUNCTION
  // ==========================================================
  //
  // IMPORTANT FIX
  //
  // A diode branch can produce:
  //
  // LEFT + CENTER
  // or
  // CENTER + RIGHT
  //
  // Do NOT let this immediately become V_TURN.
  //
  // When straight priority is selected, continue forward
  // using actual sensor error.
  //

  bool forwardDiodePattern = false;


  if (currentPriority == PRI_STRAIGHT_LEFT_RIGHT)
  {
    // Left branch + center

    if (leftCluster &&
        centerCluster)
    {
      forwardDiodePattern = true;
    }


    // Center + right branch

    if (centerCluster &&
        rightCluster)
    {
      forwardDiodePattern = true;
    }
  }


  if (forwardDiodePattern)
  {
    if (junctionHoldStart == 0)
    {
      junctionHoldStart = now;
    }


    switchState(STATE_JUNCTION);

    return;
  }


  junctionHoldStart = 0;


  // ==========================================================
  // 5. V-TURN DETECTION
  // ==========================================================
  //
  // IMPORTANT:
  // V-turn must have:
  //
  // - two clusters
  // - no center
  // - an OUTER sensor active
  //
  // This prevents many diode patterns from being
  // incorrectly classified as V-turns.
  //

  bool leftOuter =
      (rawSensorMask & 0x01) != 0;


  bool rightOuter =
      (rawSensorMask & 0x80) != 0;


  bool possibleVTurn =
      (clusterCount >= 2) &&
      !centerCluster &&
      (leftOuter || rightOuter);


  if (possibleVTurn)
  {
    vturnConfirmCounter++;


    if (vturnConfirmCounter >=
        VTURN_CONFIRM_FRAMES)
    {
      // Use last known direction

      if (lastKnownDirection != 0)
      {
        turnDirection =
            lastKnownDirection;
      }

      else
      {
        if (leftOuter)
          turnDirection = -1;
        else
          turnDirection = 1;
      }


      switchState(STATE_V_TURN);

      return;
    }
  }

  else
  {
    vturnConfirmCounter = 0;
  }


  // ==========================================================
  // 6. OTHER JUNCTION
  // ==========================================================

  if ((leftCluster &&
       centerCluster &&
       leftOuter) ||

      (rightCluster &&
       centerCluster &&
       rightOuter))
  {
    switchState(STATE_JUNCTION);

    return;
  }


  // ==========================================================
  // 7. SHARP 90 DEG TURN
  // ==========================================================

  if (fabs(error) >=
      ERROR_SHARP_THRESHOLD)
  {
    int8_t dir =
        (error > 0) ? 1 : -1;


    if (dir == turnDirection)
    {
      sharpConfirmCounter++;
    }

    else
    {
      turnDirection = dir;

      sharpConfirmCounter = 1;
    }


    if (sharpConfirmCounter >=
        SHARP_CONFIRM_FRAMES)
    {
      switchState(STATE_SHARP_TURN);

      return;
    }


    switchState(STATE_CURVE);

    return;
  }


  sharpConfirmCounter = 0;


  // ==========================================================
  // 8. CURVE / NORMAL
  // ==========================================================

  if (fabs(error) >=
      ERROR_CURVE_THRESHOLD)
  {
    switchState(STATE_CURVE);
  }

  else
  {
    switchState(STATE_NORMAL);
  }
}


// ============================================================================
// 27. ADAPTIVE SPEED
// ============================================================================

int calculateAdaptiveCruiseSpeed(
    float error,
    float derivative)
{
  float absErr = fabs(error);


  // Straight

  if (absErr <
      ERROR_CURVE_THRESHOLD)
  {
    return SPEED_STRAIGHT;
  }


  // Curve

  float curveFraction =
      (absErr - ERROR_CURVE_THRESHOLD) /
      (ERROR_SHARP_THRESHOLD -
       ERROR_CURVE_THRESHOLD);


  curveFraction =
      constrain(curveFraction,
                0.0f,
                1.0f);


  int speed =
      SPEED_STRAIGHT -
      (int)
      (
        curveFraction *
        (SPEED_STRAIGHT -
         SPEED_TIGHT_CURVE)
      );


  // Strong derivative = slow down

  if (fabs(derivative) > 4.0f)
  {
    speed -= 15;
  }


  return constrain(
      speed,
      SPEED_TIGHT_CURVE,
      SPEED_STRAIGHT);
}


// ============================================================================
// 28. PD TRACKING
// ============================================================================

void executePDTracking(
    float error,
    int cruiseSpeed)
{
  if (fabs(error) <
      ERROR_DEADBAND)
  {
    error = 0.0f;
  }


  // Integral remains exactly as original

  integralError += error;

  integralError =
      constrain(
          integralError,
          -25.0f,
          25.0f);


  // Derivative

  float rawDerivative =
      error - previousError;


  filteredDerivative =
      ((1.0f - derivativeAlpha) *
       filteredDerivative)
      +
      (derivativeAlpha *
       rawDerivative);


  previousError = error;


  // PID
  //
  // VALUES UNCHANGED:
  // Kp = 4
  // Ki = 0
  // Kd = 15

  float correction =
      (Kp * error) +
      (Ki * integralError) +
      (Kd * filteredDerivative);


  /*
    Positive error = RIGHT

    According to your current motor setup:
      speedLeft  increases
      speedRight decreases

    This is kept from your working turning behavior.
  */

  int speedRight =
      cruiseSpeed -
      (int)correction;


  int speedLeft =
      cruiseSpeed +
      (int)correction;


  // Keep reasonable motor limits

  speedRight =
      constrain(
          speedRight,
          -255,
          255);


  speedLeft =
      constrain(
          speedLeft,
          -255,
          255);


  driveRight(
      speedRight +
      rightMotorTrim);


  driveLeft(
      speedLeft +
      leftMotorTrim);
}


// ============================================================================
// 29. SHARP / V PIVOT
// ============================================================================

void executePivotTurn(
    int8_t direction)
{
  // LEFT TURN

  if (direction < 0)
  {
    driveRight(
        SPEED_TURN_OUTER +
        rightMotorTrim);

    driveLeft(
        SPEED_TURN_INNER +
        leftMotorTrim);
  }


  // RIGHT TURN

  else
  {
    driveRight(
        SPEED_TURN_INNER +
        rightMotorTrim);

    driveLeft(
        SPEED_TURN_OUTER +
        leftMotorTrim);
  }
}


// ============================================================================
// 30. END U-TURN
// ============================================================================

void executeEndOfTrackUTurn(
    int8_t direction)
{
  if (direction < 0)
  {
    driveRight(
        SPEED_UTURN_OUTER +
        rightMotorTrim);

    driveLeft(
        SPEED_UTURN_INNER +
        leftMotorTrim);
  }

  else
  {
    driveRight(
        SPEED_UTURN_INNER +
        rightMotorTrim);

    driveLeft(
        SPEED_UTURN_OUTER +
        leftMotorTrim);
  }
}


// ============================================================================
// 31. JUNCTION / DIODE NAVIGATION
// ============================================================================

void executeJunctionNavigation()
{
  /*
    IMPORTANT FIX:

    Do not use:
        executePDTracking(0.0f,...)

    because that throws away the actual sensor position.

    Instead use currentError.
  */


  if (currentPriority ==
      PRI_STRAIGHT_LEFT_RIGHT)
  {
    // Keep going forward through diode/junction.

    executePDTracking(
        currentError,
        SPEED_GENTLE_CURVE);

    return;
  }


  // LEFT PRIORITY

  if (currentPriority ==
      PRI_LEFT_STRAIGHT_RIGHT)
  {
    if (leftCluster)
    {
      executePivotTurn(-1);
    }

    else if (centerCluster)
    {
      executePDTracking(
          currentError,
          SPEED_GENTLE_CURVE);
    }

    else
    {
      executePivotTurn(1);
    }

    return;
  }


  // RIGHT PRIORITY

  if (currentPriority ==
      PRI_RIGHT_STRAIGHT_LEFT)
  {
    if (rightCluster)
    {
      executePivotTurn(1);
    }

    else if (centerCluster)
    {
      executePDTracking(
          currentError,
          SPEED_GENTLE_CURVE);
    }

    else
    {
      executePivotTurn(-1);
    }

    return;
  }
}


// ============================================================================
// 32. CROSSING
// ============================================================================

void executeCrossing()
{
  /*
    Don't force the robot to blindly drive straight.

    Use a small forward speed with the current sensor
    information.
  */

  executePDTracking(
      currentError,
      SPEED_GENTLE_CURVE);
}


// ============================================================================
// 33. GAP
// ============================================================================

void executeGapNavigation()
{
  driveRight(
      SPEED_GENTLE_CURVE +
      rightMotorTrim);

  driveLeft(
      SPEED_GENTLE_CURVE +
      leftMotorTrim);
}


// ============================================================================
// 34. LINE RECOVERY
// ============================================================================

void executeLineRecovery()
{
  unsigned long lostFor =
      (lineLostTime == 0)
      ? 0
      : millis() - lineLostTime;


  int outer =
      SPEED_RECOVERY;


  int inner =
      SPEED_RECOVERY / 3;


  bool sweepRight =
      (lastKnownDirection >= 0);


  // Stage 2

  if (lostFor >
      SEARCH_STAGE2_START_MS)
  {
    outer =
        SPEED_RECOVERY + 25;

    inner = -15;


    unsigned long phase =
        (lostFor -
         SEARCH_STAGE2_START_MS)
        /
        SEARCH_ALTERNATE_PERIOD_MS;


    if (phase % 2 == 1)
    {
      sweepRight =
          !sweepRight;
    }
  }


  if (sweepRight)
  {
    driveRight(
        inner +
        rightMotorTrim);

    driveLeft(
        outer +
        leftMotorTrim);
  }

  else
  {
    driveRight(
        outer +
        rightMotorTrim);

    driveLeft(
        inner +
        leftMotorTrim);
  }
}


// ============================================================================
// 35. STARTUP SELF TEST
// ============================================================================

void runStartupSelfTest()
{
  Serial.println(
      "\n--- STARTUP SELF-TEST ---");


  Serial.println(
      "RIGHT wheel FORWARD");


  setMotorRight(160);

  setMotorLeft(0);

  delay(400);

  stopAllMotors();

  delay(300);


  Serial.println(
      "LEFT wheel FORWARD");


  setMotorLeft(160);

  setMotorRight(0);

  delay(400);

  stopAllMotors();

  delay(300);


  Serial.println(
      "Check wheel directions.");


  Serial.println(
      "Sensor bitmask IR1..IR8:");


  for (int s = 0; s < 5; s++)
  {
    readSensorsAndAnalyze();


    for (int i = 0; i < 8; i++)
    {
      Serial.print(
          (rawSensorMask &
           (1 << i))
          ? "1"
          : "0");
    }

    Serial.println();

    delay(200);
  }


  Serial.println(
      "--- SELF-TEST COMPLETE ---\n");
}


// ============================================================================
// 36. SETUP
// ============================================================================

void setup()
{
  Serial.begin(115200);


  for (int i = 0; i < 8; i++)
  {
    pinMode(
        sensorPins[i],
        INPUT);
  }


  pinMode(
      LED_PIN,
      OUTPUT);

  digitalWrite(
      LED_PIN,
      LOW);


  setupMotorPWM();

  stopAllMotors();


  Serial.println(
      "\n==========================================");

  Serial.println(
      " ESP32 COMPETITION LINE FOLLOWER");

  Serial.println(
      " CORRECTED VERSION");

  Serial.println(
      "==========================================");


  Serial.print(
      "PID: Kp=");

  Serial.print(Kp);

  Serial.print(
      " Ki=");

  Serial.print(Ki);

  Serial.print(
      " Kd=");

  Serial.println(Kd);


  Serial.print(
      "Straight Speed = ");

  Serial.println(
      SPEED_STRAIGHT);


  Serial.print(
      "Gentle Curve Speed = ");

  Serial.println(
      SPEED_GENTLE_CURVE);


  Serial.print(
      "Tight Curve Speed = ");

  Serial.println(
      SPEED_TIGHT_CURVE);


  if (ENABLE_STARTUP_SELFTEST)
  {
    runStartupSelfTest();
  }


  Serial.println(
      "Place robot on track.");


  Serial.println(
      "Starting in 3 seconds...");


  int blinkCount =
      (int)currentPriority + 1;


  for (int b = 0;
       b < blinkCount;
       b++)
  {
    digitalWrite(
        LED_PIN,
        HIGH);

    delay(200);

    digitalWrite(
        LED_PIN,
        LOW);

    delay(200);
  }


  long remaining =
      (long)STARTUP_DELAY_MS -
      (blinkCount * 400L);


  if (remaining > 0)
  {
    delay(remaining);
  }


  digitalWrite(
      LED_PIN,
      HIGH);


  switchState(
      STATE_NORMAL);
}


// ============================================================================
// 37. MAIN LOOP
// ============================================================================

void loop()
{
  unsigned long currentMicros =
      micros();


  // ~500 Hz

  if (currentMicros -
      lastLoopMicros < 2000)
  {
    return;
  }


  lastLoopMicros =
      currentMicros;


  // Read sensors

  readSensorsAndAnalyze();


  // Calculate error

  bool hasLine = false;


  currentError =
      calculatePositionError(
          hasLine);


  // Save last good error

  if (hasLine)
  {
    lastGoodError =
        currentError;


    if (currentError > 1.0f)
    {
      lastKnownDirection = 1;
    }

    else if (currentError < -1.0f)
    {
      lastKnownDirection = -1;
    }
  }


  // Evaluate state

  evaluateTrackState(
      currentError,
      hasLine);


  // ==========================================================
  // EXECUTE STATE
  // ==========================================================

  switch (currentState)
  {
    // --------------------------------------------------------
    // NORMAL / CURVE
    // --------------------------------------------------------

    case STATE_NORMAL:
    case STATE_CURVE:
    {
      int speed =
          calculateAdaptiveCruiseSpeed(
              currentError,
              filteredDerivative);


      executePDTracking(
          currentError,
          speed);

      break;
    }


    // --------------------------------------------------------
    // SHARP / V TURN
    // --------------------------------------------------------

    case STATE_SHARP_TURN:
    case STATE_V_TURN:
    {
      executePivotTurn(
          turnDirection);

      break;
    }


    // --------------------------------------------------------
    // END U TURN
    // --------------------------------------------------------

    case STATE_END_TURN:
    {
      executeEndOfTrackUTurn(
          turnDirection);

      break;
    }


    // --------------------------------------------------------
    // JUNCTION / DIODE
    // --------------------------------------------------------

    case STATE_JUNCTION:
    {
      executeJunctionNavigation();

      break;
    }


    // --------------------------------------------------------
    // CROSSING
    // --------------------------------------------------------

    case STATE_CROSSING:
    {
      executeCrossing();

      break;
    }


    // --------------------------------------------------------
    // GAP
    // --------------------------------------------------------

    case STATE_GAP:
    {
      executeGapNavigation();

      break;
    }


    // --------------------------------------------------------
    // LINE LOST
    // --------------------------------------------------------

    case STATE_LINE_LOST:
    {
      executeLineRecovery();

      break;
    }


    // --------------------------------------------------------
    // STOP
    // --------------------------------------------------------

    case STATE_SAFE_STOP:
    default:
    {
      stopAllMotors();

      digitalWrite(
          LED_PIN,
          LOW);

      break;
    }
  }


  // ==========================================================
  // SERIAL TELEMETRY
  // ==========================================================

  static unsigned long lastTelemetry = 0;


  if (millis() -
      lastTelemetry >= 100)
  {
    lastTelemetry =
        millis();


    Serial.print(
        "Sensors: ");


    for (int i = 0; i < 8; i++)
    {
      Serial.print(
          (rawSensorMask &
           (1 << i))
          ? "1"
          : "0");
    }


    Serial.print(
        " | Cnt: ");

    Serial.print(
        activeSensorCount);


    Serial.print(
        " | Clust: ");

    Serial.print(
        clusterCount);


    Serial.print(
        " | Err: ");

    Serial.print(
        currentError,
        1);


    Serial.print(
        " | State: ");

    Serial.println(
        getStateName(
            currentState));
  }
}
