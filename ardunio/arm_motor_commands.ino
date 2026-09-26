/*
 * BigTreeTech Octopus Pro v1.1 six-axis arm controller.
 *
 * Host protocol (kept compatible with the PCA9685 version):
 *   POSE <base> <shoulder> <elbow> <wrist_pitch> <wrist_rotate> <gripper>
 *   HOME | OPEN | CLOSE | ENABLE | DISABLE | INFO | STATUS
 *   RAW_CW | RAW_CCW
 *   CURRENT <milliamps>
 *
 * Motor sockets 0 through 5 are assigned in that order. POSE values are joint
 * angles in degrees, not raw steps.
 *
 * IMPORTANT: There is no endstop homing in this first version. Put the arm at
 * HOME_ANGLE before powering it. setup() declares that position as current.
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <math.h>

constexpr uint8_t AXIS_COUNT = 6;
constexpr uint32_t SERIAL_BAUD = 115200;

enum Axis : uint8_t { BASE, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROTATE, GRIPPER };

// Octopus Pro v1.1 motor sockets 0-5.
constexpr uint8_t STEP_PIN[AXIS_COUNT] = {PF13, PG0, PF11, PG4, PF9, PC13};
constexpr uint8_t DIR_PIN[AXIS_COUNT] = {PF12, PG1, PG3, PC1, PF10, PF0};
constexpr uint8_t ENABLE_PIN[AXIS_COUNT] = {PF14, PF15, PG5, PA2, PG2, PF1};

constexpr uint16_t RAW_TEST_STEPS = 400;
constexpr uint16_t RAW_STEP_HALF_PERIOD_US = 3333;

// Octopus Pro MOTOR0 uses a single-wire UART connection on PC4. The standard
// TMC2209 module address is 0 when its MS1/MS2 address inputs are both low.
constexpr uint8_t BASE_UART_PIN = PC4;
constexpr float TMC_R_SENSE = 0.110f;
constexpr uint8_t BASE_DRIVER_ADDRESS = 0;
constexpr uint16_t DEFAULT_RUN_CURRENT_MA = 1000;
constexpr uint16_t MIN_RUN_CURRENT_MA = 300;
constexpr uint16_t MAX_RUN_CURRENT_MA = 1400;
constexpr float HOLD_CURRENT_MULTIPLIER = 0.50f;

// Calibrate for each motor, microstep setting, and gearbox:
// (motor full steps/rev * microsteps * gear ratio) / 360 degrees.
constexpr float STEPS_PER_DEGREE[AXIS_COUNT] = {
  8.8889f, 8.8889f, 8.8889f, 8.8889f, 8.8889f, 8.8889f
};
constexpr bool INVERT_DIRECTION[AXIS_COUNT] = {
  false, false, false, false, false, false
};
// STEP/DIR drivers cannot report whether a motor is physically connected.
// Keep this list aligned with the motors actually installed on the board.
constexpr bool ACTIVE_AXIS[AXIS_COUNT] = {
  true, false, false, false, false, false
};
constexpr float MAX_SPEED[AXIS_COUNT] = {
  150.0f, 1000.0f, 1000.0f, 1200.0f, 1200.0f, 800.0f
};
constexpr float ACCELERATION[AXIS_COUNT] = {
  50.0f, 500.0f, 500.0f, 600.0f, 600.0f, 400.0f
};

constexpr int MIN_ANGLE[AXIS_COUNT] = {0, 0, 0, 0, 0, 0};
constexpr int MAX_ANGLE[AXIS_COUNT] = {180, 180, 180, 180, 180, 180};
constexpr int HOME_ANGLE[AXIS_COUNT] = {90, 90, 90, 90, 90, 120};
constexpr int GRIPPER_OPEN_ANGLE = 120;
constexpr int GRIPPER_CLOSED_ANGLE = 55;

AccelStepper baseMotor(AccelStepper::DRIVER, STEP_PIN[BASE], DIR_PIN[BASE]);
AccelStepper shoulderMotor(AccelStepper::DRIVER, STEP_PIN[SHOULDER], DIR_PIN[SHOULDER]);
AccelStepper elbowMotor(AccelStepper::DRIVER, STEP_PIN[ELBOW], DIR_PIN[ELBOW]);
AccelStepper wristPitchMotor(AccelStepper::DRIVER, STEP_PIN[WRIST_PITCH], DIR_PIN[WRIST_PITCH]);
AccelStepper wristRotateMotor(AccelStepper::DRIVER, STEP_PIN[WRIST_ROTATE], DIR_PIN[WRIST_ROTATE]);
AccelStepper gripperMotor(AccelStepper::DRIVER, STEP_PIN[GRIPPER], DIR_PIN[GRIPPER]);
TMC2209Stepper baseDriver(BASE_UART_PIN, BASE_UART_PIN, TMC_R_SENSE, BASE_DRIVER_ADDRESS);

AccelStepper *motors[AXIS_COUNT] = {
  &baseMotor, &shoulderMotor, &elbowMotor,
  &wristPitchMotor, &wristRotateMotor, &gripperMotor
};
int currentAngle[AXIS_COUNT];
bool outputsEnabled = true;
bool baseUartConnected = false;
uint16_t baseRunCurrentMa = DEFAULT_RUN_CURRENT_MA;

long angleToSteps(uint8_t axis, int angle) {
  return lroundf(angle * STEPS_PER_DEGREE[axis]);
}

bool anglesAreSafe(const int angles[AXIS_COUNT]) {
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    if (angles[axis] < MIN_ANGLE[axis] || angles[axis] > MAX_ANGLE[axis]) {
      return false;
    }
  }
  return true;
}

void moveToAngles(const int angles[AXIS_COUNT]) {
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    if (ACTIVE_AXIS[axis]) {
      motors[axis]->moveTo(angleToSteps(axis, angles[axis]));
    }
  }

  bool moving;
  do {
    moving = false;
    for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
      if (ACTIVE_AXIS[axis] && motors[axis]->distanceToGo() != 0) {
        motors[axis]->run();
        moving = true;
      }
    }
  } while (moving);

  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    currentAngle[axis] = angles[axis];
  }
}

void setOutputsEnabled(bool enabled) {
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    if (!ACTIVE_AXIS[axis]) {
      continue;
    }
    if (enabled) {
      motors[axis]->enableOutputs();
    } else {
      motors[axis]->disableOutputs();
    }
  }
  outputsEnabled = enabled;
}

void setBaseEnableDirect(bool enabled) {
  pinMode(ENABLE_PIN[BASE], OUTPUT);
  digitalWrite(ENABLE_PIN[BASE], enabled ? LOW : HIGH);
  outputsEnabled = enabled;
}

void runRawBaseTest(bool clockwise) {
  setBaseEnableDirect(true);
  pinMode(STEP_PIN[BASE], OUTPUT);
  pinMode(DIR_PIN[BASE], OUTPUT);
  digitalWrite(STEP_PIN[BASE], LOW);
  digitalWrite(DIR_PIN[BASE], clockwise ? HIGH : LOW);
  delayMicroseconds(10);

  for (uint16_t step = 0; step < RAW_TEST_STEPS; ++step) {
    digitalWrite(STEP_PIN[BASE], HIGH);
    delayMicroseconds(RAW_STEP_HALF_PERIOD_US);
    digitalWrite(STEP_PIN[BASE], LOW);
    delayMicroseconds(RAW_STEP_HALF_PERIOD_US);
  }

  // Raw motion invalidates the absolute pose reference. Treat the resulting
  // location as the configured home so subsequent POSE commands stay bounded.
  motors[BASE]->setCurrentPosition(angleToSteps(BASE, HOME_ANGLE[BASE]));
  currentAngle[BASE] = HOME_ANGLE[BASE];
}

void configureBaseDriver(uint16_t runCurrentMa) {
  baseDriver.begin();
  baseDriver.toff(4);
  baseDriver.blank_time(24);
  baseDriver.rms_current(runCurrentMa, HOLD_CURRENT_MULTIPLIER);
  baseDriver.microsteps(16);
  baseDriver.en_spreadCycle(false);
  baseDriver.pwm_autoscale(true);
  baseDriver.pwm_autograd(true);
  baseDriver.GSTAT(0b111);

  baseRunCurrentMa = runCurrentMa;
  baseUartConnected = baseDriver.test_connection() == 0;
}

void setBaseCurrent(const String &command) {
  int requestedCurrent;
  if (sscanf(command.c_str(), "CURRENT %d", &requestedCurrent) != 1 ||
      requestedCurrent < MIN_RUN_CURRENT_MA || requestedCurrent > MAX_RUN_CURRENT_MA) {
    Serial.print("ERROR CURRENT range is ");
    Serial.print(MIN_RUN_CURRENT_MA);
    Serial.print("-");
    Serial.print(MAX_RUN_CURRENT_MA);
    Serial.println(" mA");
    return;
  }

  configureBaseDriver(static_cast<uint16_t>(requestedCurrent));
  if (!baseUartConnected) {
    Serial.print("WARN CURRENT requested ");
    Serial.print(baseRunCurrentMa);
    Serial.println("mA; TMC2209 UART readback unavailable");
    return;
  }

  Serial.print("OK CURRENT ");
  Serial.print(baseDriver.rms_current());
  Serial.println("mA");
}

void printInfo() {
  Serial.print("INFO BOARD=OCTOPUS_PRO_V1_1 PROTOCOL=1 ACTIVE=BASE UART_READBACK=");
  Serial.println(baseUartConnected ? "OK" : "ERROR");
}

void printStatus() {
  Serial.print("STATUS POSE=");
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    if (axis > 0) {
      Serial.print(',');
    }
    Serial.print(currentAngle[axis]);
  }
  Serial.print(" OUTPUTS=");
  Serial.print(outputsEnabled ? "ENABLED" : "DISABLED");
  Serial.print(" EN_PIN=");
  Serial.print(digitalRead(ENABLE_PIN[BASE]) == LOW ? "LOW" : "HIGH");
  Serial.print(" UART_READBACK=");
  Serial.print(baseUartConnected ? "OK" : "ERROR");
  Serial.print(" RUN_CURRENT=");
  Serial.print(baseUartConnected ? baseDriver.rms_current() : baseRunCurrentMa);
  Serial.println("mA");
}

void homeArm() {
  moveToAngles(HOME_ANGLE);
}

void moveGripperTo(int angle) {
  int pose[AXIS_COUNT];
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    pose[axis] = currentAngle[axis];
  }
  pose[GRIPPER] = angle;
  moveToAngles(pose);
}

void handlePoseCommand(const String &command) {
  int angles[AXIS_COUNT];
  const int parsed = sscanf(
    command.c_str(), "POSE %d %d %d %d %d %d",
    &angles[BASE], &angles[SHOULDER], &angles[ELBOW],
    &angles[WRIST_PITCH], &angles[WRIST_ROTATE], &angles[GRIPPER]
  );

  if (parsed != AXIS_COUNT) {
    Serial.println("ERROR Invalid POSE command");
    return;
  }
  if (!anglesAreSafe(angles)) {
    Serial.println("ERROR Joint angle outside configured limits");
    return;
  }

  moveToAngles(angles);
  Serial.println("OK");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(100);

  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    currentAngle[axis] = HOME_ANGLE[axis];
    if (!ACTIVE_AXIS[axis]) {
      continue;
    }
    motors[axis]->setEnablePin(ENABLE_PIN[axis]);
    motors[axis]->setPinsInverted(INVERT_DIRECTION[axis], false, true);
    motors[axis]->setMinPulseWidth(2);
    motors[axis]->setMaxSpeed(MAX_SPEED[axis]);
    motors[axis]->setAcceleration(ACCELERATION[axis]);
    motors[axis]->setCurrentPosition(angleToSteps(axis, HOME_ANGLE[axis]));
    motors[axis]->enableOutputs();
  }

  // TMC2209 EN is active-low. Assert it directly for this hardware diagnostic
  // so holding torque does not depend on a library abstraction.
  setBaseEnableDirect(true);
  configureBaseDriver(DEFAULT_RUN_CURRENT_MA);

  Serial.println("READY Octopus Pro v1.1 stepper arm");
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command == "HOME") {
    homeArm();
    Serial.println("OK HOME");
  } else if (command == "OPEN") {
    moveGripperTo(GRIPPER_OPEN_ANGLE);
    Serial.println("OK OPEN");
  } else if (command == "CLOSE") {
    moveGripperTo(GRIPPER_CLOSED_ANGLE);
    Serial.println("OK CLOSE");
  } else if (command == "ENABLE") {
    setOutputsEnabled(true);
    Serial.println("OK ENABLE");
  } else if (command == "DISABLE") {
    setOutputsEnabled(false);
    Serial.println("OK DISABLE");
  } else if (command == "INFO") {
    printInfo();
  } else if (command == "STATUS") {
    printStatus();
  } else if (command == "RAW_CW") {
    runRawBaseTest(true);
    Serial.println("OK RAW_CW");
  } else if (command == "RAW_CCW") {
    runRawBaseTest(false);
    Serial.println("OK RAW_CCW");
  } else if (command.startsWith("CURRENT ")) {
    setBaseCurrent(command);
  } else if (command.startsWith("POSE ")) {
    if (!outputsEnabled) {
      setOutputsEnabled(true);
    }
    handlePoseCommand(command);
  } else {
    Serial.println("ERROR Unknown command");
  }
}
