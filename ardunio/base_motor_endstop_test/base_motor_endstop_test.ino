/*
 * Octopus Pro v1.1: motor socket 0 + corresponding optical endstop test.
 *
 * Required library: AccelStepper
 * Serial settings: 115200 baud, newline line ending
 *
 * Commands:
 *   HELP          Show commands
 *   STATUS        Read the endstop and motor position
 *   ENABLE        Energize the base motor
 *   DISABLE       De-energize the base motor
 *   JOG <steps>   Move a limited number of steps (example: JOG 100)
 *   HOME          Run a distance-limited, two-pass homing test
 *
 * Keep the joint unloaded for the first test and be ready to remove motor
 * power. Confirm STATUS changes when the optical flag is moved by hand before
 * using JOG or HOME.
 */

#include <Arduino.h>
#include <AccelStepper.h>

// Octopus Pro v1.1 motor socket 0 and its corresponding endstop input.
constexpr uint8_t STEP_PIN = PF13;
constexpr uint8_t DIR_PIN = PF12;
constexpr uint8_t ENABLE_PIN = PF14;
constexpr uint8_t ENDSTOP_PIN = PG6;

// Change these two settings if STATUS or motor travel is reversed.
constexpr uint8_t ENDSTOP_TRIGGERED_STATE = LOW;
constexpr bool INVERT_MOTOR_DIRECTION = false;
constexpr int HOME_DIRECTION = -1;  // Must be either -1 or +1.

constexpr float JOG_MAX_SPEED = 300.0f;
constexpr float JOG_ACCELERATION = 150.0f;
constexpr float FAST_HOME_SPEED = 180.0f;
constexpr float SLOW_HOME_SPEED = 60.0f;
constexpr long MAX_JOG_STEPS = 800;
constexpr long MAX_HOME_TRAVEL_STEPS = 6400;
constexpr long BACKOFF_STEPS = 160;
constexpr uint32_t HOME_TIMEOUT_MS = 30000;

AccelStepper baseMotor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
bool motorEnabled = false;

bool endstopTriggered() {
  return digitalRead(ENDSTOP_PIN) == ENDSTOP_TRIGGERED_STATE;
}

void setMotorEnabled(bool enabled) {
  if (enabled) {
    baseMotor.enableOutputs();
  } else {
    baseMotor.disableOutputs();
  }
  motorEnabled = enabled;
}

void printStatus() {
  Serial.print("STATUS raw=");
  Serial.print(digitalRead(ENDSTOP_PIN));
  Serial.print(" triggered=");
  Serial.print(endstopTriggered() ? "YES" : "NO");
  Serial.print(" position_steps=");
  Serial.print(baseMotor.currentPosition());
  Serial.print(" motor=");
  Serial.println(motorEnabled ? "ENABLED" : "DISABLED");
}

void printHelp() {
  Serial.println("Commands: HELP, STATUS, ENABLE, DISABLE, JOG <steps>, HOME");
  Serial.println("Start with STATUS. Block/unblock the optical sensor by hand.");
  Serial.println("Then try small moves such as JOG 20 and JOG -20.");
}

bool moveRelative(long steps, bool stopAtEndstop) {
  baseMotor.move(steps);

  while (baseMotor.distanceToGo() != 0) {
    if (stopAtEndstop && endstopTriggered()) {
      baseMotor.setCurrentPosition(baseMotor.currentPosition());
      Serial.println("STOP Endstop triggered");
      return false;
    }
    baseMotor.run();
  }
  return true;
}

bool seekEndstop(float speed, long maximumTravel) {
  const long startPosition = baseMotor.currentPosition();
  const uint32_t startTime = millis();
  baseMotor.setSpeed(HOME_DIRECTION * speed);

  while (!endstopTriggered()) {
    baseMotor.runSpeed();

    if (labs(baseMotor.currentPosition() - startPosition) >= maximumTravel) {
      Serial.println("ERROR Home travel limit reached without triggering endstop");
      return false;
    }
    if (millis() - startTime >= HOME_TIMEOUT_MS) {
      Serial.println("ERROR Home timeout reached without triggering endstop");
      return false;
    }
  }

  baseMotor.setCurrentPosition(0);
  return true;
}

void runHomeTest() {
  setMotorEnabled(true);
  Serial.println("HOMING Start");

  // If already blocked, first move away far enough to release the sensor.
  if (endstopTriggered()) {
    Serial.println("HOMING Endstop starts triggered; backing away");
    moveRelative(-HOME_DIRECTION * BACKOFF_STEPS, false);
    if (endstopTriggered()) {
      Serial.println("ERROR Endstop did not release during backoff");
      setMotorEnabled(false);
      return;
    }
  }

  Serial.println("HOMING Fast approach");
  if (!seekEndstop(FAST_HOME_SPEED, MAX_HOME_TRAVEL_STEPS)) {
    setMotorEnabled(false);
    return;
  }

  Serial.println("HOMING Backoff");
  moveRelative(-HOME_DIRECTION * BACKOFF_STEPS, false);
  if (endstopTriggered()) {
    Serial.println("ERROR Endstop remained triggered after backoff");
    setMotorEnabled(false);
    return;
  }

  Serial.println("HOMING Slow approach");
  if (!seekEndstop(SLOW_HOME_SPEED, BACKOFF_STEPS * 2)) {
    setMotorEnabled(false);
    return;
  }

  baseMotor.setCurrentPosition(0);
  Serial.println("OK HOME position_steps=0");
}

void handleJog(const String &command) {
  long steps;
  if (sscanf(command.c_str(), "JOG %ld", &steps) != 1) {
    Serial.println("ERROR Use: JOG <steps>");
    return;
  }
  if (steps == 0 || labs(steps) > MAX_JOG_STEPS) {
    Serial.print("ERROR Jog must be between -");
    Serial.print(MAX_JOG_STEPS);
    Serial.print(" and ");
    Serial.print(MAX_JOG_STEPS);
    Serial.println(" steps, excluding zero");
    return;
  }

  // When triggered, only permit travel in the direction away from home.
  if (endstopTriggered() && steps * HOME_DIRECTION > 0) {
    Serial.println("ERROR Endstop active; movement toward home blocked");
    return;
  }

  setMotorEnabled(true);
  const bool towardHome = steps * HOME_DIRECTION > 0;
  if (moveRelative(steps, towardHome)) {
    Serial.print("OK JOG position_steps=");
    Serial.println(baseMotor.currentPosition());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);

  baseMotor.setEnablePin(ENABLE_PIN);
  baseMotor.setPinsInverted(INVERT_MOTOR_DIRECTION, false, true);
  baseMotor.setMinPulseWidth(2);
  baseMotor.setMaxSpeed(JOG_MAX_SPEED);
  baseMotor.setAcceleration(JOG_ACCELERATION);
  setMotorEnabled(false);

  delay(250);
  Serial.println("READY Octopus Pro base motor/endstop test");
  printHelp();
  printStatus();
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "HELP") {
    printHelp();
  } else if (command == "STATUS") {
    printStatus();
  } else if (command == "ENABLE") {
    setMotorEnabled(true);
    Serial.println("OK ENABLE");
  } else if (command == "DISABLE") {
    setMotorEnabled(false);
    Serial.println("OK DISABLE");
  } else if (command.startsWith("JOG ")) {
    handleJog(command);
  } else if (command == "HOME") {
    runHomeTest();
  } else {
    Serial.println("ERROR Unknown command; enter HELP");
  }
}
