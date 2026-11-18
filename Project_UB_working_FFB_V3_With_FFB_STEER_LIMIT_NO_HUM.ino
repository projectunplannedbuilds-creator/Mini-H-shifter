#include <avr/interrupt.h>
#include "Joystick.h"
#include <Wire.h>

// ==========================================
// MT6701 I2C
// ==========================================

#define MT6701_ADDR 0x06
#define ANGLE_REG_HIGH 0x03
#define ANGLE_REG_LOW  0x04

const float COUNTS_PER_REV = 16384.0;
const float STEERING_LIMIT = 270.0;

// Fixed magnet center
const uint16_t CENTER_RAW = 0;

uint16_t previousRaw = 0;
int32_t accumulatedCounts = 0;

float lastSteeringDegrees = 0;

bool mt6701Error = false;
bool hasValidReading = false;

// ==========================================
// JOYSTICK
// ==========================================

Joystick_ SimController(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  18, 0,

  true, false, true,  // X, Y, Z
  false, false, false,  // Rx, Ry, Rz
  false, false, false,  // Rudder, Throttle, Accelerator
  true, false          // Brake, Steering
);

// ==========================================
// FORCE FEEDBACK
// ==========================================

int32_t forces[2] = {0, 0};

Gains mygains[2];
EffectParams myeffectparams[2];

// ==========================================
// STEERING OVERTRAVEL PROTECTION 
// ==========================================

const float SAFETY_LIMIT = 273.0; // MAX STEERING ANGLE
const int SAFETY_MAX_PWM = 255;   // MMIN 0, MAX 255 || How much the motor resist when overtravel  

const bool SAFETY_DIRECTION_INVERTED = true;

const unsigned long SAFETY_PULSE_HALF_PERIOD_MS = 12;
const unsigned long SAFETY_PHASE_MS = 12;
// ==========================================
// L298N MOTOR DRIVER
// ==========================================

#define MOTOR_IN1 9
#define MOTOR_IN2 6

// THIS IS FOR RETURN TO CENTER. NOT USED IN FORCE FEEDBACK
const bool MOTOR_DIRECTION_INVERTED = false;
const float CENTER_DEADBAND_DEG = 3.0;
const int MIN_PWM = 200;
const int MAX_PWM = 255;
int motorIn1Command = 0;
int motorIn2Command = 0;

// ==========================================
// MOTOR DAMPING CONTROL
// ==========================================
// THIS IS FOR RETURN TO CENTER. NOT USED IN FORCE FEEDBACK
// (OFF by default, Uncomment "returnToCenter(steeringDegrees);" below)
const float KP = 1.5;       // Spring strength
const float KD = 0.08;      // Damping strength

const float STOP_ANGLE = 2.0; // return to center stops when close to center at 2°
const float STOP_SPEED = 15.0; // how fast it return to center

float previousMotorAngle = 0;
float steeringVelocity = 0;

unsigned long previousMotorTime = 0;

bool motorControlInitialized = false;

// ==========================================
// USB FORCE FEEDBACK INTERRUPT
// ==========================================
  volatile uint32_t pidISRCount = 0;
ISR(TIMER3_COMPA_vect) {
  pidISRCount++;
  SimController.getUSBPID();
}

// ==========================================
// READ I2C REGISTER
// ==========================================

bool readMT6701Register(uint8_t reg, uint8_t &value) {

  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((uint8_t)MT6701_ADDR, (uint8_t)1) != 1) {
    return false;
  }

  value = Wire.read();

  return true;
}

// ==========================================
// READ MT6701 ABSOLUTE ANGLE
// ==========================================

bool readMT6701(uint16_t &angle) {

  uint8_t highByte;
  uint8_t lowByte;

  // Read register 0x03 first
  if (!readMT6701Register(ANGLE_REG_HIGH, highByte)) {
    return false;
  }

  // Then read register 0x04
  if (!readMT6701Register(ANGLE_REG_LOW, lowByte)) {
    return false;
  }

  // Combine 14-bit angle
  angle = ((uint16_t)highByte << 6) |
          ((uint16_t)lowByte >> 2);

  angle &= 0x3FFF;

  return true;
}

// ==========================================
// INITIALIZE STEERING
// ==========================================

void initializeSteering() {

  uint16_t initialRaw;

  hasValidReading = false;

  for (int i = 0; i < 20; i++) {

    if (readMT6701(initialRaw)) {

      int32_t difference =
        (int32_t)initialRaw - CENTER_RAW;

      // Shortest initial distance from fixed center
      if (difference > 8192)
        difference -= 16384;

      if (difference < -8192)
        difference += 16384;

      accumulatedCounts = difference;
      previousRaw = initialRaw;

      lastSteeringDegrees =
        accumulatedCounts * 360.0 / COUNTS_PER_REV;

      lastSteeringDegrees = constrain(
        lastSteeringDegrees,
        -STEERING_LIMIT,
        STEERING_LIMIT
      );

      hasValidReading = true;
      mt6701Error = false;

      Serial.print("MT6701 INITIAL RAW: ");
      Serial.println(initialRaw);

      return;
    }

    delay(10);
  }

  mt6701Error = true;

  Serial.println("MT6701 I2C ERROR");
}

// ==========================================
// READ STEERING DEGREES
// ==========================================

float readSteeringDegrees() {

  uint16_t currentRaw;

  if (!readMT6701(currentRaw)) {

    // Ignore failed reading.
    // Do not change previousRaw or accumulatedCounts.
    mt6701Error = true;

    return lastSteeringDegrees;
  }

  mt6701Error = false;

  // First valid reading after failed initialization
  if (!hasValidReading) {

    int32_t difference =
      (int32_t)currentRaw - CENTER_RAW;

    if (difference > 8192)
      difference -= 16384;

    if (difference < -8192)
      difference += 16384;

    accumulatedCounts = difference;
    previousRaw = currentRaw;

    hasValidReading = true;

    lastSteeringDegrees =
      accumulatedCounts * 360.0 / COUNTS_PER_REV;


      
    return lastSteeringDegrees;
  }

  // Calculate movement since last successful reading
  int32_t difference =
    (int32_t)currentRaw - previousRaw;

  // 14-bit wraparound correction
  if (difference > 8192) {

    difference -= 16384;

  }
  else if (difference < -8192) {

    difference += 16384;

  }

  // Accumulate movement
  accumulatedCounts += difference;

  previousRaw = currentRaw;

  float degrees =
    accumulatedCounts * 360.0 / COUNTS_PER_REV;

lastSteeringDegrees = degrees;

return lastSteeringDegrees;
}
// ==========================================
// HIGH-FREQUENCY MOTOR PWM
// Pro Micro / ATmega32U4
// D9 = Timer1 OC1A
// D6 = Timer4 OC4D
// Target: 31.25 kHz on both outputs
// ==========================================

void setupMotorPWMFrequency() {

  // D9 / Timer1: Fast PWM, TOP = 511
  // 16 MHz / (511 + 1) = 31.25 kHz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  ICR1 = 511;
  OCR1A = 0;

  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  // D6 / Timer4 uses the ATmega32U4 high-speed PLL clock.
  // Select 96 MHz PLL / 1.5 = 64 MHz PCK.
  // Preserve PLLUSB and the PLL frequency divider settings.
  TCCR4B = 0;
  TCCR4A = 0;
  TCCR4C = 0;
  TCCR4D = 0;

  PLLFRQ = (PLLFRQ & ~(_BV(PLLTM1) | _BV(PLLTM0))) | _BV(PLLTM1);
  delay(1);

  // 64 MHz PCK / 8 / 256 counts = 31.25 kHz.
  // Timer4 is deliberately used in 8-bit mode.
  TC4H = 0;
  OCR4C = 255;
  TC4H = 0;
  OCR4D = 0;

  TCCR4D = 0;          // Fast PWM mode
  TCCR4C = _BV(PWM4D); // PWM channel D enabled, output disconnected
  TCCR4B = _BV(CS42);  // Asynchronous PCK / 8

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}


// 0 = LOW, 255 = HIGH, 1..254 = hardware PWM.
void writeMotorPWM(uint8_t pin, uint8_t value) {

  if (pin == MOTOR_IN1) {

    if (value == 0 || value == 255) {
      TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
      digitalWrite(MOTOR_IN1, (value == 255) ? HIGH : LOW);
    } else {
      OCR1A = (uint16_t)value * 2U;
      TCCR1A = (TCCR1A & ~_BV(COM1A0)) | _BV(COM1A1);
    }
  }

  else if (pin == MOTOR_IN2) {

    if (value == 0 || value == 255) {
      TCCR4C = _BV(PWM4D);
      digitalWrite(MOTOR_IN2, (value == 255) ? HIGH : LOW);
    } else {
      TC4H = 0;
      OCR4D = value;
      TCCR4C = _BV(PWM4D) | _BV(COM4D1);
    }
  }
}


// ==========================================
// MOTOR STOP
// ==========================================

void stopMotor() {

  writeMotorPWM(MOTOR_IN1, 0);
  writeMotorPWM(MOTOR_IN2, 0);

  motorIn1Command = 0;
  motorIn2Command = 0;
}

// ==========================================
// RETURN TO CENTER
// ==========================================

void returnToCenter(float steeringDegrees) {

  if (mt6701Error || !hasValidReading) {
    stopMotor();
    motorControlInitialized = false;
    return;
  }

  unsigned long currentTime = micros();

  if (!motorControlInitialized) {

    previousMotorAngle = steeringDegrees;
    previousMotorTime = currentTime;
    steeringVelocity = 0;

    motorControlInitialized = true;
    stopMotor();
    return;
  }

  float dt = (currentTime - previousMotorTime) / 1000000.0;

  if (dt <= 0 || dt > 0.1) {
    previousMotorAngle = steeringDegrees;
    previousMotorTime = currentTime;
    steeringVelocity = 0;
    stopMotor();
    return;
  }

  // Calculate steering speed in degrees per second
  float instantVelocity =
    (steeringDegrees - previousMotorAngle) / dt;

  // Smooth velocity to reduce sensor noise
  steeringVelocity =
    0.25 * instantVelocity +
    0.75 * steeringVelocity;

  previousMotorAngle = steeringDegrees;
  previousMotorTime = currentTime;

  // Position error + velocity damping
  float control =
    (KP * steeringDegrees) +
    (KD * steeringVelocity);

  // Stop only when both position and speed are small
  if (abs(steeringDegrees) < STOP_ANGLE &&
      abs(steeringVelocity) < STOP_SPEED) {

    stopMotor();
    return;
  }

  // Limit motor output
  control = constrain(
    control,
    -(MAX_PWM - MIN_PWM),
    MAX_PWM - MIN_PWM
  );

  int output = (int)abs(control);

  if (output < 1) {
    stopMotor();
    return;
  }

  output += MIN_PWM;
  output = constrain(output, MIN_PWM, MAX_PWM);

  // Motor direction
  bool direction = (control > 0);

  if (MOTOR_DIRECTION_INVERTED) {
    direction = !direction;
  }

  if (direction) {

    writeMotorPWM(MOTOR_IN1, output);
    writeMotorPWM(MOTOR_IN2, 0);

    motorIn1Command = output;
    motorIn2Command = 0;

  } else {

    writeMotorPWM(MOTOR_IN1, 0);
    writeMotorPWM(MOTOR_IN2, output);

    motorIn1Command = 0;
    motorIn2Command = output;
  }
}









// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true); // Bus Timeout

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  setupMotorPWMFrequency();
  stopMotor();

  delay(500);

  Serial.println("I2C STEERING STARTED");

  initializeSteering();

  // Joystick
  SimController.begin(false);
  
  SimController.setXAxis(512);
  SimController.setZAxis(512);

  SimController.setSteeringRange(0, 1023);

//  If you need more Analog input, set this bolean
//  at the top of this code to true and
//         true, false, true,      | X, Y, Z
//        false, false, false,     | Rx, Ry, Rz
//        false, false, false,     | Rudder, Throttle, Accelerator
//         true, false             | Brake, Steering

//  SimController.setRxAxis(512); // 
//  SimController.setRyAxis(512); //
//  SimController.setRzAxis(512); //

//  SimController.setThrottle(512);
//  SimController.setAccelerator(512);
//  SimController.setBrake(512);





// ==========================================
// FORCE FEEDBACK GAINS
// ==========================================
// No need to change this. if it not working problem is not here
  // First FFB axis
  mygains[0].totalGain = 100;
  mygains[0].constantGain = 100;
  mygains[0].springGain = 100;
  mygains[0].damperGain = 100;
  
  // Second FFB axis
  mygains[1].totalGain = 100;
  mygains[1].constantGain = 100;
  mygains[1].springGain = 100;
  mygains[1].damperGain = 100;
  
  SimController.setGains(mygains);

  // ==========================================
  // TIMER3 - USB FFB RECEPTION
  // ==========================================
  
  cli();
  
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  
  OCR3A = 399;
  
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  
  TIMSK3 |= (1 << OCIE3A);
  
  sei();
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

// ========================================================
//                      H-SHIFTER
// ========================================================
//============== PASTE FROM BELOW THIS LINE ===============
//        (The data we got from H_SHIFTER_VALUE.ino)
//---------------------------------------------------------
  int SHIFTER_X_Value = analogRead(A10);
  int SHIFTER_Y_Value = analogRead(A0);
  int selectedButton = -1;

  if (SHIFTER_Y_Value > 800 && SHIFTER_X_Value < 50 )
    selectedButton = 10;

  else if (SHIFTER_Y_Value < 200  && SHIFTER_X_Value < 50 )
    selectedButton = 11;

  else if (SHIFTER_Y_Value > 800 && SHIFTER_X_Value > 196 && SHIFTER_X_Value < 396 )
    selectedButton = 12;  

  else if (SHIFTER_Y_Value < 200 && SHIFTER_X_Value > 196 && SHIFTER_X_Value < 396 )
    selectedButton = 13;

  else if (SHIFTER_Y_Value > 800 && SHIFTER_X_Value > 561 && SHIFTER_X_Value < 761)
    selectedButton = 14;
 
  else if (SHIFTER_Y_Value < 200 && SHIFTER_X_Value > 561 && SHIFTER_X_Value < 761 )
    selectedButton = 15;

  else if (SHIFTER_Y_Value > 800 && SHIFTER_X_Value > 900 )
    selectedButton = 16;  
       
  else if (SHIFTER_Y_Value < 200 && SHIFTER_X_Value > 900 )
    selectedButton = 9;  
       
  for (int i = 9; i <= 16; i++) {
    SimController.setButton(i, i == selectedButton);
    }
// ------------------------------------------------------
// =============== UP TO ABOVE THIS =====================
//        (The data we got from H_SHIFTER_VALUE.ino)
// ======================================================
//                  H-SHIFTER END
// ======================================================





// ======================================================
//                    STEERING
// ======================================================

  float steeringDegrees = readSteeringDegrees();

  int steeringValue = map(
    (long)steeringDegrees,
    -270, 270,
    1023, 0
  );

  steeringValue = constrain(steeringValue, 0, 1023);
  
  //SimController.setSteering(steeringValue);
  SimController.setXAxis(steeringValue);

// ========================================
//           RETURN TO CENTER
// ========================================
     /*  Uncomment the line below
      *  If you want your steering
      *  to automatically return to center   */
//  returnToCenter(steeringDegrees);
  
// ========================================
// PEDALS
// ========================================

  SimController.setBrake(analogRead(A1));
  SimController.setZAxis(analogRead(A2));

// ========================================
// FORCE FEEDBACK PROCESSING
// ========================================
  
  // First FFB effect parameters
  myeffectparams[0].springMaxPosition = 1023;
  myeffectparams[0].springPosition = steeringValue;
  
  // Second FFB effect parameters
  myeffectparams[1].springMaxPosition = 1023;
  myeffectparams[1].springPosition = 512;
  
  // Update effect parameters
  SimController.setEffectParams(myeffectparams);
  
  // Receive game force feedback
  SimController.getForce(forces);


// ==========================================
// GAME FORCE FEEDBACK + SAFETY LOCKOUT
// ==========================================
bool positiveLimit = (steeringDegrees >= SAFETY_LIMIT);
bool negativeLimit = (steeringDegrees <= -SAFETY_LIMIT);

if (positiveLimit || negativeLimit) {

  // Alternate between strong and weaker reverse pulses
  bool phase = ((millis() / SAFETY_PHASE_MS) % 2) == 0;

  if (positiveLimit) {

    // Above +273 degrees:
    // Phase 1: CW 255
    // Phase 2: CCW 127

    if (phase) {
      writeMotorPWM(MOTOR_IN1, 250);  // CW  
      writeMotorPWM(MOTOR_IN2, 0);
    } 
    else {
      writeMotorPWM(MOTOR_IN1, 0);
      writeMotorPWM(MOTOR_IN2, 235);  // CCW
    }

  } 
  else {

    // Below -273 degrees:
    // Phase 1: CCW 255
    // Phase 2: CW 127

    if (phase) {
      writeMotorPWM(MOTOR_IN1, 0);
      writeMotorPWM(MOTOR_IN2, 250);  // CCW
    } 
    else {
      writeMotorPWM(MOTOR_IN1, 235);  // CW
      writeMotorPWM(MOTOR_IN2, 0);
    }
  }

} 
else {

  // NORMAL GAME FFB — unchanged

  int force = constrain(-forces[0], -100, 100);
  int motorPWM = map(abs(force), 0, 100, 50, 200); // LEAVE THE 0 AND 100 UNCHANGE 
  // 50, 200 || 50 is the minimum and 200 is maximum can be set  from 0 to  255  ==== 

  if (force > 2) {
    writeMotorPWM(MOTOR_IN1, motorPWM);
    writeMotorPWM(MOTOR_IN2, 0);
  } 
  else if (force < -2) {
    writeMotorPWM(MOTOR_IN1, 0);
    writeMotorPWM(MOTOR_IN2, motorPWM);
  } 
  else {
    stopMotor();
  }
}

  SimController.sendState();
// ========================================
// DEBUG
// ========================================
//
//  Serial.print("MT6701: ");
//  Serial.print(mt6701Error ? "I2C ERROR" : "OK");
//
//  Serial.print(" | Steering: ");
//  Serial.print(steeringDegrees, 2);
//
//  Serial.print(" | Raw: ");
//  Serial.print(previousRaw);
//
//  Serial.print(" | Accumulated: ");
//  Serial.print(accumulatedCounts);
//
//  Serial.print(" | IN1 PWM: ");
//  Serial.print(motorIn1Command);
//
//  Serial.print(" | IN2 PWM: ");
//  Serial.print(motorIn2Command);
//
//  Serial.print(" | Speed: ");
//  Serial.print(steeringVelocity, 1);
//  Serial.println(" deg/s");
//
//  Serial.print(" | FFB X: ");
//  Serial.print(forces[0]);
//  
//  Serial.print(" | FFB Y: ");
//  Serial.print(forces[1]);

// ========================================
//               DEBUG 2
// =========================================
/*static unsigned long lastDebug = 0;

if (millis() - lastDebug >= 1000) {
  lastDebug = millis();

  uint32_t count;

  noInterrupts();
  count = pidISRCount;
  interrupts();

  Serial.print("PID ISR Count: ");
  Serial.print(count);

  Serial.print(" | FFB X: ");
  Serial.print(forces[0]);

  Serial.print(" | FFB Y: ");
  Serial.print(forces[1]);

Serial.print(" | FFB X: ");
Serial.print(forces[0]);

Serial.print(" | FFB Y: ");
Serial.print(forces[1]);

if (steeringDegrees >= 273.0) {
  Serial.println("POSITIVE LIMIT DETECTED");
}
else if (steeringDegrees <= -273.0) {
  Serial.println("NEGATIVE LIMIT DETECTED");
}

Serial.print(" | Spring Position: ");
Serial.println(myeffectparams[0].springPosition);

  
}*/

  delay(5);
}
