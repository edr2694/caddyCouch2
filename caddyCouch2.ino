#include <Servo.h>
#include <EEPROM.h> // for saving values
#include "caddyCouch2Defs.h"

const uint8_t  LEFT_SERVO_PIN  = 5;
const uint8_t  RIGHT_SERVO_PIN = 6;
const uint8_t  RC_CH1          = A0;
const uint8_t  RC_CH2          = A1;
const uint8_t  RC_CH3          = A2;
const uint8_t  RC_CH8          = A3; // the knob, to be used for either turn specific sensitivity or absolute maximum lockout

Servo leftMotor;
Servo rightMotor;

// global changeable variables
boolean  outputEnable    = false;
boolean  reverseRight    = true;
boolean  reverseLeft     = false;
boolean  deadZoneEnable  = true;
boolean  capMaxVelo      = true;
uint16_t veloDeadZone    = 6;
uint16_t diffDeadZone    = 6;
uint16_t maxDifference   = 20;
uint8_t  maxPWM          = 180;
uint8_t  minPWM          = 0;
uint16_t CH1_MAX_INPUT   = 1924; // right stick right/left TURN
uint16_t CH1_MIN_INPUT   = 963;
uint16_t CH2_MAX_INPUT   = 1989; // right stick up/down MAIN THROTTLE
uint16_t CH2_MIN_INPUT   = 984;
uint16_t CH3_MAX_INPUT   = 1800; // left stick up/down LIMITTER
uint16_t CH3_MIN_INPUT   = 1100;
uint16_t CH8_MIN_INPUT   = 989; // secondary limiter
uint16_t CH8_MAX_INPUT   = 1984;
uint8_t  maxPosDeltaV    = 10; // max per control loop cycle
uint8_t  maxNegDeltaV    = 5;
uint16_t delayTimeMillis = 5;

// global working variables. Use sparingly
boolean  serialDebugPrintEnable = false;
boolean  newCommand             = false;
boolean  claibrateFlag          = false;
String   commandStr             = "";
uint16_t CH1_last_value         = 0;
uint16_t CH2_last_value         = 0;
uint16_t CH3_last_value         = 0;
uint16_t CH8_last_value         = 0;
int16_t CH1_scaled_input_persistent = 0;
int16_t CH2_scaled_input_persistent = 0;
int16_t CH3_scaled_input_persistent = 0;
int16_t CH8_scaled_input_persistent = 0;
uint32_t timeout                = 100;
uint32_t lastTime               = 0;
uint8_t  maxLeftPWM             = 180;
uint8_t  minLeftPWM             = 0;
uint8_t  maxRightPWM            = 180;
uint8_t  minRightPWM            = 0;



void setup() {
    Serial.begin(115200);
    leftMotor.attach(LEFT_SERVO_PIN); leftMotor.write(90);
    rightMotor.attach(RIGHT_SERVO_PIN); rightMotor.write(90);
    pinMode(RC_CH1, INPUT);
    pinMode(RC_CH2, INPUT);
    pinMode(RC_CH3, INPUT);
    pinMode(RC_CH8, INPUT);
    if (reverseLeft) {
        maxLeftPWM = 0;
        minLeftPWM = 180;
    }
    if (reverseRight) {
        maxRightPWM = 0;
        minRightPWM = 180;
    }
}

void loop() {
    if (newCommand) {
        handleCommand(); // do this first, because if anything changed we want to implement it immediately.
    }
    int16_t CH1_IN_VAL;
    int16_t CH2_IN_VAL;
    int16_t CH3_IN_VAL;
    int16_t CH8_IN_VAL;
    getSmoothReceiver(&CH1_IN_VAL, &CH2_IN_VAL, &CH3_IN_VAL, &CH8_IN_VAL);
    scaleChannelValue(); // scale that input to -99 to 99 for ease of use
    adjustOutput(); // perform control loop calculations here
    controlMotors();
    // set persistent velocities (static variables, in loop only)

}
