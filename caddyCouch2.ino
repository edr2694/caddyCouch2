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
    if (claibrateFlag) {
        handleCalibration(CH1_IN_VAL, CH2_IN_VAL, CH3_IN_VAL, CH8_IN_VAL);
    }
    scaleChannelValue(); // scale that input to -99 to 99 for ease of use
    adjustOutput(); // perform control loop calculations here
    controlMotors();
    // set persistent velocities (static variables, in loop only)

}

void getReceiver(int16_t *ch1, int16_t *ch2, int16_t *ch3, int16_t *ch8) {
  CH1_last_value = 0;
  CH2_last_value = 0;
  CH3_last_value = 0;
  CH8_last_value = 0;

  lastTime = millis();
  while (CH1_last_value == 0 && millis() - lastTime < timeout) {
    CH1_last_value = pulseIn(RC_CH1, HIGH, 20000);
  }
  lastTime = millis();
  while (CH2_last_value == 0 && millis() - lastTime < timeout) {
    CH2_last_value = pulseIn(RC_CH2, HIGH, 20000);
  }
  lastTime = millis();
  while (CH3_last_value == 0 && millis() - lastTime < timeout) {
    CH3_last_value = pulseIn(RC_CH3, HIGH, 20000);
  }
  lastTime = millis();
  while (CH8_last_value == 0 && millis() - lastTime < timeout) {
    CH8_last_value = pulseIn(RC_CH8, HIGH, 20000);
  }
}

void getSmoothReceiver(int16_t *ch1, int16_t *ch2, int16_t *ch3, int16_t *ch8) {
    int32_t ch1Average = 0;
    int32_t ch2Average = 0;
    int32_t ch3Average = 0;
    int32_t ch8Average = 0;
    for (uint8_t i = 0; i < 3; i++) {
        getReceiver();
        ch1Average += CH1_last_value;
        ch2Average += CH2_last_value;
        ch3Average += CH3_last_value;
        ch8Average += CH8_last_value;
    }
    CH1_last_value = ch1Average / 3.0;
    CH2_last_value = ch2Average / 3.0;
    CH3_last_value = ch3Average / 3.0;
    CH8_last_value = ch8Average / 3.0;
}

void handleCommand() {
    commandStr.remove(commandStr.indexOf("\n"));
    if (commandStr.length() < 1) {
        invalidCommand();
        Serial.println("Command less than 1");
    } else if (commandStr.equalsIgnoreCase(enableCmdStr) && !claibrateFlag) {
        outputEnable = true;
    } else if (commandStr.equalsIgnoreCase(disableCmdStr)) {
        outputEnable = false;
    } else if (commandStr.equalsIgnoreCase(helpCmdStr)) {
        printHelp();
    } else if (commandStr.equalsIgnoreCase(lstParamCmdStr)) {
        printParameters();
    } else if (commandStr.equalsIgnoreCase(debugCmdStr)) {
        serialDebugPrintEnable = true;
    } else if (commandStr.equalsIgnoreCase(calibrCmdStr)) {
        if (!claibrateFlag) {
            claibrateFlag = true;
        } else {
            invalidCommand();
            Serial.println("bad calibrate");
        }
    } else if (commandStr.equalsIgnoreCase(quitStr) && claibrateFlag) {
        claibrateFlag = false;
    } else if (commandStr.equalsIgnoreCase(quitStr) && serialDebugPrintEnable) {
        serialDebugPrintEnable = false;
    } else if (commandStr.equalsIgnoreCase(saveCmdStr)&& !outputEnable) {
        saveParameters();
    } else if (commandStr.equalsIgnoreCase(reloadCmdStr) && !outputEnable) {
        reloadParameters();
    } else if (commandStr.equalsIgnoreCase(lstChngParamCmdStr)) {
        listParameters();
    } else if (commandStr.substring(0,2).equalsIgnoreCase(chngParamCmdStr) && !outputEnable) {
        changeParameters();
    } else {
        invalidCommand();
        Serial.print("Bad command, len: ");Serial.print(commandStr.length());Serial.print(",");Serial.println(commandStr);
    }
    commandStr = "";
    newCommand = false;
}

void invalidCommand() {
    Serial.println(invalidCmdString);
}

void serialDebugPrint() {
  Serial.print("1:"); Serial.print(CH1_last_value);
  Serial.print(",2:"); Serial.print(CH2_last_value);
  Serial.print(",3:"); Serial.print(CH3_last_value);
  Serial.print(",8:"); Serial.print(CH8_last_value);
  Serial.print(",V:"); Serial.print(velocity);
  Serial.print(",D:"); Serial.print(difference);
  Serial.print(",M:"); Serial.print(maxSpeed);
  Serial.print(",L:"); Serial.print(leftMotorSpeed);
  Serial.print(",R:"); Serial.println(rightMotorSpeed);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    commandStr += inChar;
    if (inChar == '\n') {
        newCommand = true;
    }
  }
}

void printHelp() {
    Serial.println("******Caddy Couch Control V0.2******");
    Serial.println("Available Commands:");
    Serial.print(helpCmdStr);Serial.println(": print this message");
    Serial.print(lstParamCmdStr);Serial.println(": list all parameter values");
    Serial.print(debugCmdStr);Serial.println(": print debug values (q to quit)");
    Serial.print(enableCmdStr);Serial.println(": enable motor output");
    Serial.print(disableCmdStr);Serial.println(": disable motor output");
    Serial.print(lstChngParamCmdStr);Serial.println(": list changeable parameters");
    Serial.print(chngParamCmdStr);Serial.println(" <parameter> <value>: change a parameter value. Motors must be disabled");
    Serial.print(saveCmdStr);Serial.println(": save parameters to eeprom. Will overwrite any values stored there, and cannot be undone");
    Serial.print(reloadCmdStr);Serial.println(": reload parameters from eeprom");

}

void printParameters() {
    Serial.println("***Current Parameters***");
    Serial.println("---Motor Settings---");
    Serial.print(outEnStr);Serial.println(outputEnable);
    Serial.print(revRightStr);Serial.println(reverseRight);
    Serial.print(revLeftStr);Serial.println(reverseLeft);
    Serial.println("---Control Parameters---");
    Serial.print(maxDiffStr);Serial.println(maxDifference);
    Serial.print(maxPWMStr);Serial.println(maxPWM);
    Serial.print(minPWMStr);Serial.println(minPWM);
    Serial.print(veloDZStr);Serial.println(veloDeadZone);
    Serial.print(diffDZStr);Serial.println(diffDeadZone);
    Serial.print(dzEnaStr);Serial.println(deadZoneEnable);
    Serial.print(ch1maxStr);Serial.println(CH1_MAX);
    Serial.print(ch1minStr);Serial.println(CH1_MIN);
    Serial.print(ch2maxStr);Serial.println(CH2_MAX);
    Serial.print(ch2minStr);Serial.println(CH2_MIN);
    Serial.print(ch3maxStr);Serial.println(CH3_MAX);
    Serial.print(ch3minStr);Serial.println(CH3_MIN);
    Serial.print(ch8maxStr);Serial.println(CH8_MAX);
    Serial.print(ch8minStr);Serial.println(CH8_MIN);
}

void listParameters(void) {
    Serial.println("---Motor Settings---");
    Serial.println(outEnStr);
    Serial.println(revRightStr);
    Serial.println(revLeftStr);
    Serial.println("---Control Loop Parameters---");
    Serial.println(maxDiffStr);
    Serial.println(maxPWMStr);
    Serial.println(minPWMStr);
    Serial.println(veloDZStr);
    Serial.println(diffDZStr);
    Serial.println(dzEnaStr);
    Serial.println("---Controller Parameters---");
    Serial.println(ch1maxStr);
    Serial.println(ch1minStr);
    Serial.println(ch2maxStr);
    Serial.println(ch2minStr);
    Serial.println(ch3maxStr);
    Serial.println(ch3minStr);
    Serial.println(ch8maxStr);
    Serial.println(ch8minStr);
}

void changeParameters() {
    // first check the function for form
    String paramVal = commandStr.substring(3);
    String param;
    String value;
    // check for first index of space
    int spaceIndex = paramVal.indexOf(" ");
    if (spaceIndex <= 0) {
        // didn't find a space, or there was more than one space between cp and param
        Serial.println(invalidCmdString);
        return;
    } else {
        param = paramVal.substring(0,spaceIndex);
        value = paramVal.substring(spaceIndex+1);
    }
    // check value for spaces
    if (value.indexOf(" ") >= 0) {
        Serial.println(invalidCmdString);
        return;
    }
    // look through the parameters and see if we get a match
    findAndAdjustParam(&param, &value);
}

void findAndAdjustParam(String *param, String *value)
{
    // ok, before we start this function. Let's clear the air.
    // Yes, this sucks. I know it sucks. But the ArduinoSTL library
    // which could have been used to group these guys by type
    // using std::map has major issues, and any other option is
    // going to be just as bad if not worse, so here we are.

    // Don't worry too much about efficiency. This particular function
    // can only run if the motors have been disabled, in other words,
    // we're not interrupting important control loop buisness.

    if (param->equalsIgnoreCase(revRightStr)) {
        checkAndUpdateBoolean(value, &reverseRight);
    } else if (param->equalsIgnoreCase(revLeftStr)) {
        checkAndUpdateBoolean(value, &reverseLeft);
    } else if (param->equalsIgnoreCase(dzEnaStr)) {
        checkAndUpdateBoolean(value, &deadZoneEnable);
    } else if (param->equalsIgnoreCase(veloDZStr)) {
        checkAndUpdateUint16(value, &veloDeadZone);
    } else if (param->equalsIgnoreCase(diffDZStr)) {
        checkAndUpdateUint16(value, &diffDeadZone);
    } else if (param->equalsIgnoreCase(maxDiffStr)) {
        checkAndUpdateUint16(value, &maxDifference);
    } else if (param->equalsIgnoreCase(maxPWMStr)) {
        checkAndUpdateUint8(value, &maxPWM);
    } else if (param->equalsIgnoreCase(minPWMStr)) {
        checkAndUpdateUint8(value, &minPWM);
    } else if (param->equalsIgnoreCase(ch1maxStr)) {
        checkAndUpdateUint16(value, &CH1_MAX);
    } else if (param->equalsIgnoreCase(ch1minStr)) {
        checkAndUpdateUint16(value, &CH1_MIN);
    } else if (param->equalsIgnoreCase(ch2maxStr)) {
        checkAndUpdateUint16(value, &CH2_MAX);
    } else if (param->equalsIgnoreCase(ch2minStr)) {
        checkAndUpdateUint16(value, &CH2_MIN);
    } else if (param->equalsIgnoreCase(ch3maxStr)) {
        checkAndUpdateUint16(value, &CH3_MAX);
    } else if (param->equalsIgnoreCase(ch3minStr)) {
        checkAndUpdateUint16(value, &CH3_MIN);
    } else if (param->equalsIgnoreCase(ch8maxStr)) {
        checkAndUpdateUint16(value, &CH8_MAX);
    } else if (param->equalsIgnoreCase(ch8minStr)) {
        checkAndUpdateUint16(value, &CH8_MIN);
    } else if (param->equalsIgnoreCase(delayTmStr)) {
        checkAndUpdateUint16(value, &delayTimeMillis);
    } else if (param->equalsIgnoreCase(useKnobStr)) {
        checkAndUpdateBoolean(value, &capMaxVelo);
    } else {
        Serial.println(invalidPrmString);
    }
}

void checkAndUpdateBoolean(String *value, boolean *bVal) {
    if (!value->equalsIgnoreCase(boolTrue) && !value->equalsIgnoreCase(boolFalse)) {
            Serial.println(invalidPrmString);
    } else {
        if (value->equalsIgnoreCase(boolTrue)) {
            *bVal = true;
        } else {
            *bVal = false;
        }
    }
}

void checkAndUpdateInt16(String *value, int16_t *iVal) {
    int interVal = value->toInt();
    if (!value->equalsIgnoreCase("0") && interVal == 0) {
        // this is an error condition
        Serial.println(invalidPrmString);
    } else {
        *iVal = interVal;
    }
}

void checkAndUpdateUint16(String *value, uint16_t *uVal) {
    int interVal = value->toInt();
    if (!value->equalsIgnoreCase("0") && interVal == 0) {
        // this is an error condition
        Serial.println(invalidPrmString);
    } else if (interVal < 0) {
        Serial.println(invalidPrmString);
    } else {
        *uVal = (uint16_t)interVal;
    }
}

void checkAndUpdateUint8(String *value, uint8_t *uVal) {
    int interVal = value->toInt();
    if (!value->equalsIgnoreCase("0") && interVal == 0) {
        // this is an error condition
        Serial.println(invalidPrmString);
    } else if (interVal < 0) {
        Serial.println(invalidPrmString);
    } else {
        *uVal = (uint8_t)interVal;
    }
}

void saveParameters() {
    // See the note at the start of the find and adjust parameter function
    // #YouGetWhatYouPayFor
    // #AndThisOnesFree
    EEPROM.put(outEnAddr, outputEnable);
    EEPROM.put(revRightAddr, reverseRight);
    EEPROM.put(revLeftAddr, reverseLeft);
    EEPROM.put(dzEnAddr, deadZoneEnable);
    EEPROM.put(veloDZAddr, veloDeadZone);
    EEPROM.put(diffDZAddr, diffDeadZone);
    EEPROM.put(maxDiffAddr, maxDifference);
    EEPROM.put(maxPWMAddr, maxPWM);
    EEPROM.put(minPWMAddr, minPWM);
    EEPROM.put(CH1_MAXAddr, CH1_MAX);
    EEPROM.put(CH1_MINAddr, CH1_MIN);
    EEPROM.put(CH2_MAXAddr, CH2_MAX);
    EEPROM.put(CH2_MINAddr, CH2_MIN);
    EEPROM.put(CH3_MAXAddr, CH3_MAX);
    EEPROM.put(CH3_MINAddr, CH3_MIN);
    EEPROM.put(CH8_MINAddr, CH8_MAX);
    EEPROM.put(CH8_MAXAddr, CH8_MIN);
    EEPROM.put(delayMilsAddr, delayTimeMillis);
}

void reloadParameters() {
    // See the note at the start of the find and adjust parameter function
    // #YouGetWhatYouPayFor
    // #AndThisOnesFree
    EEPROM.get(outEnAddr, outputEnable);
    EEPROM.get(revRightAddr, reverseRight);
    EEPROM.get(revLeftAddr, reverseLeft);
    EEPROM.get(dzEnAddr, deadZoneEnable);
    EEPROM.get(veloDZAddr, veloDeadZone);
    EEPROM.get(diffDZAddr, diffDeadZone);
    EEPROM.get(maxDiffAddr, maxDifference);
    EEPROM.get(maxPWMAddr, maxPWM);
    EEPROM.get(minPWMAddr, minPWM);
    EEPROM.get(CH1_MAX_INPUTAddr, CH1_MAX_INPUT);
    EEPROM.get(CH1_MIN_INPUTAddr, CH1_MIN_INPUT);
    EEPROM.get(CH2_MAX_INPUTAddr, CH2_MAX_INPUT);
    EEPROM.get(CH2_MIN_INPUTAddr, CH2_MIN_INPUT);
    EEPROM.get(CH3_MAX_INPUTAddr, CH3_MAX_INPUT);
    EEPROM.get(CH3_MIN_INPUTAddr, CH3_MIN_INPUT);
    EEPROM.get(CH8_MIN_INPUTAddr, CH8_MAX_INPUT);
    EEPROM.get(CH8_MAX_INPUTAddr, CH8_MIN_INPUT);
    EEPROM.get(delayMilsAddr, delayTimeMillis);
}

void checkMinMax(uint16_t *min, uint16_t *max, uint16_t val) {
    if (val < *min) {
  	    *min = val; 
    }
   if (val > *max) {
        *max = val; 
    }
}

void handleCalibration(int16) {
    checkMinMax(&CH1_MIN_INPUT, &CH1_MAX_INPUT, CH1_last_value);
    checkMinMax(&CH2_MIN_INPUT, &CH2_MAX_INPUT, CH2_last_value);
    checkMinMax(&CH3_MIN_INPUT, &CH3_MAX_INPUT, CH3_last_value);
    checkMinMax(&CH8_MIN_INPUT, &CH8_MAX_INPUT, CH8_last_value);
}