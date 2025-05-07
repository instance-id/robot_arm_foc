#if (defined(RUN_MAIN) && RUN_MAIN == 1) \
 && (defined(RUN_PLANNER) && RUN_PLANNER == 1) \
 && (defined(FOC_CONTROLLER) && FOC_CONTROLLER == 24)

#include <SimpleFOC.h>
#include "SimpleCAN.h"
#include "TrapPlanner/TrapezoidalPlanner.h"

#if defined(USE_CAN)
#if defined(CAN_VERSION) && CAN_VERSION == 1

#include "CANProfile_V1.h"

#elif defined(CAN_VERSION) && CAN_VERSION == 2

#include "CANProfile_V2.h"

#endif
#endif

#define PHASE_RESISTANCE 7.5
// #define KV

#define CS PA15       // PA15 // GPIO 7 ==> PA15 ==> SPI(AS5047)
#define SPI_MISO PC11 //
#define SPI_MOSI PC12 //
#define SPI0SCK PC10  // PC10

// SHUNT SENSING
#define M0_IA _NC // Only 2 current measurements B&C, A = not available.
#define M0_IB PC0
#define M0_IC PC1

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15

#define EN_GATE PB12

// Temperature
#define M0_TEMP PC5
#define AUX_TEMP PA5

#define CAN_R PB8
#define CAN_D PB9

#define CURRENT_LIMIT 1.5

float target_value = 0;
float lastTarget = 0.0f;

float velocityLimitLow = 1.0f;
float velocityLimitHigh = 10.0f;
float velocityOutputRamp = 100000;

float totalDelta = 0.0f;
float currentDelta = 0.0f;
float newVelocityLimit = 0.0f;
float movementStartAngle = 0.0f;

ulong rampTimer = millis();

float vbusV = 0.0;
float tempDegC = 0.0;
float tempDegCAux = 0.0;
float vbusValRaw = 0.0;
float tempValRaw = 0.0;
float tempValRawAux = 0.0;
float velSpRadsPerSec = 0.0;
unsigned long printTimeMS = 0;

const byte CALIBRATE = CAN_MESSAGE::RUN_MODE::CALIBRATE;

const byte CALIBRATING = CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING;
const byte CALIBRATED = CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED;
const byte RUNNING = CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING;
const byte IDLE = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;


// --| Create Calibration State --
float calibrationAngleTarget = 14.0f;
float motorStartAngle = 0.0f;
float motorCenterAngle = 0.0f;
float motorCurrentAngle = 0.0f;
float motorForwardMaxAngle = 0.0f;
float motorReverseMaxAngle = 0.0f;
int calibrateDirection = 0;

enum CalibrationState
{
    NONE,
    CALIBRATE_START,
    CALIBRATE_REVERSE,
    CALIBRATE_CENTER,
    CALIBRATE_FORWARD,
    CALIBRATE_COMPLETE
};

// Create setter for calibrationState to print out the state on change
CalibrationState calibrationState = NONE;
byte calibrationStateByte = 0;

CalibrationState setCalibrationState(CalibrationState state)
{
    if (state != calibrationState) {
        calibrationState = state;
        calibrationStateByte = state;
        Serial.printf("Calibration State: %d\n", state);
    }

    return state;
}

CalibrationState getCalibrationState()
{
    return calibrationState;
}

byte runMode = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;

byte setRunMode(byte mode)
{
    runMode = mode;
    Serial.printf("Run Mode: 0x%x\n", mode);
    return mode;
}

byte getRunMode()
{
    return runMode;
}

// --| Settings ------------------
bool useVelocityRamp = false;

// --| Use CANBus ----------------
// --|----------------------------
byte txIdentifier = 0x005;
byte rxIdentifier = 0x001;

CANReceiver CANBroker;
CANHandler CANDevice(&CAN, &CANBroker, txIdentifier);
// --|-----------------------------------------------
// --|-----------------------------------------------

BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, CS);

SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);

TrapezoidalPlanner trapezoidal = TrapezoidalPlanner(8.0f, 2.0f, 0.25f, 0.2f);

float currentMagnitude = 0.0f;
DQVoltage_s focVoltage = DQVoltage_s();
DQCurrent_s focCurrent = DQCurrent_s();
PhaseCurrent_s phaseCurrent = PhaseCurrent_s();

float voltageQ = 0.0f;
float voltageD = 0.0f;

float currentQ = 0.0f;
float currentD = 0.0f;
float currentQFiltered = 0.0f;
float currentDFiltered = 0.0f;

float motorShaftAngle = 0.0f;
float motorShaftVelocity = 0.0f;
float motorElectricalAngle = 0.0f;
float sensorCurrentAngle = 0.0f;


Commander command = Commander(Serial);

void doMotor(char *cmd)
{ command.motor(&motor, cmd); }

void doTarget(char *cmd)
{ command.scalar(&target_value, cmd); }

void doLimit(char *cmd)
{ command.scalar(&motor.voltage_limit, cmd); }

void doVelocity(char *cmd)
{ command.scalar(&motor.velocity_limit, cmd); }

// NTC2TempV: Convert NTC voltage to temperature
#pragma region Temperature

static float Ntc2TempV(float ADCVoltage)
{
    // Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
    const float ResistorBalance = 4700.0;
    const float Beta = 3425.0F;
    const float RoomTempI = 1.0F / 298.15F; //[K]
    const float Rt = ResistorBalance * ((3.3F / ADCVoltage) - 1);
    const float R25 = 10000.0F;

    float T = 1.0F / ((log(Rt / R25) / Beta) + RoomTempI);
    T = T - 273.15;

    return T;
}

float GetTemp(int Pin)
{
    float Temp = -1;
    tempValRaw = _readADCVoltageInline(Pin, currentSense.params);
    // PrintLog("Raw temp: %.3f\n", Temp);
    Temp = Ntc2TempV(tempValRaw);
    return Temp;
}

#pragma endregion

#pragma region CANBusSetup

void CANBusSetup()
{
    CAN.logTo(&Serial);
    Serial.println("Starting CAN");

    CanFilter filter = CanFilter(ACCEPT_ALL, rxIdentifier, rxIdentifier, FILTER_ANY_FRAME);

    CAN.filter(filter);
    CAN.begin(CAN_BITRATE);
    delay(10);
}

#pragma endregion

void setup()
{
    SimpleFOCDebug::enable();

    Serial.begin(BAUDRATE);
    //    Serial.begin(BAUDRATE);

    delay(2000);
    while (!Serial);

    pinMode(M0_TEMP, INPUT);  // M0_TEMP PC5
    pinMode(AUX_TEMP, INPUT); // AUX_TEMP PA5
    analogReadResolution(12);

    command.verbose = VerboseMode::machine_readable;

    // Initialise magnetic sensor hardware
    // sensor.clock_speed = 500000;
    sensor.init(&SPI_2);

    // Link the motor to the sensor
    motor.linkSensor(&sensor);

    // Driver config
    // Power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;

    driver.pwm_frequency = 40000;

    driver.init();

    // Link the motor and the driver
    motor.linkDriver(&driver);

    // Newly added
    currentSense.linkDriver(&driver);
    // ▬▬▬▬▬▬▬▬▬▬▬▬linkCurrentSense▬▬▬▬▬▬▬▬▬▬▬▬
    //  https://docs.simplefoc.com/low_side_current_sense
    if (currentSense.init()) {
        Serial.println("Current sense init success!");
    } else {
        Serial.println("Current sense init failed!");
        return;
    }

    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // Align sensor and start FOC
    // Newly added
    motor.voltage_sensor_align = 3;
    motor.velocity_index_search = 3;

    // Choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Set motion control loop to be used
    motor.controller = MotionControlType::velocity;
    motor.target = target_value = 0;

    // Maximal voltage to be set to the motor

    // ▬▬▬▬▬▬ Velocity PID Settings ▬▬▬▬▬▬▬▬▬▬▬▬▬
    motor.PID_velocity.P = 0.066;
    motor.PID_velocity.I = 6.6;
    motor.PID_velocity.D = 0.000135;

    motor.LPF_velocity = 0.03;
    // motor.LPF_velocity.Tf = 0.01f;

    motor.PID_velocity.output_ramp = velocityOutputRamp;
    // motor.PID_velocity.limit = 20;

    // ▬▬▬▬▬▬ Angle PID Settings ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬ 
    motor.P_angle.P = 20; // Proportional Gain (default 20) (Usually only P is necessary)
    motor.P_angle.I = 0;  // Integral Gain
    motor.P_angle.D = 0;  // Derivative Gain

    // acceleration control using output ramp
    // this variable is in rad/s^2 and sets the limit of acceleration
    // motor.P_angle.output_ramp = 10000; // default 1e6 rad/s^2

    // ▬▬▬▬▬▬ Limits_motor ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
    motor.voltage_limit = driver.voltage_limit * 0.5f;
    motor.current_limit = CURRENT_LIMIT;
    motor.velocity_limit = velocityLimitHigh;

    motor.zero_electric_angle = 2.25f;      // zero_electric_angle
    motor.sensor_direction = Direction::CW; // Cw/CCW // direction

    motor.sensor_offset = 1.15f;

    // --| CANBus Setup -----
    CANBusSetup();

    // Comment out if not needed
    motor.useMonitoring(Serial);

    // downsample the monitoring data
    motor.monitor_downsample = 100;

    // --| Trapezoidal Planner Setup --
    trapezoidal.linkMotor(motor);

    // Initialize motor
    motor.init();

    motor.initFOC();

    // Add target command T
    command.add('M', doMotor, "motor");
    command.add('T', doTarget, "target angle");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doLimit, "movement velocity");

    // ▬▬▬▬▬▬▬▬▬▬ CHECK_IF_ERROR ▬▬▬▬▬▬▬▬▬▬
    //  https://docs.simplefoc.com/cheetsheet/options_reference
    if (motor.motor_status != 4) // 0 - fail initFOC
    {
        Serial.println("ERROR:" + String(motor.motor_status));
        // return;
    }

    _delay(500);

    CANBroker.motorReady = true;
    CANDevice.CANSendReady(rxIdentifier);
    setRunMode(RUNNING);

    trapezoidal.setTarget(target_value);

    printTimeMS = millis();
}

float internalLimiter(float value, float limit)
{
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    } else {
        return value;
    }
}


void setTargetValue(float target)
{
    movementStartAngle = motor.shaftAngle();
    target_value = target;
    trapezoidal.setTarget(target_value);
}

#pragma region Calibration


float calibratedTarget(float target)
{
    // Range = from motorReverseMaxAngle to motorForwardMaxAngle
    // Example values: motorReverseMaxAngle = -10.8, motorForwardMaxAngle = 13.4
    // Center = (motorForwardMaxAngle - motorReverseMaxAngle) / 2
    // input target is an angle value from -90 to 90
    // Center is considered angle 0
    // Final targetValue cannot be less than motorReverseMaxAngle or more than

    float targetValue = 0.0f;

    if (motorForwardMaxAngle == 0.0f && motorReverseMaxAngle == 0.0f) {
        return target;
    }

    float center = (motorForwardMaxAngle + motorReverseMaxAngle) / 2;
    float range = motorForwardMaxAngle - motorReverseMaxAngle;

    targetValue = center + target * (range / 180);

    if (targetValue < motorReverseMaxAngle) {
        targetValue = motorReverseMaxAngle;
    } else if (targetValue > motorForwardMaxAngle) {
        targetValue = motorForwardMaxAngle;
    }

    String message = "Target: " + String(target) + " Center: " + String(center) + " Range: " + String(range) + " TargetValue: " + String(targetValue);
    Serial.println(message);

    return targetValue;
}

bool compareFloat(float pos1, float pos2, float allowedError)
{
    if (abs(pos1 - pos2) < allowedError) {
        return true;
    } else {
        return false;
    }
}

void performCalibrationMovement()
{
    if (getCalibrationState() == CALIBRATE_START) {
        setCalibrationState(CALIBRATE_REVERSE);
    }

    if (getCalibrationState() == CALIBRATE_CENTER) {

        if (target_value != motorCenterAngle) {
            setTargetValue(motorCenterAngle);
        }

        if (compareFloat(motorCurrentAngle, motorStartAngle, 0.1f)) {
            setCalibrationState(CALIBRATE_FORWARD);
        }
    }

    if (getCalibrationState() == CALIBRATE_REVERSE) {
        if (motorReverseMaxAngle == 0.0f) {
            if (target_value != -calibrationAngleTarget) {
                setTargetValue(-calibrationAngleTarget);
            }
        } else {
            setTargetValue(motorStartAngle);
            setCalibrationState(CALIBRATE_CENTER);
        }
    }

    if (getCalibrationState() == CALIBRATE_FORWARD) {
        if (motorForwardMaxAngle == 0.0f) {
            if (target_value != calibrationAngleTarget) {
                setTargetValue(calibrationAngleTarget);
            }
        } else {
            motorCenterAngle = (motorForwardMaxAngle + motorReverseMaxAngle) / 2;

            if (target_value != motorCenterAngle) {
                setTargetValue(motorCenterAngle);
            }

            setCalibrationState(CALIBRATE_COMPLETE);
            setRunMode(CALIBRATED);
        }
    }

    if (getCalibrationState() == CALIBRATE_COMPLETE) {
        CANDevice.CANSendInt(getRunMode(), rxIdentifier, CAN_MESSAGE::REPORT_RUN_MODE);
        setRunMode(RUNNING);
    }
}

void calibrateTargetHit(int endstopValue)
{
    if (endstopValue == 0) {
        motorReverseMaxAngle = motorCurrentAngle;
        Serial.printf("Calibrated Motor Reverse Max Angle: %f\n", motorReverseMaxAngle);
        setTargetValue(motorReverseMaxAngle + 0.4f);
    } else if (endstopValue == 1) {
        motorForwardMaxAngle = motorCurrentAngle;
        Serial.printf("Calibrated Motor Forward Max Angle: %f\n", motorForwardMaxAngle);
        setTargetValue(motorForwardMaxAngle - 0.4f);
    }
}

#pragma endregion

void CANBusLoop()
{
    if (CAN.available() > 0) {
        uint8_t packedMessage[8];
        CanMsg const rxMsg = CAN.read();

        CANDevice.HandleCanMessage(rxMsg, rxMsg.data);

        if (CANBroker.motorTarget != 0.0f && lastTarget != CANBroker.motorTarget) {
            lastTarget = CANBroker.motorTarget;
            Serial.printf("Motor Value: %f\n", CANBroker.motorTarget);
        }

        // --| Handle Endstop ------------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::ENDSTOP_PRESSED) {
            Serial.printf("Endstop Value: %d\n", CANBroker.endstopValue);

            motor.disable();

            if (CANBroker.runMode == CALIBRATE && getCalibrationState() != CALIBRATE_COMPLETE) {
                calibrateTargetHit(CANBroker.endstopValue);
                motor.enable();
            }

            if (getRunMode() == RUNNING) {
                if (CANBroker.endstopValue == 0) {
                    Serial.printf("Endstop Reverse Hit: %f\n", motorReverseMaxAngle);
                    setTargetValue(motorReverseMaxAngle + 0.2f);
                } else {
                    Serial.printf("Endstop Forward Hit: %f\n", motorForwardMaxAngle);
                    setTargetValue(motorForwardMaxAngle - 0.2f);
                }

                motor.enable();
            }

            CANBroker.ReceivedID = -1;
        }

        // --| Set Run Mode --------------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_RUN_MODE) {
            if (CANBroker.runMode == CAN_MESSAGE::RUN_MODE::CALIBRATE && getRunMode() != CALIBRATING) {
                CANDevice.CANSendReport(rxIdentifier, txIdentifier, CAN_MESSAGE::REPORT_RUN_MODE, CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING);

                setRunMode(CALIBRATING);
                setCalibrationState(CALIBRATE_START);

                motorStartAngle = motorCurrentAngle;
                setTargetValue(0);

                motor.enable();

                CANBroker.ReceivedID = -1;
            }
        }

        // --| Set Enable ----------------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_ENABLE) {
            if (CANBroker.motorEnabled == 1) {
                motor.enable();
            } else {
                motor.disable();
            }

            CANBroker.ReceivedID = -1;
        }

        // --| Set Target ----------------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_TARGET) {

            setTargetValue(calibratedTarget(CANBroker.motorTarget));

            CANBroker.ReceivedID = -1;
        }

        // --| Set Velocity Limit --------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_VELOCITY) {
            motor.velocity_limit = internalLimiter(CANBroker.motorVelocity, velocityLimitHigh);
            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_OUTPUT_RAMP) {

            motor.PID_velocity.output_ramp = CANBroker.motorVelocityRamp;
            Serial.printf("Output Ramp: %f\n", motor.PID_velocity.output_ramp);

            CANBroker.ReceivedID = -1;
        }
    }
}


void velocityRamp(float startAngle, float currentAngle, float targetAngle)
{
    // Angle is in radians
    // Start movement with motor.velocity_limit = velocityLimitLow
    // increase motor.velocity_limit toward velocityLimitHigh
    // when currentAngle is 1/4 way to targetAngle,  motor.velocity_limit should be velocityLimitHigh
    // when currentAngle is 3/4 way to targetAngle begin to decrease motor.velocity_limit toward velocityLimitLow
    // when currentAngle is at targetAngle, motor.velocity_limit should be velocityLimitLow


    // Calculate total distance from start to target.
    totalDelta = abs(targetAngle - startAngle);

    // Calculate how far we traveled from the start.
    currentDelta = abs(currentAngle - startAngle);

    // Calculate our progress as a value from 0 to 1.
    float progress = currentDelta / totalDelta;

    float rampUpStart = 0.40f;
    float fullSpeedStart = 0.5f;
    float rampDownStart = 0.60f;

    newVelocityLimit = 0.0f;

    if (progress < rampUpStart) { // starting phase: ramping up
        newVelocityLimit = velocityLimitLow + ((velocityLimitHigh - velocityLimitLow) * progress / rampUpStart);
    } else if (progress < rampDownStart) { // ending phase: starts slowing down
        newVelocityLimit = velocityLimitHigh;
    } else {  // near target ramp down phase
        newVelocityLimit = velocityLimitHigh - ((velocityLimitHigh - velocityLimitLow) * (progress - rampDownStart) / (1.0f - rampDownStart));
    }

    if (millis() - rampTimer > 400) {
        // Serial.printf("Progress: %f Current Delta: %f Total Delta: %f New Velocity Limit: %f\n", progress, currentDelta, totalDelta, newVelocityLimit);
        rampTimer = millis();
    }

    newVelocityLimit = std::min(std::max(newVelocityLimit, velocityLimitLow), velocityLimitHigh);

    motor.velocity_limit = newVelocityLimit;
    motor.PID_velocity.limit = newVelocityLimit;
}


int32_t downsample = 5;    // depending on your MCU's speed, you might want a value between 5 and 50...
int32_t downsample_cnt = 0;


float previousTempC = 0.0f;
float previousTempCAux = 0.0f;

LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};
LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};

void loop()
{
    sensor.update();

    phaseCurrent = currentSense.getPhaseCurrents();
    sensorCurrentAngle = sensor.getAngle();
    motorCurrentAngle = motor.shaftAngle();

    motorElectricalAngle = motor.electricalAngle();
    currentMagnitude = currentSense.getDCCurrent(motorElectricalAngle);

    focCurrent = currentSense.getFOCCurrents(motorElectricalAngle);
    focVoltage = motor.voltage;

    currentQ = (focCurrent.q);
    currentD = (focCurrent.d);

    currentQFiltered = LPF_current_q(currentQ) * 1000;
    currentDFiltered = LPF_current_d(currentD) * 1000;

    voltageQ = focVoltage.q;
    voltageD = focVoltage.d;


    // --| CANBus Loop --------------------------
    CANBusLoop();

    // --| Rate Limited Print -------------------
    if (millis() - printTimeMS > 500) {
        printTimeMS = millis();

        if (!CANBroker.systemReady && !CANBroker.motorReady) {
            CANDevice.CANSendReady(rxIdentifier);
        }

        tempDegC = int(GetTemp(M0_TEMP) * 10) / 10.0;
        tempDegCAux = int(GetTemp(AUX_TEMP) * 100) / 100.0;

        tempValRaw = _readADCVoltageInline(M0_TEMP, currentSense.params);
        tempValRawAux = _readADCVoltageInline(AUX_TEMP, currentSense.params);

#if defined(PRINT_MOTOR_VALUES) && PRINT_MOTOR_VALUES == 1
        String message = "Target: " + String(target_value) +
                         " Velocity: " + String(motor.PID_velocity.limit) +
                         " Angle: " + String(motorCurrentAngle) +
                         " Temperature: " + String(tempDegC) +
                         " MaxForward: " + String(motorForwardMaxAngle) +
                         " MaxReverse: " + String(motorReverseMaxAngle);

        Serial.println(message);
#endif

        if (previousTempC != tempDegC)
            CANDevice.CANSendFloat(tempDegC, txIdentifier, CAN_MESSAGE::SEND_TEMP);

        // if (previousTempCAux != tempDegCAux) CANDevice.CANSendFloat(currentMagnitude, txIdentifier, CAN_MESSAGE::SEND_CURRENT);

        previousTempC = tempDegC;
        previousTempCAux = tempDegCAux;

    }

    if (useVelocityRamp && (getRunMode() == CALIBRATED || getRunMode() == RUNNING)) {
        // velocityRamp(movementStartAngle, motorCurrentAngle, target_value);
    }

    if (CANBroker.runMode == CALIBRATE && getRunMode() == CALIBRATING) {
        performCalibrationMovement();
    }

    if (getRunMode() == CALIBRATING || getRunMode() == RUNNING) {

    }


    if (downsample > 0 && --downsample_cnt <= 0) {
        motor.target = trapezoidal.run();
        downsample_cnt = downsample;
    }

    motor.move();
    motor.loopFOC();

    // function intended to be used with serial plotter to monitor motor variables
    // significantly slowing the execution down!!!!
    // motor.monitor();

    // user communication
    command.run();
}


#endif
