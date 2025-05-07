#if (defined(RUN_MAIN) && RUN_MAIN == 1) && (defined(RUN_PLANNER) && RUN_PLANNER == 0) && (defined(FOC_CONTROLLER) && FOC_CONTROLLER == 21)

#include "SimpleCAN.h"
#include "TrapPlanner/TrapezoidalPlanner.h"
#include <Helpers/VelocityRamp.h>
#include <SimpleFOC.h>

#if defined(USE_CAN)
#if defined(CAN_VERSION) && CAN_VERSION == 1

#include "CANProfile_V1.h"

#elif defined(CAN_VERSION) && CAN_VERSION == 2

#include "CANProfile_V2.h"
#include "Helpers/stringf.h"

#endif

#endif

#define PHASE_RESISTANCE 16.0f
// #define KV

#define CS       PA15   // PA15 // GPIO 7 ==> PA15 ==> SPI(AS5047)
#define SPI_MISO PC11   //
#define SPI_MOSI PC12   //
#define SPI0SCK  PC10   // PC10

// SHUNT SENSING
#define M0_IA _NC   // Only 2 current measurements B&C, A = not available.
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
#define M0_TEMP  PC5
#define AUX_TEMP PA5

#define CAN_R PB8
#define CAN_D PB9

float target_value = 0;
float lastTarget = 0.0f;

float vbusV = 0.0;
float tempDegC = 0.0;
float motorWatts = 0.0;
float tempDegCAux = 0.0;
float vbusValRaw = 0.0;
float tempValRaw = 0.0;
float tempValRawAux = 0.0;
float velSpRadsPerSec = 0.0;

unsigned long rampTimer = millis();
unsigned long printTimeMS = millis();
unsigned long printTimeMS2 = millis();
unsigned long printTimeMS3 = millis();
unsigned long positionTimeMS = millis();
int positionReportFreq = 100;

// --| Run Mode Request ----------
const byte CALIBRATE = CAN_MESSAGE::RUN_MODE::CALIBRATE;

// --| Run Mode Responses --------
const byte CALIBRATING = CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING;
const byte CALIBRATED = CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED;
const byte RUNNING = CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING;
const byte TEARDOWN = CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN;

// --| Task Status ---------------
const byte TASK_IDLE = CAN_MESSAGE::TASK_STATUS::IDLE;
const byte TASK_RUNNING = CAN_MESSAGE::TASK_STATUS::RUNNING;
const byte TASK_COMPLETE = CAN_MESSAGE::TASK_STATUS::COMPLETE;
const byte TASK_ERROR = CAN_MESSAGE::TASK_STATUS::ERROR;

byte taskStatus = TASK_IDLE;

// --| Settings ------------------
bool useVelocityRamp = true;
float calculatedVelocityLimit = 0.0f;

#include <Helpers/HelperMethods.h>
#include <array>
std::array<float, 2> velocityLimits = {5.0f, 10.0f};

// --| CANBus Init ---------------
// --|----------------------------
byte txIdentifier = 0x005;
byte rxIdentifier = 0x001;
byte filterIdentifier = 0x001 << 4;
byte rxMask = 0x7F0;

CANReceiver CANBroker;
CANHandler CANDevice(&CAN, &CANBroker, txIdentifier);
// --|-----------------------------------------------
// --|-----------------------------------------------

// , PHASE_RESISTANCE
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

SPIClass SPI_2((PC12), (PC11), (PC10));
// SPIClass SPI_2((SPI_MOSI), (SPI_MISO), (SPI0SCK));

MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, CS);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);

// --| Monitoring Variables ------
float currentMagnitude = 0.0f;
ABCurrent_s abCurrent = ABCurrent_s();
DQVoltage_s focVoltage = DQVoltage_s();
DQCurrent_s focCurrent = DQCurrent_s();
PhaseCurrent_s phaseCurrent = PhaseCurrent_s();

float motorPower = 0.0f;

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

float motorStartAngle = 0.0f;
float motorCenterAngle = 0.0f;
float motorCurrentAngle = 0.0f;
float motorPreviousAngle = 0.0f;

float motorForwardMaxAngle = 10.8f;
float motorReverseMaxAngle = -10.8f;

int calibrateDirection = 0;

// Create Calibration State Enum
enum CalibrationState { NONE, CALIBRATE_START, CALIBRATE_REVERSE, CALIBRATE_CENTER, CALIBRATE_FORWARD, CALIBRATE_COMPLETE };

// Create Calibration State Variable
CalibrationState calibrationState = NONE;
byte runMode = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;

// --| Command Handler -----------
#pragma region Command Handler //@formatter:off
Commander command = Commander(Serial);

void doMotor(char *cmd) { command.motor(&motor, cmd); }
void doTarget(char *cmd) { command.scalar(&target_value, cmd); }
void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

#pragma endregion   //@formatter:on

// --| NTC2TempV -----------------
// Convert NTC voltage to temperature
#pragma region Temperature

static float Ntc2TempV(float ADCVoltage) {
    // Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
    const float ResistorBalance = 4700.0;
    const float Beta = 3425.0F;
    const float RoomTempI = 1.0F / 298.15F;   //[K]
    const float Rt = ResistorBalance * ((3.3F / ADCVoltage) - 1);
    const float R25 = 10000.0F;

    float T = 1.0F / ((log(Rt / R25) / Beta) + RoomTempI);
    T = T - 273.15;

    return T;
}

float GetTemp(int Pin) {
    float Temp = -1;
    tempValRaw = _readADCVoltageInline(Pin, currentSense.params);
    // PrintLog("Raw temp: %.3f\n", Temp);
    Temp = Ntc2TempV(tempValRaw);
    return Temp;
}

#pragma endregion

#pragma region CANBusSetup

CanFilter filter = CanFilter(FilterType::MASK_STANDARD, filterIdentifier, rxMask);

void CANBusSetup() {
    CAN.logTo(&Serial);
    Serial.println("Starting CAN");

    CAN.filter(filter);
    CAN.begin(CAN_BITRATE);
}

#pragma endregion

void setup() {
    Serial.begin(BAUDRATE);
    SimpleFOCDebug::enable(&Serial);

    delay(2000);
    while (!Serial)
        ;

    pinMode(M0_TEMP, INPUT);    // M0_TEMP PC5
    pinMode(AUX_TEMP, INPUT);   // AUX_TEMP PA5
    analogReadResolution(12);

    command.verbose = VerboseMode::machine_readable;

    // Initialise magnetic sensor hardware
    sensor.init(&SPI_2);
    sensor.clock_speed = 500000;
    // sensor.clock_speed = 20000;

    // Link the motor to the sensor
    motor.linkSensor(&sensor);

    // Driver config
    // Power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;

    driver.pwm_frequency = 40000;
    // driver.pwm_frequency = 20000; // 20000 max STM32

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
    // motor.voltage_sensor_align = 3;
    // motor.velocity_index_search = 3;

    // Choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Set motion control loop to be used
    motor.controller = MotionControlType::angle;
    motor.target = target_value = 0;

    // Maximal voltage to be set to the motor

    // ▬▬▬▬▬▬ Velocity PID Settings ▬▬▬▬▬▬▬▬▬▬▬▬▬
    // motor.PID_velocity.P = 0.066;
    // motor.PID_velocity.I = 6.6;
    // motor.PID_velocity.D = 0.000135;

    motor.LPF_velocity.Tf = 0.01f;

    motor.PID_velocity.limit = 20;
    motor.PID_velocity.output_ramp = velocityOutputRamp;

    // ▬▬▬▬▬▬ Angle PID Settings ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
    // motor.P_angle.P = 20; // Proportional Gain (default 20) (Usually only P is necessary)
    // motor.P_angle.I = 0;  // Integral Gain
    // motor.P_angle.D = 0;  // Derivative Gain

    // acceleration control using output ramp
    // this variable is in rad/s^2 and sets the limit of acceleration
    // motor.P_angle.output_ramp = 5000; // default 1e6 rad/s^2
    motor.LPF_angle.Tf = 0.001f;

    // ▬▬▬▬▬▬ Limits_motor ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
    motor.voltage_limit = driver.voltage_limit;
    motor.current_limit = CURRENT_LIMIT;
    motor.velocity_limit = velocityLimitHigh;

    // motor.zero_electric_angle = 5.62f;        // zero_electric_angle
    motor.sensor_direction = Direction::CW;   // Cw/CCW // direction

    float offset_value = -9.25f;
    motor.sensor_offset = offset_value;
    // motor.sensor_offset = motor.shaftAngle();

    // --| CANBus Setup -----
    CANBusSetup();

    // Comment out if not needed
    motor.useMonitoring(Serial);

    // downsample the monitoring data
    motor.monitor_downsample = 100;

    // Initialize motor
    motor.init();

    motor.initFOC();

    // Add target command T
    command.add('M', doMotor, "motor");
    command.add('T', doTarget, "target angle");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doVelocity, "movement velocity");

    // ▬▬▬▬▬▬▬▬▬▬ CHECK_IF_ERROR ▬▬▬▬▬▬▬▬▬▬
    //  https://docs.simplefoc.com/cheetsheet/options_reference

    auto motor_status = motorStatusToString(motor.motor_status);
    if (motor.motor_status != 4)   // 0 - fail initFOC
    {
        auto error = stringf("--- ERROR: Motor ID: 0x%x Status: %s", txIdentifier, motor_status);
        SIMPLEFOC_DEBUG(error.c_str());
    } else {
        auto success = stringf("Success: Motor ID: 0x%x Status: %s", txIdentifier, motor_status);
        SIMPLEFOC_DEBUG(success.c_str());
    }

    motor.disable();

    _delay(500);

    CANBroker.motorReady = true;
    CANDevice.CANSendReady(rxIdentifier, txIdentifier);

    Serial.printf("Driver Stats: voltage_power_supply %f voltage_limit %f pwm_frequency %d\n", driver.voltage_power_supply, driver.voltage_limit, driver.pwm_frequency);
    Serial.printf("Motor Stats: voltage_limit %f current_limit %f velocity_limit: %f\n", motor.voltage_limit, motor.current_limit, motor.velocity_limit);
    Serial.printf("Motor Angle: %f Sensor Angle: %f Offset: %f\n", motor.shaftAngle(), sensor.getAngle(), motor.sensor_offset);

    printTimeMS = millis();
}

// --| Set/Get Functions ---------
// --|----------------------------
void setRunMode(byte mode) {
    runMode = mode;
    CANDevice.CANSendReport(rxIdentifier, txIdentifier, CAN_MESSAGE::REPORT_RUN_MODE, runMode);
    std::string runModeStr = get_message_type_string(CAN_MESSAGE::REPORT_RUN_MODE, runMode);
    Serial.printf("setRunMode: Reported Run Mode: %s for motor: %d\n", runModeStr.c_str(), txIdentifier);

    // Serial.printf("setRunMode: Reported Run Mode: %s for motor: %d\n", get_message_type_string(CAN_MESSAGE::REPORT_RUN_MODE, runMode), txIdentifier);
}

void setCalibrationState(CalibrationState state) {
    calibrationState = state;
    Serial.printf("Calibration State: %d\n", calibrationState);
}

float internalLimiter(float value, float limit) {
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    } else {
        return value;
    }
}

void taskBegin() {
    if (taskStatus != TASK_RUNNING) {
        taskStatus = TASK_RUNNING;
        CANDevice.CANSendReport(rxIdentifier, txIdentifier, CAN_MESSAGE::REPORT_TASK_STATUS, TASK_RUNNING);
    }
}

void taskComplete() {
    taskStatus = TASK_COMPLETE;
    CANDevice.CANSendReport(rxIdentifier, txIdentifier, CAN_MESSAGE::REPORT_TASK_STATUS, TASK_COMPLETE);
}

void setTargetValue(float target) {
    taskBegin();
    Serial.printf("Setting Target Value: %f\n", target);

    movementStartAngle = motor.shaftAngle();
    target_value = target;
}

// --| Set Parameter -------------
void updateParameter(const std::string &parameterName, float &parameter, const float receivedValue) {
    parameter = receivedValue;
    Serial.printf("%s: %f \n", parameterName.c_str(), parameter);
}

void setParameter() {
    if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::DRIVER_VOLTAGE_LIMIT) {
        updateParameter("Driver Voltage Limit", driver.voltage_limit, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_CURRENT_LIMIT) {
        updateParameter("Motor Current Limit", motor.current_limit, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_VELOCITY_LIMIT) {
        updateParameter("Motor Velocity Limit", motor.velocity_limit, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_P_GAIN) {
        updateParameter("Motor P Gain", motor.PID_velocity.P, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_I_GAIN) {
        updateParameter("Motor I Gain", motor.PID_velocity.I, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_D_GAIN) {
        updateParameter("Motor D Gain", motor.PID_velocity.D, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_ANGLE_P_GAIN) {
        updateParameter("Motor Angle P Gain", motor.P_angle.P, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_PID_VELOCITY_LIMIT) {
        updateParameter("Motor PID Velocity Limit", motor.PID_velocity.limit, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_PID_VELOCITY_RAMP) {
        updateParameter("Motor PID Velocity Ramp", motor.PID_velocity.output_ramp, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_P_ANGLE_RAMP) {
        updateParameter("Motor P Angle Ramp", motor.P_angle.output_ramp, CANBroker.receivedFloatVal);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_SENSOR_OFFSET) {
        if (CANBroker.receivedFloatVal == 1.234f) {
            motor.sensor_offset = sensor.getSensorAngle();
        } else {
            motor.sensor_offset = CANBroker.receivedFloatVal;
        }
        Serial.printf("Motor Sensor Offset: %f \n", motor.sensor_offset);
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_ENABLED) {
        if (CANBroker.receivedFloatVal == 1) {
            motor.enable();
        } else {
            motor.disable();
        }
    }
}

#pragma region Calibration

float calibratedTarget(float target) {
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

// --| Compare Float ----------------------------
// --|-------------------------------------------
bool compareFloat(float pos1, float pos2, float allowedError) {
    if (abs(pos1 - pos2) < allowedError) {
        return true;
    } else {
        return false;
    }
}

// --| Perform Calibration Movement -------------
// --|-------------------------------------------
void performCalibrationMovement() {
    if (calibrationState == CALIBRATE_COMPLETE) {
        // runMode = RUNNING;
        // CANDevice.CANSendReport(rxIdentifier, txIdentifier, CAN_MESSAGE::REPORT_RUN_MODE, runMode);

        // Serial.println("Calibration Complete");
    }

    if (calibrationState == CALIBRATE_START) {
        setCalibrationState(CALIBRATE_REVERSE);
    }

    if (calibrationState == CALIBRATE_CENTER) {
        setTargetValue(motorStartAngle);

        if (compareFloat(motorCurrentAngle, motorStartAngle, 0.1f)) {
            setCalibrationState(CALIBRATE_FORWARD);
        }
    }

    if (calibrationState == CALIBRATE_REVERSE) {
        if (motorReverseMaxAngle == 0.0f) {
            if (compareFloat(motorCurrentAngle, target_value, 0.1f)) {
                setTargetValue(target_value - 0.3f);
            }
        } else {
            setTargetValue(motorStartAngle);
            setCalibrationState(CALIBRATE_CENTER);
        }
    }

    if (calibrationState == CALIBRATE_FORWARD) {
        if (motorForwardMaxAngle == 0.0f) {
            if (compareFloat(motorCurrentAngle, target_value, 0.1f)) {
                setTargetValue(motorStartAngle + 2.0f);
                // setTargetValue(target_value + 0.3f);
            }
        } else {
            motorCenterAngle = (motorForwardMaxAngle + motorReverseMaxAngle) / 2;

            setTargetValue(motorCenterAngle);
            setCalibrationState(CALIBRATE_COMPLETE);
            setRunMode(CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED);

            // CANDevice.CANSendReport(rxIdentifier, txIdentifier, CAN_MESSAGE::REPORT_RUN_MODE, runMode);
            Serial.println("Calibration Complete");
        }
    }
}

void calibrateTargetHit(int endstopValue) {
    if (endstopValue == 0) {
        motorReverseMaxAngle = motorCurrentAngle;
        setTargetValue(motorReverseMaxAngle + 0.4f);
    } else if (endstopValue == 1) {
        motorForwardMaxAngle = motorCurrentAngle;
        setTargetValue(motorForwardMaxAngle - 0.4f);
    }
}

#pragma endregion

void CANBusLoop() {
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

            if (CANBroker.runMode == CAN_MESSAGE::RUN_MODE::CALIBRATE && runMode == CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING) {
                calibrateTargetHit(CANBroker.endstopValue);
            }

            if (runMode == CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING) {
                if (CANBroker.endstopValue == 0) {
                    setTargetValue(motorCurrentAngle + 0.2f);
                } else {
                    setTargetValue(motorCurrentAngle - 0.2f);
                }
            }

            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::REQUEST_READY) {
            CANDevice.CANSendReady(rxIdentifier, txIdentifier);

            CANBroker.ReceivedID = -1;
        }

        // --| Set Run Mode --------------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_RUN_MODE) {
            // --| Calibrate --------------------
            // --|-------------------------------
            if (CANBroker.runMode == CAN_MESSAGE::RUN_MODE::CALIBRATE) {
                Serial.println("Setting Run Mode: Calibrate");

                setRunMode(CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING);

                setCalibrationState(CALIBRATE_START);

                motorStartAngle = motor.shaftAngle();
                setTargetValue(motorStartAngle - 0.3f);

                motor.velocity_limit = velocityCalibrationLimit;
                motor.PID_velocity.limit = velocityCalibrationLimit;
#if defined(MOTOR_DISABLED) && MOTOR_DISABLED == 0
                motor.enable();
                Serial.println("Motor Enabled");
#endif
            }

            // --| Run --------------------------
            // --|-------------------------------
            if (CANBroker.runMode == CAN_MESSAGE::RUN_MODE::RUN) {
                setRunMode(CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING);
                // setTargetValue(calibratedTarget(CANBroker.motorTarget));
            }

            // --| Teardown ---------------------
            // --|-------------------------------
            if (CANBroker.runMode == CAN_MESSAGE::RUN_MODE::TEARDOWN) {
                Serial.println("Setting Run Mode: Teardown");

                setRunMode(CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN);
                setCalibrationState(NONE);

                setTargetValue(motorStartAngle);
            }

            CANBroker.ReceivedID = -1;
        }

        // --| Set Enable ----------------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_ENABLE) {
            if (CANBroker.motorEnabled == 1) {
#if defined(MOTOR_DISABLED) && MOTOR_DISABLED == 0
                motor.enable();
#endif
            } else {
                motor.disable();
            }

            CANBroker.ReceivedID = -1;
        }

        // --| Set Target ----------------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_TARGET) {
            setTargetValue(calibratedTarget(CANBroker.motorTarget));

            CANDevice.CANSendReport(rxIdentifier, txIdentifier, CAN_MESSAGE::REPORT_TASK_STATUS, TASK_RUNNING);

            CANBroker.ReceivedID = -1;
        }

        // --| Set Velocity Limit --------------------
        // --|----------------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_VELOCITY) {
            motor.velocity_limit = internalLimiter(CANBroker.motorVelocity, velocityLimitHigh);
            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_PARAMETER) {
            Serial.printf("CANBroker Match: %d Value: %f Destination: 0x%x:%d TxID: 0x%x\n", CANBroker.setParameterType, CANBroker.receivedFloatVal, CANBroker.destinationId, CANBroker.destinationId, txIdentifier);

            setParameter();
        }

        CANBroker.ReceivedID = -1;
    }
}

int32_t downsample = 20;   // depending on your MCU's speed, you might want a value between 5 and 50...
int32_t downsample_cnt = 0;

float previousTempC = 0.0f;
float previousTempCAux = 0.0f;

LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};
LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};

float getPower() {
    float aPow = abCurrent.alpha * motor.Ua;
    float bPow = abCurrent.beta * motor.Ub;
    return _sqrt(aPow * aPow + bPow * bPow);
}

// --| Send Position Report ------
void sendPositionReport() { CANDevice.CANReportParameter(rxIdentifier, txIdentifier, CAN_MESSAGE::PARAMETERS::CURRENT_POSITION, motorCurrentAngle); }

void loop() {
    // sensor.update();

    phaseCurrent = currentSense.getPhaseCurrents();
    sensorCurrentAngle = sensor.getAngle();

    motorPreviousAngle = motorCurrentAngle;
    motorCurrentAngle = motor.shaftAngle();

    motorElectricalAngle = motor.electricalAngle();
    currentMagnitude = currentSense.getDCCurrent(motorElectricalAngle);

    focCurrent = currentSense.getFOCCurrents(motorElectricalAngle);
    abCurrent = currentSense.getABCurrents(phaseCurrent);

    focVoltage = motor.voltage;

    currentQ = (focCurrent.q);
    currentD = (focCurrent.d);

    currentQFiltered = LPF_current_q(currentQ) * 1000;
    currentDFiltered = LPF_current_d(currentD) * 1000;

    voltageQ = focVoltage.q;
    voltageD = focVoltage.d;

    motorWatts = (driver.voltage_limit * currentMagnitude);
    motorPower = getPower();

    velSpRadsPerSec = motor.shaftVelocity();

    // --| CANBus Loop --------------------------
    CANBusLoop();

    // --| Rate Limited Print -------------------
    if (millis() - printTimeMS > MONITOR_FREQ) {
        if (!CANBroker.systemReady && !CANBroker.motorReady) {
            CANDevice.CANSendReady(rxIdentifier, txIdentifier);
        }

        // vbusValRaw = _readADCVoltageInline(A_V, currentSense.params);
        // vbusV = vbusValRaw * 10.7711;

        tempDegC = int(GetTemp(M0_TEMP) * 10) / 10.0;
        tempDegCAux = int(GetTemp(AUX_TEMP) * 100) / 100.0;

        // tempValRaw = _readADCVoltageInline(M0_TEMP, currentSense.params);
        // tempValRawAux = _readADCVoltageInline(AUX_TEMP, currentSense.params);

        //@formatter:off
        // clang-format off
#if defined(PRINT_MOTOR_VALUES) && PRINT_MOTOR_VALUES == 1
        String message =
            "T: " + String(target_value, 4) +
            " V: " + String(motor.PID_velocity.limit) +
            " A: " + String(motorCurrentAngle, 4) +
            " SA: " + String(sensorCurrentAngle) +
            " VQ: " + String(voltageQ) +
            " Tmp: " + String(tempDegC) +
            " Min: " + String(motorReverseMaxAngle) +
            " Max: " + String(motorForwardMaxAngle) +
            " R: " + String(runMode) +
            " Tsk: " + String(taskStatus) +
            " En: " + String(motor.enabled);

        Serial.println(message);
        // Serial.print('\r');
#endif
        // clang-format on
        //@formatter:on

        // If the difference between the current and previous temperature is greater than 0.1

        if (previousTempC != tempDegC && abs(tempDegC - previousTempC) > 0.5)
            CANDevice.CANSendFloat(tempDegC, txIdentifier, CAN_MESSAGE::SEND_TEMP);

        // if (previousTempCAux != tempDegCAux) CANDevice.CANSendFloat(currentMagnitude, txIdentifier, CAN_MESSAGE::SEND_CURRENT);

        previousTempC = tempDegC;
        previousTempCAux = tempDegCAux;

        printTimeMS = millis();
    }

    motor.loopFOC();

    if (useVelocityRamp && (runMode == CALIBRATED || runMode == RUNNING) && taskStatus == TASK_RUNNING) {
        if (downsample > 0 && --downsample_cnt <= 0) {
            calculatedVelocityLimit = velocityRamp(movementStartAngle, motorCurrentAngle, target_value, velocityLimits[0], velocityLimits[1]);

            motor.velocity_limit = calculatedVelocityLimit;
            motor.PID_velocity.limit = calculatedVelocityLimit;

            downsample_cnt = downsample;
        }
    }

    if (
        // CANBroker.runMode == CALIBRATE &&
        runMode == CALIBRATING) {
        performCalibrationMovement();
    }

    // Reset task status
    if (taskStatus == TASK_COMPLETE) {
        taskStatus = TASK_IDLE;

        motor.velocity_limit = velocityLimitHigh;
        motor.PID_velocity.limit = velocityLimitHigh;

        if (runMode == CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN) {
            runMode = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;
            CANDevice.CANSendReport(rxIdentifier, txIdentifier, CAN_MESSAGE::REPORT_RUN_MODE, runMode);
        }
    }

    if (runMode == CALIBRATING || runMode == CALIBRATED || runMode == RUNNING || runMode == TEARDOWN) {
        motor.move(target_value);
    }

    if ((taskStatus == TASK_RUNNING) && (motorCurrentAngle != motorCurrentAngle) && (millis() - positionTimeMS > positionReportFreq)) {
        // sendPositionReport();
        positionTimeMS = millis();
    }

    if (taskStatus == TASK_RUNNING && compareFloat(motorCurrentAngle, target_value, 0.02f)) {
        taskComplete();
    }

    // function intended to be used with serial plotter to monitor motor variables
    // significantly slowing the execution down!!!!
    // motor.monitor();

    // user communication
    // command.run();
}

#endif
