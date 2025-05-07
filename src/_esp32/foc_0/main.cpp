#include <memory>
#if (defined(RUN_MAIN) && RUN_MAIN == 1) && (defined(FOC_CONTROLLER) && FOC_CONTROLLER == 20)

/*

2204-260KV

Size: below 30cm/11.8inch
Option: 2204-260KV, 2206-260KV
Motor KV: 160 RPM/V
Motor resistance (Rm): 5.6 ohms
No load current: 0.06A/7.4V
Maximum continuous current: 1.3A
Maximum power: 15W
Motor diameter: 28mm
Motor body length: 15mm
Bolt hole spacing: 12mm
Bolt thread: M2X4
Stator arms: 12N
Pole count: 14P

*/

#define M2204260KV 'M2204260KV'

#if defined(PRINT_DEBUG) && PRINT_DEBUG == 1
bool printDebug = true;
#else
bool printDebug = false;
#endif

#include "SimpleCAN.h"
#include "Wire.h"
#include <Arduino.h>
#include <Helpers/VelocityRamp.h>
#include <SimpleFOC.h>
// #include "SPI.h"

#if defined(USE_CAN) && USE_CAN == 1
#if defined(CAN_VERSION) && CAN_VERSION == 1

#include "CANProfile_V1.h"

#elif defined(CAN_VERSION) && CAN_VERSION == 2

#include "CANProfile_V2.h"
#include "Helpers/HelperMethods.h"
#include "Helpers/stringf.h"

// ESP32-S3 TX/RX Pins
#define TX_PIN      43
#define RX_PIN      44
#define PIN_CAN0_TX GPIO_NUM_43
#define PIN_CAN0_RX GPIO_NUM_44

ulong logPrintTime = 0;
ulong printTimeMS = millis();
ulong endstopTimeMS = millis();
ulong positionTimeMS = millis();

int positionReportFreq = 300;

// --| CANBus Identifiers --------
// --|----------------------------
byte txIdentifier = 0x004;
byte txIdentifier2 = 0x006;
byte rxIdentifier = 0x001;
byte filterIdentifier = 0x001 << 4;
// byte rxMask = 0x00F;
byte rxMask = 0x7F0;

byte motorIDLookupTable[2] = {0x004, 0x006};
typedef std::map<int, int> MotorIndexMap;

MotorIndexMap motorIndexMap = {
    {0x004, 0},
    {0x006, 1},
};

// --| CANBus Initialization -----
// --|----------------------------
CANReceiver canBroker;
CANHandler CANDevice(&CAN, &canBroker, txIdentifier, txIdentifier2);

#endif
#endif

// --| Motor Pins ----------------
#define PIN_IN1    4
#define PIN_IN2    5
#define PIN_IN3    6
#define PIN_ENABLE 7

#define PIN2_IN1    11
#define PIN2_IN2    12
#define PIN2_IN3    13
#define PIN2_ENABLE 17

// --| Current Sense Pins --------
#define CURRENT_SENSE_0_0 1
#define CURRENT_SENSE_0_1 2
#define CURRENT_SENSE_1_0 15
#define CURRENT_SENSE_1_1 16

// --| Motor Constants ------------
#define PHASE_RESISTANCE 7.5

// --| Encoder_1 Configuration ----------
// --|-----------------------------------
#if defined(ENCODER_TYPE) && ENCODER_TYPE == 0
#include "SimpleFOCDrivers.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#define SSI_SDA_PIN  13
#define SSI_CLK_PIN  12
#define SSI_CS_PIN   16
#define SSI_MODE_PIN 15

MagneticSensorMT6701SSI sensor1(SSI_CS_PIN);
#elif defined(ENCODER_TYPE) && ENCODER_TYPE == 1
#include "MT6701_I2C.h"
#define I2C_SCL_PIN 9
#define I2C_SDA_PIN 8

TwoWire I2Cone = TwoWire(0);

MT6701_I2CConfig_s mt6701_config = {.chip_address = 0b0000110, .bit_resolution = 14, .angle_register = 0x03, .data_start_bit = 8};
MT6701_Serial_I2C sensor_0 = MT6701_Serial_I2C(mt6701_config);
#endif

// --| Encoder_2 Configuration ----------
// --|-----------------------------------
#if defined(ENCODER2_TYPE) && ENCODER2_TYPE == 0

#include "SimpleFOCDrivers.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#define SSI_SDA_PIN  45   // MISO
#define SSI_CLK_PIN  37   // CLK/SCK
#define SSI_CS_PIN   39   // CS
#define SSI_MODE_PIN 47   // SSI Select

MagneticSensorMT6701SSI sensor_1(SSI_CS_PIN);
SPIClass SPI1(HSPI);
#endif

// --| Motor Constructors ------------------
// --|--------------------------------------
BLDCMotor motor_0 = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
BLDCDriver3PWM driver_0 = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);

BLDCMotor motor_1 = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(PIN2_IN1, PIN2_IN2, PIN2_IN3, PIN2_ENABLE);

DQCurrent_s motor_0_current;
DQCurrent_s motor_1_current;

// --| Current Sense Setup -----------------
// --|--------------------------------------
InlineCurrentSense current_sense_0 = InlineCurrentSense(0.01f, 50.0f, CURRENT_SENSE_0_0, CURRENT_SENSE_0_1);
InlineCurrentSense current_sense_1 = InlineCurrentSense(0.01f, 50.0f, CURRENT_SENSE_1_0, CURRENT_SENSE_1_1);

// --| Global Variables ----------
float allowedVariation = 0.5f;
float base_min_pos = BASE_MIN_POS;
float base_max_pos = BASE_MAX_POS;
float arm_1_min_pos = ARM_1_MIN_POS;
float arm_1_max_pos = ARM_1_MAX_POS;

enum RestArmState {
    REST_ARM_DOWN = 0,
    REST_ARM_UP = 1,
};

RestArmState restArmState = REST_ARM_DOWN;

// --| Global Variables ----------
// --| Run Mode Request ----------
const byte CALIBRATE = CAN_MESSAGE::RUN_MODE::CALIBRATE;

// --| Run Mode Responses --------
const byte CALIBRATING = CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING;
const byte CALIBRATED = CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED;
const byte RUNNING = CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING;
const byte TEARDOWN = CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN;
const byte IDLE = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;

// --| Task Status ---------------
const byte TASK_IDLE = CAN_MESSAGE::TASK_STATUS::IDLE;
const byte TASK_RUNNING = CAN_MESSAGE::TASK_STATUS::RUNNING;
const byte TASK_COMPLETE = CAN_MESSAGE::TASK_STATUS::COMPLETE;
const byte TASK_ERROR = CAN_MESSAGE::TASK_STATUS::ERROR;

std::vector<Sensor *> sensors;
std::vector<BLDCMotor *> motors;
std::vector<BLDCDriver *> drivers;
std::vector<InlineCurrentSense *> currentSensors;
std::vector<DQCurrent_s> currentValues;

std::array useVelocityRamp = {false, true};
std::array sendPositionReports = {false, false};
std::array sensorCurrentAngles = {0.0f, 0.0f};
std::array motorCurrentAngles = {0.0f, 0.0f};
std::array motorPreviousAngles = {0.0f, 0.0f};
std::array motorCenterAngles = {0.0f, 0.0f};
std::array motorStartAngles = {0.0f, 0.0f};
std::array movementStartAngles = {0.0f, 0.0f};
std::array targetValues = {0.0f, 0.0f};
std::array lastTargets = {0.0f, 0.0f};
std::array motorMinPos = {base_min_pos, arm_1_min_pos};
std::array motorMaxPos = {base_max_pos, arm_1_max_pos};
std::array runModes = {IDLE, IDLE};
std::array<int, MOTOR_COUNT> taskStatuses = {TASK_IDLE, TASK_IDLE};
std::array enabledMotors = {0.0f, 0.0f};
std::array reductionFactors = {static_cast<float>(MOTOR_0_REDUCTION), static_cast<float>(MOTOR_1_REDUCTION)};

std::array motorCurrentMagnitudes = {0.0f, 0.0f};
std::array motorElectricAngles = {0.0f, 0.0f};

std::array newVelocityLimits = {0.0f, 0.0f};
std::array velocityLimitsLow = {3.0f, 3.0f};
std::array velocityLimitsHigh = {10.0f, 35.0f};

int32_t downsample = 20;   // depending on your MCU's speed, you might want a value between 5 and 50...
int32_t downsample_cnt = 0;

std::array downsampleCounts = {0, 0};

// --| Set/Get Functions --------------
// --|---------------------------------
#pragma region Set/Get Functions

// --| Task Status ---------------
void taskBegin(const int motorID) {
    taskStatuses[motorIndexMap[motorID]] = TASK_RUNNING;
    CANDevice.CANSendReport(rxIdentifier, motorID, CAN_MESSAGE::REPORT_TASK_STATUS, taskStatuses[motorIndexMap[motorID]]);
}

void taskComplete(const int motorID) {
    taskStatuses[motorIndexMap[motorID]] = TASK_COMPLETE;
    CANDevice.CANSendReport(rxIdentifier, motorID, CAN_MESSAGE::REPORT_TASK_STATUS, TASK_COMPLETE);
    CANDevice.CANStoreParameter(rxIdentifier, motorID, CAN_MESSAGE::PARAMETERS::CURRENT_POSITION, motorCurrentAngles[motorIndexMap[motorID]]);
}

// --| Motor Enabled ---------------
void setMotorEnabled(const float enabled, const int motorID) {
    if (enabled == 1) {
        motors[motorIndexMap[motorID]]->enable();
    } else {
        motors[motorIndexMap[motorID]]->disable();
    }
}

// --| Target Value ---------------
void setTargetValue(const float target, const int motorID) {
    taskBegin(motorID);
    Serial.printf("Setting Target Value: %f\n", target);

    movementStartAngles[motorIndexMap[motorID]] = motors[motorIndexMap[motorID]]->shaftAngle();
    targetValues[motorIndexMap[motorID]] = target;
}

float getTargetValue(const int motorID) { return targetValues[motorIndexMap[motorID]]; }

void setLastTargetValue(const float target, const int motorID) { lastTargets[motorIndexMap[motorID]] = target; }

// --| Task Status ---------------
void setTaskStatus(const byte taskStatus, const int motorID) { taskStatuses[motorIndexMap[motorID]] = taskStatus; }

byte getTaskStatus(const int motorID) { return taskStatuses[motorIndexMap[motorID]]; }

// --| Run Mode ------------------
void setRunMode(const byte runMode, const int motorID) { runModes[motorIndexMap[motorID]] = runMode; }
byte getRunMode(const int motorID) { return runModes[motorIndexMap[motorID]]; }

// --| Start Angle ---------------
void setStartAngle(const float startAngle, const int motorID) { motorStartAngles[motorIndexMap[motorID]] = startAngle; }
float getStartAngle(const int motorID) { return motorStartAngles[motorIndexMap[motorID]]; }
float getMotorReduction(const int motorID) { return reductionFactors[motorIndexMap[motorID]]; }

#pragma endregion

// Instantiate the commander
Commander command = Commander(Serial);

void doMotor(char *cmd) { command.motor(&motor_0, cmd); }
void doTarget(char *cmd) { command.scalar(&targetValues[1], cmd); }
void doEnable(char *cmd) { command.scalar(&enabledMotors[1], cmd); }
void doLimit(char *cmd) { command.scalar(&motor_0.voltage_limit, cmd); }
void doVelocity(char *cmd) { command.scalar(&motor_0.velocity_limit, cmd); }

// --| Setup Functions -----------
// --|----------------------------

// --| Pin Setup ------------
void pinSetup() {
    pinMode(RX_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);
    // vQueueAddToRegistry();
}

// --| CANBus Init ----------
CanFilter filter = CanFilter(FilterType::MASK_STANDARD, filterIdentifier, rxMask);
void canBusInit() {
    // Create filter that would allow messages to be received from 0x001
    CAN.logTo(&Serial);
    CAN.filter(filter);
    CAN.begin(CAN_BITRATE);
}

void currentSenseInit() {
    pinMode(CURRENT_SENSE_0_0, INPUT);
    pinMode(CURRENT_SENSE_0_1, INPUT);
    pinMode(CURRENT_SENSE_1_0, INPUT);
    pinMode(CURRENT_SENSE_1_1, INPUT);
}

void sendMotorReady() {
    for (size_t i = 0; i < sizeof(motorIDLookupTable); ++i) {
        taskStatuses[i] = TASK_IDLE;
        CANDevice.CANSendReady(rxIdentifier, motorIDLookupTable[i]);
    }
}

// --| Motor_0_Setup -------------
// --|----------------------------
#pragma region Motor_0_Setup
bool motor_0_Setup() {
    // --| Encoder_1 Init ----------
#if defined(ENCODER_TYPE) && ENCODER_TYPE == 0
    SPI.setFrequency(100000);
    sensor1.init(&SPI);
#elif defined(ENCODER_TYPE) && ENCODER_TYPE == 1
    if (I2Cone.setPins(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("Sensor_0 Pins Set!");
        sensor_0.init(&I2Cone);
        I2Cone.setClock(400000);
    } else {
        Serial.println("Sensor_0 Pins Failed!");
        return false;
    }
#endif

    // Link the motor to the sensor
    motor_0.linkSensor(&sensor_0);

    // --| Motor_1 Driver Init ----------
    driver_0.voltage_power_supply = POWER_SUPPLY;
    driver_0.voltage_limit = VOLTAGE_LIMIT;

    if (driver_0.init()) {
        Serial.println("Driver_0 init success!");
    } else {
        Serial.println("Driver_0 init failed!");
        return false;
    }

    current_sense_0.linkDriver(&driver_0);

    // Link the motor and the driver
    motor_0.linkDriver(&driver_0);

    // Choose FOC modulation (optional)
    motor_0.foc_modulation = SpaceVectorPWM;

    // Set motion control loop to be used
    motor_0.controller = angle;

    // Maximal voltage to be set to the motor
    motor_0.voltage_limit = driver_0.voltage_limit * 0.5f;
    motor_0.current_limit = CURRENT_LIMIT - 0.6f;

    // Controller configuration
    // Default parameters in defaults.h
    // Velocity PI controller parameters
    // --| Motor_1 PID Config ----------
    motor_0.PID_velocity.P = 0.066;
    motor_0.PID_velocity.I = 6.6;
    motor_0.PID_velocity.D = 0.000135;

    motor_0.PID_velocity.output_ramp = 100;

    // Velocity low pass filtering time constant
    // the lower the less filtered
    motor_0.LPF_velocity.Tf = 0.01f;

    // Angle P controller
    motor_0.P_angle.P = 20;
    // motor.LPF_angle.Tf = 0.001f;
    motor_0.P_angle.output_ramp = 5000;

    // Maximal velocity of the position control
    motor_0.velocity_limit = 5;

    // Maximal velocity of the position control
    motor_0.velocity_limit = 5;

    motor_0.zero_electric_angle = 0.87;
    motor_0.sensor_direction = CW;

    motor_0.sensor_offset = motor_0.shaftAngle();

    // Comment out if not needed
    // motor_0.useMonitoring(Serial);

    // Initialize motor
    motor_0.init();

    // Current Sense Init
    if (current_sense_0.init()) {
        Serial.println("Current Sense 0 init success!");
        current_sense_0.gain_b *= -1;
        motor_0.voltage_sensor_align = 1;

        motor_0.linkCurrentSense(&current_sense_0);
        current_sense_0.skip_align = true;
    } else {
        Serial.println("Current Sense 0 init failed!");
        return false;
    }

    // Align sensor and start FOC
    motor_0.initFOC();

    motor_0.monitor_downsample = 1000;
    return true;
}
#pragma endregion

// --| Motor_1_Setup -------------
// --|----------------------------
#pragma region Motor_1_Setup
bool motor_1_Setup() {
    // --| Motor_2 Config ---------------
    // --|-------------------------------

    // --| Encoder_2 Init ----------
#if defined(ENCODER2_TYPE) && ENCODER2_TYPE == 0
    SPI1.setFrequency(100000);
    SPI1.begin(SSI_CLK_PIN, SSI_SDA_PIN);

    sensor_1.init(&SPI1);
#endif
    motor_1.linkSensor(&sensor_1);

    // --| Motor_2 Driver Init ----------
    driver_1.voltage_power_supply = POWER_SUPPLY;
    driver_1.voltage_limit = VOLTAGE_LIMIT;

    if (driver_1.init()) {
        Serial.println("Driver_1 init success!");
    } else {
        Serial.println("Driver_1 init failed!");
        return false;
    }

    current_sense_1.linkDriver(&driver_1);

    motor_1.linkDriver(&driver_1);

    motor_1.foc_modulation = SpaceVectorPWM;
    motor_1.controller = angle;

    motor_1.voltage_limit = driver_1.voltage_limit * 0.8f;
    motor_1.current_limit = CURRENT_LIMIT - 0.5f;

    // --| Motor_2 PID Config ----------
    motor_1.PID_velocity.P = 0.066;
    motor_1.PID_velocity.I = 6.6;
    motor_1.PID_velocity.D = 0.000135;

    motor_1.PID_velocity.output_ramp = 300;

    // Velocity low pass filtering time constant
    // the lower the less filtered
    motor_1.LPF_velocity.Tf = 0.01f;

    // Angle P controller
    motor_1.P_angle.P = 20;
    // motor.LPF_angle.Tf = 0.001f;
    // motor_1.P_angle.output_ramp = 5000;

    // Maximal velocity of the position control
    motor_1.PID_velocity.limit = velocityLimitsHigh[1];
    motor_1.velocity_limit = velocityLimitsHigh[1];

    motor_1.zero_electric_angle = 3.50;
    motor_1.sensor_direction = CW;

    float sensorAngle = sensor_1.getSensorAngle();
    float sensor_offset_value = ARM_1_OFFSET;

    SIMPLEFOC_DEBUG(stringf("Sensor GetAngle: %f", sensorAngle).c_str());

    if (sensorAngle > 0.0f) {
        sensor_offset_value += sensorAngle;
    }

    motor_1.sensor_offset = sensor_offset_value;

    // Comment out if not needed
    // motor_1.useMonitoring(Serial);

    // Initialize motor
    motor_1.init();

    // Current Sense Init
    if (current_sense_1.init()) {
        Serial.println("Current Sense 0 init success!");
        current_sense_1.gain_b *= -1;
        motor_1.voltage_sensor_align = 1;
        motor_1.linkCurrentSense(&current_sense_1);

        current_sense_1.skip_align = true;
    } else {
        Serial.println("Current Sense 0 init failed!");
        return false;
    }

    // Align sensor and start FOC
    motor_1.initFOC();

    motor_1.monitor_downsample = 1000;
    return true;
}
#pragma endregion

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    delay(4000);

    SimpleFOCDebug::enable(&Serial);

    canBusInit();

    // currentSenseInit();

    if (!motor_0_Setup())
        return;
    if (!motor_1_Setup())
        return;

    // Add target command T
    command.add('M', doMotor, "motor");
    command.add('E', doEnable, "enable");
    command.add('T', doTarget, "target angle");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doLimit, "movement velocity");

    sensors.push_back(&sensor_0);
    sensors.push_back(&sensor_1);

    drivers.push_back(&driver_0);
    drivers.push_back(&driver_1);

    motors.push_back(&motor_0);
    motors.push_back(&motor_1);

    currentSensors.push_back(&current_sense_0);
    currentSensors.push_back(&current_sense_1);

    currentValues.push_back(motor_0_current);
    currentValues.push_back(motor_1_current);

    for (int i = 0; i < motors.size(); ++i) {
        auto motor_status = motorStatusToString(motors[i]->motor_status);

        // ▬▬▬▬▬▬▬▬▬▬ CHECK_IF_ERROR ▬▬▬▬▬▬▬▬▬▬
        //  https://docs.simplefoc.com/cheetsheet/options_reference
        if (motors[i]->motor_status != 4)   // 0 - fail initFOC
        {
            auto error = stringf("--- ERROR: Motor %d: ID: 0x%x Status: %s", i, motorIDLookupTable[i], motor_status);
            SIMPLEFOC_DEBUG(error.c_str());
        } else {
            auto success = stringf("Success: Motor %d: ID: 0x%x Status: %s", i, motorIDLookupTable[i], motor_status);
            SIMPLEFOC_DEBUG(success.c_str());
        }

        Serial.printf("Driver_%d Stats: voltage_power_supply %f voltage_limit %f pwm_frequency %ld\n", i, drivers[i]->voltage_power_supply, drivers[i]->voltage_limit, drivers[i]->pwm_frequency);
        Serial.printf("Motor_%d Stats: voltage_limit %f current_limit %f velocity_limit: %f\n", i, motors[i]->voltage_limit, motors[i]->current_limit, motors[i]->velocity_limit);
        Serial.printf("Motor_%d Angle: %f Sensor_0 Angle: %f Offset: %f\n", i, motors[i]->shaftAngle(), sensors[i]->getAngle(), motors[i]->sensor_offset);

        motors[i]->disable();
    }

    _delay(500);

    canBroker.motorReady = true;

    sendMotorReady();

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target angle using serial terminal:"));
}

// --| Compare Float -----------------------
bool compareFloat(const float pos1, const float pos2, const float allowedError) { return abs(pos1 - pos2) < allowedError; }

// --| Extract Float -----------------------
void extractFloat(float receivedValue, float &value1, float &value2) {
    value1 = static_cast<float>(static_cast<int>(receivedValue));
    value2 = static_cast<float>(static_cast<int>((receivedValue - value1) * 100));
}

// --| Update Parameter --------------------
void updateParameter(const std::string &parameterName, float &parameter, const float receivedValue) {
    parameter = receivedValue;
    Serial.printf("%s: %f \n", parameterName.c_str(), parameter);
}

// --| Process Common Parameters -----------
void processCommonParameters(byte paramType, BLDCDriver *driver, BLDCMotor *motor, Sensor *sensor, int motorIndex) {
    if (!driver || !motor || !sensor) {
        Serial.println("Invalid driver or motor or sensor pointer");
        return;
    }

    switch (paramType) {   //@formatter:off //clang-format off
    case CAN_MESSAGE::PARAMETERS::DRIVER_VOLTAGE_LIMIT: {
        updateParameter("Driver Voltage Limit", driver->voltage_limit, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_CURRENT_LIMIT: {
        updateParameter("Motor Current Limit", motor->current_limit, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_VELOCITY_LIMIT: {
        updateParameter("Motor Velocity Limit", motor->velocity_limit, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_P_GAIN: {
        updateParameter("Motor P Gain", motor->PID_velocity.P, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_I_GAIN: {
        updateParameter("Motor I Gain", motor->PID_velocity.I, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_D_GAIN: {
        updateParameter("Motor D Gain", motor->PID_velocity.D, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_ANGLE_P_GAIN: {
        updateParameter("Motor Angle P Gain", motor->P_angle.P, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_PID_VELOCITY_LIMIT: {
        updateParameter("Motor PID Velocity Limit", motor->PID_velocity.limit, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_PID_VELOCITY_RAMP: {
        updateParameter("Motor PID Velocity Ramp", motor->PID_velocity.output_ramp, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_P_ANGLE_RAMP: {
        updateParameter("Motor P Angle Ramp", motor->P_angle.output_ramp, canBroker.receivedFloatVal);
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_SENSOR_OFFSET:
        if (canBroker.receivedFloatVal == 1.234f) {
            motor->sensor_offset = sensor->getAngle();
        } else {
            motor->sensor_offset = canBroker.receivedFloatVal;
        }
        Serial.printf("Motor Sensor Offset: %f \n", motor->sensor_offset);
        break;
    case CAN_MESSAGE::PARAMETERS::MOTOR_ENABLED: {
        if (canBroker.receivedFloatVal == 1) {
            Serial.printf("Enabling Motor 0x%x\n", motorIndex);
            motor->enable();
        } else {
            Serial.printf("Disabling Motor 0x%x\n", motorIndex);
            motor->disable();
        }
        break;
    }
    case CAN_MESSAGE::PARAMETERS::MOTOR_MIN_MAX_VELOCITY: {
        extractFloat(canBroker.receivedFloatVal, velocityLimitsLow[motorIndex], velocityLimitsHigh[motorIndex]);
        Serial.printf("Motor Min Velocity: %f Max Velocity: %f \n", velocityLimitsLow[motorIndex], velocityLimitsHigh[motorIndex]);
        break;
    }
    default:
        break;
    }   //@formatter:on //clang-format on
}

// --| Set Parameter -----------------------
void setParameter(const int motorID) {
    const int motorIndex = motorIndexMap[motorID];
    if (motorIndex < 0 || motorIndex >= std::size(motorIDLookupTable)) {
        Serial.printf("Invalid motor ID: %d\n", motorID);
        return;
    }

    BLDCDriver *driver = drivers[motorIndex];
    BLDCMotor *motor = motors[motorIndex];
    Sensor *sensor = sensors[motorIndex];

    processCommonParameters(canBroker.setParameterType, driver, motor, sensor, motorIndex);
}

float ensureAngleRange(const float angle, int motorID) {
    const float min = motorIndexMap[motorID] == 0 ? base_min_pos : arm_1_min_pos;
    const float max = motorIndexMap[motorID] == 0 ? base_max_pos : arm_1_max_pos;

    if (angle <= min) {
        return min;
    }
    if (angle >= max) {
        return max;
    }

    return angle;
}

// --| CANBus Loop ---------------
// --|----------------------------
void CANBusLoop() {
    if (CAN.available() > 0) {
        uint8_t packedMessage[8];
        CanMsg const rxMsg = CAN.read();

        CANDevice.HandleCanMessage(rxMsg, rxMsg.data);

        if (canBroker.ReceivedID == 0 || canBroker.ReceivedID == -1) {
            return;
        }

        if (canBroker.motorTarget != 0.0f && lastTargets[0] != canBroker.motorTarget) {
            lastTargets[0] = canBroker.motorTarget;
            Serial.printf("Motor Value: %f\n", canBroker.motorTarget);
        }

        if (canBroker.ReceivedID == CAN_MESSAGE::REQUEST_READY) {
            sendMotorReady();
        }

        // --| Set Target ----------------------------
        // --|----------------------------------------
        if (canBroker.ReceivedID == CAN_MESSAGE::SET_TARGET) {
            int motorID = canBroker.motorID;

            float target = ensureAngleRange((canBroker.motorTarget * static_cast<float>(DEG_TO_RAD)) * getMotorReduction(motorID), motorID);

            Serial.printf("Setting Target Value: %f\n", target);
            setTargetValue(target, motorID);

            canBroker.ReceivedID = -1;
        }

        // --| Set Run Mode --------------------------
        // --|----------------------------------------
        if (canBroker.ReceivedID == CAN_MESSAGE::SET_RUN_MODE) {
            // --| Calibrate --------------------
            // --|-------------------------------
            if (canBroker.runMode == CAN_MESSAGE::RUN_MODE::CALIBRATE) {
                int motorID = canBroker.motorID;

                Serial.println("Setting Run Mode: Calibrate");

                setRunMode(CALIBRATING, motorID);

                CANDevice.CANSendReport(rxIdentifier, motorID, CAN_MESSAGE::REPORT_RUN_MODE, getRunMode(motorID));
                Serial.println("Reported Run Mode: Calibrating for Motor: " + String(motorID));

                Serial.println("Run Mode: Calibrating");
                Serial.println("Calibration State: Start");

                setStartAngle(motors[motorIndexMap[motorID]]->shaftAngle(), motorID);
                motors[motorIndexMap[motorID]]->velocity_limit = 20.0f;

                setTargetValue(getStartAngle(motorID), motorID);

#if defined(MOTOR_DISABLED) && MOTOR_DISABLED == 0
                setMotorEnabled(1, motorID);
                Serial.println("Motor Enabled");
#endif

                setRunMode(CALIBRATED, motorID);
                Serial.printf("Run Mode: Calibrated for Motor: %d\n", motorID);
                CANDevice.CANSendReport(rxIdentifier, motorID, CAN_MESSAGE::REPORT_RUN_MODE, getRunMode(motorID));
                Serial.printf("Reported Run Mode: Calibrated for Motor: %d\n", motorID);
            }

            if (canBroker.runMode == CAN_MESSAGE::RUN_MODE::RUN) {
                int motorID = canBroker.motorID;
                Serial.println("Setting Run Mode: Run");

                setMotorEnabled(1, motorID);
                setRunMode(RUNNING, motorID);

                CANDevice.CANSendReport(rxIdentifier, motorID, CAN_MESSAGE::REPORT_RUN_MODE, getRunMode(motorID));

                Serial.println("Reported Run Mode: Running");
            }

            // --| Teardown ---------------------
            // --|-------------------------------
            if (canBroker.runMode == CAN_MESSAGE::RUN_MODE::TEARDOWN) {
                int motorID = canBroker.motorID;

                Serial.println("Setting Run Mode: Teardown");
                setRunMode(TEARDOWN, motorID);
                CANDevice.CANSendReport(rxIdentifier, motorID, CAN_MESSAGE::REPORT_RUN_MODE, getRunMode(motorID));

                // if (motorID == motorIDLookupTable[1]) {
                //     motors[motorIndexMap[motorID]]->velocity_limit = .0f;
                // }
                setTargetValue(getStartAngle(motorID), motorID);

                Serial.println("Reported Run Mode: Teardown");
            }

            canBroker.ReceivedID = -1;
        }

        if (canBroker.ReceivedID == CAN_MESSAGE::SET_PARAMETER) {
            Serial.printf("SET_PARAMETER Match: %d Value: %f Destination: 0x%x:%d TxID: 0x%x\n", canBroker.setParameterType, canBroker.receivedFloatVal, canBroker.motorID, canBroker.motorID, txIdentifier);

            int motorID = canBroker.motorID;
            setParameter(motorID);
        }

        canBroker.ReceivedID = -1;
    }
}

// --| Loop Functions -----------------
// --|---------------------------------

// --| Send Position Report ------
void sendPositionReport(int motor, float angle) { CANDevice.CANReportParameter(rxIdentifier, motor, CAN_MESSAGE::PARAMETERS::CURRENT_POSITION, angle); }

// --| Serial Motor Enable -------
void SerialMotorEnable() {
    if (!motors[1]->enabled && enabledMotors[1] > 0.0f) {
        motors[1]->enable();
        runModes[1] = RUNNING;
    } else if (enabledMotors[1] == -1.0f) {
        motors[1]->disable();
        runModes[1] = IDLE;
    }
}

// --| Motor Range Check ---------
void motorRangeCheck() {
    for (size_t i = 0; i < sizeof(motorIDLookupTable); ++i) {
        if (motors[i]->enabled && motorCurrentAngles[i] > motorMaxPos[i] + allowedVariation || motorCurrentAngles[i] < motorMinPos[i] - allowedVariation) {
            motors[i]->disable();

            Serial.printf("Motor 0x%x Disabled: Out of Range\n", motorIDLookupTable[i]);
        }
    }
}

// --| Update Sensors and Angles -
void updateSensorsAndAngles() {
    for (size_t i = 0; i < sizeof(motorIDLookupTable); ++i) {
        // sensors[i]->update();

        motorCurrentMagnitudes[i] = currentSensors[i]->getDCCurrent();
        motorElectricAngles[i] = motors[i]->electricalAngle();
        currentValues[i] = currentSensors[i]->getFOCCurrents(motorElectricAngles[i]);

        motorPreviousAngles[i] = motorCurrentAngles[i];
        sensorCurrentAngles[i] = sensors[i]->getAngle();
        motorCurrentAngles[i] = motors[i]->shaftAngle();
    }
}

// --| Check Task Status and Move
void checkTaskStatusAndMove() {
    for (size_t i = 0; i < sizeof(motorIDLookupTable); ++i) {
        int motorID = motorIDLookupTable[i];

        if (taskStatuses[i] == TASK_COMPLETE) {
            taskStatuses[i] = TASK_IDLE;
            if (runModes[i] == CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN) {
                runModes[i] = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;
                CANDevice.CANSendReport(rxIdentifier, motorID, CAN_MESSAGE::REPORT_RUN_MODE, runModes[i]);
            }
        }

        if (useVelocityRamp[i] && (runModes[i] == CALIBRATED || runModes[i] == RUNNING || runModes[i] == TEARDOWN)
            // && taskStatuses[i] == TASK_RUNNING
        ) {
            if (downsample > 0 && --downsampleCounts[i] <= 0) {
                newVelocityLimits[i] = velocityRamp(movementStartAngles[i], motorCurrentAngles[i], targetValues[i], velocityLimitsLow[i], velocityLimitsHigh[i]);

                motors[i]->velocity_limit = newVelocityLimits[i];
                motors[i]->PID_velocity.limit = newVelocityLimits[i];

                downsampleCounts[i] = downsample;
            }
        }

        if (runModes[i] == CALIBRATING || runModes[i] == RUNNING || runModes[i] == TEARDOWN) {
            motors[i]->move(targetValues[i]);
        }

        if (sendPositionReports[i] && (taskStatuses[i] == TASK_RUNNING) && (motorPreviousAngles[i] != motorCurrentAngles[i]) && (millis() - positionTimeMS > positionReportFreq)) {
            sendPositionReport(motorID, motorCurrentAngles[i]);
            positionTimeMS = millis();
        }

        if (taskStatuses[i] == TASK_RUNNING && compareFloat(motorCurrentAngles[i], targetValues[i], 0.02f)) {
            taskComplete(motorID);
        }
    }
}

// --| Print Debug Info ----------
void printDebugInfo() {
    float freq = (taskStatuses[0] == TASK_RUNNING || taskStatuses[1] == TASK_RUNNING) ? 0.2f : 1.0f;

    if (printDebug && millis() - printTimeMS > MONITOR_FREQ * freq) {
        String message;

        for (size_t i = 0; i < sizeof(motorIDLookupTable); ++i) {
            message += "T:" + String(targetValues[i], 2) + " Q:" + String(motors[i]->voltage.q, 2) + " MV:" + String(motors[i]->shaftVelocity(), 2) + " V:" + String(motors[i]->PID_velocity.limit) + " A:" + String(motorCurrentAngles[i], 2) + " SA:" + String(sensorCurrentAngles[i]) +
                       " R:" + String(runModes[i]) + " Tk:" + String(taskStatuses[i]) + " En: " + String(motors[i]->enabled);

            if (i < sizeof(motorIDLookupTable) - 1) {
                message += " | ";
            }
        }

        String currentMessage;

        for (size_t i = 0; i < sizeof(motorIDLookupTable); ++i) {
            currentMessage += "C:" + String(currentValues[i].q, 2) + " M:" + String(motorCurrentMagnitudes[i], 2) + " E:" + String(motorElectricAngles[i], 2);

            if (i < sizeof(motorIDLookupTable) - 1) {
                currentMessage += " | ";
            }
        }

        Serial.println(message);
        // Serial.print('\r');
        // Serial.print(currentMessage);
        // Serial.print('\r');

        printTimeMS = millis();
    }
}

// --| Main Loop -----------------
// --|----------------------------
void loop() {
    updateSensorsAndAngles();

    SerialMotorEnable();

    // --| Motor Range Check -----
    motorRangeCheck();

    // --| CANBus Loop -----------
    CANBusLoop();

    for (size_t i = 0; i < sizeof(motorIDLookupTable); ++i) {
        motors[i]->loopFOC();
    }

    checkTaskStatusAndMove();

    printDebugInfo();

    // user communication
    command.run();
}
#endif
