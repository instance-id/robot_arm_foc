#if defined(RUN_MAIN) && RUN_MAIN == 1 && defined(ESP_CONTROLLER) && ESP_CONTROLLER == 1

#include "Helpers/VelocityRamp.h"
#include "SimpleCAN.h"
#include <Arduino.h>
#include <arduino-timer.h>
#include <cmath>
#include <map>

#if defined(USE_CAN)
#if defined(CAN_VERSION) && CAN_VERSION == 1
#include "CANProfile_V1.h"
#elif defined(CAN_VERSION) && CAN_VERSION == 2

#include "CANProfile_V2.h"

#endif
#endif

typedef std::map<uint8_t, uint8_t> stateMap;

// Stop Pin Set 1
#define STP_PIN_1 7
#define STP_PIN_2 15

#define STP_PIN_3 16
#define STP_PIN_4 17

// #define STP_PIN_5 18
#define STP_PIN_6 8

// ESP32-S3 TX/RX Pins
#define TX_PIN      43
#define RX_PIN      44
#define PIN_CAN0_TX GPIO_NUM_43
#define PIN_CAN0_RX GPIO_NUM_44

ulong printTimeMS = 0;
ulong log_print_time = 0;
ulong endstop_time_ms = millis();

// --| CANBus Identifiers --------
// --|----------------------------
byte txIdentifier = 0x001;
byte rxIdentifier = 0x002;
byte allMotorsIdentifier = 0x98;
byte broadcastIdentifier = 0x99;

// Can Identifier Lookup Table
byte canIDLookupTable[8] = {0x001, 0x020, 0x005, 0x006, 0x007, 0x99};
byte motorIDLookupTable[MOTOR_COUNT + BASE_MOTOR] = {
#if defined(MOTOR_COUNT) && MOTOR_COUNT > 0
#if defined(BASE_MOTOR) && BASE_MOTOR == 1
    0x004,
#endif
    0x005
#endif
#if defined(MOTOR_COUNT) && MOTOR_COUNT > 1
    ,
    0x006
#endif
#if defined(MOTOR_COUNT) && MOTOR_COUNT > 2
    ,
    0x007
#endif
};

// --| Motor Info ----------------
// --|----------------------------
#pragma region Motor_Info
// Motor Lookup Table
int motorCount = MOTOR_COUNT + BASE_MOTOR;
int registeredMotors = 0;

// create appendable list of byte
std::vector<byte> registeredMotorIDs;

int motorReadyCount = 0;

byte systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;

typedef std::map<int, int> MotorTaskStatusMap;
std::map<int, int> motorTeardownMap;

MotorUpdateMap motorUpdateMap = MotorUpdateMap();
MotorTaskStatusMap motorTaskStatusMap = MotorTaskStatusMap();

// Motor/Endstop Assignment. Two endstops per motor.
byte motorEndstopLookupTable[MOTOR_COUNT][2] = {

//@formatter:off
#if defined(MOTOR_COUNT) && MOTOR_COUNT > 0
    {STP_PIN_1, STP_PIN_2}
#endif
#if defined(MOTOR_COUNT) && MOTOR_COUNT > 1
    ,
    {STP_PIN_3, STP_PIN_4}
#endif
#if defined(MOTOR_COUNT) && MOTOR_COUNT > 2
    ,
    {STP_PIN_5, STP_PIN_6}
#endif
};

byte endstopPins[MOTOR_COUNT * 2] = {
    // #if defined(MOTOR_COUNT) && MOTOR_COUNT > 0
    //     STP_PIN_1, STP_PIN_2
    // #endif
    // #if defined(MOTOR_COUNT) && MOTOR_COUNT > 1
    //     ,
    //     STP_PIN_3, STP_PIN_4
    // #endif
    // #if defined(MOTOR_COUNT) && MOTOR_COUNT > 2
    //     ,
    //     STP_PIN_5, STP_PIN_6
    // #endif
};
//@formatter:on

float target_value = 0;
float motorCurrentAngle = 0.0f;
float restArmAngle = 0.0f;
float currentArmSensorAngle = 0.0f;
float previousArmSensorAngle = 0.0f;

ulong positionTimeMS = millis();
int positionReportFreq = 100;

float restArmStartPos = 0.0f;

float rest_pos_start = POS_TARGET_START;
float rest_pos_end = POS_TARGET_END;

float arm_0_min = ARM_0_MIN;
float arm_0_max = ARM_0_MAX;

enum RestArmState
{
    REST_ARM_DOWN = 0,
    REST_ARM_UP = 1,
};

RestArmState restArmState = REST_ARM_DOWN;

bool motor_enabled = false;
bool enable_endstop = true;
bool alternative_loop = false;

bool useVelocityRamp = true;
int vRampDownSample = 10;
int vRampDownSampleCount = 0;

std::array velocityLimits = {5.0f, 15.0f};
float calculatedVelocityLimit = 0.0f;

bool waiterStarted = false;
unsigned long motorDisableWaiter = 500;
unsigned long motorDisableWaiterStart = 0;

// --| Task Status ---------------
const byte TASK_IDLE = CAN_MESSAGE::TASK_STATUS::IDLE;
const byte TASK_RUNNING = CAN_MESSAGE::TASK_STATUS::RUNNING;
const byte TASK_COMPLETE = CAN_MESSAGE::TASK_STATUS::COMPLETE;
const byte TASK_ERROR = CAN_MESSAGE::TASK_STATUS::ERROR;

byte taskStatus = TASK_IDLE;

#pragma endregion

byte enabledPins[MOTOR_COUNT * 2];
stateMap endstopStateMap;

// --| CANBus Initialization -----
// --|----------------------------
CANReceiver CANBroker;
CANHandler CANDevice(&CAN, &CANBroker, txIdentifier);

// --| Helper Functions ----------
// --|----------------------------
byte getMotorFromEndStop(int endstopPin) {
    for (int i = 0; i < registeredMotors; i++) {
        if (motorEndstopLookupTable[i][0] == endstopPin || motorEndstopLookupTable[i][1] == endstopPin) {
            Serial.printf("Motor Lookup: %d Pin %d matched either pin: %d | %d \n", motorIDLookupTable[i], endstopPin, motorEndstopLookupTable[i][0], motorEndstopLookupTable[i][1]);
            return motorIDLookupTable[i];
        }
    }

    Serial.printf("Motor Lookup: No Motor Found for Pin %d \n", endstopPin);
    return 0;
}

#pragma region Motor_Control
#if defined(USE_MOTOR) && USE_MOTOR == 1

#include "MT6701_I2C.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include <SimpleFOC.h>

#define MT6701_SSI_CLOCK 100000   // Define 100kHz SSI speed before <MT6701.h> include
#include "SPI.h"
#include "Wire.h"
#include <MT6701.h>

#define PIN_ENABLE 7
#define PIN_IN1    4
#define PIN_IN2    5
#define PIN_IN3    6

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

BLDCMotor motor = BLDCMotor(11, 5.57, 197);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);

TwoWire I2Cone = TwoWire(0);

#if defined(ENCODER2_TYPE) && ENCODER2_TYPE == 0

#elif defined(ENCODER2_TYPE) && ENCODER2_TYPE == 1

MT6701_I2CConfig_s mt6701_config = {.chip_address = 0b0000110, .bit_resolution = 14, .angle_register = 0x03, .data_start_bit = 8};
MT6701_Serial_I2C sensor = MT6701_Serial_I2C(mt6701_config);
#elif defined(ENCODER2_TYPE) && ENCODER2_TYPE == 2
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

#endif

#if defined(ENCODER_TYPE) && ENCODER_TYPE == 0
#define SSI_SDA_PIN  13
#define SSI_CLK_PIN  12
#define SSI_CS_PIN   16
#define SSI_MODE_PIN 15

MagneticSensorMT6701SSI sensor1(SSI_CS_PIN);
#elif defined(ENCODER_TYPE) && ENCODER_TYPE == 1
#define I2C_SDA_PIN_2 13
#define I2C_SCL_PIN_2 12

TwoWire I2Ctwo = TwoWire(1);

MT6701_I2CConfig_s mt6701_config = {.chip_address = 0b0000110, .bit_resolution = 14, .angle_register = 0x03, .data_start_bit = 8};
MT6701_Serial_I2C sensor1 = MT6701_Serial_I2C(mt6701_config);
#endif

Commander cmdr = Commander(Serial);

float monitorSpeed = MONITOR_FREQ;
float adjustedMonitorSpeed = monitorSpeed;

void doMotor(char *cmd) { cmdr.motor(&motor, cmd); }
void doTarget(char *cmd) { cmdr.scalar(&target_value, cmd); }
void doLimit(char *cmd) { cmdr.scalar(&motor.voltage_limit, cmd); }
void doMonitorSpeed(char *cmd) { cmdr.scalar(&monitorSpeed, cmd); }
void doVelocity(char *cmd) { cmdr.scalar(&motor.velocity_limit, cmd); }

bool motorProcessBeginCallback();

bool motorProcessEndCallback();

bool motorProcessTestCallback();

bool motorProcessDisableCallback();

auto timer = timer_create_default();

#endif
#pragma endregion

// --| Setup Functions -----------
// --|----------------------------

// --| Pin Setup ------------
void pinSetup() {
    int totalEndstopPins = sizeof(endstopPins) / sizeof(endstopPins[0]);
    Serial.printf("Total Endstop Pins: %d \n", totalEndstopPins);

    // for (int i = 0; i < totalEndstopPins; i++) {
    //     pinMode(endstopPins[i], INPUT);

    //     endstopStateMap[endstopPins[i]] = digitalRead(endstopPins[i]);
    //     enabledPins[i] = endstopPins[i];
    //     Serial.printf("Enabled ENDSTOP PIN: %d State: %d \n", endstopPins[i],
    //     endstopStateMap[endstopPins[i]]);
    // }

    Serial.printf("Total Enabled Endstop Pins: %d \n", sizeof(enabledPins) / sizeof(enabledPins[0]));

    pinMode(RX_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);
}

void motorDataSetup() {
    motorUpdateMap = MotorUpdateMap();
    motorTaskStatusMap = MotorTaskStatusMap();

    motorTeardownMap[0x004] = 500;
    motorTeardownMap[0x005] = 1800;
    motorTeardownMap[0x006] = 500;
}

void taskBegin() {
    taskStatus = TASK_RUNNING;
    CANDevice.CANSendReport(txIdentifier, txIdentifier, CAN_MESSAGE::REPORT_TASK_STATUS, TASK_RUNNING);
}

void taskComplete() {
    taskStatus = TASK_COMPLETE;
    CANDevice.CANSendReport(txIdentifier, txIdentifier, CAN_MESSAGE::REPORT_TASK_STATUS, TASK_COMPLETE);
}

CanFilter filter = CanFilter(ACCEPT_ALL, rxIdentifier, rxIdentifier, FILTER_ANY_FRAME);

// --| CANBus Init ----------
void canBusInit() {
    CAN.logTo(&Serial);
    CAN.filter(filter);
    CAN.begin(CAN_BITRATE);
}

// --| Motor Setup ----------
#pragma region Motor_Setup
#if defined(USE_MOTOR) && USE_MOTOR == 1
void MotorSetup() {
    cmdr.verbose = VerboseMode::machine_readable;
    Serial.begin(115200);
    while (!Serial);

    SimpleFOCDebug::enable(&Serial);

    if (I2Cone.setPins(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("Sensor 1 Pins Set!");
        sensor.init(&I2Cone);
        I2Cone.setClock(400000);
    } else {
        Serial.println("Sensor 1 Pins Failed!");
        return;
    }

#if defined(ENCODER_TYPE) && ENCODER_TYPE == 0
    SPI.setFrequency(100000);
    sensor1.init(&SPI);
#elif defined(ENCODER_TYPE) && ENCODER_TYPE == 1
    if (I2Ctwo.setPins(I2C_SDA_PIN_2, I2C_SCL_PIN_2)) {
        Serial.println("Sensor 2 Pins Set!");
        sensor1.init(&I2Ctwo);
        I2Ctwo.setClock(400000);
    } else {
        Serial.println("Sensor 2 Pins Failed!");
        return;
    }
#endif

    // Link the motor to the sensor
    motor.linkSensor(&sensor);

    motor.voltage_sensor_align = 3;
    motor.velocity_index_search = 3;

    // Driver config
    // Power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;

    // driver.pwm_frequency = 100000;
    // driver.pwm_frequency = 20000;
    driver.pwm_frequency = 40000;

    if (driver.init())
        Serial.println("Driver init success!");
    else {
        Serial.println("Driver init failed!");
        return;
    }

    // Link the motor and the driver
    motor.linkDriver(&driver);

    // Choose FOC modulation (optional)
    motor.foc_modulation = SpaceVectorPWM;

    // Set motion control loop to be used
    motor.controller = angle;

    // Maximal voltage to be set to the motor
    motor.voltage_limit = driver.voltage_limit * 0.5f;
    motor.current_limit = CURRENT_LIMIT;

    // Controller configuration
    // Default parameters in defaults.h
    // Velocity PI controller parameters

    motor.PID_velocity.P = 0.066;
    motor.PID_velocity.I = 6.6;
    motor.PID_velocity.D = 0.000135;

    // motor.PID_velocity.P = 0.8f;
    // motor.PID_velocity.I = 20;
    // motor.PID_velocity.D = 0.001;
    motor.PID_velocity.output_ramp = 200.0f;

    motor.LPF_velocity.Tf = 0.01f;

    // Angle P controller
    motor.P_angle.P = 20.0;
    // motor.P_angle.P = 14.0;
    motor.P_angle.I = 0.0;
    motor.P_angle.D = 0.0;
    motor.LPF_angle.Tf = 0.001;

    // motor.P_angle.output_ramp = 10000.0;
    // Maximal velocity of the position control
    motor.velocity_limit = velocityLimits[0];

    motor.zero_electric_angle = 5.39f; // zero_electric_angle
    motor.sensor_direction = CCW; // Cw/CCW // direction

    // Comment out if not needed
    motor.useMonitoring(Serial);

    // Initialize motor
    motor.init();

    // Align sensor and start FOC
    motor.initFOC();

    // --| Sensor Offset Calibration
    motor.sensor_offset = motor.shaftAngle();
    currentArmSensorAngle = sensor1.getAngle();

    Serial.printf("Driver Stats: voltage_power_supply %f voltage_limit %f "
                  "pwm_frequency %ld\n",
                  driver.voltage_power_supply, driver.voltage_limit, driver.pwm_frequency);

    Serial.printf("Motor Stats: voltage_limit %f current_limit %f velocity_limit: %f\n", motor.voltage_limit, motor.current_limit, motor.velocity_limit);

    Serial.printf("Motor Angle: %f Sensor Angle: %f Offset: %f\n", motor.shaftAngle(), sensor.getAngle(), motor.sensor_offset);
    Serial.printf("Sensor 1: %f Sensor 2: %f\n", sensor.getAngle(), currentArmSensorAngle);

    velocityLimitLow = velocityLimits[0];
    velocityLimitHigh = velocityLimits[1];

    // field_status = sensor1.fieldStatusRead();
    // Serial.println("Encoder Status: " + String(field_status));

    // --| Get initial sensor angle

    motor.disable();

    // Add target command T
    cmdr.add('M', doMotor, "motor");
    cmdr.add('T', doTarget, "target angle");
    cmdr.add('L', doLimit, "voltage limit");
    cmdr.add('V', doLimit, "movement velocity");
    cmdr.add('S', doMonitorSpeed, "monitor speed");

    // ▬▬▬▬▬▬▬▬▬▬ CHECK_IF_ERROR ▬▬▬▬▬▬▬▬▬▬
    //  https://docs.simplefoc.com/cheetsheet/options_reference
    if (motor.motor_status != 4) // 0 - fail initFOC
    {
        Serial.println("ERROR:" + String(motor.motor_status));
        // return;
    }
}

void requestReady() {
    CANDevice.CANSendInt(
        0,
        broadcastIdentifier,
        CAN_MESSAGE::REQUEST_READY,
        0
    );
}

void requestValues() {
    CANDevice.CANRegisterDevice(0x002, 0x001);
    CANDevice.CANRetrieveParameter(0x002, 0x001, CAN_MESSAGE::PARAMETERS::ALL_POSITIONS, 0);
}

#endif
#pragma endregion

// --| Main Setup -----------
#pragma region Main Setup

void setup() {
    Serial.begin(BAUDRATE);
    while (!Serial);

    delay(3000);

    pinSetup();
    motorDataSetup();
    canBusInit();

#if defined(USE_MOTOR) && USE_MOTOR == 1
    MotorSetup();
    // requestValues();
    requestReady();
#endif

    printTimeMS = millis();
}

#pragma endregion

// --| Runtime Loops --------
// --|-----------------------
#pragma region Motor Calibration

void startMotorCalibration() {
    for (int i = 0; i < registeredMotors; i++) {
        byte motorID = registeredMotorIDs[i];

        if (motorUpdateMap[motorID] == CAN_MESSAGE::CURRENT_RUN_MODE::IDLE) {
            Serial.printf("Calibrating Motor: 0x%x \n", motorID);
            CANDevice.CANSendInt(static_cast<int>(CAN_MESSAGE::RUN_MODE::CALIBRATE), motorID, CAN_MESSAGE::SET_RUN_MODE, static_cast<byte>(CAN_MESSAGE::RUN_MODE::CALIBRATE));
            break;
        }
    }

    bool allCalibrated = true;
    for (auto const &[key, val]: motorUpdateMap) {
        if (val != CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED) {
            allCalibrated = false;
            break;
        }
    }

    if (allCalibrated) {
        systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED;
        Serial.println("All Motors are Calibrated: Set Mode to RUN");

        for (int i = 0; i < registeredMotors; i++) {
            byte motorID = registeredMotorIDs[i];
            CANDevice.CANSendInt(static_cast<int>(CAN_MESSAGE::RUN_MODE::RUN), motorID, CAN_MESSAGE::SET_RUN_MODE, static_cast<byte>(CAN_MESSAGE::RUN_MODE::CALIBRATE));
        }
    }
}

#pragma endregion

// --| CANBus Loop ---------------
#pragma region CANBus Loop

bool motor_moving = false;

// --| Timed Run Mode Set --------

struct run_mode_args
{
    int motorID;
    int status;
};

bool setRunMode(void *argument) {
    const auto *args = static_cast<run_mode_args *>(argument);
    const int motorID = args->motorID;
    const int status = args->status;

    CANDevice.CANSendInt(status, motorID, CAN_MESSAGE::SET_RUN_MODE, status);

    delete args;
    return true;
}

// --| Motor Control Functions ---
void dcEnable() {
    if (!motor.enabled)
        motor.enable();

    motor_enabled = true;
    Serial.printf("Enabling Motor: %d\n", motor_enabled);
}

void dcDisable() {
    motor_moving = false;
    motor_enabled = false;
    waiterStarted = false;
    Serial.printf("Motor Enabled: %d\n", motor_enabled);

    taskComplete();

    CANDevice.CANStoreParameter(0x002, 0x001, CAN_MESSAGE::PARAMETERS::CURRENT_POSITION, motorCurrentAngle);

    if (motor.enabled)
        motor.disable();

    switch (systemRunMode) {
        case CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN: {
            if (restArmState == REST_ARM_DOWN) {
                for (auto const &[key, val]: motorUpdateMap) {
                    byte motorID = key;

                    auto *args = new run_mode_args{motorID, static_cast<int>(CAN_MESSAGE::RUN_MODE::TEARDOWN)};
                    args->motorID = motorID;
                    args->status = static_cast<int>(CAN_MESSAGE::RUN_MODE::TEARDOWN);

                    timer.in(motorTeardownMap[motorID], setRunMode, args);
                }
            } else {
                Serial.println("Teardown cannot be completed with the arm in the up position!");
            }
            break;
        }
    }
}

void moveDCForward() {
    taskBegin();
    dcEnable();

    motor_moving = true;
    movementStartAngle = motor.shaftAngle();
    target_value = rest_pos_start;
    Serial.println("Motor Forward");
}

void moveDCReverse() {
    taskBegin();
    dcEnable();

    motor_moving = true;
    movementStartAngle = motor.shaftAngle();
    target_value = rest_pos_end;
    Serial.println("Motor Reverse");
}

// --| Motor Process Callbacks ------------------
// --|-------------------------------------------
bool restMotorReady = false;
//@formatter:off
// clang-format off

bool motorProcessBeginCallback(void *argument) { moveDCReverse(); return true; }
bool motorProcessEndCallback(void *argument) { moveDCForward(); return true; }
bool motorProcessDisableCallback(void *argument) { dcDisable(); return true; }
bool motorProcessTestCallback(void *argument) { Serial.println("Motor Process Test Callback"); return true; }

// --| Compare Float ----------------------------
// --|-------------------------------------------
bool compareFloat(float pos1, float pos2, float allowedError)
{
    if (abs(pos1 - pos2) < allowedError) { return true; }
    return false;
}

bool floatHasChanged(float pos1, float pos2, float allowedError)
{
    if (abs(pos1 - pos2) > allowedError) { return true; }
    return false;
}

// clang-format on
//@formatter:on

// --| Set Parameter ----------------------------
// --|-------------------------------------------
void updateParameter(const std::string &parameterName, float &parameter, const float receivedValue) {
    parameter = receivedValue;
    Serial.printf("%s: %f \n", parameterName.c_str(), parameter);
}

float combineToFloat(int value1, int value2) {
    if (value1 < 0 || value1 > 99 || value2 < 0 || value2 > 99) {
        Serial.printf("Invalid input values: %d, %d\n", value1, value2);
    }
    // Combine values: value1 occupies the integral part, value2 occupies the hundredths place
    return static_cast<float>(value1) + static_cast<float>(value2) / 100.0f;
}

void extractFloat(float receivedValue, float &value1, float &value2) {
    value1 = static_cast<float>(static_cast<int>(receivedValue));
    value2 = static_cast<float>(static_cast<int>((receivedValue - value1) * 100));
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
        if (CANBroker.motorEnabled == 1) {
            motor.enable();
        } else {
            motor.disable();
        }
    } else if (CANBroker.setParameterType == CAN_MESSAGE::PARAMETERS::MOTOR_MIN_MAX_VELOCITY) {
        extractFloat(CANBroker.receivedFloatVal, velocityLimits[0], velocityLimits[1]);
        Serial.printf("Motor Min Velocity: %f Max Velocity: %f \n", velocityLimits[0], velocityLimits[1]);
    }
}

void CANBusLoop() {
    if (CAN.available() > 0) {
        uint8_t packedMessage[8];
        CanMsg const rxMsg = CAN.read();

        CANDevice.HandleCanMessage(rxMsg, rxMsg.data);

        // --| Set Run Mode ---------------------
        // --|-----------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_RUN_MODE) {
            systemRunMode = CANBroker.runMode;

            switch (CANBroker.runMode) // clang-format off
            { //@formatter:off
            case CAN_MESSAGE::RUN_MODE::IDLE: { Serial.println("Setting System Run Mode to IDLE"); break; }
            case CAN_MESSAGE::RUN_MODE::RUN: { Serial.println("Setting System Run Mode to RUNNING"); break; }
            case CAN_MESSAGE::RUN_MODE::CALIBRATE: { Serial.println("Setting System Run Mode to CALIBRATING"); break; }
            case CAN_MESSAGE::RUN_MODE::TEARDOWN: { Serial.println("Setting System Run Mode to TEARDOWN"); break; }
            case CAN_MESSAGE::RUN_MODE::SHUTDOWN: { Serial.println("Setting System Run Mode to SHUTDOWN"); break; }
            default: break;
            } //@formatter:on // clang-format on

            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SEND_READY) {
            int motorID = CANBroker.motorID;
            Serial.printf("Received Ready Signal from Motor: 0x%x \n", motorID);

            motorUpdateMap[motorID] = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;
            motorTaskStatusMap[motorID] = TASK_IDLE;

            // if registeredMotorIDs does not contain motorID, add it
            if (std::ranges::find(registeredMotorIDs, motorID) == registeredMotorIDs.end()) {
                registeredMotorIDs.push_back(motorID);

                // Sort registeredMotorIDs
                std::sort(registeredMotorIDs.begin(), registeredMotorIDs.end());
            }

            registeredMotors = static_cast<int>(registeredMotorIDs.size());

            Serial.printf("Motor Ready Count: %d \n", registeredMotors);

            CANBroker.ReceivedID = -1;
        }

        // --| Motor Reporting ------------------
        // --|-----------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::REPORT_RUN_MODE) {
            int motorID = CANBroker.motorUpdate.Device;
            motorUpdateMap[motorID] = CANBroker.motorUpdate.Value;

            Serial.printf("Update MotorID: 0x%x Run Mode: %d \n", motorID,
                          CANBroker.runMode);

#if defined(USE_MOTOR) && USE_MOTOR == 1
            if (motorID == motorIDLookupTable[1] && motorUpdateMap[motorID] == CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING) {
                restMotorReady = true;

                Serial.printf("Motor: 0x%x is Calibrating \n", motorID);
            }
#endif

            if (motorUpdateMap[motorID] == CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED) {
                Serial.printf("Motor: 0x%x is Calibrated \n", motorID);

                startMotorCalibration();
            }

            if (motorUpdateMap[motorID] == CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING) {
                Serial.printf("Motor: 0x%x is Running \n", motorID);

                startMotorCalibration();
            }

            if (motorUpdateMap[motorID] == CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN) {
                Serial.printf("Motor: 0x%x is in Teardown \n", motorID);
            }

            if (systemRunMode == CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED) {
                // Iterate motorUpdateMap
                bool allRunning = true;
                for (auto const &[key, val]: motorUpdateMap) {
                    if (val != CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING) {
                        allRunning = false;
                        break;
                    }
                }

                if (allRunning) {
                    Serial.println("All Motors are Running: Set System Mode to RUNNING");
                    systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING;
                }
            }

            CANBroker.ReceivedID = -1;
        }

        // --| Controller Action ----------------
        // --|-----------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::CONTROLLER_ACTION) {
            if (CANBroker.controllerAction == CAN_MESSAGE::CONTROLLER_ACTIONS::STOP) {
                Serial.println("Stopping All Motors");
                systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;

                for (int i = 0; i < motorCount; i++) {
                    byte motorID = motorIDLookupTable[i];
                    CANDevice.CANSendInt(static_cast<int>(CAN_MESSAGE::RUN_MODE::IDLE), motorID, CAN_MESSAGE::SET_RUN_MODE, static_cast<byte>(CAN_MESSAGE::RUN_MODE::IDLE));
                }
            }

            if (CANBroker.controllerAction == CAN_MESSAGE::CONTROLLER_ACTIONS::START) {
                Serial.println("Starting All Motors");
                systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING;

                for (int i = 0; i < motorCount; i++) {
                    byte motorID = motorIDLookupTable[i];
                    CANDevice.CANSendInt(static_cast<int>(CAN_MESSAGE::RUN_MODE::RUN), motorID, CAN_MESSAGE::SET_RUN_MODE, static_cast<byte>(CAN_MESSAGE::RUN_MODE::RUN));
                }
            }

            if (CANBroker.controllerAction == CAN_MESSAGE::CONTROLLER_ACTIONS::RESET) {
                // --| Test Callback --------------
                timer.in(1000, motorProcessTestCallback);

                Serial.println("Resetting Controller State");

                motorDataSetup();
                requestReady();

                systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;
            }

            if (CANBroker.controllerAction == CAN_MESSAGE::CONTROLLER_ACTIONS::CALIBRATE) {
                if (systemRunMode == CAN_MESSAGE::CURRENT_RUN_MODE::IDLE || systemRunMode == CAN_MESSAGE::CURRENT_RUN_MODE::RUNNING) {
                    Serial.println("Setting Motor Run Mode to CALIBRATE");
                    systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING;

                    startMotorCalibration();
                } else if (systemRunMode == CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATING) {
                    Serial.println("System is currently calibrating");
                } else if (systemRunMode == CAN_MESSAGE::CURRENT_RUN_MODE::CALIBRATED) {
                    Serial.println("System is already calibrated");
                }
            }

            if (CANBroker.controllerAction == CAN_MESSAGE::CONTROLLER_ACTIONS::TEARDOWN) {
                Serial.println("Tearing Down Controller");
                systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN;

                timer.in(500, motorProcessEndCallback);
            }

            if (CANBroker.controllerAction == CAN_MESSAGE::CONTROLLER_ACTIONS::SHUTDOWN) {
                Serial.println("Shutting Down Controller");
                systemRunMode = CAN_MESSAGE::CURRENT_RUN_MODE::IDLE;

                for (int i = 0; i < motorCount; i++) {
                    byte motorID = motorIDLookupTable[i];
                    CANDevice.CANSendInt(static_cast<int>(CAN_MESSAGE::RUN_MODE::IDLE), motorID, CAN_MESSAGE::SET_RUN_MODE, static_cast<byte>(CAN_MESSAGE::RUN_MODE::IDLE));
                }
            }

            // --| DC Motor Control -------------
            // --|-------------------------------
            switch (CANBroker.controllerAction) //clang-format off
            { //@formatter:off
            case CAN_MESSAGE::CONTROLLER_ACTIONS::DC_ENABLE: { dcEnable(); break; }
            case CAN_MESSAGE::CONTROLLER_ACTIONS::DC_DISABLE: { dcDisable(); break; }
            case CAN_MESSAGE::CONTROLLER_ACTIONS::DC_FORWARD: { moveDCForward(); break; }
            case CAN_MESSAGE::CONTROLLER_ACTIONS::DC_REVERSE: {
                // --| Limit: Rest Position ----------------
                if (currentArmSensorAngle >= ARM_0_CALIB) {
                    Serial.println("Main arm is in the rest position. Will not move rest "
                                   "arm motor.");
                    break;
                }
                moveDCReverse();
                break;
            }
            case CAN_MESSAGE::CONTROLLER_ACTIONS::DC_STOP: {
                /* motor_stop = true; */
                Serial.println("Motor Stop");
                break;
            }
            default: break;
            } //@formatter:on

            CANBroker.ReceivedID = -1;
        } // clang-format on

        // --| Set Parameter --------------------
        // --|-----------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_PARAMETER) {
            Serial.printf("CANBroker Match: %d Value: %f Destination: 0x%x:%d TxID: 0x%x\n", CANBroker.setParameterType, CANBroker.receivedFloatVal, CANBroker.destinationId, CANBroker.destinationId, txIdentifier);

            if (CANBroker.destinationId == static_cast<int>(txIdentifier)) {
                setParameter();
                Serial.printf("Setting Parameter: %d Value: %f \n", CANBroker.setParameterType, CANBroker.receivedFloatVal);
            } else if (CANBroker.destinationId == allMotorsIdentifier) {
                for (int i = 0; i < motorCount; i++) {
                    byte motorID = motorIDLookupTable[i];

                    CANDevice.CANSendParameter(CANBroker.receivedFloatVal, motorID, 0, CANBroker.setParameterType);
                }
            } else {
                CANDevice.CANSendParameter(CANBroker.receivedFloatVal, CANBroker.destinationId, 0, CANBroker.setParameterType);
            }

            CANBroker.ReceivedID = -1;
        }

        // --| Retrieve Parameter ---------------
        // --|-----------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::RETRIEVE_PARAMETER) {
            Serial.printf("CANBroker Match: %d Value: %f Destination: 0x%x:%d TxID: 0x%x\n", CANBroker.setParameterType, CANBroker.receivedFloatVal, CANBroker.destinationId, CANBroker.destinationId, txIdentifier);

            if (CANBroker.destinationId == static_cast<int>(txIdentifier)) {
                setParameter();
                Serial.printf("Setting Parameter: %d Value: %f \n", CANBroker.setParameterType, CANBroker.receivedFloatVal);
            } else if (CANBroker.destinationId == allMotorsIdentifier) {
                for (int i = 0; i < motorCount; i++) {
                    const byte motorID = motorIDLookupTable[i];

                    CANDevice.CANSendParameter(CANBroker.receivedFloatVal, motorID, 0, CANBroker.setParameterType);
                }
            } else {
                CANDevice.CANSendParameter(CANBroker.receivedFloatVal, CANBroker.destinationId, 0, CANBroker.setParameterType);
            }

            CANBroker.ReceivedID = -1;
        }

        // --| Task Status Reporting ------------
        // --|-----------------------------------
        if (CANBroker.ReceivedID == CAN_MESSAGE::REPORT_TASK_STATUS) {
            Serial.printf("Task Status: %d \n", CANBroker.taskStatus);

            int motorID = CANBroker.destinationId;

            motorTaskStatusMap[motorID] = CANBroker.taskStatus;

            if (motorUpdateMap[motorID] == CAN_MESSAGE::CURRENT_RUN_MODE::TEARDOWN) {
                if (CANBroker.taskStatus == TASK_COMPLETE) {
                    Serial.printf("Teardown complete for motor: 0x%x. Disabling...\n", motorID);

                    CANDevice.CANSendParameter(0, motorID, 0, CAN_MESSAGE::PARAMETERS::MOTOR_ENABLED);
                }
            }

            CANBroker.ReceivedID = -1;
        }

        CANBroker.ReceivedID = -1;
    }
}

#pragma endregion
bool didPrint = false;
unsigned long sensorAngleTime = millis();

int positionalEndstop_0 = 0;
int positionalEndstop_1 = 0;

// --| Main Loop ------------
#pragma region Main Loop

// --| Send Position Report ------
void sendPositionReport(int motorID, float position) {
    // --| Send Position Report ------
    CANDevice.CANReportParameter(rxIdentifier, txIdentifier, CAN_MESSAGE::PARAMETERS::CURRENT_POSITION, currentArmSensorAngle);
}

bool loopCalled = false;

void loop() {
    loopCalled = false;

    if (LOOP_FOC && !motor_enabled && !loopCalled) {
        motor.loopFOC();
        loopCalled = true;
    }

    motorCurrentAngle = motor.shaftAngle();
    restArmAngle = sensor.getAngle();

    currentArmSensorAngle = sensor1.getAngle();

    if ((motorTaskStatusMap[motorIDLookupTable[1]] == TASK_RUNNING) && (floatHasChanged(currentArmSensorAngle, previousArmSensorAngle, 0.03f)) && (millis() - positionTimeMS > positionReportFreq)) {
        sendPositionReport(motorIDLookupTable[1], currentArmSensorAngle);
        positionTimeMS = millis();
    }

    // --| Positional Endstop Handler (Encoder) ------
    // --|--------------------------------------------
    int armMotor = motorIDLookupTable[0];
    if (MOTOR_COUNT + BASE_MOTOR > 1) {
        armMotor = motorIDLookupTable[1];
    }

    if (enable_endstop && (currentArmSensorAngle <= arm_0_min || currentArmSensorAngle >= arm_0_max)) {
        if (currentArmSensorAngle <= arm_0_min) {
            if (positionalEndstop_0 == 0) {
                CANDevice.CANSendInt(0, armMotor, CAN_MESSAGE::ENDSTOP_PRESSED, 0);
                positionalEndstop_0 = 1;
            }
        } else if (currentArmSensorAngle >= arm_0_max) {
            if (positionalEndstop_1 == 0) {
                CANDevice.CANSendInt(1, armMotor, CAN_MESSAGE::ENDSTOP_PRESSED, 1);
                positionalEndstop_1 = 1;
            }
        }
    } else {
        positionalEndstop_0 = 0;
        positionalEndstop_1 = 0;
    }

    if (enable_endstop && (positionalEndstop_0 == 1 || positionalEndstop_1 == 1) && (millis() - sensorAngleTime > 100)) {
        // Serial.printf("Positional Endstop 0: %d 1: %d Sensor Angle: %f %f\n",
        // positionalEndstop_0, positionalEndstop_1, armSensorAngle,
        // sensor1.getAngle());
        sensorAngleTime = millis();
    }

    // --| Rest Arm Position Handler ------------
    // --|---------------------------------------
    if (motor_enabled && restArmState != REST_ARM_DOWN && compareFloat(motorCurrentAngle, rest_pos_start, 0.1f)) {
        restArmState = REST_ARM_DOWN;
        Serial.println("Rest Arm State: DOWN");
    } else if (motor_enabled && restArmState != REST_ARM_UP && compareFloat(motorCurrentAngle, rest_pos_end, 0.1f)) {
        restArmState = REST_ARM_UP;
        Serial.println("Rest Arm State: UP");
    }

    // --| Endstop Handler ------------
    // --|-----------------------------
    int endStopPinsCount = sizeof(enabledPins) / sizeof(enabledPins[0]);
    endStopPinsCount = 0;

    for (int i = 0; i < endStopPinsCount; i++) {
        int es = enabledPins[i];
        int stp = digitalRead(es);

        if (stp == 0 && endstopStateMap[es] != stp) {
            byte motorCANId = getMotorFromEndStop(es);

            Serial.printf("ENDSTOP: %d STP: %d MotorID: 0x%x \n", es, stp, motorCANId);

            if (motorCANId != 0 && millis() - endstop_time_ms > 200) {
                CANDevice.CANSendInt(int(i), motorCANId, CAN_MESSAGE::ENDSTOP_PRESSED, int(i));
                endstop_time_ms = millis();
            }
        }

        if (endstopStateMap[es] != stp && millis() - printTimeMS > 100) {
            Serial.print("ENDSTOP: ");
            Serial.print(enabledPins[i]);
            Serial.print(" STP: ");
            Serial.println(stp);

            didPrint = true;
        }

        endstopStateMap[es] = stp;
    }

    // --| Run CANBus Loop ------------
    // --|-----------------------------
    CANBusLoop();

#if defined(USE_MOTOR) && USE_MOTOR == 1
    // --| Motor Loop -----------------
    // --|-----------------------------

    if (taskStatus == TASK_RUNNING) {
        adjustedMonitorSpeed = monitorSpeed * 0.2f;
    } else {
        adjustedMonitorSpeed = monitorSpeed;
    }

    if (millis() - printTimeMS > adjustedMonitorSpeed) {
        String dataMessage = "T: " + String(target_value, 4) + " V: " + String(calculatedVelocityLimit, 4) + " MV: " + String(motor.shaftVelocity(), 4) + " CA: " + String(motorCurrentAngle, 4) + " SA: " + String(currentArmSensorAngle) + " RA: " + String(restArmAngle) +
                             " Tsk: " + String(taskStatus) + " En: " + String(motor.enabled);

        Serial.println(dataMessage);
        // Serial.print('\r');

        printTimeMS = millis();
    }

    if (restMotorReady && currentArmSensorAngle <= ARM_0_CALIB) {
        restMotorReady = false;
        timer.in(500, motorProcessBeginCallback);
    }

    if (taskStatus == TASK_COMPLETE) {
        taskStatus = TASK_IDLE;
    }

    if (motor_enabled) {
        // --| Waiter for Motor Disable ----
        // --|------------------------------
        if (motor_moving && compareFloat(target_value, motorCurrentAngle, 0.05f)) {
            if (!waiterStarted) {
                timer.in(500, motorProcessDisableCallback);
                motorDisableWaiterStart = millis();
                waiterStarted = true;
            }
        }

        motor.loopFOC();

        if (useVelocityRamp && vRampDownSample > 0 && --vRampDownSampleCount <= 0) {
            calculatedVelocityLimit = velocityRamp(movementStartAngle, motorCurrentAngle, target_value, velocityLimits[0], velocityLimits[1]);

            motor.velocity_limit = calculatedVelocityLimit;
            motor.PID_velocity.limit = calculatedVelocityLimit;

            vRampDownSampleCount = vRampDownSample;
        }

        if (motor.enabled && motorCurrentAngle >= (rest_pos_start + 0.08f) || motorCurrentAngle <= (rest_pos_end - 0.08f)) {
            Serial.printf("Motor is out of bounds. Disabling... Current: %f Start: %f End: %f \n", motorCurrentAngle, rest_pos_start - 0.08f, rest_pos_end + 0.08f);
            motor.disable();
        }

        motor.move(target_value);

        // --| Motor Command Handler -------
        // --|------------------------------
        // cmdr.run();
    }
#endif

    previousArmSensorAngle = currentArmSensorAngle;
    sensor1.update();

    // --| Timer Loop ----------------------
    // --|----------------------------------
    timer.tick();
}

#pragma endregion
#endif
