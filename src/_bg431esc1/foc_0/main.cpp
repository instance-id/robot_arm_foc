#if defined(RUN_MAIN) && RUN_MAIN == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 22

#include <SimpleFOC.h>

// --| Use CANBus ----------------
// --|----------------------------

#include <cmath>
#include "SimpleCAN.h"


#if defined(USE_CAN)
#if defined(CAN_VERSION) && CAN_VERSION == 1

#include "CANProfile_V1.h"

#elif defined(CAN_VERSION) && CAN_VERSION == 2

#include "CANProfile_V2.h"
#include "Helpers/stringf.h"

#endif
#endif

float target_angle = 0;
long printTimeMS = 0;
float lastTarget = 0.0f;

float vbusV = 0.0;
float tempDegC = 0.0;
float vbusValRaw = 0.0;
float tempValRaw = 0.0;
float velSpRadsPerSec = 0.0;

// The actual CAN bus class, which handles all communication.
// You may need to adjust the used pins!
byte txIdentifier = 0x22;
byte rxIdentifier = 0x20;

CANReceiver CANBroker;
CANHandler CANDevice(&CAN, &CANBroker, txIdentifier);

int DeviceID = 0x21;

#define PHASE_RESISTANCE 7.5
// #define KV
#define CURRENT_LIMIT 1.5

// Make Variable Global
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
PhaseCurrent_s current;
Commander command = Commander(Serial);

void doMotor(char *cmd) { command.motor(&motor, cmd); }

void doTarget(char *cmd) { command.scalar(&target_angle, cmd); }

void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

// NTC2TempV: Convert NTC voltage to temperature
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

#pragma region CANBus Setup


void CANBusSetup()
{
#define BUTTON PC10
    pinMode(BUTTON, INPUT);

    CAN.logTo(&Serial);
    Serial.println("Starting CAN");
    CanFilter filter = CanFilter(ACCEPT_ALL, rxIdentifier, rxIdentifier, FILTER_ANY_FRAME);
    CAN.filter(filter);
    CAN.begin(CAN_BITRATE);
    delay(10);
}

bool SendEnable = true;
bool buttonPressed = false;
bool buttonReleased = false;

float velocityLimitHigh = 20;
float internalLimiter(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    else if (value < -limit)
    {
        return -limit;
    }
    else
    {
        return value;
    }
}

void CANBusLoop()
{
    static uint32_t LastAction = millis();
    static uint32_t LastFloatAction = millis();
    static uint32_t LastRTR = millis();

    // Test of regular messages:
    // What is sent next to the CAN bus depends on what was received last.
    // When a PING was received, send a PONG and vice versa.
    // To get the whole thing started, a PONG is sent every 5s without having received anything.
    // This is just for testing. Usually you would invoke actions for incomming messages
    // directly in the broker class.
    if (CAN.available() > 0)
    {
        uint8_t packedMessage[8];
        CanMsg const rxMsg = CAN.read();

        CANDevice.HandleCanMessage(rxMsg, rxMsg.data);

        if (CANBroker.motorTarget != 0.0f && lastTarget != CANBroker.motorTarget)
        {
            lastTarget = CANBroker.motorTarget;
            Serial.printf("Motor Value: %f\n", CANBroker.motorTarget);
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_ENABLE)
        {
            motor.enabled = CANBroker.motorEnabled;
            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_TARGET)
        {

            if (motor.controller == MotionControlType::velocity)
            {
                target_angle = CANBroker.motorTarget * 10;
            }
            else
            {
                target_angle = CANBroker.motorTarget;
            }

            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_VELOCITY)
        {
            motor.velocity_limit = internalLimiter(CANBroker.motorVelocity, velocityLimitHigh);
            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_CONTROL_MODE)
        {
            if (CANBroker.motorControlMode == 0)
            {
                motor.controller = MotionControlType::torque;
            }
            else if (CANBroker.motorControlMode == 1)
            {
                motor.controller = MotionControlType::velocity;
            }
            else if (CANBroker.motorControlMode == 2)
            {
                motor.controller = MotionControlType::angle;
            }

            Serial.printf("Control mode set to %d\n", CANBroker.motorControlMode);

            CANBroker.ReceivedID = -1;
        }

#if defined ARDUINO_B_G431B_ESC1
        if (digitalRead(BUTTON) == LOW && !buttonPressed)
        {
            buttonPressed = true;
            CANDevice.CANSendInt((int)1, DeviceID, CAN_MESSAGE::BUTTON_PRESSED);
        }
        else if (digitalRead(BUTTON) == HIGH && buttonPressed)
        {
            buttonPressed = false;
            CANDevice.CANSendInt((int)0, DeviceID, CAN_MESSAGE::BUTTON_PRESSED);
        }

#endif
    }

    // Get some statistics on bus errors.
    static int LastTxErrors = 0;
    static int LastRxErrors = 0;
    static int LastOtherErrors = 0;
    static uint32_t LastStatus = 0;
    uint32_t Status = 0;
}


#pragma endregion

void setup()
{
    // Use monitoring with serial
    SimpleFOCDebug::enable();
    Serial.begin(BAUDRATE);

    delay(2000);
    while (!Serial)
        ;

    Wire.setClock(400000);

    // command.verbose = VerboseMode::machine_readable;
    pinMode(BUTTON, INPUT);

    // Initialise magnetic sensor hardware
    sensor.init();

    // Link the motor to the sensor
    motor.linkSensor(&sensor);

    // Driver config
    // Power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;

    driver.init();

    // Link the motor and the driver
    motor.linkDriver(&driver);

    // Newly added
    currentSense.linkDriver(&driver);
    currentSense.init();

    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // Align sensor and start FOC
    // Newly added
    motor.voltage_sensor_align = 3;
    motor.velocity_index_search = 3;

    // Choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Set motion control loop to be used
    motor.controller = MotionControlType::angle;
    // motor.controller = MotionControlType::velocity;
    motor.target = 0;

    // Maximal voltage to be set to the motor
    motor.voltage_limit = driver.voltage_limit * 0.5f;
    motor.current_limit = CURRENT_LIMIT;
    // motor.voltage_limit = driver.voltage_limit;

    motor.PID_velocity.P = 0.066;
    motor.PID_velocity.I = 6.6;
    motor.PID_velocity.D = 0.000135;

    motor.PID_velocity.output_ramp = 300;

    // Velocity low pass filtering time constant
    // the lower the less filtered
    //     motor.LPF_velocity.Tf = 0.01f;

    // Angle P controller
    // motor.P_angle.P = 20;

    // Maximal velocity of the position control
    motor.velocity_limit = 50;

#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1
    CANBusSetup();
#endif

    // Comment out if not needed
    motor.useMonitoring(Serial);

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
    if (motor.motor_status != 4)
    { // 0 - fail initFOC
        Serial.println("ERROR:" + String(motor.motor_status));
        // return;
    }

    motor.enabled = false;

    _delay(500);

    CANBroker.motorReady = true;
    CANDevice.CANSendReady(rxIdentifier);

    printTimeMS = millis();
}

void loop()
{
#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1
    CANBusLoop();
#endif

    float currentMagnitude = currentSense.getDCCurrent();
    float motorElectricalAngle = motor.electricalAngle();
    DQCurrent_s focCurrent = currentSense.getFOCCurrents(motorElectricalAngle);

    if (millis() - printTimeMS > 500)
    {
        vbusValRaw = _readADCVoltageInline(A_VBUS, currentSense.params);
        vbusV = vbusValRaw * 10.7711;

        printTimeMS = millis();
        tempValRaw = _readADCVoltageInline(A_TEMPERATURE, currentSense.params);
        tempDegC = Ntc2TempV(tempValRaw);

        CANDevice.CANSendFloat(tempDegC, DeviceID, CAN_MESSAGE::SEND_TEMP);
        CANDevice.CANSendFloat(currentMagnitude, DeviceID, CAN_MESSAGE::SEND_CURRENT);
    }

    motor.loopFOC();

    motor.move(target_angle);

    current = currentSense.getPhaseCurrents();

    // motor.monitor();

    // user communication
    command.run();
}

#endif
