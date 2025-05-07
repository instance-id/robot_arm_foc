#if (defined(RUN_MAIN) && RUN_MAIN == 1) && (defined(FOC_CONTROLLER) && FOC_CONTROLLER == 24)

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

#include <SimpleFOC.h>
#include "Wire.h"
#include "SimpleCAN.h"
#include <Arduino.h>
// #include "SPI.h"

#if defined(USE_CAN) && USE_CAN == 1
#if defined(CAN_VERSION) && CAN_VERSION == 1

#include "CANProfile_V1.h"

#elif defined(CAN_VERSION) && CAN_VERSION == 2

#include "CANProfile_V2.h"
#include "Helpers/stringf.h"

// ESP32-S3 TX/RX Pins
#define TX_PIN 43
#define RX_PIN 44
#define PIN_CAN0_TX GPIO_NUM_43
#define PIN_CAN0_RX GPIO_NUM_44

ulong log_print_time = 0;
ulong printTimeMS = millis();
ulong endstop_time_ms = millis();

// --| CANBus Identifiers --------
// --|----------------------------
byte txIdentifier = 0x004;
byte rxIdentifier = 0x001;

// --| CANBus Initialization -----
// --|----------------------------
CANReceiver CANBroker;
CANHandler CANDevice(&CAN, &CANBroker, txIdentifier);

#endif
#endif

#define PIN_IN1 4
#define PIN_IN2 5
#define PIN_IN3 6
#define PIN_ENABLE 7

#define CURRENT_SENSE_1 15
#define CURRENT_SENSE_2 16

#define PHASE_RESISTANCE 7.5

#if defined(ENCODER_TYPE) && ENCODER_TYPE == 0
#include "SimpleFOCDrivers.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#define SSI_SDA_PIN 13
#define SSI_CLK_PIN 12
#define SSI_CS_PIN 16
#define SSI_MODE_PIN 15

MagneticSensorMT6701SSI sensor1(SSI_CS_PIN);
#elif defined(ENCODER_TYPE) && ENCODER_TYPE == 1
#include "MT6701_I2C.h"
#define I2C_SCL_PIN 10
#define I2C_SDA_PIN 12

TwoWire I2Cone = TwoWire(0);

MT6701_I2CConfig_s mt6701_config = {
    .chip_address = 0b0000110,
    .bit_resolution = 14,
    .angle_register = 0x03,
    .data_start_bit = 8};
MT6701_Serial_I2C sensor = MT6701_Serial_I2C(mt6701_config);

#elif defined(ENCODER_TYPE) && ENCODER_TYPE == 2

TwoWire I2Cone = TwoWire(0);
#define I2C_SCL_PIN 9
#define I2C_SDA_PIN 8
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

#endif

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);

InlineCurrentSense current_sense1 = InlineCurrentSense(0.01f, 50.0f, CURRENT_SENSE_1, CURRENT_SENSE_2);

// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Angle set point variable
float target_value = 0.0f;
float lastTarget = 0.0f;

// Instantiate the commander
Commander command = Commander(Serial);

void doMotor(char *cmd) { command.motor(&motor, cmd); }
void doTarget(char *cmd) { command.scalar(&target_value, cmd); }
void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

// --| Setup Functions -----------
// --|----------------------------

// --| Pin Setup ------------
void pinSetup()
{
    pinMode(RX_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);

    // vQueueAddToRegistry();
}

CanFilter filter = CanFilter(ACCEPT_ALL, rxIdentifier, rxIdentifier, FILTER_ANY_FRAME);

// --| CANBus Init ----------
void canBusInit()
{
    CAN.logTo(&Serial);
    CAN.filter(filter);
    CAN.begin(CAN_BITRATE);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    delay(2000);

    SimpleFOCDebug::enable(&Serial);

    // canBusInit();

#if defined(ENCODER_TYPE) && ENCODER_TYPE == 0
    SPI.setFrequency(100000);
    sensor1.init(&SPI);
#elif defined(ENCODER_TYPE) && ENCODER_TYPE == 1
    if (I2Cone.setPins(I2C_SDA_PIN, I2C_SCL_PIN))
    {
        Serial.println("Sensor 2 Pins Set!");
        sensor.init(&I2Cone);
        I2Cone.setClock(400000);
    }
    else
    {
        Serial.println("Sensor 2 Pins Failed!");
        return;
    }
#elif defined(ENCODER_TYPE) && ENCODER_TYPE == 2

    if (I2Cone.setPins(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("Sensor 2 Pins Set!");
        sensor.init(&I2Cone);
        I2Cone.setClock(400000);
    } else {
        Serial.println("Sensor 2 Pins Failed!");
        return;
    }

#endif

    // Link the motor to the sensor
    motor.linkSensor(&sensor);

    // Driver config
    // Power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;

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
    motor.current_limit = CURRENT_LIMIT - 0.6f;

    // Controller configuration
    // Default parameters in defaults.h
    // Velocity PI controller parameters
    motor.PID_velocity.P = 0.066;
    motor.PID_velocity.I = 6.6;
    motor.PID_velocity.D = 0.000135;

    motor.PID_velocity.output_ramp = 300;

    // Velocity low pass filtering time constant
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.01f;

    // Angle P controller
    motor.P_angle.P = 20;
    motor.LPF_angle.Tf = 0.001;

    // Maximal velocity of the position control
    motor.velocity_limit = 3;

    // motor.zero_electric_angle = 0.87;
    // motor.sensor_direction = CW;

    motor.sensor_offset = motor.shaftAngle();

    // Comment out if not needed
    motor.useMonitoring(Serial);

    // Initialize motor
    motor.init();

    // Align sensor and start FOC
    motor.initFOC();

    motor.monitor_downsample = 1000;

    // Add target command T
    command.add('M', doMotor, "motor");
    command.add('T', doTarget, "target angle");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doLimit, "movement velocity");

    Serial.printf("Driver Stats: voltage_power_supply %f voltage_limit %f pwm_frequency %ld\n",
                  driver.voltage_power_supply, driver.voltage_limit, driver.pwm_frequency);

    Serial.printf("Motor Stats: voltage_limit %f current_limit %f velocity_limit: %f\n",
                  motor.voltage_limit, motor.current_limit, motor.velocity_limit);

    Serial.printf("Motor Angle: %f Sensor Angle: %f Offset: %f\n", motor.shaftAngle(), sensor.getAngle(), motor.sensor_offset);

    _delay(500);

    CANBroker.motorReady = true;
    CANDevice.CANSendReady(rxIdentifier);

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target angle using serial terminal:"));
}

float motorAngle = 0;
float angleValue = 0;
float precAngle = 0;
int32_t rot = 0;
float mechAngle = 0;

void setTargetValue(float target)
{
    target_value = target;
}

// --| CANBus Loop ---------------
// --|----------------------------
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
            setTargetValue(CANBroker.motorTarget);
            CANBroker.ReceivedID = -1;
        }
    }
}

// --| Main Loop -----------------
// --|----------------------------
void loop()
{
    // CANBusLoop();

    // main FOC algorithm function
    // the faster you run this function the better
    // Arduino UNO loop  ~1kHz
    // Bluepill loop ~10kHz
    motor.loopFOC();

    // Motion control function
    // velocity, position or voltage (defined in motor.controller)
    // this function can be run at much lower frequency than loopFOC() function
    // You can also use motor.move() and set the motor.target in the code
    motor.move(target_value);

    // function intended to be used with serial plotter to monitor motor variables
    // significantly slowing the execution down!!!!

    if (printDebug && millis() - printTimeMS > 500) {
        motorAngle = motor.shaftAngle();
        angleValue = sensor.getAngle();
        rot = sensor.getFullRotations();
        precAngle = sensor.getPreciseAngle();
        mechAngle = sensor.getMechanicalAngle();

        // float angle = sensor.getAngle();
        Serial.printf("T: %f MAngle: %f SAngle: %f PrecAngle: %f Rot: %d MechAngle: %f", target_value, motorAngle, angleValue, precAngle, rot, mechAngle);
        Serial.print("\t");
        Serial.println(sensor.getVelocity());

        printTimeMS = millis();
    }

    // user communication
    command.run();
}
#endif
