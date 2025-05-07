#if defined(RUN_MAIN) && RUN_MAIN == 3

#include <SimpleFOC.h>

#define PHASE_RESISTANCE 7.5
// #define KV


#define CS PA15       // PA15 // GPIO 7 ==> PA15 ==> SPI(AS5047)
#define SPI_MISO PC11 //
#define SPI_MOSI PC12 //
#define SPI0SCK PC10  // PC10

// SHUNT SENSING
#define M0_IA _NC // Seulement 2 mesures de courant B&C, A = pas dispo lui.
#define M0_IB PC0
#define M0_IC PC1

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15
// M1 & M2 common enable pin
#define EN_GATE PB12

// Temp
#define M0_TEMP PC5
#define AUX_TEMP PA5

#define CAN_R PB8
#define CAN_D PB9

float target_value = 0;
long printTimeMS = 0;
float vbusV = 0.0;
float tempDegC = 0.0;
float vbusValRaw = 0.0;
float tempValRaw = 0.0;
float velSpRadsPerSec = 0.0;

// --| Use CANBus ----------------
// --|----------------------------
#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1

#include <cmath>
#include "SimpleCAN.h"
#include "CANProfile_V1.h"

// Instantiation of the class which receives messages from the CAN bus.
// This class depends on your application!
CANReceiver CANBroker;

// The actual CAN bus class, which handles all communication.
// You may need to adjust the used pins!
// #if defined ARDUINO_PT_SENSOR || defined ARDUINO_B_G431B_ESC1
// CANHandler CANDevice(CreateCanLib(A_CAN_TX, A_CAN_RX), &CANBroker);
CANHandler CANDevice(CreateCanLib((uint32_t)PB9, (uint32_t)PB8), &CANBroker);
// #endif
int DeviceID = 0x21;
#endif

BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);
MagneticSensorSPI sensor = MagneticSensorSPI(CS, 14, 0x3FFF);
 // // alternative constructor (chipselsect, bit_resolution, angle_read_register, )
SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);

// Make Variable Global
// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
// BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
// LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
PhaseCurrent_s current;
Commander command = Commander(Serial);

void doMotor(char *cmd) { command.motor(&motor, cmd); }

void doTarget(char *cmd) { command.scalar(&target_value, cmd); }

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

#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1

void CANBusSetup()
{
#if defined ARDUINO_PT_SENSOR
    DeviceID = 0x35;
#elif defined ARDUINO_B_G431B_ESC1
#define BUTTON PC10
    pinMode(BUTTON, INPUT);
    DeviceID = 0x21;
#elif defined CONFIG_IDF_TARGET_ESP32S3
    pinMode(RX_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);
    DeviceID = 0x22;
#else
    randomSeed(micros());
    DeviceID = random(1, 126);
#endif
    //**************************************************************

    CANDevice.Init();

    // Set bus termination on/off (may not be available on all platforms).
    if (CAN_OK != CANDevice.Can1->SetBusTermination(true))
        Serial.println("Setting CAN bus termination via software not possible");

        // Note: Blinking will only work if LED_BUILTIN is defined and the board supports it!
#if defined(LED_BUILTIN)
    CANDevice.Can1->EnableBlinkOnActivity(LED_BUILTIN);
#endif
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
    if (SendEnable)
    {
        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_ENABLE)
        {
            motor.enabled = CANBroker.motorEnabled;
            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_TARGET)
        {

            if (motor.controller == MotionControlType::velocity)
            {
                target_value = CANBroker.motorTarget * 10;
            }
            else
            {
                target_value = CANBroker.motorTarget;
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
            CANDevice.CANSendInt((int)1, MAKE_CAN_ID(DeviceID, CAN_MESSAGE::BUTTON_PRESSED));
        }
        else if (digitalRead(BUTTON) == HIGH && buttonPressed)
        {
            buttonPressed = false;
            CANDevice.CANSendInt((int)0, MAKE_CAN_ID(DeviceID, CAN_MESSAGE::BUTTON_PRESSED));
        }

#endif
    }

    // Get some statistics on bus errors.
    static int LastTxErrors = 0;
    static int LastRxErrors = 0;
    static int LastOtherErrors = 0;
    static uint32_t LastStatus = 0;
    uint32_t Status = 0;
    char StatusStr[MAX_STATUS_STR_LEN] = {0};

    if (LastRTR + 3000 < millis())
    {

        CANDevice.Can1->GetStatus(&Status, StatusStr);
        if (CANDevice.Can1->GetTxErrors() != LastTxErrors || CANDevice.Can1->GetRxErrors() != LastRxErrors || CANDevice.Can1->GetOtherErrors() != LastOtherErrors || LastStatus != Status)
        {
            LastTxErrors = CANDevice.Can1->GetTxErrors();
            LastRxErrors = CANDevice.Can1->GetRxErrors();
            LastOtherErrors = CANDevice.Can1->GetOtherErrors();
            LastStatus = Status;

            Serial.printf("New Status=%s, RxErrors=%d, TxErrors=%d, Other=%d\n", StatusStr, LastTxErrors, LastRxErrors, LastOtherErrors);
        }
        LastRTR = millis();
    }

    CANDevice.Can1->Loop();
}

#endif

#pragma endregion

#define CURRENT_LIMIT 1.5

void setup()
{
    // Use monitoring with serial
    SimpleFOCDebug::enable();
    Serial.begin(BAUDRATE);

    delay(2000);
    while (!Serial)
        ;

    pinMode(M0_TEMP, INPUT_PULLUP);  // M0_TEMP PC5
    pinMode(AUX_TEMP, INPUT_PULLUP); // AUX_TEMP PA5

    // Wire.setClock(400000);

    // command.verbose = VerboseMode::machine_readable;
    // pinMode(BUTTON, INPUT);

    // Initialise magnetic sensor hardware
    sensor.init(&SPI_2);
    // sensor.init();

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
    // motor.controller = MotionControlType::angle;
    motor.controller = MotionControlType::velocity;
    motor.target = target_value = 1;

    // Maximal voltage to be set to the motor
    motor.voltage_limit = driver.voltage_limit * 0.5f;
    motor.current_limit = CURRENT_LIMIT;
    // motor.voltage_limit = driver.voltage_limit;

    // Contoller configuration
    // Default parameters in defaults.h
    // Velocity PI controller parameters
    // motor.PID_velocity.P = 0.2f;
    // motor.PID_velocity.I = 20;
    // motor.PID_velocity.D = 0;

    motor.PID_velocity.P = 0.066;
    motor.PID_velocity.I = 6.6;
    motor.PID_velocity.D = 0.000135;

    motor.LPF_velocity = 0.03;

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

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target angle using serial terminal:"));
    _delay(1000);

    printTimeMS = millis();
}

void loop()
{
#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1
    CANBusLoop();
#endif

    //    if (millis() - printTimeMS > 100) {
    //        printTimeMS = millis();
    //        Serial.print('\t');
    //        Serial.print(motor.current_limit);
    //        Serial.println("");
    //
    //    }

    float currentMagnitude = currentSense.getDCCurrent();
    float motorElectricalAngle = motor.electricalAngle();
    DQCurrent_s focCurrent = currentSense.getFOCCurrents(motor_electrical_angle);

    // Raise target_value from 1 to 40 over 20 seconds (rounded to the nearest whole number), then lower it from 40 to 1 over 20 seconds and repeat this cycle every 40 seconds
    target_value = 20 + 20 * sin(PI * millis() / 20000);
    if (target_value < 1)
    {
        target_value = 1;
    }
    else if (target_value > 40)
    {
        target_value = 40;
    }

    //     if (millis() - printTimeMS > 500)
    //     {
    //         // vbusValRaw = _readADCVoltageInline(A_VBUS, currentSense.params);
    //         // vbusV = vbusValRaw * 10.7711;

    //         printTimeMS = millis();
    //         tempValRaw = _readADCVoltageInline(M0_TEMP, currentSense.params);
    //         // tempValRaw = _readADCVoltageInline(A_TEMPERATURE, currentSense.params);
    //         tempDegC = Ntc2TempV(tempValRaw);

    //         // Serial.print("VBUS: ");
    //         // Serial.print(vbusV);
    //         // Serial.print("Temperature: ");
    //         // Serial.print(tempDegC);
    //         // Serial.print("Current: ");
    //         // Serial.print(currentMagnitude);
    //         // Serial.print("Current A: ");
    //         // Serial.print(current.q);
    //         // Serial.print("Current B: ");
    //         // Serial.print(current.d);
    //         // Serial.print("Target: ");
    //         // Serial.print(target_value);

    //         // Combined Message String
    //         // "VBUS: " + vbusV + "Temperature: " + tempDegC + "Current: " + currentMagnitude + "Current A: " + current.q + "Current B: " + current.d + "Target: " + target_value
    //         // String message = "VBUS: " + String(vbusV) + " Temperature: " + String(tempDegC) + " Current: " + String(currentMagnitude) + " Current A: " + String(focCurrent.q) + " Current B: " + String(focCurrent.d) + " Target: " + String(target_value);
    //         // Serial.println(message);
    // #if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1
    //         CANDevice.CANSendFloat(tempDegC, MAKE_CAN_ID(DeviceID, CAN_MESSAGE::SEND_TEMP));
    // #endif
    //         // CANDevice.CANSendFloat(currentMagnitude, MAKE_CAN_ID(DeviceID, CAN_MESSAGE::SEND_CURRENT));
    //     }

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

    // current = currentSense.getPhaseCurrents();

    // function intended to be used with serial plotter to monitor motor variables
    // significantly slowing the execution down!!!!
    // motor.monitor();

    // user communication
    command.run();
}

#endif
