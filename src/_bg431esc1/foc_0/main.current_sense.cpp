#if defined(RUN_MAIN) && RUN_MAIN == 1 && defined(RUN_CURRENT_SENSE) && RUN_CURRENT_SENSE == 1

#include <SimpleFOC.h>
#include <cmath>
#include "SimpleCAN.h"
#include "CANProfile_V1.h"

#define BUTTON PC10
// B-G431B-ESC1 Button

#if defined ARDUINO_B_G431B_ESC1
#define BUTTON PC10
#endif

#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1
// Instantiation of the class which receives messages from the CAN bus.
// This class depends on your application!
CANReceiver CANBroker;

// The actual CAN bus class, which handles all communication.
// You may need to adjust the used pins!
#if defined ARDUINO_PT_SENSOR || defined ARDUINO_B_G431B_ESC1
CANHandler CANDevice(CreateCanLib(A_CAN_TX, A_CAN_RX), &CANBroker);
#endif

int MyDeviceID = 0;
#endif

// Motor instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// pend encoder instance
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// instantiate the commander
Commander command = Commander(Serial);

void onMotor(char *cmd) { command.motor(&motor, cmd); }

float tempValRaw = 0.0;
float tempDegC = 0.0;

float vbusValRaw = 0.0;
float vbusV = 0.0;

float pot_val_V = 0.0;

long printTimeMs = 0;
float velSpRadsPerSec = 0.0;
float target = 0.0;

static float Ntc2TempV(float ADCVoltage) {
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

void setup() {



    pinMode(BUTTON, INPUT);

    // configure i2C
    Wire.setClock(400000);

    // initialise magnetic sensor hardware
    sensor.init();

    // link the motor to the encoder
    motor.linkSensor(&sensor);

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    driver.init();

    // link the motor and the driver
    motor.linkDriver(&driver);

    // link current sense and the driver
    currentSense.linkDriver(&driver);

    currentSense.init();
    // no need for aligning
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // aligning voltage [V]
    motor.voltage_sensor_align = 1;

    // control loop type and torque mode
    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller = MotionControlType::angle;
    motor.motion_downsample = 0.0;

    // velocity loop PID
    motor.PID_velocity.P = 0.4;
    motor.PID_velocity.I = 6.0;
    // Low pass filtering time constant
    motor.LPF_velocity.Tf = 0.008;

    // angle loop PID
    motor.P_angle.P = 15.0;

    // Low pass filtering time constant
    motor.LPF_angle.Tf = 0.005;

    // current q loop PID
    motor.PID_current_q.P = 2.0;
    motor.PID_current_q.I = 250.0;
    // Low pass filtering time constant
    motor.LPF_current_q.Tf = 0.001;

    // current d loop PID
    motor.PID_current_d.P = 2.0;
    motor.PID_current_d.I = 250.0;
    // Low pass filtering time constant
    motor.LPF_current_d.Tf = 0.001;

    // Limits
    motor.velocity_limit = 45.0;
    motor.voltage_limit = 6.2;
    motor.current_limit = 0.55;

    // sensor zero offset - home position
    motor.sensor_offset = 0.0;

    // pwm modulation settings
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.modulation_centered = 1.0;

    // use monitoring with serial
    Serial.begin(115200);

    // comment out if not needed
    motor.useMonitoring(Serial);

    // define the motor id
    command.add('M', onMotor, "motor");

    // downsampling
    motor.monitor_downsample = 10; // default 10
    motor.motion_downsample = 2.0;

#pragma region CANBus Setup
#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1
#if defined ARDUINO_PT_SENSOR
    MyDeviceID = 0x35;
#elif defined ARDUINO_B_G431B_ESC1
    pinMode(BUTTON, INPUT);
    MyDeviceID = 0x21;
#elif defined CONFIG_IDF_TARGET_ESP32S3
    pinMode(RX_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);
    MyDeviceID = 0x22;
#else
    randomSeed(micros());
    MyDeviceID = random(1, 126);
#endif
    //**************************************************************

    CANDevice.Init();

    // Set bus termination on/off (may not be available on all platforms).
    if (CAN_OK != CANDevice.Can1->SetBusTermination(true))
        Serial.println("Setting CAN bus termination via software not possible");

    // Note: Blinking will only work if LED_BUILTIN is defined and the board supports it!
#if defined(GPIO_LED_PIN)
    CANDevice.Can1->EnableBlinkOnActivity(GPIO_LED_PIN);
#endif
#endif
#pragma endregion

    // initialize motor
    motor.init();

    // align encoder and start FOC
    // motor.initFOC(1.46, CCW);
    motor.initFOC();
    // set the inital target value
    motor.target = 0.0;

    _delay(1000);

    printTimeMS = millis();
}

bool SendEnable = true;
bool buttonPressed = false;
bool buttonReleased = false;

void CANBusLoop() {
    static uint32_t LastAction = millis();
    static uint32_t LastFloatAction = millis();
    static uint32_t LastRTR = millis();

    // Test of regular messages:
    // What is sent next to the CAN bus depends on what was received last.
    // When a PING was received, send a PONG and vice versa.
    // To get the whole thing started, a PONG is sent every 5s without having received anything.
    // This is just for testing. Usually you would invoke actions for incomming messages
    // directly in the broker class.
    if (SendEnable) {


        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_ENABLE) {
            motor.enabled = CANBroker.motorEnabled;
            CANBroker.ReceivedID = -1;
        }

        if (CANBroker.ReceivedID == CAN_MESSAGE::SET_TARGET) {
            target = CANBroker.motorTarget;
            Serial.printf("Received Target: d%", target);
            CANBroker.ReceivedID = -1;
        }

#if defined ARDUINO_B_G431B_ESC1
        if (digitalRead(BUTTON) == LOW && !buttonPressed) {
            buttonPressed = true;
            Serial.println("Button Pressed");
            CANDevice.CANSendInt((int) 1, MAKE_CAN_ID(MyDeviceID, CAN_MESSAGE::BUTTON_PRESSED));
        } else if (digitalRead(BUTTON) == HIGH && buttonPressed) {
            buttonPressed = false;
            Serial.println("Button Released");
            CANDevice.CANSendInt((int) 0, MAKE_CAN_ID(MyDeviceID, CAN_MESSAGE::BUTTON_PRESSED));
        }

#endif

#pragma region Response/Interaction
        // if ((CANBroker.ReceivedID == CANID_PP_PING && LastAction + 1000 < millis()) || (LastAction + RandWait + 5000 < millis())) {
        //     Serial.println("Sending Pong");
        //     CANDevice.CANSendText("Pong", MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG));
        //     LastAction = millis();

        //     // Make sure we don't react twice to the same message.
        //     CANBroker.ReceivedID = -1;
        // } else if (CANBroker.ReceivedID == CANID_PP_PONG && LastAction + RandWait + 1000 < millis()) {
        //     Serial.println("Sending Ping");
        //     CANDevice.CANSendText("Ping", MAKE_CAN_ID(MyDeviceID, CANID_PP_PING));
        //     LastAction = millis();

        //     // Make sure we don't react twice to the same message.
        //     CANBroker.ReceivedID = -1;
        // } else
        // if (CANBroker.ReceivedID == CANID_RTRINT && CANBroker.RTR)
        // {
        //     // React to an RTR request message. The reply should be the number "1234". If something else is
        //     // received, check the byte order used by the devices!
        //     CANBroker.RTR = false;
        //     CANDevice.CANSendInt(1234, MAKE_CAN_ID(MyDeviceID, CANID_RTRINT));

        //     // Make sure we don't react twice to the same message.
        //     CANBroker.ReceivedID = -1;
        // }

        // #if defined CONFIG_IDF_TARGET_ESP32S3
        // else if (CANBroker.ReceivedID == CANID_PP_PRESSED
        //          // && CONFIG_IDF_TARGET_ESP32S3
        // )
        // {

        //     Serial.println("ESP32-S3 Received Button Press");

        //     // Make sure we don't react twice to the same message.
        //     CANBroker.ReceivedID = -1;
        // }
        // #endif

        // Every 3s just send a float value. This can be used to check if all devices on
        // the bus use the same floating point number representation and byte order.
        // if (LastFloatAction + RandWait + 3000 < millis())
        // {
        //     float NewVal = CANBroker.receivedFloatVal * 2.5;
        //     if (NewVal == 0)
        //         NewVal = 1.0f;
        //     if (NewVal > 1000000)
        //         NewVal = -1.0;
        //     if (NewVal < -1000000)
        //         NewVal = 1.0;

        //     Serial.printf("Sending: %.3f\n", NewVal);
        //     CANDevice.CANSendFloat(NewVal, MAKE_CAN_ID(MyDeviceID, CANID_FLOAT));
        //     LastFloatAction = millis();
        // }

        // Test of RTR messages
        // Every 5s request an int value. The Response should be the number 1234 in binary form.
        // if (LastRTR + RandWait + 5000 < millis())
        // {
        //     Serial.printf("Request int\n");
        //     CANDevice.CANRequestInt(MyDeviceID);
        //     LastRTR = millis();
        // }
#pragma endregion
    }

    // Get some statistics on bus errors.
    static int LastTxErrors = 0;
    static int LastRxErrors = 0;
    static int LastOtherErrors = 0;
    static uint32_t LastStatus = 0;
    uint32_t Status = 0;
    char StatusStr[MAX_STATUS_STR_LEN] = {0};

    if (LastRTR + 3000 < millis()) {

        CANDevice.Can1->GetStatus(&Status, StatusStr);
        if (CANDevice.Can1->GetTxErrors() != LastTxErrors || CANDevice.Can1->GetRxErrors() != LastRxErrors || CANDevice.Can1->GetOtherErrors() != LastOtherErrors || LastStatus != Status) {
            LastTxErrors = CANDevice.Can1->GetTxErrors();
            LastRxErrors = CANDevice.Can1->GetRxErrors();
            LastOtherErrors = CANDevice.Can1->GetOtherErrors();
            LastStatus = Status;

            Serial.printf("New Status=%s, RxErrors=%d, TxErrors=%d, Other=%d\n", StatusStr, LastTxErrors, LastRxErrors, LastOtherErrors);
        }
        LastRTR = millis();
    }

    delay(5);

    // Update message queues.
    CANDevice.Can1->Loop();
}

void loop() {

#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 1
    CANBusLoop();
#endif

    float currentMagnitude = currentSense.getDCCurrent();
    float motorElectricalAngle = motor.electricalAngle();
    DQCurrent_s current = currentSense.getFOCCurrents(motorElectricalAngle);

    if (millis() - printTimeMS > 100) {

        vbusValRaw = _readADCVoltageInline(A_VBUS, currentSense.params);
        vbusV = vbusValRaw * 10.7711;

        tempValRaw = _readADCVoltageInline(A_TEMPERATURE, currentSense.params);
        tempDegC = Ntc2TempV(tempValRaw);

        pot_val_V = _readADCVoltageInline(A_POTENTIOMETER, currentSense.params);

        printTimeMS = millis();
        Serial.print(vbusV);
        Serial.print('\t');
        Serial.print(tempDegC);
        Serial.print('\t');
        Serial.print(currentMagnitude);
        Serial.print('\t');
        Serial.print(current.q);
        Serial.print('\t');
        Serial.print(current.d);
        Serial.print('\t');
        Serial.print(target);

        // Pot goes from 0 to 3.3
        // Change current limit dynamically
        motor.current_limit = 1.3 * (pot_val_V / 3.3) + 0.10; // amps
        motor.PID_velocity.limit = motor.current_limit;

#if defined(ENABLE_CAN_V1) && ENABLE_CAN_V1 == 0
        target = 40 * (pot_val_V / 3.3);
#endif

        Serial.print('\t');
        Serial.print(motor.current_limit);
        Serial.println("");

        // Change velocity limit dynamically
        // velSpRadsPerSec = pot_val_V * 9.0 + 1.0;
        // motor.velocity_limit = velSpRadsPerSec;
        // motor.P_angle.limit = velSpRadsPerSec;
    }

     if (digitalRead(BUTTON) == HIGH){
       target = 0.0;
     } else {
       target = 6.0 * PI;
     }

    motor.move(target);
    motor.loopFOC();
}

#endif
