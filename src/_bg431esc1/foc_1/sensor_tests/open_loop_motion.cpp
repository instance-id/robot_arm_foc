#if defined(RUN_OPEN_LOOP) && RUN_OPEN_LOOP == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 23

// Open loop motor control example
#include <SimpleFOC.h>

// GBM5208-75T BLDC motor
#define PHASE_RESISTANCE 7.5
// #define KV

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// target variable
float target_position = 0;

LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
PhaseCurrent_s current;

// instantiate the commander
Commander command = Commander(Serial);

long printTimeMS = 0;
float vbusV = 0.0;
float tempDegC = 0.0;
float vbusValRaw = 0.0;
float tempValRaw = 0.0;

void doMotor(char *cmd) { command.motor(&motor, cmd); }
void doTarget(char *cmd) { command.scalar(&target_position, cmd); }
void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

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

void setup()
{

    Serial.begin(115200);
    while (!Serial)
        ;

    delay(2000);

    SimpleFOCDebug::enable(&Serial);

    command.verbose = VerboseMode::machine_readable;

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    // limit the maximal dc voltage the driver can set
    // as a protection measure for the low-resistance motors
    // this value is fixed on startup
    // driver.voltage_limit = VOLTAGE_LIMIT;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);

    // Newly added
    currentSense.linkDriver(&driver);
    currentSense.init();

    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);


    // limiting motor movements
    // limit the voltage to be set to the motor
    // start very low for high resistance motors
    // currnet = resistance*voltage, so try to be well under 1Amp
    motor.voltage_limit = VOLTAGE_LIMIT * 0.5f; // [V]
    motor.current_limit = CURRENT_LIMIT;

    motor.voltage_sensor_align = 3;
    motor.velocity_index_search = 3;

    // limit/set the velocity of the transition in between
    // target angles
    motor.velocity_limit = 2; // [rad/s] cca 50rpm
    // open loop control config
    motor.controller = MotionControlType::angle_openloop;

    // init motor hardware
    motor.init();

    // add target command T
    command.add('M', doMotor, "motor");
    command.add('T', doTarget, "target angle");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doLimit, "movement velocity");

    if (motor.motor_status != 4)
    { // 0 - fail initFOC
        Serial.println("ERROR:" + String(motor.motor_status));
        // return;
    }

    motor.useMonitoring(Serial);
    motor.monitor_downsample = 10000;

    Serial.println("Motor ready!");
    Serial.println("Set target position [rad]");
    _delay(1000);

    target_position = 0;
}

void loop()
{

    float currentMagnitude = currentSense.getDCCurrent();
    float motorElectricalAngle = motor.electricalAngle();
    DQCurrent_s focCurrent = currentSense.getFOCCurrents(motorElectricalAngle);

    float motorAngle = motor.shaftAngle();

    if (millis() - printTimeMS > 500)
    {
        vbusValRaw = _readADCVoltageInline(A_VBUS, currentSense.params);
        vbusV = vbusValRaw * 10.7711;

        printTimeMS = millis();
        tempValRaw = _readADCVoltageInline(A_TEMPERATURE, currentSense.params);
        tempDegC = Ntc2TempV(tempValRaw);

        // CANDevice.CANSendFloat(tempDegC, DeviceID, CAN_MESSAGE::SEND_TEMP);
        // CANDevice.CANSendFloat(currentMagnitude, DeviceID, CAN_MESSAGE::SEND_CURRENT);

        String msg = "Angle: " + String(motorAngle) + ", " + "Vbus: " + String(vbusV) + "V, Temp: " + String(tempDegC) + "°C" + ", Current: " + String(currentMagnitude) + "A";
        Serial.println(msg);
    }

    // open  loop angle movements
    // using motor.voltage_limit and motor.velocity_limit
    // angles can be positive or negative, negative angles correspond to opposite motor direction
    motor.move(target_position);

    // motor.monitor();

    current = currentSense.getPhaseCurrents();

    // user communication
    command.run();
}

#endif
