#if defined(RUN_CLOSED_LOOP) && RUN_CLOSED_LOOP == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 21

// Open loop motor control example
#include <SimpleFOC.h>

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

// GBM5208-75T BLDC motor
#define PHASE_RESISTANCE 7.5
// #define KV

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, CS);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);

// target variable
float target_position = 0;

// instantiate the commander
Commander command = Commander(Serial);

void doMotor(char *cmd) { command.motor(&motor, cmd); }

void doTarget(char *cmd) { command.scalar(&target_position, cmd); }

void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

void setup()
{
    Serial.begin(BAUDRATE);
    SimpleFOCDebug::enable(&Serial);

    delay(2000);
    while (!Serial)
        ;

    pinMode(M0_TEMP, INPUT);  // M0_TEMP PC5
    pinMode(AUX_TEMP, INPUT); // AUX_TEMP

    command.verbose = VerboseMode::machine_readable;

    sensor.init(&SPI_2);

    // Link the motor to the sensor
    motor.linkSensor(&sensor);

    // Driver config
    // Power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT / 2;

    // driver.pwm_frequency = 40000;

    driver.init();

    // Link the motor and the driver
    motor.linkDriver(&driver);

    // Newly added
    currentSense.linkDriver(&driver);
    // ▬▬▬▬▬▬▬▬▬▬▬▬linkCurrentSense▬▬▬▬▬▬▬▬▬▬▬▬
    //  https://docs.simplefoc.com/low_side_current_sense
    if (currentSense.init())
    {
        Serial.println("Current sense init success!");
    }
    else
    {
        Serial.println("Current sense init failed!");
        return;
    }

    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // Align sensor and start FOC
    // Newly added
    // motor.voltage_sensor_align = 3;
    // motor.velocity_index_search = 3;

    // ▬▬▬▬▬▬ Velocity PID Settings ▬▬▬▬▬▬▬▬▬▬▬▬▬
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 5;
    motor.LPF_velocity.Tf = 0.01;
    // motor.PID_velocity.P = 0.066;
    // motor.PID_velocity.I = 6.6;
    // motor.PID_velocity.D = 0.000135;
    // motor.LPF_velocity.Tf = 0.01f;

    motor.PID_velocity.limit = 20;

    motor.voltage_limit = driver.voltage_limit * 0.5f;
    motor.current_limit = CURRENT_LIMIT;

    motor.velocity_limit = 2;

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::angle;

    motor.useMonitoring(Serial);

    // init motor hardware
    motor.init();

    motor.initFOC();

    motor.sensor_offset = motor.shaftAngle();
    motor.monitor_downsample = 1000;

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

    Serial.println("Motor ready!");
    Serial.println("Set target position [rad]");
    _delay(1000);
}

unsigned long prev_time = millis();

void loop()
{
    motor.loopFOC();

    // using motor.voltage_limit and motor.velocity_limit
    // angles can be positive or negative, negative angles correspond to opposite motor direction
    motor.move(target_position);
    motor.monitor();

    if (millis() - prev_time > 500)
    {
        prev_time = millis();
    }

    // user communication
    command.run();
}

#endif
