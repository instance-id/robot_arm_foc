#if defined(RUN_OPEN_LOOP) && RUN_OPEN_LOOP == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 24

// Open loop motor control example
#include <SimpleFOC.h>

#define CS PA15       // PA15 // GPIO 7 ==> PA15 ==> SPI(AS5047)
#define SPI_MISO PC11 //
#define SPI_MOSI PC12 //
#define SPI0SCK PC10  // PC10

// BLDC motor & driver instance
#define I2C_SCL_PIN 10
#define I2C_SDA_PIN 12

#define PIN_ENABLE 13
#define PIN_IN1 4
#define PIN_IN2 5
#define PIN_IN3 6
#define PHASE_RESISTANCE 7.5

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);
// target variable
float target_position = 0;

// instantiate the commander
Commander command = Commander(Serial);

void doTarget(char *cmd) { command.scalar(&target_position, cmd); }

void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    SimpleFOCDebug::enable(&Serial);

    command.verbose = VerboseMode::machine_readable;

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;

    driver.voltage_limit = VOLTAGE_LIMIT;
    driver.pwm_frequency = 40000;

    if (driver.init())
        Serial.println("Driver init success!");
    else
    {
        Serial.println("Driver init failed!");
        return;
    }

    // link the motor and the driver
    motor.linkDriver(&driver);

    // motor.voltage_sensor_align = 3;
    // motor.velocity_index_search = 3;

    // limiting motor movements
    // limit the voltage to be set to the motor
    // start very low for high resistance motors
    // currnet = resistance*voltage, so try to be well under 1Amp
    motor.voltage_limit = VOLTAGE_LIMIT * 0.5; // [V]
    motor.current_limit = CURRENT_LIMIT - 0.8;
    // motor.voltage_sensor_align = 3;
    // limit/set the velocity of the transition in between
    // target angles
    motor.velocity_limit = 5; // [rad/s] cca 50rpm
    // open loop control config
    motor.controller = MotionControlType::angle_openloop;

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

    // init motor hardware
    if (motor.init())
        Serial.println("Motor init success!");
    else
    {
        Serial.println("Motor init failed!");
        return;
    }

    // add target command T
    command.add('T', doTarget, "target angle");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doLimit, "movement velocity");

    target_position = 3;

    Serial.println("Motor ready!");
    Serial.println("Set target position [rad]");
    _delay(1000);
}

void loop()
{
    // open  loop angle movements
    // using motor.voltage_limit and motor.velocity_limit
    // angles can be positive or negative, negative angles correspond to opposite motor direction
    motor.move(target_position);

    // user communication
    command.run();
}

#endif
