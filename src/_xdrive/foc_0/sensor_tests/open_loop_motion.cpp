#if defined(RUN_OPEN_LOOP) && RUN_OPEN_LOOP == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 21

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

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

// target variable
float target_position = 0;

// instantiate the commander
Commander command = Commander(Serial);

void doTarget(char *cmd) { command.scalar(&target_position, cmd); }

void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

void setup()
{
    command.verbose = VerboseMode::machine_readable;

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;
    driver.pwm_frequency = 40000;
    // limit the maximal dc voltage the driver can set
    // as a protection measure for the low-resistance motors
    // this value is fixed on startup
    // driver.voltage_limit = VOLTAGE_LIMIT;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);

    // limiting motor movements
    // limit the voltage to be set to the motor
    // start very low for high resistance motors
    // currnet = resistance*voltage, so try to be well under 1Amp
    motor.voltage_limit = VOLTAGE_LIMIT * 0.8f; // [V]
    motor.current_limit = CURRENT_LIMIT;
    motor.voltage_sensor_align = 3;
    // limit/set the velocity of the transition in between
    // target angles
    motor.velocity_limit = 2; // [rad/s] cca 50rpm
    // open loop control config
    motor.controller = MotionControlType::angle_openloop;

    // init motor hardware
    motor.init();

    _delay(3000);

    Serial.printf("Driver Stats: voltage_power_supply %f voltage_limit %f pwm_frequency %d\n",
                  driver.voltage_power_supply, driver.voltage_limit, driver.pwm_frequency);

    Serial.printf("Motor Stats: voltage_limit %f current_limit %f velocity_limit: %f\n",
                  motor.voltage_limit, motor.current_limit, motor.velocity_limit);

    Serial.printf("Motor Angle: %f Offset: %f\n", motor.shaftAngle(), motor.sensor_offset);


    // add target command T
    command.add('T', doTarget, "target angle");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doLimit, "movement velocity");

    Serial.begin(115200);
    Serial.println("Motor ready!");
    Serial.println("Set target position [rad]");
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
