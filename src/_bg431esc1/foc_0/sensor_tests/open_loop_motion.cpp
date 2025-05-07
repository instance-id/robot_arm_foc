#if defined(RUN_OPEN_LOOP) && RUN_OPEN_LOOP == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 22

// Open loop motor control example
#include <SimpleFOC.h>

// GBM5208-75T BLDC motor
#define POLE_PAIRS 11
#define PHASE_RESISTANCE 7.5
//#define KV
#define CURRENT_LIMIT 1.5

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// target variable
float target_position = 0;

// instantiate the commander
Commander command = Commander(Serial);

void doTarget(char *cmd) { command.scalar(&target_position, cmd); }

void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

void setup() {

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

    // limiting motor movements
    // limit the voltage to be set to the motor
    // start very low for high resistance motors
    // currnet = resistance*voltage, so try to be well under 1Amp
    motor.voltage_limit = VOLTAGE_LIMIT * 0.5f; // [V]
    motor.current_limit = CURRENT_LIMIT;
    motor.voltage_sensor_align = 3;
    // limit/set the velocity of the transition in between
    // target angles
    motor.velocity_limit = 5; // [rad/s] cca 50rpm
    // open loop control config
    motor.controller = MotionControlType::angle_openloop;

    // init motor hardware
    motor.init();

    // add target command T
    command.add('T', doTarget, "target angle");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doLimit, "movement velocity");

    Serial.begin(115200);
    Serial.println("Motor ready!");
    Serial.println("Set target position [rad]");
    _delay(1000);
}

void loop() {
    // open  loop angle movements
    // using motor.voltage_limit and motor.velocity_limit
    // angles can be positive or negative, negative angles correspond to opposite motor direction
    motor.move(target_position);

    // user communication
    command.run();
}

#endif
