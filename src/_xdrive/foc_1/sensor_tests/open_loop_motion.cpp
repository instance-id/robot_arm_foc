#if defined(RUN_OPEN_LOOP) && RUN_OPEN_LOOP == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 24

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
#define POLE_PAIRS 20
// #define PHASE_RESISTANCE 7.5
//#define KV
#define CURRENT_LIMIT 0.5

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);

// target variable
float target_position = 0;

// instantiate the commander
Commander command = Commander(Serial);

void doTarget(char *cmd) { command.scalar(&target_position, cmd); }

void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

void doVelocity(char *cmd) { command.scalar(&motor.velocity_limit, cmd); }

void setup()
{
    pinMode(M0_TEMP, INPUT); // M0_TEMP PC5
    pinMode(AUX_TEMP, INPUT); // AUX_TEMP PA5

    command.verbose = VerboseMode::machine_readable;

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;

    driver.voltage_limit = VOLTAGE_LIMIT;
    driver.pwm_frequency = 40000;

    driver.init();

    // link the motor and the driver
    motor.linkDriver(&driver);
    
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
    
    motor.voltage_sensor_align = 3;
    motor.velocity_index_search = 3;

    // limiting motor movements
    // limit the voltage to be set to the motor
    // start very low for high resistance motors
    // currnet = resistance*voltage, so try to be well under 1Amp
    motor.voltage_limit = VOLTAGE_LIMIT; // [V]
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
