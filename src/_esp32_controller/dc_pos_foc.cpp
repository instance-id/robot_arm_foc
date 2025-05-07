/**
 * Position control example.
 * This example controls a DC motor via a 2PWM-compatible driver hardware,
 * such as a L298N based driver.
 * The example is using a SC60228 magnetic position sensor to monitor the rotor position.
 *
 * SimpleFOC's position mode is used to move the motor to specific positions (angles)
 * chosen by the user. Positive or negative values determine the direction of
 * rotation. The angle is expressed in radians.
 *
 * SimpleFOC's commander object is used to enable serial control of the
 * desired angle. After connecting the serial console, type "M100" to
 * set the postion to 100 rad, or "M-2.5" to set it to negative 2.5
 * rad.
 * Many other motor parameters can be set via the commander, please see
 * our documentation for details on how to use it.
 */

#if defined(RUN_DC_TEST) && RUN_DC_TEST == 2 && defined(ESP_CONTROLLER) && ESP_CONTROLLER == 1

#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "SimpleDCMotor.h"

#define MOTOR_IN1 9
#define MOTOR_IN2 10
#define I2C_SCL 41
#define I2C_SDA 42

DCMotor motor = DCMotor();
DCDriver2PWM driver = DCDriver2PWM(MOTOR_IN1, MOTOR_IN2);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Commander object, used for serial control
Commander commander = Commander(Serial);

// motor control function - this is needed to link the incoming commands
// to the motor object
void onMotor(char *cmd)
{ commander.motor(&motor, cmd); }

/**
 * Setup function, in which you should intialize sensor, driver and motor,
 * and the serial communications and commander object.
 * Before calling the init() methods of these objects you can set relevant
 * parameters on them.
 */

bool runLoop = false;

void setup()
{

    // to use serial control we have to initialize the serial port
    Serial.begin(115200); // init serial communication
    // wait for serial connection - doesn't work with all hardware setups
    // depending on your application, you may not want to wait
    while (!Serial);
    
     // wait for serial connection
    // enable debug output to the serial port
    SimpleFOCDebug::enable(&Serial);

    Wire.setPins(I2C_SDA, I2C_SCL);

    // basic driver setup - set power supply voltage
    driver.voltage_power_supply = 5;
    // if you want, you can limit the voltage used by the driver.
    // This value has to be same as or lower than the power supply voltage.
    driver.voltage_limit = 5;
    
    // Optionally set the PWM frequency.
    driver.pwm_frequency = 5000;
    
    // init driver
    driver.init();
    
    // init sensor
    sensor.init();

    Wire.setClock(400000);
    
    // pinMode(10, OUTPUT);
    // link driver to motor
    motor.linkDriver(&driver);
    // link sensor to motor
    motor.linkSensor(&sensor);

    // set a voltage limit on the motor, optional. The value set here
    // has to be lower than the power supply voltage.
    // motor.voltage_limit = 6.0f;
    // // motor.velocity_limit = 500.0f;
    // // control type - for this example we use position mode.
    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;
    // init motor
    motor.init();
    // set the PID parameters for velocity control. Velocity PID is the basis also
    // for position mode. If you have not yet tested the velocity mode we strongly
    // suggest you do this first, and find the PID parameters for that mode as
    // the initial values to use here.
    // Please consult our documentation and forums for tips on PID tuning. The values
    // can be different depending on your PSU voltage, the driver, the sensor
    // and the motor used.
    // motor.PID_velocity.P = 0.05f;
    // motor.PID_velocity.I = 0.02f;
    // motor.PID_velocity.D = 0.0f;

    // output ramp limits the rate of change of the velocity, e.g. limits the acceleration.
    // motor.PID_velocity.output_ramp = 200.0f;

    // low pass filter time constant. higher values smooth the velocity measured
    // by the sensor, at the cost of latency and control responsiveness.
    // Generally speaking, the lower this value can be while still producing good
    // response, the better.
    motor.LPF_velocity.Tf = 0.05f;
    motor.LPF_angle.Tf = 0.01;

    // angle P-controller P parameter setting. Normally this can
    // be set to a fairly high value.
    // motor.P_angle.P = 15.0f;

    // set the target velocity to 0, we use the commander to set it later

    motor.sensor_offset = motor.shaftAngle();
    motor.target = 0.0f;



    // enable motor
    motor.enable();

    // add the motor and its control function to the commander
    commander.add('M', onMotor, "dc motor");

    // enable monitoring on Serial port
    motor.useMonitoring(Serial);

    motor.monitor_downsample = 1000;
    Serial.println("Initialization complete.");

    Serial.println("Delay 3 seconds");


    Serial.println("Begin Loop");
}

float timeSpan = 5000;
float totalRunTime = 10000;
float timeStart = 0;
bool hasRun = false;

int downsample = 50;
int downsampleCount = 0;

void loop()
{
    //
    // if (!runLoop)
    //     return;
    //
    // driver.setPwm(-5.0f);
    //
    // // Run the motor for 1 second
    // if (!hasRun) {
    //     timeStart = millis();
    //     hasRun = true;
    // }
    //
    // if (millis() - timeStart > totalRunTime) {
    //     Serial.println("Stopping motor");
    //     driver.setPwm(0.0f);
    //     runLoop = false;
    // } else if (millis() - timeStart > timeSpan) {
    //     driver.setPwm(5.0f);
    // }


    if (downsample > 0 && --downsampleCount <= 0) {

        motor.move();
        downsampleCount = downsample;
    }

    // call motor.move() once per iteration, ideally at a rate of 1kHz or more.
    // rates of more than 10kHz might need a delay, as the sensor may not be able to
    // update quickly enough (depends on sensor)
    // motor.move(); // target position can be set via commander input

    // call commander.run() once per loop iteration, it will process incoming commands
    // commander.run();

    // call motor.monitor() once per loop iteration, it will print the motor state
    // motor.monitor();
}

#endif
