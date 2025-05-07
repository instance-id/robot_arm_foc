#if defined(RUN_CAN_COMMANDER) && RUN_CAN_COMMANDER == 1 && defined(ESP_CONTROLLER) && ESP_CONTROLLER == 1

#include <SimpleFOC.h>
#include "SimpleCAN.h"
#include "MT6701_I2C.h"
#include "SimpleFOCDrivers.h"
#include "Wire.h"
#include "can/CANCommander.h"
#include "can/modules/MetricsModule.h"
#include "can/modules/BaseModule.h"
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#define TX_PIN      43
#define RX_PIN      44
#define PIN_CAN0_TX GPIO_NUM_43
#define PIN_CAN0_RX GPIO_NUM_44

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

#define CS 10
#define SPI_MOSI 11
#define SPI_SCK 12
#define SPI_MISO 13

#define PIN_ENABLE 7
#define PIN_IN1 4
#define PIN_IN2 5
#define PIN_IN3 6
#define PHASE_RESISTANCE 5.57
#define KV_RATING 197

BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_ENABLE);

#define SSI_SDA_PIN  13
#define SSI_CLK_PIN  12
#define SSI_CS_PIN   16
#define SSI_MODE_PIN 15

MagneticSensorMT6701SSI sensor1(SSI_CS_PIN);

MT6701_I2CConfig_s mt6701_config = {
    .chip_address = 0b0000110,
    .bit_resolution = 14,
    .angle_register = 0x03,
    .data_start_bit = 8
};

MT6701_Serial_I2C sensor0 = MT6701_Serial_I2C(mt6701_config);

TwoWire I2Cone = TwoWire(0);


byte txIdentifier = 0x001;
byte rxIdentifier = 0x002;
byte allMotorsIdentifier = 0x98;
byte broadcastIdentifier = 0x99;

CanFilter filter = CanFilter(ACCEPT_ALL, rxIdentifier, rxIdentifier, FILTER_ANY_FRAME);

// --| CANBus Init ----------
void canBusInit()
{
    // CAN.logTo(&Serial);
    // CAN.filter(filter);
    CAN.begin(CAN_BITRATE);
}

CANCommander commander = CANCommander();

void setup()
{
    Serial.begin(MONITOR_SPEED);
    while (!Serial);

    SimpleFOCDebug::enable(&Serial);

    pinMode(RX_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);

    canBusInit();

    if (I2Cone.setPins(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("Sensor 1 Pins Set!");
        sensor0.init(&I2Cone);
        I2Cone.setClock(400000);
    } else {
        Serial.println("Sensor 1 Pins Failed!");
        return;
    }

    commander.linkMotor(0x00, &motor);
    commander.monitor(0, BaseModule::MODULES::MODULE_METRICS, MetricsModule::FIELDS::M_METRICS_VELOCITY_ANGLE, 1200);

    // configure i2C
    // SPI_2.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS); //SCLK, MISO, MOSI, SS
    // Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    // Wire.setPins(I2C_SDA, I2C_SCL);
    // Wire.begin();

    // I2Cone.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

    // Initialize magnetic sensor hardware
    // sensor0.init(&I2Cone);
    // Wire.setClock(400000);

    // sensor.init(&SPI_2);

    delay(3000);
    Serial.println("Sensor ready");
}

double start_time = millis();

void loop()
{
    // iterative function updating the sensor internal variables
    // it is usually called in motor.loopFOC()
    // this function reads the sensor hardware and
    // has to be called before getAngle nad getVelocity
    sensor0.update();

    // display the angle and the angular velocity to the terminal

    // float angle = sensor0.getAngle();
    // float precAngle = sensor0.getPreciseAngle();
    // int32_t rot = sensor0.getFullRotations();
    // float mechAngle = sensor0.getMechanicalAngle();
    //
    // // float angle = sensor.getAngle();
    // Serial.printf("Angle: %f PrecAngle: %f Rot: %d MechAngle: %f", angle, precAngle, rot, mechAngle);
    //
    // Serial.print("\t");
    //
    // Serial.println(sensor0.getVelocity());
    //
    // _delay(100);
    //

    if (millis() - start_time > 1200) {
        float angle = sensor0.getAngle();

        commander.sendFloat(0x00, angle);
        // commander.sendFloat(0x01, ub);
        // commander.sendFloat(0x02, uc);
        // commander.sendFloat(0x03, current_magnitude);

        start_time = millis();
    }


    commander.run();
}

#endif
