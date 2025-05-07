#if defined(RUN_SENSOR) && RUN_SENSOR == 2 && defined(ESP_CONTROLLER) && ESP_CONTROLLER == 1

#include <SimpleFOC.h>
#include "Wire.h"

#include "MT6701_I2C.h"

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
//  chip_address  I2C chip address
//  bit_resolution  resolution of the sensor
//  angle_register_msb  angle read register msb
//  bits_used_msb  number of used bits in msb register
//
// make sure to read the chip address and the chip angle register msb value from the datasheet
// also in most cases you will need external pull-ups on SDA and SCL lines!!!!!
//
// For AS5058B
// MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);


// #define I2C_SCL_PIN 12
// #define I2C_SDA_PIN 13
#define I2C_SCL_PIN 10
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN_2 9
#define I2C_SDA_PIN_2 8

#define CS 10
#define SPI_MOSI 11
#define SPI_SCK 12
#define SPI_MISO 13


MT6701_I2CConfig_s mt6701_config = {
    .chip_address = 0b0000110,
    .bit_resolution = 14,
    .angle_register = 0x03,
    .data_start_bit = 8
};

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MT6701_Serial_I2C sensor2 = MT6701_Serial_I2C(mt6701_config);

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

void setup()
{
    Serial.begin(MONITOR_SPEED);
    while (!Serial);

    SimpleFOCDebug::enable(&Serial);


    // configure i2C

    // SPI_2.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS); //SCLK, MISO, MOSI, SS

    // Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    // Wire1.setPins(I2C_SDA_PIN_2, I2C_SCL_PIN_2);
    // Wire.setPins(I2C_SDA, I2C_SCL);
    // Wire.begin();
    // Wire1.begin();

    I2Cone.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    I2Ctwo.begin(I2C_SDA_PIN_2, I2C_SCL_PIN_2, 400000);

    // Initialize magnetic sensor hardware
    sensor.init(&I2Cone);
    sensor2.init(&I2Ctwo);
    // Wire.setClock(400000);
    // Wire1.setClock(400000);

    // sensor.init(&Wire);
    // sensor2.init(&Wire1);

    delay(3000);
    Serial.println("Sensor ready");
}

void loop()
{
    // iterative function updating the sensor internal variables
    // it is usually called in motor.loopFOC()
    // this function reads the sensor hardware and
    // has to be called before getAngle nad getVelocity
    sensor.update();
    sensor2.update();

    // display the angle and the angular velocity to the terminal

    float angle = sensor.getAngle();
    float precAngle = sensor.getPreciseAngle();
    int32_t rot = sensor.getFullRotations();
    float mechAngle = sensor.getMechanicalAngle();

    float angle2 = sensor2.getAngle();
    float precAngle2 = sensor2.getPreciseAngle();
    int32_t rot2 = sensor2.getFullRotations();
    float mechAngle2 = sensor2.getMechanicalAngle();

    // float angle = sensor.getAngle();
    // Serial.printf("Angle: %f PrecAngle: %f Rot: %d MechAngle: %f Angle2: %f PrecAngle2: %f Rot2: %d MechAngle2: %f", angle, precAngle, rot, mechAngle, angle2, precAngle2, rot2, mechAngle2);


    Serial.printf("Angle: %f Angle2: %f\n", angle, angle2);

    Serial.print("\t");

    // Serial.println(sensor.getVelocity());

    _delay(100);
}

#endif
