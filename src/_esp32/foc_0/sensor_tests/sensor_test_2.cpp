#if defined(RUN_SENSOR) && RUN_SENSOR == 2 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 20

#include <SimpleFOC.h>
#include "MT6701_I2C.h"
#include "Wire.h"

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
// #define I2C_SCL_PIN 10
// #define I2C_SDA_PIN 12


#define I2C_SCL_PIN 9
#define I2C_SDA_PIN 8

// #define I2C_SCL 	41
// #define I2C_SDA 	42

#define CS 10
#define SPI_MOSI 11
#define SPI_SCK 12
#define SPI_MISO 13

// Example of AS5600 configuration
//  MagneticSensorSPI sensor = MagdgneticSensorSPI(TLE, 10);
// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

MT6701_I2CConfig_s mt6701_config = {
    .chip_address = 0b0000110,
    .bit_resolution = 14,
    .angle_register = 0x03,
    .data_start_bit = 8};
MT6701_Serial_I2C mt6701 = MT6701_Serial_I2C(mt6701_config);



// MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, CS);
// SPIClass SPI_2 = SPIClass(FSPI);

TwoWire I2Cone = TwoWire(0);

void setup()
{
    Serial.begin(MONITOR_SPEED);
    while (!Serial);

    SimpleFOCDebug::enable(&Serial);
    // configure i2C
    // SPI_2.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS); //SCLK, MISO, MOSI, SS
    // Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    // Wire.setPins(I2C_SDA, I2C_SCL);
    // Wire.begin();

    I2Cone.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

    // Initialize magnetic sensor hardware
    mt6701.init(&I2Cone);
    Wire.setClock(400000);

    // sensor.init(&SPI_2);

    delay(3000);
    Serial.println("Sensor ready");
}

void loop()
{
    // iterative function updating the sensor internal variables
    // it is usually called in motor.loopFOC()
    // this function reads the sensor hardware and
    // has to be called before getAngle nad getVelocity
    mt6701.update();

    // display the angle and the angular velocity to the terminal

    float angle = mt6701.getAngle();
    float precAngle = mt6701.getPreciseAngle();
    int32_t rot = mt6701.getFullRotations();
    float mechAngle = mt6701.getMechanicalAngle();

    // float angle = sensor.getAngle();
    Serial.printf("Angle: %f PrecAngle: %f Rot: %d MechAngle: %f", angle, precAngle, rot, mechAngle);

    Serial.print("\t");

    Serial.println(mt6701.getVelocity());

    _delay(100);
}

#endif
