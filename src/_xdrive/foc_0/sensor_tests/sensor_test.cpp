#if defined(RUN_SENSOR) && RUN_SENSOR == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 21

#include <SimpleFOC.h>
#include "AS5047P.h"

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

// Example of AS5600 configuration
// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
#define CS PA15       // PA15 // GPIO 7 ==> PA15 ==> SPI(AS5047)
#define SPI_MISO PC11 //
#define SPI_MOSI PC12 //
#define SPI0SCK PC10  // PC10

//MagneticSensorSPI sensor = MagneticSensorSPI(CS, 14, 0x3FFF);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, CS);

SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);

void setup() {
    // monitoring port
    Serial.begin(MONITOR_SPEED);
    // Serial.begin(115200);

    SimpleFOCDebug::enable(&Serial);

    delay(5000);

    // configure i2C
    // Wire.setClock(400000);

    sensor.clock_speed = 20000;

    // initialise magnetic sensor hardware
    // sensor.init();
    sensor.init(&SPI_2);

    Serial.println("Sensor ready");
    _delay(1000);
}

void loop() {
    // iterative function updating the sensor internal variables
    // it is usually called in motor.loopFOC()
    // this function reads the sensor hardware and
    // has to be called before getAngle nad getVelocity
    sensor.update();

    // display the angle and the angular velocity to the terminal

    float angle = sensor.getAngle();
    float precAngle = sensor.getPreciseAngle();
    int32_t rot = sensor.getFullRotations();
    float mechAngle = sensor.getMechanicalAngle();

    // float angle = sensor.getAngle();
    Serial.printf("Angle: %f PrecAngle: %f Rot: %d MechAngle: %f", angle, precAngle, rot, mechAngle);

    Serial.print("\t");

    Serial.println(sensor.getVelocity());

    _delay(100);
}

#endif
