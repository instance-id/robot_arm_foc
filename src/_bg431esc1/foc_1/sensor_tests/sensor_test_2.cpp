#if defined(RUN_SENSOR) && RUN_SENSOR == 2 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 23

#include <Arduino.h>
#include <SimpleFOC.h>
#include "Wire.h"
#include "MT6701_I2C.h"
// #include "SimpleFOCDrivers.h"
// #include <encoders/mt6701/MagneticSensorMT6701SSI.h>

TwoWire I2Cone = TwoWire();

MT6701_I2CConfig_s mt6701_config = {
    .chip_address = 0b0000110,
    .bit_resolution = 14,
    .angle_register = 0x03,
    .data_start_bit = 8};
// MT6701_Serial_I2C sensor = MT6701_Serial_I2C(mt6701_config);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);

void setup()
{
    // I2Cone = TwoWire(PIN_WIRE_SDA, PIN_WIRE_SCL);

    // monitoring port
    Serial.begin(MONITOR_SPEED);
    while (!Serial)
        ;

    delay(1000);

    SimpleFOCDebug::enable(&Serial);

    // I2Cone.begin(PIN_WIRE_SDA, PIN_WIRE_SCL, 400000);
    // sensor.init(&I2Cone);
    sensor.init();


    // Wire.setClock(400000);

    // sensor.clock_speed = 20000;
    // initialise magnetic sensor hardware
    // sensor.init(&SPI_2);

    Serial.println("Sensor ready");
    _delay(1000);
}

float sensorAngle = 0.0f;
unsigned long last_time = millis();

void loop()
{
    // iterative function updating the sensor internal variables
    // it is usually called in motor.loopFOC()
    // this function reads the sensor hardware and
    // has to be called before getAngle nad getVelocity
    sensor.update();
    sensorAngle = sensor.getAngle();

    // display the angle and the angular velocity to the terminal

    // float angle = sensor.getPreciseAngle();

    if (millis() - last_time > 500)
    {
        last_time = millis();
        Serial.printf("Angle: %f", sensorAngle);
        Serial.print("\t");
        Serial.println(sensor.getVelocity());
    }
}
#endif
