#if defined(RUN_SENSOR) && RUN_SENSOR == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 23

#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>


MagneticSensorMT6701I2C sensor1();
// MagneticSensorSPI sensor = MagneticSensorSPI(CS, 14, 0x3FFF);
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, CS);

// SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);

void setup()
{
  // monitoring port
  Serial.begin(MONITOR_SPEED);
  // Serial.begin(115200);

  SimpleFOCDebug::enable(&Serial);

  delay(5000);

  // configure i2C
  Wire.setClock(400000);

  // sensor.clock_speed = 20000;

  // initialise magnetic sensor hardware
  sensor.init();
  // sensor.init(&SPI_2);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop()
{
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and
  // has to be called before getAngle nad getVelocity
  sensor.update();

  // display the angle and the angular velocity to the terminal

  // float angle = sensor.getPreciseAngle();

  float angle = sensor.getAngle();
  Serial.printf("Angle: %f", angle);

  Serial.print("\t");

  Serial.println(sensor.getVelocity());

  _delay(100);
}
#endif
