#if defined(RUN_XDRIVE_TEST) && RUN_XDRIVE_TEST == 2

/*
  SimpleFOC MKS DRIVE MINI + AS5047P + iPower GM5208-24
  12v <=> 56v
  60 Amp (120 Amp max)
  SimpleFOC_STM32F405RGT6

  MCU ==> STM32F405RGT6
  DRIVER ==> DRV8301DCAR
  MOSFET ==> NTMFS5C628NLT1G
  CAN ==> SN65HVD232DR

  Odrive 3.6 ==> Odesk 4.2
  https://github.com/makerbase-motor/ODrive-MKS/tree/main/Hardware

  AS5047P
  https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/as5047

  Makerbase-Mini contrôleur de servomoteur sans balais XDrive, haute précision, basé sur ODriLi3.6 avec AS5047P à bord
  https://fr.aliexpress.com/item/1005006480243178.html

  Moteur iPower GM5208-24
  https://shop.iflight.com/ipower-motor-gm5208-24-brushless-gimbal-motor-pro1347?search=5208
  https://fr.aliexpress.com/item/4001296722586.html
  https://eu.robotshop.com/fr/products/ipower-motor-gm5208-24-brushless-gimbal-motor
  Model: GM5208-24
  Motor Out Diameter: Ф59.5±0.05mm
  Motor Height: 31.2±0.2mm
  Hollow Shaft(OD): Ф15-0.008/-0.012 mm
  Hollow Shaft(ID): Ф12.6+0.05/0 mm
  Wire Length: 610±3mm
  Cable AWG: #24
  Motor Weight: 204±0.5g
  Wire plug: 2.5mm dupont connector
  No-load current: 0.09±0.1 A
  No-load volts: 20V
  No-load Rpm: 396~436 RPM
  Load current: 1A
  Load volts: 20V
  Load torque(g·cm): 1800-2500
  Motor internal resistance: 13.7Ω±5%（Resistance varies with temperature）
  High voltage test: DC500V 10mA @1sec
  Rotor housing runout: ≤0.1mm
  Steering (axle extension): clockwise
  High-low temperature test:
  High temperature: Keep at 60℃ for 100 hours, and the motor can work normally after 24 hours at room temperature
  Low temperature: Keep at -20℃ for 100 hours, and the motor can work normally after 24 hours at room temperature
  Maximum power: ≤40W
  Working Voltage: 3-5S
  Working temperature: -20~60℃;10~90%RH
*/

#include <Arduino.h>
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
#define CS PA15       // PA15 // GPIO 7 ==> PA15 ==> SPI(AS5047)
#define SPI_MISO PC11 //
#define SPI_MOSI PC12 //
#define SPI0SCK PC10  // PC10

// SHUNT SENSING
#define M0_IA _NC // Seulement 2 mesures de courant B&C, A = pas dispo lui.
#define M0_IB PC0
#define M0_IC PC1

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15

// M1 & M2 common enable pin
#define EN_GATE PB12

// Pole pair
#define PP 11

// Temp
#define M0_TEMP PC5

#define PHASE_RESISTANCE 7.5

// Motor instance
// BLDCMotor motor = BLDCMotor(PP);
BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

// https://docs.simplefoc.com/low_side_current_sense
// low side current sensing define
// 0.0005 Ohm resistor
// gain of 10x
// current sensing on B and C phases, phase A not connected
// LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);

// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬AS5047_SPI▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, CS);
MagneticSensorSPI sensor = MagneticSensorSPI(CS, 14, 0x3FFF); // // alternative constructor (chipselsect, bit_resolution, angle_read_register, )
// https://github.com/simplefoc/Arduino-FOC/blob/ee2dfdeee62bc28fdee5820fb1d26a0af4dc80c9/src/sensors/MagneticSensorSPI.cpp
// https://github.com/simplefoc/Arduino-FOC/blob/master/examples/utils/sensor_test/magnetic_sensors/magnetic_sensor_spi/magnetic_sensor_spi_alternative_examples/stm32_spi_alt_example/stm32_spi_alt_example.ino
// these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
// SPIClass SPI_2(mosi, miso, sclk);
SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);

// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬HardwareSerial_Serial1▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
#define PIN_SERIAL1_RX PA3
#define PIN_SERIAL1_TX PA2
#define SERIAL_PORT_HARDWARE Serial1
HardwareSerial Serial1(PIN_SERIAL1_RX, PIN_SERIAL1_TX); // RX, TX

// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬USB_OTG_Serial▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
//  https://www.stm32duino.com/viewtopic.php?p=12540&hilit=USBD_VID#p12540
/*
  USB OTG
  USB_P ==> PA12 USART1_RTS USB_P
  USB_N ==> PA11 USART1_CTS USB_N

  -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC
  ;-D PIO_FRAMEWORK_ARDUINO_USB_LOWSPEED_FULLMODE ;This is not needed anymore, more in documentations.
  -D USBCON
  -D USBD_VID=0x0483
  -D USBD_PID=0x5740
  -D USB_MANUFACTURER="SimpleFOC"
  -D USB_PRODUCT="\"SimpleFOC_STM32F405RGT6\""
  -D HAL_PCD_MODULE_ENABLED

  // https://github.com/stm32duino/Arduino_Core_STM32/blob/main/cores/arduino/WSerial.h
  -DSERIAL_UART_INSTANCE=1
  -D HAL_OPAMP_MODULE_ENABLED
  -DSIMPLEFOC_STM32_DEBUG
  -DPIO_FRAMEWORK_ARDUINO_ENABLE_CDC
  -DUSBD_USE_CDC
  -DUSBCON
*/

#include <HardwareSerial.h>
HardwareSerial SerialUSB1(USART1); // Enable USB Serial

// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬Commander▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
//  https://docs.simplefoc.com/communication
//  instantiate the commander
Commander command = Commander(Serial1);

// motor SimpleFOCStudio ==> M
void doMotor(char *cmd)
{
  command.motor(&motor, cmd);
  // command.target(&motor, cmd); // ok
  // command.motion(&motor, cmd);
}

// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬AUX▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
// AUX_L PB10
// AUX_H PB11

// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬setup▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
void setup()
{
  // ▬▬▬▬▬▬▬▬▬▬▬▬CAN▬▬▬▬▬▬▬▬▬▬▬▬
  // CAN_R PB8
  // CAN_D PB9

  // ▬▬▬▬▬▬▬▬▬▬▬▬AUX▬▬▬▬▬▬▬▬▬▬▬▬
  // AUX_L PB10
  // AUX_H PB11

  // ▬▬▬▬▬▬▬▬▬▬▬▬Temp▬▬▬▬▬▬▬▬▬▬▬▬
  // pinMode(M0_TEMP, INPUT_PULLUP); // M0_TEMP PC5
  // pinMode(AUX_TEMP, INPUT_PULLUP); // AUX_TEMP PA5

  // ▬▬▬▬▬▬▬▬▬▬▬▬GPIO1/GPIO2/GPIO3/GPIO4▬▬▬▬▬▬▬▬▬▬▬▬
  // pinMode(PA0, OUTPUT); // GPIO1 ADC123_IN0 UART4_TX
  // digitalWrite(PA0, LOW); // HIGH/LOW

  // pinMode(PA1, OUTPUT); // GPIO2 ADC123_IN1 UART4_RX
  // digitalWrite(PA1, LOW); // HIGH/LOW

  // PA2 utilisé pour RX/TX avec SoftwareSerial
  // pinMode(PA2, OUTPUT); // GPIO3 ADC123_IN2 USART2_TX
  // digitalWrite(PA2, LOW); // HIGH/LOW

  // PA3 utilisé pour RX/TX avec SoftwareSerial
  // pinMode(PA3, OUTPUT); // GPIO4 ADC123_IN3 USART2_RX
  // digitalWrite(PA3, LOW); // HIGH/LOW

  // ▬▬▬▬▬▬▬▬▬▬▬▬HardwareSerial_Serial1▬▬▬▬▬▬▬▬▬▬▬▬
  pinMode(PIN_SERIAL1_RX, INPUT_PULLUP); // sur GPIO4 ==>  PA3
  pinMode(PIN_SERIAL1_TX, OUTPUT);       // sur GPIO3 ==>  PA2
  Serial1.begin(115200);                 // 921600 115200 230400
  _delay(1000);

  // ▬▬▬▬▬▬▬▬▬▬▬▬USB_OTG_Serial▬▬▬▬▬▬▬▬▬▬▬▬
  SerialUSB1.begin(115200);

  // ▬▬▬▬▬▬▬▬▬▬▬▬magnetic_sensor_AS5047P▬▬▬▬▬▬▬▬▬▬▬▬
  // sensor.clock_speed = 1000000;
  sensor.init(&SPI_2);
  motor.linkSensor(&sensor); // initialise magnetic sensor hardware
  _delay(1000);

  // ▬▬▬▬▬▬▬▬▬▬▬▬driver▬▬▬▬▬▬▬▬▬▬▬▬
  driver.pwm_frequency = 40000;        // 20000 max STM32  // pwm frequency to be used [Hz]
  driver.voltage_power_supply = 24.0f; // power supply voltage [V]
  driver.voltage_limit = 18.0f;        // Max DC voltage allowed - default voltage_power_supply
  /*
    https://docs.simplefoc.com/bldcdriver6pwm
    Le paramètre de la zone morte est défini comme la quantité du cycle de service qui est réservée
    entre les changements du mosfet actif. Chaque fois que le côté haut/bas est désactivé
    et que le côté bas/haut est activé, la moitié de la zone morte est injectée.
    Ce paramètre est équivalent au temps mort,
    le temps mort peut être calculé comme suit : dead_time = 1/pwm_frequency*dead_zone
  */
  // driver.dead_zone = 0.05f; // dead_zone [0,1] - default 0.02f - 2% // 1/20000*dead_zone=?
  driver.init(); // driver init
  _delay(1000);
  motor.linkDriver(&driver); // link the motor and the driver

  // ▬▬▬▬▬▬▬▬▬▬▬▬mode▬▬▬▬▬▬▬▬▬▬▬▬
  //  control loop type and torque mode
  //  https://docs.simplefoc.com/foc_current_torque_mode
  motor.torque_controller = TorqueControlType::foc_current; // foc_current || dc_current || voltage
  // https://docs.simplefoc.com/angle_loop
  motor.controller = MotionControlType::angle; // angle velocity torque  // Control loop type
  // choose FOC modulation
  // FOCModulationType::SinePWM; (default)
  // FOCModulationType::SpaceVectorPWM;
  // FOCModulationType::Trapezoid_120;
  // FOCModulationType::Trapezoid_150;
  motor.foc_modulation = FOCModulationType::SinePWM; // pwm modulation settings
  motor.modulation_centered = 1;                     // 1

  // ▬▬▬▬▬▬▬▬▬▬▬▬ALL_PID▬▬▬▬▬▬▬▬▬▬▬▬

  motor.PID_velocity.P = 0.066;
  motor.PID_velocity.I = 6.6;
  motor.PID_velocity.D = 0.000135;

  motor.LPF_velocity = 0.03;

  motor.PID_velocity.output_ramp = 300;

  // ▬▬▬▬▬▬▬▬▬▬▬▬velocity loop PID
  //  motor.PID_velocity.P = 2.0f; // 1.2
  //  motor.PID_velocity.I = 30.0f; // 80.0
  //  motor.PID_velocity.D = 0.001f; // 0.001
  //  motor.PID_velocity.output_ramp = 1000.0f;
  //  motor.LPF_velocity.Tf = 0.01f; //0.01  // Low pass filtering time constant

  // //▬▬▬▬▬▬▬▬▬▬▬▬angle loop PID
  // // https://docs.simplefoc.com/angle_loop
  // motor.P_angle.P = 20.0f; // 14.0
  // motor.P_angle.I = 0.0f; // usually only P controller is enough
  // motor.P_angle.D = 0.0f; // usually only P controller is enough
  // // this variable is in rad/s^2 and sets the limit of acceleration
  // motor.P_angle.output_ramp = 10000.0f; // 10000.0
  // motor.LPF_angle.Tf = 0.0f; // 0.01  // Low pass filtering time constant

  // //▬▬▬▬▬▬▬▬▬▬▬▬current q loop PID
  // motor.PID_current_q.P = 0.6f; // 3
  // motor.PID_current_q.I = 150.0f; // 300
  // motor.PID_current_q.D = 0.0f;
  // motor.PID_current_q.output_ramp = 0.0f;
  // motor.LPF_current_q.Tf = 0.005f;  // Low pass filtering time constant

  // //▬▬▬▬▬▬▬▬▬▬▬▬current d loop PID
  // motor.PID_current_d.P = 0.6f; // 3
  // motor.PID_current_d.I = 150.0f; // 300
  // motor.PID_current_d.D = 0.0f;
  // motor.PID_current_d.output_ramp = 0.0f;
  // motor.LPF_current_d.Tf = 0.005f;  // Low pass filtering time constant

  // ▬▬▬▬▬▬▬▬▬▬▬▬Limits_motor▬▬▬▬▬▬▬▬▬▬▬▬
  motor.velocity_limit = 25.0f;                      // [rad/s] (15 stable pour mode Angle, pas de retour)
  motor.voltage_limit = 0.5f * driver.voltage_limit; // [Volts] // Calcul ==> 5.57[Ohms]*1.0[Amps]=5,57[Volts] // [V] - if phase resistance not defined

  // https://www.digikey.fr/fr/resources/conversion-calculators/conversion-calculator-ohms
  motor.current_limit = 1.0f;    // Current limit [Amps] - if phase resistance defined
  motor.phase_resistance = 13.7; // [Ohms] motor phase resistance // I_max = V_dc/R

  // motor.KV_rating = 39; // [rpm/Volt] - default not set // motor KV rating [rpm/V]
  //  commenter les 2 ci-dessous pour avoir le test au demarrage
  motor.zero_electric_angle = 4.15f;       // zero_electric_angle
  motor.sensor_direction = Direction::CCW; // Cw/CCW // direction

  // ▬▬▬▬▬▬▬▬▬▬▬▬monitoring▬▬▬▬▬▬▬▬▬▬▬▬
  //  https://docs.simplefoc.com/monitoring
  motor.useMonitoring(Serial1);
  // motor.useMonitoring(rtt);
  // motor.monitor_variables =  _MON_TARGET | _MON_CURR_Q | _MON_CURR_D;

  // ▬▬▬▬▬▬▬▬▬▬▬▬motion_downsample▬▬▬▬▬▬▬▬▬▬▬▬
  //  https://docs.simplefoc.com/bldcmotor#step-71-motion-control-downsampling
  motor.monitor_downsample = 0; // default 10 // 0 = disable monitor at first - optional

  // ▬▬▬▬▬▬▬▬▬▬▬▬SimpleFOCDebug▬▬▬▬▬▬▬▬▬▬▬▬
  //  https://docs.simplefoc.com/debugging
  // SimpleFOCDebug::enable(NULL);
  // SimpleFOCDebug::enable();
  SimpleFOCDebug::enable(&Serial1); // ok pour simpleFOCStudio
  // SimpleFOCDebug::enable(&rtt);

  // ▬▬▬▬▬▬▬▬▬▬▬▬command ==> VerboseMode▬▬▬▬▬▬▬▬▬▬▬▬
  //  https://docs.simplefoc.com/commander_interface
  //  VerboseMode::nothing           - display nothing - good for monitoring
  //  VerboseMode::on_request        - display only on user request
  //  VerboseMode::user_friendly     - display textual messages to the user (default)
  //  VerboseMode::machine_readable  - display machine readable messages
  // command.verbose = VerboseMode::nothing;

  // ▬▬▬▬▬▬▬▬▬▬▬▬init▬▬▬▬▬▬▬▬▬▬▬▬
  motor.init(); // initialise motor

  // ▬▬▬▬▬▬▬▬▬▬▬▬Current▬▬▬▬▬▬▬▬▬▬▬▬
  //  https://docs.simplefoc.com/low_side_current_sense
  currentSense.linkDriver(&driver); // link the driver

  // ▬▬▬▬▬▬▬▬▬▬▬▬linkCurrentSense▬▬▬▬▬▬▬▬▬▬▬▬
  //  https://docs.simplefoc.com/low_side_current_sense
  if (currentSense.init())
  {
    Serial1.println("Current sense init success!");
  }
  else
  {
    Serial1.println("Current sense init failed!");
    return;
  }
  // motor.motor_status
  // If monitoring is enabled for the motor during the initFOC the monitor will display the alignment status:
  // 0 - fail
  // 1 - success and nothing changed
  // 2 - success but pins reconfigured
  // 3 - success but gains inverted
  // 4 - success but pins reconfigured and gains inverted
  // If you are sure in your configuration and if you wish to skip the alignment procedure you can specify set the skip_align flag before calling motor.initFOC():
  // Si vous êtes sûr de votre configuration et si vous souhaitez ignorer la procédure d'alignement, vous pouvez spécifier le drapeau skip_align avant d'appeler motor.initFOC() :
  /*
    //currentSense.gain_a *=-1.0; // N/A
    currentSense.gain_b *= -1.0; // invert phase b gain
    currentSense.gain_c *= -1.0; // invert phase c gain

    // default values of per phase gains
    float shunt_resistor = 0.0005f;
    float gain = 10.0f;
    //currentSense.gain_a = 1.0 / shunt_resistor / gain; // N/A
    currentSense.gain_b = 1.0 / shunt_resistor / gain;
    currentSense.gain_c = 1.0 / shunt_resistor / gain;
  */
  currentSense.skip_align = true; // true false // skip alignment procedure
  motor.linkCurrentSense(&currentSense);

  // ▬▬▬▬▬▬▬▬▬▬▬▬initFOC▬▬▬▬▬▬▬▬▬▬▬▬
  motor.initFOC(); // init FOC

  // ▬▬▬▬▬▬▬▬▬▬▬▬command▬▬▬▬▬▬▬▬▬▬▬▬
  //  https://docs.simplefoc.com/commander_interface
  //  add the motor to the commander interface
  command.decimal_places = 4;                         // default 3
  command.add('M', doMotor, "motor exemple ==> M10"); // The letter (here 'M') you will provide to the SimpleFOCStudio

  // ▬▬▬▬▬▬▬▬▬▬▬▬target▬▬▬▬▬▬▬▬▬▬▬▬
  motor.target = 0;

  // ▬▬▬▬▬▬▬▬▬▬▬▬CHECK_IF_ERROR▬▬▬▬▬▬▬▬▬▬▬▬
  //  https://docs.simplefoc.com/cheetsheet/options_reference
  if (motor.motor_status != 4)
  { // 0 - fail initFOC
    Serial1.println("ERROR:" + String(motor.motor_status));
    // return;
  }

  _delay(1000);

  // https://docs.simplefoc.com/cheetsheet/build_flags
  // https://docs.simplefoc.com/cheetsheet/options_reference
} // End setup
// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬End_setup▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬

// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬loop▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
float valVEL;
float sensor_getAngle;
float valWATTS;
String toSEND;
long timestamp = millis();

// current https://docs.simplefoc.com/low_side_current_sense
PhaseCurrent_s current; // getPhaseCurrents
float currentMagnitude;
int old_motor_enabled;
float memory_max_current = motor.current_limit;

void loop()
{
  motor.loopFOC(); // main FOC algorithm function
  current = currentSense.getPhaseCurrents();
  currentMagnitude = currentSense.getDCCurrent(); // https://docs.simplefoc.com/inline_current_sense#example-code

  long now = millis();
  if (now - 250 > timestamp)
  { // 250 ==> 4x per second
    if (motor.motor_status == 0)
    { // 0 - fail initFOC
      Serial1.println("ERROR:" + String(motor.motor_status));
    }
    timestamp = now;
  }

  /*
    //▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬mySerial_send▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
    // On envoie au M5StickC-plus la vitesse en RAD/S
    long now = millis();
    if (now - 25 > timestamp) { // 250 ==> 4x per second
      //Serial1.println(shaft_velocity);

      sensor_getAngle = sensor.getAngle(); // get the angle, in radians, including full rotations
      valVEL = sensor.getVelocity();
      //valVEL = motor.shaft_velocity;
      toSEND = ("V" + String(valVEL) + "\n");
      Serial1.write((char*)toSEND.c_str());

      // P=VxI
      //valWATTS = ((motor.voltage.d+motor.voltage.q)*currentMagnitude);
      valWATTS = (driver.voltage_limit * currentMagnitude);
      toSEND = ("C" + String(valWATTS));
      Serial1.write((char*)toSEND.c_str());

      timestamp = now;
    }
  */
  command.run(Serial1);
  motor.move();
} // End loop
// ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬End_loop▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
#endif
