#if defined(RUN_MAIN) && RUN_MAIN == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 22 && USE_CAN == 1 && defined(GBM520875T)

#include <Arduino.h>
#include <SimpleCan.h>
#include <SimpleFOC.h>


#define P_MIN -100
#define P_MAX  100
#define Pre	   0.01

// Handle CAN messages from CAN bus
static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
static void init_CAN(void);
static void Button_Down(void);

// Function Prototypes
uint16_t combineBytes(uint8_t lsb, uint8_t msb);
float scaleAndPrecision(uint16_t raw_value);

// pass in optional shutdown and terminator pins that disable transceiver and add 120ohm resistor respectively
SimpleCan can1(A_CAN_SHDN,A_CAN_TERM);
SimpleCan::RxHandler can1RxHandler(16, handleCanMessage);

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Motor instance
BLDCMotor motor = BLDCMotor(11,0.4,380);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// velocity set point variable
float target_velocity = 0;

void setup(){

  // configure i2C
  Wire.setClock(400000);
  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0.0;
  // default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  motor.velocity_limit = 25;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE;
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC(0.84,CW);

  Serial.println(F("Motor ready."));

  delay(7000);
	Serial.println("Init");

	pinMode(LED_BUILTIN, OUTPUT);
	attachInterrupt(digitalPinToInterrupt(A_BUTTON), Button_Down, LOW);

	delay(100);
	init_CAN();
}

void loop(){
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor.monitor();
}

static void init_CAN()
{
	Serial.println(can1.init(CanSpeed::Mbit1) == HAL_OK
					   ? "CAN: initialized."
					   : "CAN: error when initializing.");

	FDCAN_FilterTypeDef sFilterConfig;

	// Configure Rx filter
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x121;
	sFilterConfig.FilterID2 = 0x7FF;

	can1.configFilter(&sFilterConfig);
	can1.configGlobalFilter(FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	can1.activateNotification(&can1RxHandler);

	Serial.println(can1.start() == HAL_OK
					   ? "CAN: started."
					   : "CAN: error when starting.");
}

static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
	
	uint16_t received_value = combineBytes(rxData[0], rxData[1]);
	float scaled_value = scaleAndPrecision(received_value);
	// Serial.print(rxHeader.Identifier, HEX);
	// Serial.print(" ");
	// Serial.print(rxHeader.DataLength);
	// Serial.print(" ");
	// Serial.print(received_value);
	// Serial.print(" ");
	// Serial.println(scaled_value);
  target_velocity = scaled_value;
	digitalToggle(LED_BUILTIN);
}

static void Button_Down()
{

	static uint8_t press_count = 0;

	press_count++;

	TxHeader.Identifier = 0x321;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	TxData[0] = press_count;
	TxData[1] = 0xAD;

	Serial.print("CAN: sending message ");
	Serial.println(can1.addMessageToTxFifoQ(&TxHeader, TxData) == HAL_OK
					   ? "was ok."
					   : "failed.");
}

uint16_t combineBytes(uint8_t lsb, uint8_t msb) {
  return static_cast<uint16_t>((msb << 8) | lsb);
}

float scaleAndPrecision(uint16_t raw_value) {
  uint16_t raw_min = 0;       // Minimum raw value
  uint16_t raw_max = 65535;   // Maximum raw value

  float decimal_value = ((static_cast<float>(raw_value) - raw_min) * (P_MAX - P_MIN) / (raw_max - raw_min)) + P_MIN;
  return decimal_value;
}

#endif