#include "Arduino.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"
#include <encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h>
#include <Adafruit_NeoPixel.h>
#include "esp_task_wdt.h"
//#include <encoders/smoothing/SmoothingSensor.h>
//#include <encoders/mt6816/MagneticSensorMT6816.h>
//#include "encoders/esp32hwencoder/ESP32HWEncoder.h"

#include "..\.pio\libdeps\esp32dev\Simple FOC\src\current_sense\hardware_specific\esp32\esp32_adc_driver.cpp"


#define VBUS_SENSE 1
#define PCB_TEMP 2
#define LED_DIN 48
#define NUM_LEDS 1
#define PIXEL_COUNT 1

#define DRV8316_EN 47
#define DRV8316_CS 11
#define nFAULT 33
#define DRV_OFF 34
#define INHA 26
#define INLA 21
#define INHB 18
#define INLB 17
#define INHC 16
#define INLC 15
#define CS_A 7
#define CS_B 8
#define CS_C 9


float target = 0;
LowPassFilter LPF_target(2);  //  the higher the longer new values need to take effect

float R3 = 100; //kOhm
float R4 = 5.1; //kOhm

Adafruit_NeoPixel strip(PIXEL_COUNT, LED_DIN, NEO_GRB + NEO_KHZ800);

//BLDCMotor motor = BLDCMotor(6,0.30625,4000,6.83*1e-6); //12.7 uH for tiny motor 4300 kV
//BLDCMotor motor = BLDCMotor(11,0.05125,690,4.56*1e-6); //6.1 uH for large motor 690 kV
//BLDCMotor motor = BLDCMotor(4,0.1375,1100,19.45*1e-6); //Plastic motor 957 kV?
//BLDCMotor motor = BLDCMotor(7,0.01,3500,0.45*1e-6); //Orange motor 2700 kV?
//BLDCMotor motor = BLDCMotor(4,0.525,1000,3.84*1e-7); //PCB motor 
BLDCMotor motor = BLDCMotor(1,5.75,250,3.02*1e-4);
//BLDCMotor motor = BLDCMotor(6,0.1,12000,0.82*1e-6); //7300 kV

//BLDCMotor motor = BLDCMotor(11);
// magnetic sensor instance - MT6816 SPI mode
//MagneticSensorMT6816 sensor = MagneticSensorMT6816(40);
//MagneticSensorPWM sensor2 = MagneticSensorPWM(37,879,4119,16,4111);

MXLEMMINGObserverSensor sensor = MXLEMMINGObserverSensor(motor);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(INHA,INLA,INHB,INLB,INHC,INLC,DRV8316_CS,false); // use the right pins for your setup!
LowsideCurrentSense current_sense = LowsideCurrentSense(600,CS_A,CS_B,CS_C); //Default gain 0.15V/A

//SmoothingSensor smooth(sensor, motor);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&target, cmd); }
void doFW(char* cmd) { command.scalar(&motor.current_sp_field_weakening, cmd); }
void doFL(char* cmd) { command.scalar(&sensor.flux_linkage, cmd); }

void InnerFOCTask( void * parameter );


//void doFW(char* cmd) { motor.current_sp_field_weakening = float(*cmd); }
//void changeKV(char* cmd) { motor.KV_rating(int(cmd)); }
//void changeL(char* cmd) { command.scalar(&target, cmd); }
//void doPWM(){sensor2.handlePWM();}
/*
void doFW(char* cmd) { 
  // calculate the KV
  //Serial.println(motor.shaft_velocity/motor.target/_SQRT3*30.0f/_PI);
	//driver.setActiveSynchronousRectificationEnabled(true);
	Serial.print("Field weakening new target:\t");
	Serial.println(float(*cmd));
	motor.current_sp_field_weakening = float(*cmd);
}*/


void printDRV8316Status() {
	DRV8316Status status = driver.getStatus();
	Serial.println("DRV8316 Status:");
	Serial.print("Fault: ");
	Serial.println(status.isFault());
	Serial.print("Buck Error: ");
	Serial.print(status.isBuckError());
	Serial.print("  Undervoltage: ");
	Serial.print(status.isBuckUnderVoltage());
	Serial.print("  OverCurrent: ");
	Serial.println(status.isBuckOverCurrent());
	Serial.print("Charge Pump UnderVoltage: ");
	Serial.println(status.isChargePumpUnderVoltage());
	Serial.print("OTP Error: ");
	Serial.println(status.isOneTimeProgrammingError());
	Serial.print("OverCurrent: ");
	Serial.print(status.isOverCurrent());
	Serial.print("  Ah: ");
	Serial.print(status.isOverCurrent_Ah());
	Serial.print("  Al: ");
	Serial.print(status.isOverCurrent_Al());
	Serial.print("  Bh: ");
	Serial.print(status.isOverCurrent_Bh());
	Serial.print("  Bl: ");
	Serial.print(status.isOverCurrent_Bl());
	Serial.print("  Ch: ");
	Serial.print(status.isOverCurrent_Ch());
	Serial.print("  Cl: ");
	Serial.println(status.isOverCurrent_Cl());
	Serial.print("OverTemperature: ");
	Serial.print(status.isOverTemperature());
	Serial.print("  Shutdown: ");
	Serial.print(status.isOverTemperatureShutdown());
	Serial.print("  Warning: ");
	Serial.println(status.isOverTemperatureWarning());
	Serial.print("OverVoltage: ");
	Serial.println(status.isOverVoltage());
	Serial.print("PowerOnReset: ");
	Serial.println(status.isPowerOnReset());
	Serial.print("SPI Error: ");
	Serial.print(status.isSPIError());
	Serial.print("  Address: ");
	Serial.print(status.isSPIAddressError());
	Serial.print("  Clock: ");
	Serial.print(status.isSPIClockFramingError());
	Serial.print("  Parity: ");
	Serial.println(status.isSPIParityError());
	if (status.isFault())
		driver.clearFault();
	delayMicroseconds(1); // ensure 400ns delay
	DRV8316_PWMMode val = driver.getPWMMode();
	Serial.print("PWM Mode: ");
	Serial.println(val);
	delayMicroseconds(1); // ensure 400ns delay
	bool lock = driver.isRegistersLocked();
	Serial.print("Lock: ");
	Serial.println(lock);
}

void testAlignmentAndCogging(int direction) {

  motor.move(0);
  _delay(200);

  sensor.update();
  float initialAngle = sensor.getAngle();

  const int shaft_rotation = 720; // 720 deg test - useful to see repeating cog pattern
  int sample_count = int(shaft_rotation * motor.pole_pairs); // test every electrical degree

  float stDevSum = 0;

  float mean = 0.0f;
  float prev_mean = 0.0f;


  for (int i = 0; i < sample_count; i++) {

    float shaftAngle = (float) direction * i * shaft_rotation / sample_count;
    float electricAngle = (float) shaftAngle * motor.pole_pairs;
    // move and wait
    motor.move(shaftAngle * PI / 180);
    _delay(5);

    // measure
    sensor.update();
    float sensorAngle = (sensor.getAngle() - initialAngle) * 180 / PI;
    float sensorElectricAngle = sensorAngle * motor.pole_pairs;
    float electricAngleError = electricAngle - sensorElectricAngle;

    // plot this - especially electricAngleError
    Serial.print(electricAngle);
    Serial.print("\t");
    Serial.print(sensorElectricAngle );
    Serial.print("\t");
    Serial.println(electricAngleError);

    // use knuth standard deviation algorithm so that we don't need an array too big for an Uno
    prev_mean = mean;
    mean = mean + (electricAngleError-mean)/(i+1);
    stDevSum = stDevSum + (electricAngleError-mean)*(electricAngleError-prev_mean);

  }

  Serial.println();
  Serial.println(F("ALIGNMENT AND COGGING REPORT"));
  Serial.println();
  Serial.print(F("Direction: "));
  Serial.println(direction);
  Serial.print(F("Mean error (alignment): "));
  Serial.print(mean);
  Serial.println(" deg (electrical)");
  Serial.print(F("Standard Deviation (cogging): "));
  Serial.print(sqrt(stDevSum/sample_count));
  Serial.println(F(" deg (electrical)"));
  Serial.println();
  Serial.println(F("Plotting 3rd column of data (electricAngleError) will likely show sinusoidal cogging pattern with a frequency of 4xpole_pairs per rotation"));
  Serial.println();

}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 3 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 3*65536. Adding 256 to firstPixelHue each time
  // means we'll make 3*65536/256 = 768 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue,255,100)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

void onStatus(char* cmd) { printDRV8316Status(); }
void clearFaultDriver(char* cmd) { 
	driver.clearFault(); 
	digitalWrite(DRV8316_EN, LOW); // Enable Driver
	_delay(100);
	digitalWrite(DRV8316_EN, HIGH); // Enable Driver
	driver.setCurrentSenseGain(Gain_0V6);//Gain_1V2  Gain_0V6 Gain_0V3
	driver.setSlew(Slew_200Vus);//Gain_0V25  Gain_0V375
	driver.setOCPLevel(Curr_24A);
	driver.setOCPDeglitchTime(Deglitch_1us6);
	driver.setOCPMode(NoAction);
}
/*void enableSmoothing(char* cmd) {
  float enable;
  command.scalar(&enable, cmd);
  motor.linkSensor(enable == 0 ? (Sensor*)&sensor : (Sensor*)&smooth);
}*/

TaskHandle_t FOC_TASK;

void setup() {
  	pinMode(DRV8316_EN, OUTPUT);
	digitalWrite(DRV8316_EN, HIGH); // Enable Driver
	
	pinMode(nFAULT, INPUT);

	adcInit(PCB_TEMP);
	adcInit(4);
	adcInit(5);
	adcInit(6);
	/*pinMode(4, ANALOG);
	pinMode(5, ANALOG);
	pinMode(6, ANALOG);*/

	strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  	strip.show();  // Initialize all pixels to 'off'


	Serial.begin(1000000);
	while (!Serial);
	rainbow(5);
	delay(100);

	Serial.println("Initializing...");

	SimpleFOCDebug::enable(&Serial);

	sensor.init();

	//sensor2.init();
	//sensor2.enableInterrupt(doPWM);
	//smooth.phase_correction = -_PI_6;

	motor.linkSensor(&sensor);

	Serial.print("\nBus Voltage:");
	Serial.print("\t");
	Serial.print(analogReadMilliVolts(VBUS_SENSE));
	Serial.print("\t");
	Serial.println(((float)analogReadMilliVolts(VBUS_SENSE)/1000)*(R3+R4)/(R4));


	driver.voltage_power_supply = 24;
	//driver.voltage_power_supply = 4.6;
	driver.pwm_frequency = 25000;
	driver.init();
  
	//Serial.println("\nOld Current Sense gain:");
	//Serial.println(driver.getCurrentSenseGain());
	driver.setCurrentSenseGain(Gain_0V6);//Gain_1V2  Gain_0V6 Gain_0V15 Gain_0V3

	//Serial.println("New Current Sense gain:");
	//Serial.println(driver.getCurrentSenseGain());


	//Serial.println("\nOld Slew rate:");
	//Serial.println(driver.getSlew());
	driver.setSlew(Slew_200Vus);//Gain_0V25  Gain_0V375
	driver.setOCPLevel(Curr_24A);
	driver.setOCPDeglitchTime(Deglitch_1us6);
	driver.setOCPMode(NoAction);
	//Serial.println("New Slew rate:");
	//Serial.println(driver.getSlew());
	
	//driver.setRecirculationMode(CoastMode);
	//driver.setActiveAsynchronousRectificationEnabled(true);
	//driver.setActiveSynchronousRectificationEnabled(true);

	// link the driver to the current sense
	current_sense.linkDriver(&driver);

	motor.linkDriver(&driver);  // link driver
	
	motor.voltage_sensor_align  = 0.5;                            // aligning voltage
	motor.foc_modulation        = FOCModulationType::SpaceVectorPWM; // Only with Current Sense
	//motor.foc_modulation        = FOCModulationType::Trapezoid_120; // Only with Current Sense
	motor.controller            = MotionControlType::torque;    // set motion control loop to be used
	motor.torque_controller     = TorqueControlType::foc_current;

	motor.voltage_limit = 1;

	//motor.controller = MotionControlType::angle_openloop;
  	//motor.voltage_limit=motor.voltage_sensor_align;
  

	
	// Limit the voltage to have enough low side ON time for phase current sampling
	driver.voltage_limit = driver.voltage_power_supply * 0.95;

	if (motor.controller == MotionControlType::torque || motor.controller == MotionControlType::angle || motor.controller == MotionControlType::velocity){
		if (motor.torque_controller == TorqueControlType::foc_current || motor.torque_controller == TorqueControlType::dc_current){
		// When current sensing is used, reduce the voltage limit to have enough low side ON time for phase current sampling  
		    motor.voltage_limit = driver.voltage_power_supply * 0.54;
		}else{
		    motor.voltage_limit = driver.voltage_power_supply * 0.58;
		}
	}else{
		// For openloop angle and velocity modes, use very small limit
		motor.voltage_limit = driver.voltage_power_supply * 0.03;
	}
  

	/*
	//motor.voltage_limit = 0.25;   // 12 Volt limit 
	motor.current_limit = 1;    // 2 Amp current limit
	motor.velocity_limit = 100; // 100 rad/s velocity limit*/


	
	// velocity loop PID
	motor.PID_velocity.P = 0.1;
	motor.PID_velocity.I = 0.02;

	motor.PID_velocity.limit= 2000;
	// Low pass filtering time constant 
	motor.LPF_velocity.Tf = 0.05;
	// angle loop PID
	motor.P_angle.P = 0.5;
	// Low pass filtering time constant 
	motor.LPF_angle.Tf = 0.0;
	

	/*
	// current q loop PID 
	motor.PID_current_q.P = 0.1;
	motor.PID_current_q.I = 0.1;
	// Low pass filtering time constant 
	motor.LPF_current_q.Tf = 0.02;
	// current d loop PID
	motor.PID_current_d.P = 0.1;
	motor.PID_current_d.I = 0.1;
	// Low pass filtering time constant 
	motor.LPF_current_d.Tf = 0.02;
	
	
	// contoller configuration based on the control type
	motor.PID_velocity.P = 0.2f;
	motor.PID_velocity.I = 20;
	motor.PID_velocity.D = 0;
	*/
	/*
	motor.PID_current_q.P = 1;
	motor.PID_current_q.I= 0.5;
	motor.PID_current_q.output_ramp= 2;
	motor.PID_current_q.limit= 2;
	motor.LPF_current_q.Tf = 0.01f;
	
	
	motor.PID_current_d.P= 1;
	motor.PID_current_d.I = 0.5;
	motor.PID_current_d.output_ramp= 2;
	motor.PID_current_d.limit= 2;
	motor.LPF_current_d.Tf = 0.01f;

	motor.LPF_angle.Tf = 0.0;
	motor.LPF_velocity.Tf = 0.5;
	*/
	/*
	motor.PID_current_q.P = 1;
    motor.PID_current_q.I= 10;
    motor.PID_current_d.P= 1;
    motor.PID_current_d.I = 10;
    motor.LPF_current_q.Tf = 0.002f; // 1ms default
    motor.LPF_current_d.Tf = 0.002f; // 1ms default
	motor.PID_current_q.limit= 1;
	motor.PID_current_d.limit= 1;
	*/

	motor.PID_current_q.P = 3; //1 //small motor 
    motor.PID_current_q.I= 20; //10
    motor.PID_current_d.P= 3; //1
    motor.PID_current_d.I = 20;//10
	motor.PID_current_d.D = 0.001;
    motor.LPF_current_q.Tf = 0.003f; // 1ms default
    motor.LPF_current_d.Tf = 0.003f; // 1ms default
	motor.PID_current_q.limit= 8;
	motor.PID_current_d.limit= 6;
	motor.current_limit = 0.5;    // 2 Amp current limit
	/*
	// Limits 
	
	//motor.voltage_limit = 2;   // 12 Volt limit 
	//motor.current_limit = 2;    // 2 Amp current limit

	// comment out if not needed
	motor.useMonitoring(Serial);
	//motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // monitor the two currents d and q
	
	
	//motor.useMonitoring(Serial);
	motor.monitor_downsample = 100; // set downsampling can be even more > 100
	motor.monitor_variables =  _MON_VEL;// _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
	motor.monitor_decimals = 2; //!< monitor outputs decimal places
	*/

	//motor.sensor_direction = Direction::CW;

	// init motor hardware
	motor.useMonitoring(Serial);
	motor.monitor_downsample = 1;
	motor.monitor_variables =  _MON_CURR_Q | _MON_CURR_D; //_MON_VEL;// _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
	motor.monitor_decimals = 4; //!< monitor outputs decimal places

	motor.init();

	// current sense init hardware
	current_sense.skip_align = true;
	current_sense.init();

	// link the current sense to the motor
	motor.linkCurrentSense(&current_sense);

	// !!! The MXLEMMING observer sensor doesn't need sensor alignment
	motor.sensor_direction= Direction::CW;
	motor.zero_electric_angle = 0;

	motor.initFOC();

	// set the initial motor target
	//motor.target = 0; // unit depends on control mode 

	// add target command T
	command.add('T',doTarget, "target ");
	command.add('M', onMotor, "motor");
	command.add('S', onStatus, "Status");
	command.add('R', clearFaultDriver, "Reset");
	command.add('F', doFW, "Field weakening");
	command.add('L', doFL, "Flux linkage");
	
	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target voltage : - commnad T"));
	Serial.println(F("Calculate the motor KV : - command K"));
	_delay(1000);


	//printDRV8316Status();
	//Serial.println("Current Sense gain:");
	//Serial.println(driver.getCurrentSenseGain());

	//driver.setRecirculationMode(CoastMode);
	driver.setActiveAsynchronousRectificationEnabled(true);
	//driver.setActiveSynchronousRectificationEnabled(true);
	Serial.print("New Slew rate:\t");
	Serial.println(driver.getSlew());

	
	xTaskCreatePinnedToCore(
      InnerFOCTask, /* Function to implement the task */
      "FOC loop", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &FOC_TASK,  /* Task handle. */
      0); /* Core where the task should run */


	/*
	Serial.println(F("Press any key to start"));
	while (!Serial.available()) { }
	Serial.read();
	motor.shaft_angle = 0;
	testAlignmentAndCogging(1);

	motor.move(0);
	motor.shaft_angle = 0;

	Serial.println(F("Press any key to test in CCW direction"));
	while (!Serial.available()) { }
	Serial.read();
	
	testAlignmentAndCogging(-1);

	Serial.println(F("Complete"));

	motor.voltage_limit = 0;
	motor.move(0);
	while (true) ; //do nothing;
	*/
	//LowPassFilter Motor_Start(5);

	motor.controller = MotionControlType::velocity_openloop;    // set motion control loop to be used
	float startup_target = -100;
	target = 0;
	//LowPassFilter position_test(0.002);

	_delay(2000);
	motor.zero_electric_angle = -0.1;
	target = startup_target;
	unsigned long startup_timer = millis();
	while ((millis() - startup_timer) < 10000){
		//motor.move(Motor_Start(startup_target));
		motor.monitor();
		command.run();
		/*sensor.update();
		Serial.print("Angles:");
		Serial.print("\t");
		Serial.print(motor.shaft_angle,5);
		Serial.print("\t");
		Serial.print(sensor.getMechanicalAngle(),5);
		Serial.print("\t");
		Serial.println(motor.shaft_angle - sensor.electrical_angle);*/
		
		_delay(5);
	}

	LPF_target.Tf = 0.001;
	//target = -0.15;
	motor.current_sp_field_weakening = 0;
	sensor.flux_linkage = 0.035;
	sensor.update();

	Serial.print("Calculated offset for start");
	Serial.print("\t");
	Serial.println(motor.shaft_angle - sensor.electrical_angle);
	//motor.zero_electric_angle = sensor.electrical_angle - motor.shaft_angle ;

	motor.controller = MotionControlType::velocity;
	//motor.controller = MotionControlType::torque;

	if (target > 0){
		//Serial.println(motor.voltage.q);
		motor.PID_current_q.reset_FF(motor.voltage.q);
	}
	else if (target < 0){
		motor.PID_current_q.reset_FF(-motor.voltage.q);
	}
	//motor.PID_current_q.reset_FF(motor.voltage.q);
	
	_delay(50);
	LPF_target.Tf = 1;

	//_delay(5000);
	//sensor.flux_linkage=0.035;
	//motor.PID_velocity.P = 0.1;
	//motor.PID_velocity.I = 5;
}
    
//LowPassFilter LPF_target(0.5);  //  the higher the longer new values need to take effect
unsigned long timer_start, timer_start_2, stall_timer = millis();

void loop(){
	//motor.loopFOC();
	//motor.move(LPF_target(target));
		
	if (millis()-timer_start_2 > 20){
		motor.monitor();
		timer_start_2 = millis();
	}

	/*
	sensor2.update();
	
	if (millis()-timer_start > 1000){
		timer_start = millis();
		sensor.update();
		sensor2.update();
		Serial.println("\nSensor comparison: ");
		Serial.print(sensor.getAngle());
		Serial.print("\t");
		Serial.println(sensor.getVelocity());
		Serial.print(sensor2.getAngle());
		Serial.print("\t");
		Serial.println(sensor2.getVelocity());
	}*/
	//sensor.update();
	/*
	// display the angle and the angular velocity to the terminal
	Serial.print(sensor.getAngle());
	Serial.print("\t");
	Serial.println(sensor.getVelocity());
	_delay(200);
	*/

	taskYIELD();
	if (millis()-timer_start > 1000){
		command.run();
		
		if (motor.shaft_velocity < (20000/motor.pole_pairs/10)){
			motor.monitor();

			/*
			sensor.update();
			Serial.print("Shaft angle:");
			Serial.print("\t");
			Serial.print(_normalizeAngle(motor.shaft_angle));
			Serial.print("\t");
			Serial.print(sensor.getMechanicalAngle());
			Serial.print("\t");*/

			//PCB temperature reading
			Serial.print("PCB Temperature:");
			Serial.print("\t");
			Serial.println(((3100.0f*analogRead(PCB_TEMP)/4095.0f)-500)/10);
			Serial.print("Flux values, A, FL:");
			Serial.print("\t");
			Serial.print(sensor.flux_alpha,5);
			Serial.print("\t");
			Serial.print(sensor.flux_beta,5);
			Serial.print("\t");
			Serial.print(motor.shaft_angle,5);
			Serial.print("\t");
			Serial.print(sensor.getMechanicalAngle(),5);
			Serial.print("\t");
			Serial.println(sensor.flux_linkage,5); 

			strip.setPixelColor(0, strip.Color(  0,   0,   0));         //  Set pixel's color (in RAM)
			strip.show();  
		}
		
		if (!digitalRead(nFAULT)){
			strip.setPixelColor(0, strip.Color(  100,   0,   0));         //  Set pixel's color (in RAM)
    		strip.show();

			motor.disable();
			sensor.update();
			Serial.println("Fault or warning detected!");
			printDRV8316Status();
			_delay(3000);
			//motor.enable();
		}
		timer_start = millis();

	}
	/*
	if (millis()-timer_start_2 > 10){
		/*float bemf_a = adcRead(4);
		delayMicroseconds(1);
		float bemf_b = adcRead(5);
		delayMicroseconds(1);
		float bemf_c = adcRead(6);
		float bemf_a = analogRead(4);
		float bemf_b = analogRead(5);
		float bemf_c = analogRead(6);
		Serial.print(bemf_a);
		Serial.print("\t");
		Serial.print(bemf_b);
		Serial.print("\t");
		Serial.print(bemf_c);
		Serial.println("\t");

		timer_start_2 = millis();
	}*/

	/*
	if (millis()-stall_timer > 250 && (motor.current.d * motor.shaft_velocity ) < 0 && (motor.current.d * motor.target ) > 0 && !Stall_recovery){
		Serial.println("\nStall detected!");
		motor.controller == MotionControlType::torque;
		motor.current_limit = 0.5;    // 2 Amp current limit
		LPF_target.Tf = 0.001;

		prestall_target = target;  //motor.target;

		if (motor.target < 0){
			target = -0.5;    // 2 Amp current limit
			Stall_recovery_dir = 0;
		}
		else{
			target = 0.5;    // 2 Amp current limit
			Stall_recovery_dir = 1;
		}
		//target = 0.5;    // 2 Amp current limit
		Stall_recovery = true;
		unsigned long temp = millis();
		while ((millis() - temp) < 300){
			motor.move(target);
			motor.loopFOC();
		}

		if (Stall_recovery){
			motor.current_limit = 2;    // 2 Amp current limit
			LPF_target.Tf = 0.5;

			if (Stall_recovery_dir == 0){
				target = -1;    // 2 Amp current limit
			}
			else if (Stall_recovery_dir == 1){
				target = 1;    // 2 Amp current limit
			}

			//motor.move(target);
			unsigned long temp = millis();
			while ((millis() - temp) < 200){
				//motor.move(target);
				motor.move(target);
				motor.loopFOC();
			}


			target = prestall_target;

			temp = millis();
			while ((millis() - temp) < 200){
				//motor.move(target);
				motor.move(LPF_target(target));;
				motor.loopFOC();
			}


			Stall_recovery = false;
			LPF_target.Tf = 0.5;

			

		}
		stall_timer = millis();
	}
	*/

}



void InnerFOCTask( void * parameter ) 
{
	disableCore0WDT();
	esp_task_wdt_add(FOC_TASK);
	long lastMillis = 0;
	long loops = 0;
	int count = 0;
	for (;;) 
	{
		long currentMillis = millis();
  		loops++;
		motor.loopFOC();
		motor.move(LPF_target(target));
		/*
		if (count > 2){
			motor.move(LPF_target(target));
			count = 0;
		}*/
		//count++;

		/*if(currentMillis - lastMillis > 10000){
			Serial.print("Loops last second:");
			Serial.println(loops/10);

			lastMillis = currentMillis;
			loops = 0;
		}*/
		
		esp_task_wdt_reset();
	}
}

