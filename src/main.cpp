#include "Arduino.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"
#include <encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h>
#include <Adafruit_NeoPixel.h>
#include "esp_task_wdt.h"


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


float target = 0; 				//global variable for target
LowPassFilter LPF_target(2);  	//  the higher the longer new values need to take effect

float R3 = 100; //kOhm
float R4 = 5.1; //kOhm

Adafruit_NeoPixel strip(PIXEL_COUNT, LED_DIN, NEO_GRB + NEO_KHZ800);

//Different motor presets
//BLDCMotor motor = BLDCMotor(6,0.30625,4000,6.83*1e-6); //12.7 uH for tiny motor 4300 kV
//BLDCMotor motor = BLDCMotor(11,0.05125,690,4.56*1e-6); //6.1 uH for large motor 690 kV
//BLDCMotor motor = BLDCMotor(4,0.1375,1100,19.45*1e-6); //Plastic motor 957 kV?
//BLDCMotor motor = BLDCMotor(7,0.01,3500,0.45*1e-6); //Orange motor 2700 kV?
//BLDCMotor motor = BLDCMotor(4,0.525,1000,3.84*1e-7); //PCB motor 
BLDCMotor motor = BLDCMotor(1,7,250,3.02*1e-4); //Experimental Coreless motor 
//BLDCMotor motor = BLDCMotor(6,0.1,12000,0.82*1e-6); //7300 kV

DRV8316_CSAGain CS_Gain = Gain_0V3; //Single variable for all Current sense gain
unsigned int CS_Gain_mV = 300;///150 * 2^(CS_Gain); //Needed for Low Side Current Sense 


MXLEMMINGObserverSensor sensor = MXLEMMINGObserverSensor(motor);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(INHA,INLA,INHB,INLB,INHC,INLC,DRV8316_CS,false); // use the right pins for your setup!
LowsideCurrentSense current_sense = LowsideCurrentSense(CS_Gain_mV,CS_A,CS_B,CS_C); //Default gain is 0.15V/A

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&target, cmd); }
void doFW(char* cmd) { command.scalar(&motor.current_sp_field_weakening, cmd); }
void doFL(char* cmd) { command.scalar(&sensor.flux_linkage, cmd); }

void InnerFOCTask( void * parameter );

//Function to read status registers of DRV8316C
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

// Rainbow cycle for the onboard LED, used to signal that ESC has started up
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

void configDRV8316(DRV8316_CSAGain gain, DRV8316_Slew slew, DRV8316_OCPLevel OCP, DRV8316_OCPMode OCPMode, DRV8316_OCPDeglitch OCPDeglitch){
	driver.setCurrentSenseGain(gain);
	_delay(10);
	while (driver.getCurrentSenseGain() != gain)
	{
		Serial.println("Gain didn't set, trying again!");
		driver.setCurrentSenseGain(gain);//Gain_1V2  Gain_0V6 Gain_0V3 Gain0_15
		_delay(10);
	}

	driver.setSlew(slew);
	_delay(10);
	while (driver.getSlew() != slew)
	{
		Serial.println("Slew didn't set, trying again!");
		driver.setSlew(slew);
		_delay(10);
	}

	driver.setOCPLevel(OCP);
	_delay(10);
	while (driver.getOCPLevel() != OCP)
	{
		Serial.println("OCP didn't set, trying again!");
		driver.setOCPLevel(OCP);
		_delay(10);
	}

	driver.setOCPDeglitchTime(OCPDeglitch);
	_delay(10);
	while (driver.getOCPDeglitchTime() != OCPDeglitch)
	{
		Serial.println("OCPdeglitch didn't set, trying again!");
		driver.setOCPDeglitchTime(OCPDeglitch);
		_delay(10);
	}

	driver.setOCPMode(OCPMode);
	_delay(10);
	while (driver.getOCPMode() != OCPMode)
	{
		Serial.println("OCPMode didn't set, trying again!");
		driver.setOCPMode(OCPMode);
		_delay(10);
	}

}

void onStatus(char* cmd) { printDRV8316Status(); }
void clearFaultDriver(char* cmd) { 
	driver.clearFault(); 
	digitalWrite(DRV8316_EN, LOW); // Disable Driver
	_delay(100);
	digitalWrite(DRV8316_EN, HIGH); // Enable Driver

	configDRV8316(CS_Gain, Slew_200Vus, Curr_24A, NoAction, Deglitch_1us6);

}


TaskHandle_t FOC_TASK;

void setup() {
  	pinMode(DRV8316_EN, OUTPUT);
	digitalWrite(DRV8316_EN, HIGH); // Enable Driver
	pinMode(nFAULT, INPUT);

	strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  	strip.show();  // Initialize all pixels to 'off'

	Serial.begin(1000000);
	while (!Serial);
	rainbow(5);
	delay(100);

	Serial.println("Initializing...");
	SimpleFOCDebug::enable(&Serial);
	sensor.init();
	motor.linkSensor(&sensor);

	Serial.print("\nBus Voltage:");
	Serial.print("\t");
	Serial.println(((float)analogReadMilliVolts(VBUS_SENSE)/1000)*(R3+R4)/(R4));
	Serial.println(CS_Gain_mV);

	driver.voltage_power_supply = 24;
	//driver.voltage_power_supply = 4.6;
	driver.pwm_frequency = 25000;
	driver.init();
  
	configDRV8316(CS_Gain, Slew_200Vus, Curr_24A, NoAction, Deglitch_1us6);
	
	//driver.setRecirculationMode(CoastMode); //Regen breaking disabled
	//driver.setActiveAsynchronousRectificationEnabled(true); //Not sure if it breaks something
	//driver.setActiveSynchronousRectificationEnabled(true); //not needed as already done by with the 6PWM

	// link the driver to the current sense
	current_sense.linkDriver(&driver);

	motor.linkDriver(&driver);  // link driver
	
	motor.voltage_sensor_align  = 0.5;                            // aligning voltage
	motor.foc_modulation        = FOCModulationType::SpaceVectorPWM; // Only with Current Sense
	//motor.foc_modulation        = FOCModulationType::Trapezoid_120; // Experimental
	motor.controller            = MotionControlType::torque;    // set motion control loop to be used
	motor.torque_controller     = TorqueControlType::foc_current;


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
  
	
	// velocity loop PID, needs to be tuned for the motor
	motor.PID_velocity.P = 0.05;
	motor.PID_velocity.I = 0.02;
	//motor.PID_velocity.limit= 2000;
	motor.LPF_velocity.Tf = 0.05; // Low pass filtering time constant 

	//This values are experimental and require tuning for every motor
	motor.PID_current_q.P = 3; //1 //small motor 
    motor.PID_current_q.I= 20; //10
    motor.PID_current_d.P= 3; //1
    motor.PID_current_d.I = 20;//10
	motor.PID_current_d.D = 0.001;
    motor.LPF_current_q.Tf = 0.003f; // 1ms default
    motor.LPF_current_d.Tf = 0.003f; // 1ms default
	motor.PID_current_q.limit= 15;
	motor.PID_current_d.limit= 6;
	motor.current_limit = 0.5;    // 2 Amp current limit
	
	// init motor hardware
	//motor.useMonitoring(Serial);
	motor.monitor_downsample = 1;
	motor.monitor_variables =  _MON_CURR_Q | _MON_CURR_D; //_MON_VEL;// _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
	motor.monitor_decimals = 4; //!< monitor outputs decimal places

	motor.init();

	// current sense init hardware
	current_sense.skip_align = true; //Current sense align is skipped, as sensor location is already known
	current_sense.init();

	// link the current sense to the motor
	motor.linkCurrentSense(&current_sense);

	// !!! The MXLEMMING observer sensor doesn't need sensor alignment
	motor.sensor_direction= Direction::CW;
	motor.zero_electric_angle = 0;

	motor.initFOC();

	// add commands
	command.add('T',doTarget, "target ");
	command.add('M', onMotor, "motor");
	command.add('S', onStatus, "Status");
	command.add('R', clearFaultDriver, "Reset");
	command.add('F', doFW, "Field weakening");
	command.add('L', doFL, "Flux linkage");
	
	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target value   : - commnad T"));
	Serial.println(F("DRV8316C driver status : - command S"));
	Serial.println(F("DRV8316C hard reset    : - command R"));
	Serial.println(F("Set d-axis current     : - command F")); //As this ESC mainly uses sensorless FOC, this value mainly adjusts commutation angle 
	Serial.println(F("Change Flux linkage    : - command L")); 
	_delay(1000);


	
	//SimpleFOC tasks running on the second core
	xTaskCreatePinnedToCore(
      InnerFOCTask, /* Function to implement the task */
      "FOC loop", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &FOC_TASK,  /* Task handle. */
      0); /* Core where the task should run */


	motor.controller = MotionControlType::velocity_openloop;    // set motion control loop to be used
	
	//Rotor alignment procedure
	target = 0;  
	_delay(2000); //Tweak this value so that rotor has enough time to settle, the larger load and lower pole count increases the settle time

	motor.zero_electric_angle = -0.1; //Motor not always stable at 0 offset, 0.1 offset in the rotation is good enough for the start
	// = -500;
	target = -100;
	unsigned long startup_timer = millis();

	//Open-loop startup
	while ((millis() - startup_timer) < 10000){
		motor.monitor();
		command.run();
		_delay(5);
	}

	//LPF_target.Tf = 0.001; //speedup target change for torque control
	//target = -0.15;
	motor.current_sp_field_weakening = 0;
	sensor.flux_linkage = 0.017; //Important to match Flux linkage at target speed, otherwise switchover is not smooth 0.035
	//sensor.flux_linkage = 0.008;
	sensor.update();

	motor.controller = MotionControlType::velocity;
	//motor.controller = MotionControlType::torque;

	//smooth switchover from openloop to closed loop
	if (target > 0){
		//Serial.println(motor.voltage.q);
		motor.PID_current_q.reset_FF(motor.voltage.q);
	}
	else if (target < 0){
		motor.PID_current_q.reset_FF(-motor.voltage.q);
	}

	_delay(50);
	LPF_target.Tf = 1;

}
    
unsigned long timer_start, timer_start_2, count_timer = millis();
unsigned long loops = 0;

float velocity_avg, Iq_avg, Id_avg, Vq_avg, Vd_avg, Vin_avg, Iin_avg, temp_avg = 0;

void loop(){

	if(millis() - count_timer > 10){
		velocity_avg += motor.shaft_velocity;
		Iq_avg += motor.current.q;		
		Id_avg += motor.current.d;
		Vq_avg += motor.voltage.q;
		Vd_avg += motor.voltage.d;
		//Vin_avg += (((float)analogReadMilliVolts(VBUS_SENSE)/1000)*(R3+R4)/(R4));
		Iin_avg += current_sense.getDCCurrent(motor.electrical_angle);
		//temp_avg += (analogReadMilliVolts(PCB_TEMP)-500)/10;
		loops++;
		count_timer = millis();
	}

	if (millis()-timer_start_2 > 100){
		command.run();
		motor.monitor();
		if (!digitalRead(nFAULT)){
			strip.setPixelColor(0, strip.Color(  100,   0,   0));         //  Set pixel's color (in RAM)
    		strip.show();
			motor.disable();
			Serial.println("Fault or warning detected!");
			printDRV8316Status();
			_delay(3000);
			//motor.enable();
		}
		timer_start_2 = millis();
	}

	
	if (millis()-timer_start > 10000){
		command.run();
		
		if (motor.shaft_velocity < (20000/motor.pole_pairs/9.54)){
			motor.monitor();
			velocity_avg /= loops;
			Iq_avg /= loops;
			Id_avg /= loops;
			Vq_avg /= loops;
			Vd_avg /= loops;
			Vin_avg = (((float)analogReadMilliVolts(VBUS_SENSE)/1000)*(R3+R4)/(R4));
			Iin_avg /= loops;
			temp_avg = (analogReadMilliVolts(PCB_TEMP)-500)/10;

			//Parameter printing
			Serial.print("vel, Iq, Id, Vq, Vd, Vin, Iin, temp:");
			Serial.print("\t");
			Serial.printf("%.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f \n", velocity_avg, Iq_avg, Id_avg, Vq_avg, Vd_avg, Vin_avg, Iin_avg, temp_avg);

			Serial.print("Flux values, A, FL:");
			Serial.print("\t");
			Serial.println(sensor.flux_linkage,5); 

			strip.setPixelColor(0, strip.Color(  0,   0,   0));         //  Set pixel's color (in RAM)
			strip.show();
		}
		velocity_avg, Iq_avg, Id_avg, Vq_avg, Vd_avg, Vin_avg, Iin_avg, temp_avg = 0;
		loops = 0;

		timer_start = millis();

	}

	taskYIELD();
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

