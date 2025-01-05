#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"
//#include <encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h>
//#include <encoders/smoothing/SmoothingSensor.h>
#include <encoders/mt6816/MagneticSensorMT6816.h>

float target = 0;

// magnetic sensor instance - MT6816 SPI mode
MagneticSensorMT6816 sensor = MagneticSensorMT6816(40);


//BLDCMotor motor = BLDCMotor(6,1.2,4300,12.7*1e-6); //12.7 uH for tiny motor 4300 kV
BLDCMotor motor = BLDCMotor(12,0.25,690,6.1*1e-6); //6.1 uH for large motor 690 kV


//MXLEMMINGObserverSensor sensor = MXLEMMINGObserverSensor(motor);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(26,21,18,17,16,15,11,false); // use the right pins for your setup!
LowsideCurrentSense current_sense = LowsideCurrentSense(150,7,8,9);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&target, cmd); }
//void changeKV(char* cmd) { motor.KV_rating(int(cmd)); }
//void changeL(char* cmd) { command.scalar(&target, cmd); }

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


void onStatus(char* cmd) { printDRV8316Status(); }
void clearFaultDriver(char* cmd) { driver.clearFault(); }



void setup() {
  	pinMode(47, OUTPUT);
	digitalWrite(47, HIGH); // Enable Driver
  
	Serial.begin(460800);
	while (!Serial);
	delay(3000);

	Serial.println("Initializing...");

	SimpleFOCDebug::enable(&Serial);

	sensor.init();
	motor.linkSensor(&sensor);

	driver.voltage_power_supply = 7.4;
	driver.init();
  
	//Serial.println("\nOld Current Sense gain:");
	//Serial.println(driver.getCurrentSenseGain());
	driver.setCurrentSenseGain(Gain_0V15);//Gain_1V2  Gain_0V6

	//Serial.println("New Current Sense gain:");
	//Serial.println(driver.getCurrentSenseGain());


	//Serial.println("\nOld Slew rate:");
	//Serial.println(driver.getSlew());
	driver.setSlew(Slew_200Vus);//Gain_0V25  Gain_0V375
	
	//Serial.println("New Slew rate:");
	//Serial.println(driver.getSlew());
	
	//driver.setRecirculationMode(CoastMode);
	//driver.setActiveAsynchronousRectificationEnabled(true);
	//driver.setActiveSynchronousRectificationEnabled(true);

	// link the driver to the current sense
	current_sense.linkDriver(&driver);

	motor.linkDriver(&driver);  // link driver
	motor.voltage_sensor_align  = 1;                            // aligning voltage
	motor.foc_modulation        = FOCModulationType::SpaceVectorPWM; // Only with Current Sense
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
  
  //motor.voltage_limit = 0.25;   // 12 Volt limit 
  motor.current_limit = 1;    // 2 Amp current limit
  motor.velocity_limit = 100; // 100 rad/s velocity limit


  /*
  // velocity loop PID
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 0.5;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.02;
  // angle loop PID
  motor.P_angle.P = 1.0;
  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.0;
  */

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
  */

 // contoller configuration based on the control type
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;

  motor.PID_current_q.P = 1;
  motor.PID_current_q.I= 0.5;
  motor.PID_current_q.output_ramp= 2;
  motor.PID_current_q.limit= 2;
  motor.LPF_current_q.Tf = 0.005f;
  

  motor.PID_current_d.P= 1;
  motor.PID_current_d.I = 0.5;
  motor.PID_current_d.output_ramp= 2;
  motor.PID_current_d.limit= 2;
  motor.LPF_current_d.Tf = 0.005f;
  
  

  /*
  // Limits 
  
  //motor.voltage_limit = 2;   // 12 Volt limit 
  //motor.current_limit = 2;    // 2 Amp current limit

  // comment out if not needed
  //motor.useMonitoring(Serial);
  //motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // monitor the two currents d and q
  //motor.monitor_downsample = 1000;
  */
  //motor.useMonitoring(Serial);
  	motor.monitor_downsample = 100; // set downsampling can be even more > 100
  	motor.monitor_variables =  _MON_VEL;// _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
	motor.monitor_decimals = 2; //!< monitor outputs decimal places

 // init motor hardware
	motor.init();

	// current sense init hardware
	//current_sense.skip_align = true;
	current_sense.init();


	// link the current sense to the motor
	motor.linkCurrentSense(&current_sense);

	// !!! The MXLEMMING observer sensor doesn't need sensor alignment
	//motor.sensor_direction= Direction::CW;
  	//motor.zero_electric_angle = 0;

	motor.initFOC();

	// set the initial motor target
	motor.target = 0; // unit depends on control mode 

	// add target command T
	command.add('T',doTarget, "target ");
	command.add('M', onMotor, "motor");
	command.add('S', onStatus, "Status");
	command.add('R', clearFaultDriver, "Reset");

	_delay(100);

  //printDRV8316Status();
  //Serial.println("Current Sense gain:");
  //Serial.println(driver.getCurrentSenseGain());

  //driver.setRecirculationMode(CoastMode);
  driver.setActiveAsynchronousRectificationEnabled(true);
  //driver.setActiveSynchronousRectificationEnabled(true);

}
    
LowPassFilter LPF_target(0.5);  //  the higher the longer new values need to take effect


void loop(){

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
  //motor.monitor();
	motor.loopFOC();
	motor.move(LPF_target(target));
	command.run();
}