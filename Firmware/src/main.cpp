#include "Arduino.h"
#include "main.h"

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
	Serial.println(((float)analogReadMilliVolts(VBUS_SENSE)/1000)*(R3+R4)/(R4)); //Measure DC bus voltage
	Serial.println(CS_Gain_mV);

	driver.voltage_power_supply = 24;
	//driver.voltage_power_supply = 4.6;
	driver.pwm_frequency = 25000;
	driver.init();
  
	configDRV8316(CS_Gain, Slew_200Vus, Curr_24A, NoAction, Deglitch_1us6);
	
	//driver.setRecirculationMode(CoastMode); 					//Regen breaking disabled
	//driver.setActiveAsynchronousRectificationEnabled(true); 	//Not sure if it breaks something
	//driver.setActiveSynchronousRectificationEnabled(true); 	//not needed as already done by with the 6PWM

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
	motor.PID_current_q.P = 3; 			
    motor.PID_current_q.I= 20; 			
    motor.PID_current_d.P= 3; 			
    motor.PID_current_d.I = 20;			
	motor.PID_current_d.D = 0.001;
    motor.LPF_current_q.Tf = 0.003f; 	// 1ms default
    motor.LPF_current_d.Tf = 0.003f; 	// 1ms default
	motor.PID_current_q.limit= 15;
	motor.PID_current_d.limit= 6;
	motor.current_limit = 0.5;    		//0.5 Amp current limit
	
	// init motor hardware
	//motor.useMonitoring(Serial);
	motor.monitor_downsample = 1;
	motor.monitor_variables =  _MON_CURR_Q | _MON_CURR_D; //_MON_VEL;// _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents
	motor.monitor_decimals = 4; //!< monitor outputs decimal places

	motor.init();

	// current sense init hardware
	current_sense.skip_align = true; //Current sense align is skipped, as sensor location and rotation is already known
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


	motor.controller = MotionControlType::velocity_openloop;    // set motion control to open loop for startup
	
	//Rotor alignment procedure
	target = 0;  
	_delay(2000); //Tweak this value so that rotor has enough time to settle, the larger load and lower pole count increases the settle time

	motor.zero_electric_angle = -0.1; //Motor not always stable at 0 offset, 0.1 offset in the rotation is good enough for the start
	target = -100; //Rampup to 100 rad/s
	unsigned long startup_timer = millis();

	//Open-loop ramp-up 
	while ((millis() - startup_timer) < 10000){
		motor.monitor();
		command.run();
		_delay(5);
	}

	//LPF_target.Tf = 0.001; 	//speedup target change for torque control
	//target = -0.15;			//Custom Torque target

	motor.current_sp_field_weakening = 0; 	//Field weakening is set to 0A

	//To sucessfully sensorlessly control BLDC, it is important to match motor Flux linkage at target speed, otherwise switchover is not smooth 
	//The initial Flux linkage is calulated automatically using provided KV value, but can change depending on rotational speed
	//Experimental coreless machine custom flux lingages: 
	//255mm prop: 0.035 Wb, 152mm prop: 0.017 Wb, 100mm prop 0.008 
	sensor.flux_linkage = 0.008; //Custom flux linkage override to kv estimated flux linkage 
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
    


void loop(){

	//Data collection loop, 100 Hz
	if(millis() - count_timer > 10){
		velocity_avg += motor.shaft_velocity;
		Iq_avg += motor.current.q;		
		Id_avg += motor.current.d;
		Vq_avg += motor.voltage.q;
		Vd_avg += motor.voltage.d;
		//Vin_avg += (((float)analogReadMilliVolts(VBUS_SENSE)/1000)*(R3+R4)/(R4)); //Overuses ADC and conflicts with current sensing and motor stuttering
		Iin_avg += current_sense.getDCCurrent(motor.electrical_angle);
		//temp_avg += (analogReadMilliVolts(PCB_TEMP)-500)/10; 						//Overuses ADC and conflicts with current sensing and motor stuttering
		loops++;
		count_timer = millis();
	}

	//Fault monitoring and handling, 10 Hz
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

	//Collected data printing to serial interface, 0.1 Hz
	if (millis()-timer_start > 10000){
		command.run();
		
		//Disable data printing to reduce stuttering above 20 000 erpm 
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


//High speed FOC loop running on Core 0 
void InnerFOCTask( void * parameter ) 
{
	//InnerFOCTask is only freeRTOS task that runs on Core 0, so  
	//Arduino built-in watchdog is disabled and manually reset every loop
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

		//Performance measurement
		/*if(currentMillis - lastMillis > 10000){
			Serial.print("Loops last second:");
			Serial.println(loops/10);

			lastMillis = currentMillis;
			loops = 0;
		}*/
		
		esp_task_wdt_reset();
	}
}

