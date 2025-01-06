#include "Arduino.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"
#include <encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h>
#include <Adafruit_NeoPixel.h>
#include "esp_task_wdt.h"



//Auxiliary pins
#define VBUS_SENSE 1
#define PCB_TEMP 2
#define LED_DIN 48
#define NUM_LEDS 1
#define PIXEL_COUNT 1

//Motor driver IC pins
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
float velocity_avg, Iq_avg, Id_avg, Vq_avg, Vd_avg, Vin_avg, Iin_avg, temp_avg = 0;

unsigned long timer_start, timer_start_2, count_timer = millis();
unsigned long loops = 0;

Adafruit_NeoPixel strip(PIXEL_COUNT, LED_DIN, NEO_GRB + NEO_KHZ800);

//Different motor presets
//BLDCMotor motor = BLDCMotor(6,0.30625,4000,6.83*1e-6);    //12.7 uH for tiny motor 4300 kV
//BLDCMotor motor = BLDCMotor(11,0.05125,690,4.56*1e-6);    //6.1 uH for large motor 690 kV
//BLDCMotor motor = BLDCMotor(4,0.1375,1100,19.45*1e-6);    //Plastic motor 957 kV
//BLDCMotor motor = BLDCMotor(7,0.01,3500,0.45*1e-6);       //Orange motor 2700 kV?
//BLDCMotor motor = BLDCMotor(4,0.525,1000,3.84*1e-7);      //PCB motor 
BLDCMotor motor = BLDCMotor(1,7,250,3.02*1e-4);             //Experimental Coreless motor 
//BLDCMotor motor = BLDCMotor(6,0.1,12000,0.82*1e-6);       //7300 kV

DRV8316_CSAGain CS_Gain = Gain_0V3;     //Single variable for all Current sense gain
unsigned int CS_Gain_mV = 300;          //150 * 2^(CS_Gain); //Needed for Low Side Current Sense 


MXLEMMINGObserverSensor sensor = MXLEMMINGObserverSensor(motor);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(INHA,INLA,INHB,INLB,INHC,INLC,DRV8316_CS,false);   //Use the right pins for your setup!
LowsideCurrentSense current_sense = LowsideCurrentSense(CS_Gain_mV,CS_A,CS_B,CS_C);             //Default gain is 0.15V/A

TaskHandle_t FOC_TASK;

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
  for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...

      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
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
