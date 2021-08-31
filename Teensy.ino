/*
	Created by: Hudzaifah Al Fatih
	This code is adapted from the one created by Brian Miller
	https://circuitcellar.com/research-design-hub/impedance-spectroscopy-using-the-ad5933/
*/

#define BLYNK_PRINT Serial
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <math.h>

#include <Wire.h>
#include "AD5933.h"
#include "I2CEEPROM.h"

#define PIN_MPO V0
#define PIN_MYO V1

#define START_FREQ  (1000)
#define FREQ_INCR   (500)
#define NUM_INCR    (200)
#define REF_RESIST  (100000)
#define NUM_SAMPLE_SETTLING_CYCLES  (25)


char auth[] = "yourAUTH";
char ssid[] = "yourSSID";
char pass[] = "yourpassword";

double impedance, MPO, MYO;

#define ESP8266_BAUD 115200
ESP8266 wifi(&Serial);

BLYNK_READ()
{
  // This command writes Arduino's uptime in seconds to Virtual Pin (5)
  impedance = getImpedance();
  MPO, MYO;
  MPO = 6.57*log((3.19-9.28)/(impedance-9.28)-1);
  MYO = 8.81*log((1.90-9.30)/(impedance-9.30)-1);
  Blynk.virtualWrite(PIN_MPO, MPO);
  Blynk.virtualWrite(PIN_MYO, MYO);
}

void notifyOnButtonPress()
{
  if (MYO == 90)  {
    // Note:
    //   We allow 1 notification per 5 seconds for now.
	Blynk.notify("Your MYO level is above normal.");

    // You can also use {DEVICE_NAME} placeholder for device name,
    // that will be replaced by your device name on the server side.
    //Blynk.notify("Yaaay... {DEVICE_NAME}  button is pressed!");
  }
  else if (MPO == 100) {
	Blynk.notify("Your MPO level is above normal.");
  }
}

void setup() {
	// Debug console
	pinMode(1, OUTPUT);
	digitalWrite(1, LOW);
	pinMode(3, OUTPUT);
	digitalWrite(3, LOW);
	pinMode(4, OUTPUT);
	digitalWrite(4, HIGH);
	// Begin I2C
	Wire.begin();

	// Begin serial at 9600 baud for output
	Serial.begin(9600);
	delay(2000);
	Serial.println("AD5933 Electrochemical Impedance Spectrometer");

	// Perform initial configuration. Fail if any one of these fail.
	if (!(AD5933::reset() &&
		AD5933::setRange(1) &&
		AD5933::setInternalClock(true) &&
		AD5933::setStartFrequency(START_FREQ) &&
		AD5933::setIncrementFrequency(incrementFrequency) &&
		AD5933::setNumberIncrements(NUM_INCR) &&
		AD5933::setPGAGain(PGA_GAIN_X5)))
	{
		Serial.println("FAILED in initialization!");
		while (true);
	}
	AD5933::setNumberSettlingTimeCycles(NUM_SAMPLE_SETTLING_CYCLES);

	// Set ESP8266 baud rate
	Serial.begin(ESP8266_BAUD);
	Blynk.begin(auth, wifi, ssid, pass);
	attachInterrupt(notifyOnButtonPress, CHANGE);
}

void loop() {
	Blynk.run();
} 

//float gain[NUM_INCR+1];  
//int phase[1];
byte cmd = 0;
unsigned long startFrequency = START_FREQ;
float incrementFrequency= FREQ_INCR;
unsigned int numIncrements;
unsigned int PGA_gain;
unsigned int range;
unsigned int nsti;
unsigned int refResistor = REF_RESIST;
unsigned int TIA_gain = 0;
int  calibrationBand = 0;

uint16_t  EEaddress;
uint16_t EEtemp;
I2CEEPROM ee(0x50);

void SetReferenceResistor() {
	refResistor = Serial.parseInt();
	Serial.println(refResistor);   //echo it
}

void SetCalibrationBand() {
	calibrationBand = Serial.parseInt();
	Serial.println(calibrationBand);   //echo it
	switch (calibrationBand) {

	case 1:
		refResistor = 100;
		TIA_gain = 1;   // TIA feedback resistor = 220 ohms
		range = 2;    // excitation voltage = 1Vpp
		PGA_gain = 1; // AD5933 PGA gain=1
		break;
	case 2:
		refResistor = 500;
		TIA_gain = 1;   // TIA feedback resistor = 220 ohms
		range = 2;    // excitation voltage = 1Vpp
		PGA_gain = 5; // AD5933 PGA gain=5
		break; 
	case 3:
		refResistor = 1000;
		TIA_gain = 1;   // TIA feedback resistor = 220 ohms
		range = 1;    // excitation voltage = 2Vpp
		PGA_gain = 5; // AD5933 PGA gain=5
		break;
	case 4:
		refResistor = 10000;
		TIA_gain = 0;   // TIA feedback resistor = 220 ohms
		range = 4;    // excitation voltage = 2Vpp
		PGA_gain = 1; // AD5933 PGA gain=5
		break;
	case 5:
		refResistor = 100000;
		TIA_gain = 0;   // TIA feedback resistor = 100K ohms
		range = 1;    // excitation voltage = 2 Vpp
		PGA_gain = 1; // AD5933 PGA gain=1
		break;
	case 6:
		refResistor = 1000000;
		TIA_gain = 0;   // TIA feedback resistor = 100K ohms
		range = 1;    // excitation voltage = 2 Vpp
		PGA_gain = 5; // AD5933 PGA gain=1
		break;
	default:
		refResistor = 100;
		TIA_gain = 1;   // TIA feedback resistor = 20 ohms
		range = 2;    // excitation voltage = 1Vpp
		PGA_gain = 1; // AD5933 PGA gain=5
	}
	// now set up the 3 parameters on AD5933 board
	// set up the TIA feedback resistor via mux
	if (TIA_gain == 1) digitalWrite(1, HIGH);
	if (TIA_gain == 0) digitalWrite(1, LOW);
    // set AD5933 DAC output voltage amplitude
	AD5933::setRange((byte)range);
	// set ADC PGA gain 
	if (PGA_gain == 1) {
		AD5933::setPGAGain(PGA_GAIN_X1);
	}
	if (PGA_gain == 5) {
		AD5933::setPGAGain(PGA_GAIN_X5);
	}
}

void SetStartFrequency() {
	startFrequency = Serial.parseInt();
	Serial.println(startFrequency);   //echo it
	AD5933::setStartFrequency(startFrequency);
}

void SetIncrFrequency() {
	incrementFrequency = Serial.parseFloat();
	Serial.println(incrementFrequency);   //echo it
	AD5933::setIncrementFrequency(incrementFrequency);
}

void SetNumIncrements() {
	numIncrements = Serial.parseInt();
	Serial.println(numIncrements);   //echo it
	AD5933::setNumberIncrements(numIncrements);
}

void SetPgaGain() {
	PGA_gain = Serial.parseInt();
	Serial.println(PGA_gain);   //echo it
	if (PGA_gain == 1) {
		AD5933::setPGAGain(PGA_GAIN_X1);
	}
	if (PGA_gain == 5) {
		AD5933::setPGAGain(PGA_GAIN_X5);
	}
}

void SetRange() {
	range = Serial.parseInt();
	Serial.println(range);   //echo it
	AD5933::setRange((byte) range);
}

void SetNumberSettlingTimeIntervals() {
	nsti = Serial.parseInt();
	Serial.println(nsti);
	AD5933::setNumberSettlingTimeCycles(nsti);
}

void SetTIAGain() {
	TIA_gain = Serial.parseInt();
	Serial.println(TIA_gain);   //echo it
	if (TIA_gain == 1) digitalWrite(1, HIGH);
	if (TIA_gain == 0) digitalWrite(1, LOW);
}

void frequencySweepRaw() {
	union Data {
		float g;
		uint8_t b[4];
	};
	union Data data;
	digitalWrite(4, LOW);
	digitalWrite(3, HIGH);
    // Create variables to hold the impedance data and track frequency
	int real, imag, i = 0;
	float cfreq = startFrequency;

    // Initialize the frequency sweep
	/*
    if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // place in standby
          AD5933::setControlMode(CTRL_INIT_START_FREQ) && // init start freq
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
         {
             Serial.println("Could not initialize frequency sweep...");
         }
		 */
	AD5933::setPowerMode(POWER_STANDBY);        // place in standby
	AD5933::setControlMode(CTRL_INIT_START_FREQ);  // init start freq
	delay(1000);
	AD5933::setControlMode(CTRL_START_FREQ_SWEEP); // begin frequency sweep

    // Perform the actual sweep
	EEaddress = 1024*(calibrationBand - 1);  // each calibration table is allocated 1024 bytes (up to 256 floating point values)
    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        // Get the frequency data for this frequency point
        if (!AD5933::getComplexData(&real, &imag)) {
            Serial.println("Could not get raw frequency data...");
        }

        // Print out the frequency data
        Serial.print(cfreq);
        Serial.print(", ");
        Serial.print(real);
        Serial.print(", ");
        Serial.print(imag);
		Serial.print(", ");
        // Compute impedance
        double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
		// load the gain factor from EEPROM calibration array
		// Calibration is done from 1000 Hz  to 101000 Hz in 200 steps
		// so applicable calibration entry equal
		// eeprom address  = EEaddress + 4 * (INT ((frequency-1000) /500))
		EEtemp = 4 * (int)((cfreq - 1000.0) / 500);
//		Serial.print(EEtemp);     // tested and EEtemp values were OK
//		Serial.print(", ");
		EEtemp += EEaddress;  // add in the calibration Band offset
		data.b[0] = ee.read(EEtemp);
		data.b[1] = ee.read(EEtemp+1);
		data.b[2] = ee.read(EEtemp+2);
		data.b[3] = ee.read(EEtemp + 3);
		float gain = data.g;
        double impedance = 1/(magnitude*gain);
        Serial.println(impedance);

        // Increment the frequency
        cfreq += incrementFrequency;
        AD5933::setControlMode(CTRL_INCREMENT_FREQ);
    }

    Serial.println("1");   // indicates success

    // Set AD5933 power mode to standby when finished
	AD5933::setPowerMode(POWER_STANDBY);
	digitalWrite(4, HIGH);
	digitalWrite(3, LOW);
 }

double getImpedance() {
	union Data {
		float g;
		uint8_t b[4];
	};
	union Data data;
	digitalWrite(4, LOW);
	digitalWrite(3, HIGH);
    // Create variables to hold the impedance data and track frequency
	int real, imag, i = 0;
	float cfreq = startFrequency;

    // Initialize the frequency sweep
	/*
    if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // place in standby
          AD5933::setControlMode(CTRL_INIT_START_FREQ) && // init start freq
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
         {
             Serial.println("Could not initialize frequency sweep...");
         }
		 */
	AD5933::setPowerMode(POWER_STANDBY);        // place in standby
	AD5933::setControlMode(CTRL_INIT_START_FREQ);  // init start freq
	delay(1000);
	AD5933::setControlMode(CTRL_START_FREQ_SWEEP); // begin frequency sweep

    // Perform the actual sweep
	EEaddress = 1024*(calibrationBand - 1);  // each calibration table is allocated 1024 bytes (up to 256 floating point values)
    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        // Get the frequency data for this frequency point
        if (!AD5933::getComplexData(&real, &imag)) {
            Serial.println("Could not get raw frequency data...");
        }

        // Print out the frequency data
        Serial.print(cfreq);
        Serial.print(", ");
        Serial.print(real);
        Serial.print(", ");
        Serial.print(imag);
		Serial.print(", ");
        // Compute impedance
        double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
		// load the gain factor from EEPROM calibration array
		// Calibration is done from 1000 Hz  to 101000 Hz in 200 steps
		// so applicable calibration entry equal
		// eeprom address  = EEaddress + 4 * (INT ((frequency-1000) /500))
		EEtemp = 4 * (int)((cfreq - 1000.0) / 500);
//		Serial.print(EEtemp);     // tested and EEtemp values were OK
//		Serial.print(", ");
		EEtemp += EEaddress;  // add in the calibration Band offset
		data.b[0] = ee.read(EEtemp);
		data.b[1] = ee.read(EEtemp+1);
		data.b[2] = ee.read(EEtemp+2);
		data.b[3] = ee.read(EEtemp + 3);
		float gain = data.g;
        double impedance = 1/(magnitude*gain);
        Serial.println(impedance);

        // Increment the frequency
        cfreq += incrementFrequency;
        AD5933::setControlMode(CTRL_INCREMENT_FREQ);
    }

    Serial.println("1");   // indicates success

    // Set AD5933 power mode to standby when finished
	AD5933::setPowerMode(POWER_STANDBY);
	digitalWrite(4, HIGH);
	digitalWrite(3, LOW);
	return impedance;
 }

void Calibrate() {

	digitalWrite(4, LOW);
	digitalWrite(3, HIGH);
	float impedAccum;
	union Data {
		float g;
		uint8_t b[4];
	};
	union Data data;
	// Create variables to hold the impedance data and track frequency
	int real, imag, i = 0;
	int  cfreq = startFrequency;
	EEaddress = 1024*(calibrationBand-1); // each calibration band is allocated 1024 bytes (256 floats)
	// Initialize the frequency sweep
	/*
	if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // place in standby
		AD5933::setControlMode(CTRL_INIT_START_FREQ) && // init start freq
		AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
	{
		Serial.println("Could not initialize frequency sweep...");
	}
	*/
	AD5933::setPowerMode(POWER_STANDBY);        // place in standby
	AD5933::setControlMode(CTRL_INIT_START_FREQ);  // init start freq
	delay(1000);
	AD5933::setControlMode(CTRL_START_FREQ_SWEEP); // begin frequency sweep


	// Perform the actual sweep
	while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
		impedAccum = 0;
		// Get the impedance value for this frequency point
		if (!AD5933::getComplexData(&real, &imag)) {
			Serial.println("Could not get raw frequency data...");
		}
		// calculate the gain
		impedAccum = (float)(1.0 / refResistor) / sqrt(pow(real, 2) + pow(imag, 2));
		AD5933::setControlMode(CTRL_REPEAT_FREQ);
		if (!AD5933::getComplexData(&real, &imag)) {
			Serial.println("Could not get raw frequency data...");
		}
		// calculate the gain
		impedAccum += (float)(1.0 / refResistor) / sqrt(pow(real, 2) + pow(imag, 2));
		AD5933::setControlMode(CTRL_REPEAT_FREQ);
		if (!AD5933::getComplexData(&real, &imag)) {
			Serial.println("Could not get raw frequency data...");
		}
		// calculate the gain
		impedAccum += (float)(1.0 / refResistor) / sqrt(pow(real, 2) + pow(imag, 2));
		AD5933::setControlMode(CTRL_REPEAT_FREQ);
		if (!AD5933::getComplexData(&real, &imag)) {
			Serial.println("Could not get raw frequency data...");
		}
		// calculate the gain
		impedAccum += (float)(1.0 / refResistor) / sqrt(pow(real, 2) + pow(imag, 2));
		data.g = impedAccum/4;

		// Print out the frequency and real/imag data
		Serial.print(cfreq);
		Serial.print(", ");
		Serial.print(real);
		Serial.print(", ");
		Serial.print(imag);
		Serial.print(", ");

		//data.g = (float)(1.0 / refResistor) / sqrt(pow(real, 2) + pow(imag, 2));
   		Serial.printf("%1.7e", data.g);  
		Serial.println("");
		// store gain to EEPROM 
		ee.write(EEaddress++, data.b[0]);
		ee.write(EEaddress++, data.b[1]);
		ee.write(EEaddress++, data.b[2]);
		ee.write(EEaddress++, data.b[3]);

		// Increment the frequency
		cfreq += FREQ_INCR;
		AD5933::setControlMode(CTRL_INCREMENT_FREQ);
	}
	Serial.println("1");   

						   // Set AD5933 power mode to standby when finished
	AD5933::setPowerMode(POWER_STANDBY);
	digitalWrite(4, HIGH);
	digitalWrite(3, LOW);
}