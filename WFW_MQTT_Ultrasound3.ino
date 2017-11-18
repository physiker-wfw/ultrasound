//Project for HOMIE-like MQTT control of water level and temperature
// Version 1.4 
// not yet ready 18.11.2017 @ WFW
//
#include <Homie.h>
#include <SoftwareSerial.h>  //For Ultrasound serial interfacing
#include <OneWire.h>
#include <DallasTemperature.h>

#define echoPin D6
#define trigPin D5
#define ONE_WIRE_BUS D7   // on pin D7 (a 4.7K resistor is necessary)

// SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic = false, unsigned int buffSize = 64)
SoftwareSerial mySerial = SoftwareSerial(echoPin, trigPin);
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

HomieNode ZisterneNode("zisterne", "sensor");

int min_timeout = 10000; //in ms
float diff = 0.05;
long lastMsg = 0;
byte readByte;
byte read_buffer[4];
byte crcCalc;
String message;
word values[71];
int valuesLength = 70;
int valuesDrop = 16;
long now = 0; //in ms
float temp = 0.0;
float tmp = 0.0;
float difftemp = 0.05;
float alarmtemperature = 1.6;
int alarmindex = 0;

void setup() {
	Serial.begin(115200);
	Serial << endl << endl;
	Homie_setFirmware("NaundorfITZisterne", "1.4");
	Homie_setBrand("NaundorfIT");
	Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
	ZisterneNode.advertise("ultrasoundlevel");
	ZisterneNode.advertise("temperature");	
	ZisterneNode.advertise("alarm");
	Homie.setup();
	Serial << "Sensor ZisterneNode startet! " << endl;
	mySerial.begin(9600);
	sensors.begin();
}

void setupHandler() {
}
void loop() { Homie.loop(); }

void loopHandler() {
	double vol = 0.123;
	vol = usvolumen();
	Serial << "Volume [liters]: " << vol << endl;
	ZisterneNode.setProperty("ultrasoundlevel").send(String(vol));
	readTsensor(temp);



	// Empty the data buffer (data that were accumulated during pause):
	while (mySerial.available() > 0) { 
		readByte = mySerial.read(); 
	}
}



//##################################################################
bool checkBound(float newValue, float prevValue, float maxDiff) {
	//return(true);    //Use this line to switch off checking for changes
	return newValue < prevValue - maxDiff || newValue >= prevValue + maxDiff;
}

//  ##################################################################
word usdist() {
	word distance, sum;
	for (int n = valuesLength; n>0; n--) {
		while (!usdistance(distance)) { delay(100); }    // Try different delays to give time to other processes
		values[n - 1] = distance;
	}
	BubbleSort();
	sum = 0;
	for (int n = valuesLength - valuesDrop; n>valuesDrop; n--) {    // Throw smallest (valuesDrop) and largest (valuesDrop) value away
		sum += values[n - 1];                     // and average the remaining values.
	}
	sum = sum / (valuesLength - 2 * valuesDrop);
	return sum;
}

//  ##################################################################
double usvolumen() {
	word distance;
	double sum;
	for (int n = valuesLength; n>0; n--) {
		while (!usdistance(distance)) { delay(100); }    // Try different delays to give time to other processes
		values[n - 1] = distance;
	}
	BubbleSort();
	sum = 0.0;
	for (int n = valuesLength - valuesDrop; n>valuesDrop; n--) {    // Throw smallest (valuesDrop) and largest (valuesDrop) value away
		sum += values[n - 1];                     // and average the remaining values.
	}
	sum = sum / (valuesLength - 2.0 * valuesDrop);
	sum = 1940.0 - sum;                         // water height from ground in mm
	sum = sum * 37.;                           // Water volumen in liters
	return sum;
}
// ########################################################
// Bubble Sort Function for Descending Order 
void BubbleSort() {
	int i, j, flag = 1;    // set flag to 1 to start first pass
	int temp;             // holding variable
	for (i = 1; (i <= valuesLength) && flag; i++) {
		flag = 0;
		for (j = 0; j < (valuesLength - 1); j++) {
			if (values[j + 1] > values[j]) {     // ascending order simply changes to <
				temp = values[j];             // swap elements
				values[j] = values[j + 1];
				values[j + 1] = temp;
				flag = 1;               // indicates that a swap occurred.
			}
		}
	}
	return;
}
//  ##################################################################
bool usdistance(word& distance) {
	int validdata = false;
	// Check the availability of data in COM port
	if (mySerial.available() < 1) { return false; }

	// Shift the read buffer to lower index
	for (byte loopstep = 0; loopstep <= 2; loopstep++) {
		read_buffer[loopstep] = read_buffer[loopstep + 01];
	}

	// Read in a byte and put it into the buffer (at position 3)
	readByte = mySerial.read();
	read_buffer[03] = readByte;

	// Analyze the buffer: Startbyte is FF, then the two data bytes, then checksum
	if (read_buffer[00] != 0xff) {
		return false; // this is not the beginning of the data 
	};
	crcCalc = read_buffer[00] + read_buffer[01] + read_buffer[02];
	if (read_buffer[03] != crcCalc) {
		return false; // checksum data packet does not coincide
	};

	// Calculation of the distance: Two bytes word gives distance in millimeters
	distance = (read_buffer[01] * 0xff) + read_buffer[02];
	// Create 16 bits values from 8 bits data
	// distance = read_buffer[1] << 8 | read_buffer[2];
	return true;
}
//####################################################################
void readTsensor(float t) {
	now = millis();
	if (now - lastMsg > min_timeout) {
		sensors.requestTemperatures(); // Send the command to get temperatures
		lastMsg = now;
		float newTemp = sensors.getTempCByIndex(0);
		// The following if structure tests the temperature: If for 2 min (12x 10s) 
		// the temperature is below the alaramtemperature then a MQTT alarm is send.
		// It is cleared if it is for 2 min above.
		if (newTemp < alarmtemperature) {
			alarmindex += 1;
			if (alarmindex > 11) {
				ZisterneNode.setProperty("alarm").send("freezing");
			}
		}
		else
		{
			alarmindex -= 1;
			if (alarmindex < 0) {
				alarmindex = 0;
				ZisterneNode.setProperty("alarm").send("");
			}
		}
		if (checkBound(newTemp, tmp, difftemp)) {
			tmp = newTemp;
			ZisterneNode.setProperty("temperature").send(String(tmp));
			Serial << "Temperature changed to " << tmp << endl;
		}
		t = newTemp;
	}
	return;
}