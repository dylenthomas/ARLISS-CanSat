#include "BasicLinearAlgebra.h"
#include "Neo6M_UBXParser.h"
#include "Motor_PWM_Nano33.h"
#include "ArduinoBLE.h"

constexpr bool debugging = false; // set to true if you want to see outputs to Serial

const float a = 6378137.0; // equitorial radius of the earth 

constexpr int mR_reverse_pin = 3;
constexpr int mR_forward_pin = 9;
constexpr int mL_reverse_pin = 6;
constexpr int mL_forward_pin = 5;
constexpr int PWMFreq = 25000; // Hz
const int minPW = 0; // us
const int maxPW = 1000; // us
const float minDutyCycle = 0.0; // *100%
const float maxDutyCycle = 1.0; // *100%
const int stoppingDist = 3; // m

const float Kp = 5; // distance error gain
const float KaP = 2;  // heading proportional error gain
const float KaD = -5; // heading derivative error gain
const float KaI = 1; // heading integral error gain

Motor_Control motorRight(mR_forward_pin, mR_reverse_pin);
Motor_Control motorLeft(mL_forward_pin, mL_reverse_pin);

const char* uuidService = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* uuidNotify = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* uuidCommands = "19b10002-e8f2-537e-4f6c-d104768a1214";

const char* BLEname = "Nano33BLE";

BLEService nanoService(uuidService);
BLECharacteristic notifier(uuidNotify, BLENotify, 64);
BLEByteCharacteristic commands(uuidCommands, BLEWrite);
BLEDevice central;

BLA::Matrix<2> xyInitial = { 0, 0 };

GPSParser gps;
GPSParser::posllhData GPS_posllh = gps.getPOSLLH();
GPSParser::statusData GPS_status = gps.getSTATUS();
GPSParser::velnedData GPS_velned = gps.getVELNED();

int targetX = 0;
int targetY = 0;
int targetHdg = 0;

void findHeading() {
	const float acceptableHeadingAcc = 2;
	const float findHeadingDC = 0.8;
	float headingAcc = GPS_velned.cAcc;

	motorRight.setFreqAndDuty(FORWARD, PWMFreq, findHeadingDC);
	motorLeft.setFreqAndDuty(FORWARD, PWMFreq, findHeadingDC);

	while (headingAcc > acceptableHeadingAcc || headingAcc == 0.0) {
		while (Serial1.available()) {
			gps.addByte(Serial1.read());
		}
		if (gps.velnedChanged()) {
			GPS_velned = gps.getVELNED();
		}

		headingAcc = GPS_velned.cAcc;
		log("Heading Accuracy: " + String(headingAcc));
	}
	delay(500);
	log("Heading accuracy is acceptable.");

	motorRight.setFreqAndDuty(FORWARD, PWMFreq, 0.0);
	motorLeft.setFreqAndDuty(FORWARD, PWMFreq, 0.0);
}

BLA::Matrix<2> computeGPSpos(double lat, double lon) {
	float x = a * lat * DEG_TO_RAD;
	float y = a * lon * DEG_TO_RAD;
	BLA::Matrix<2> xy = { x, y };

	return xy;
}

bool getTarget(String command) {
	/*
	Command structure:
		X_Y - where X is the target X and Y is the target Y both in meters relative to current position
	*/

	int underscores = 0;
	int underscoreInd = 0; // there should only be one underscore
	for (int i = 0; i < command.length(); i++) {
		if (command.charAt(i) == '_') { 
			underscoreInd = i;
			underscores++;
		}
	}
	if (underscores != 1) { return false; }
	
	targetX = command.substring(0, underscoreInd).toInt();
	targetY = command.substring(underscoreInd + 1).toInt();

	return true;
}

float wrapToPi(float a) {
	if (a > PI) { a -= 2 * PI; }
	else if (a <= -PI) { a += 2 * PI; }
	return a;
}

void log(String data) {
	Serial.println(data);
	notifier.writeValue(data.c_str(), data.length());
}

void log(const char* data) {
	log(String(data));
}

static float constrainFloat(float x, float min, float max) {
	return ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)));
}

void setup() {
	Serial.begin(115200);
	while (!Serial && debugging);

	Serial1.begin(9600); // gps serial

	if (!BLE.begin()) {
		Serial.println("Bluetooth failed to initialize.");
		while (1);
	}
	else {
		Serial.println("Bluetooth started");
	}

	/* configure bluetooth */
	BLE.setDeviceName(BLEname);
	BLE.setLocalName(BLEname);
	BLE.setAdvertisedService(nanoService);
	nanoService.addCharacteristic(notifier);
	nanoService.addCharacteristic(commands);
	BLE.addService(nanoService);

	/* initialize characteristics */
	notifier.writeValue((uint8_t)0);
	commands.writeValue(0);

	Serial.println("Started advertising bluetooth.");

	/* wait for central device to connect */
	while (!central) {
		central = BLE.central();
		BLE.advertise();
	}

	Serial.print("Connected to central: ");
	Serial.println(central.address());

	/* wait for central to connect to notifications */
	while (!notifier.subscribed()) {
		central = BLE.central();
	}

	log("<CONNECTED>");

	/* wait for a valid initial location from gps */
	log("Watiting for initial GPS position...");
	while (!gps.posllhChanged() || !GPS_status.gpsFixOk) {
		while (Serial1.available()) {
			unsigned char c = Serial1.read();
			gps.addByte(c);
		}

		if (gps.statusChanged()) {
			GPS_status = gps.getSTATUS();
		}
	}
	GPS_posllh = gps.getPOSLLH();
	xyInitial = computeGPSpos(GPS_posllh.lat, GPS_posllh.lon);

	log("Initial GPS position found.");
	delay(250);
	log("Looking for current heading...");
	delay(2000);
	findHeading();

	/* Tell the client the robot is ready to recieve commands */
	log("<READY>");
}

String ble_string = "";
bool ble_terminated = false;
bool commandRecieved = false;

bool wasPaused = false;

unsigned long lastprint = millis();
int printdelay = 200; // ms

void loop() {
	central = BLE.central();
	
	BLA::Matrix<2> xyPos;
	float heading;
	float speed;

	float u_r = 0.0;
	float u_l = 0.0;
	float alpha = 0.0;
	float F = 0.0;
	float Tp = 0.0;
	float Td = 0.0;
	float Ti = 0.0;
	float T = 0.0;
	float theta = 0.0;
	float yError = 0.0;
	float xError = 0.0;
	float rho = 0.0;
	float u_r_internal = 0.0;
	float u_l_internal = 0.0;
	unsigned long last_t = millis();
	float last_alpha = 0.0;
	float dt = 0;
	float de = 0.0;

	while (central.connected()) {
		if (wasPaused) {
			motorRight.resume();
			motorLeft.resume();
			wasPaused = false;
		}

		if (commands.written()) {
			char c = commands.value();

			if (c == '\0') {
				ble_terminated = true;
			}
			else {
				ble_string += c;
			}
		}

		if (ble_terminated) {
			if (!getTarget(ble_string)) {
				log("Invalid command format, Recieved: " + ble_string);
				delay(250);
			}
			else {
				commandRecieved = true;
				log("Recieved distances: X: " + String(targetX) + ", Y: " + String(targetY));
				delay(250);
			}

			ble_string = "";
			ble_terminated = false;
		}

		while (Serial1.available()) {
			gps.addByte(Serial1.read());
		}

		//if (gps.posllhChanged() && gps.velnedChanged()) {
		//	GPS_posllh = gps.getPOSLLH();
		//	GPS_velned = gps.getVELNED();

		//	xyPos = computeGPSpos(GPS_posllh.lat, GPS_posllh.lon) - xyInitial;
		//	heading = (GPS_velned.heading * DEG_TO_RAD) - PI; // convert to [-pi, pi]
		//	speed = sqrt(pow(GPS_velned.velN, 2) + pow(GPS_velned.velE, 2));

			//log("[" + String(millis()) + "]" + " - X:" + String(xyPos(0)) + ",Y: " + String(xyPos(1)) + ",Heading: " + String(heading) + ",Speed: " + String(speed) + "\n");
		//}

		//if (gps.statusChanged()) {
		//	GPS_status = gps.getSTATUS();
		//}

		/* Control Algorithim 
			Take the total distance we want to travel and traverse it in steps.
			Instead of trying to make a controller converge onto the final destination, make it converge to a point at the end of a unit vector
				which is pointed in the direction of the desired heading or angular displacement
		*/
	
		if (commandRecieved && gps.posllhChanged() && gps.velnedChanged()) {
			// Start the controller
			xyPos = computeGPSpos(GPS_posllh.lat, GPS_posllh.lon) - xyInitial;
			xError = targetX - xyPos(0);
			yError = targetY - xyPos(1);
			rho = sqrt(pow(xError, 2) + pow(yError, 2));
		}

		if (rho > stoppingDist && gps.posllhChanged() && gps.velnedChanged()) {
			GPS_posllh = gps.getPOSLLH();
			GPS_velned = gps.getVELNED();

			xyPos = computeGPSpos(GPS_posllh.lat, GPS_posllh.lon) - xyInitial;
			heading = (GPS_velned.heading * DEG_TO_RAD) - PI; // convert to [-pi, pi]
			speed = GPS_velned.gSpeed;
			//speed = sqrt(pow(GPS_velned.velN, 2) + pow(GPS_velned.velE, 2));

			xError = targetX - xyPos(0);
			yError = targetY - xyPos(1);
			rho = sqrt(pow(xError, 2) + pow(yError, 2)); // distance error
			theta = atan2(yError, xError); // angular displacement
			alpha = wrapToPi(theta - heading); // angular error
			
			dt = (millis() - last_t) / 1000;
			de = alpha - last_alpha;
			last_alpha = alpha;
			last_t = millis();

			//F = Kp * rho;
			F = Kp;
			Tp = KaP * alpha;
			Td = KaD *  de / dt;
			Ti += KaI * alpha * dt;
			T = Tp + Td + Ti;

			u_l_internal = F - T;
			u_r_internal = F + T;

			u_l = constrainFloat(abs(u_l_internal), minDutyCycle, maxDutyCycle);
			u_r = constrainFloat(abs(u_r_internal), minDutyCycle, maxDutyCycle);

			if (u_l_internal >= 0) {
				//motorLeft.setPulseWidth(FORWARD, u_l_control);
				motorLeft.setFreqAndDuty(FORWARD, PWMFreq, u_l);
			}
			else {
				//motorLeft.setPulseWidth(REVERSE, u_l_control);
				motorLeft.setFreqAndDuty(REVERSE, PWMFreq, u_l);
			}

			if (u_r_internal >= 0) {
				//motorRight.setPulseWidth(FORWARD, u_r_control);
				motorRight.setFreqAndDuty(FORWARD, PWMFreq, u_r);
			}
			else {
				//motorRight.setPulseWidth(REVERSE, u_r_control);
				motorRight.setFreqAndDuty(REVERSE, PWMFreq, u_r);
			}
		}
		else if (rho < stoppingDist) {
			motorRight.pause();
			motorLeft.pause();
			log("I have reached my position \\o/");
			while (1);
		}
		
		if (millis() - lastprint > printdelay && commandRecieved) {
			//log("Control commands: u_r: " + String(u_r) + ", u_l: " + String(u_l) + ", rho: " + String(rho) + ", alpha: " + String(alpha) + ", F: " + String(F) + ", T: " + String(T));
			log("Td: " + String(Td) + ", a: " + String(alpha) + ", F: " + String(F) + ", T: " + String(T) + ", u_r: " + String(u_r_internal) + ", u_l: " + String(u_l_internal));
			lastprint = millis();
		}
	}

	while (!central.connected()) {
		central = BLE.central();
		BLE.advertise();

		motorRight.pause();
		motorLeft.pause();
		wasPaused = true;
	}
}