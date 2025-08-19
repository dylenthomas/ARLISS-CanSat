#include "Neo6M_UBXParser.h"
#include "Motor_PWM_Nano33.h"
#include "ArduinoBLE.h"
#include "Arduino_BMI270_BMM150.h"

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

const float Kp = 1; // distance error gain
const float KaP = 0.25; // heading proportional error gain
const float KaD = 0.0; // heading derivative error gain
const float KaI = 0.0; // heading integral error gain

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

double lat_initial = 0.0;
double lon_initial = 0.0;
float initial_heading;

GPSParser gps;
GPSParser::posllhData GPS_posllh = gps.getPOSLLH();
GPSParser::statusData GPS_status = gps.getSTATUS();
GPSParser::velnedData GPS_velned = gps.getVELNED();


void findHeading() {
	const float acceptableHeadingAcc = 2;
	const float findHeadingDC = 1.0;
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
	initial_heading = GPS_velned.heading * DEG_TO_RAD;
	delay(500);
	log("Heading accuracy is acceptable.");

	motorRight.setFreqAndDuty(FORWARD, PWMFreq, 0.0);
	motorLeft.setFreqAndDuty(FORWARD, PWMFreq, 0.0);
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

	if (!IMU.begin()) {
    log("Failed to initialize IMU!");
    while (1);
  }

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
	lat_initial = GPS_posllh.lat;
	lon_initial = GPS_posllh.lon;

	log("Initial GPS position found.");
	delay(250);
	log("Looking for current heading...");
	delay(2000);
	findHeading();

	log("<READY>");
}

bool wasPaused = false;

unsigned long lastprint = millis();
int printdelay = 200; // ms

void loop() {
	central = BLE.central();
	
	float heading = initial_heading;
	float speed;

	float u_r = 0.0;
	float u_l = 0.0;
	float alpha;
	float F;
	float Tp;
	float Td;
	float Ti = 0.0;
	float T;
	float u_r_internal = 0.0;
	float u_l_internal = 0.0;
	unsigned long last_time = micros();
	float last_alpha = 0.0;
	float dt = 0;
	float de = 0.0;

	float gx;
	float gy;
	float gz;

	double calc_heading = 0.0;
	double dt_heading = 0.0;
	unsigned long last_calc_heading = micros();
	float calc_hding_zero_thres = 0.1

	int iter = 0;

	while (central.connected()) {
		if (wasPaused) {
			motorRight.resume();
			motorLeft.resume();
			wasPaused = false;
		}

		while (Serial1.available()) {
			gps.addByte(Serial1.read());
		}
		
		if (IMU.gyroscopeAvailable()) {
			IMU.readGyroscope(gx, gy, gz);
			dt_heading = (micros() - last_calc_heading) * 1e-6;
			calc_heading += gz * dt_heading * DEG_TO_RAD;
			calc_heading = wrapToPi(calc_heading);
			if (calc_heading <= calc_hding_zero_thres) {
				calc_heading = 0.0;
			}

			last_calc_heading = micros();
		}

		if (gps.posllhChanged() && gps.velnedChanged()) {
			GPS_velned = gps.getVELNED();

			heading = wrapToPi(GPS_velned.heading * DEG_TO_RAD + calc_heading);
			speed = GPS_velned.gSpeed;
		}

		alpha = wrapToPi(initial_heading - heading); // angular error
			
		dt = max((micros() - last_time) * 1e-6, 1.0);
		de = wrapToPi(alpha - last_alpha);
		last_alpha = alpha;
		last_time = micros();

		F = Kp;
		Tp = KaP * alpha;
		Td = KaD *  de / dt;
		Ti += KaI * alpha * dt;

		Td = constrain(Td, -2, 2);
		Ti = constrain(Ti, -0.5, 0.5);

		T = Tp + Td + Ti;

		u_l_internal = F - T;
		u_r_internal = F + T;

		u_l = constrainFloat(abs(u_l_internal), minDutyCycle, maxDutyCycle);
		u_r = constrainFloat(abs(u_r_internal), minDutyCycle, maxDutyCycle);

		if (iter == 0) {
			if (u_l_internal >= 0) {
				motorLeft.setFreqAndDuty(FORWARD, PWMFreq, u_l);
			}
			else {
				motorLeft.setFreqAndDuty(REVERSE, PWMFreq, u_l);
			}

			if (u_r_internal >= 0) {
				motorRight.setFreqAndDuty(FORWARD, PWMFreq, u_r);
			}
			else {
				motorRight.setFreqAndDuty(REVERSE, PWMFreq, u_r);
			}
			iter++;
		}
		else {
			if (u_r_internal >= 0) {
				motorRight.setFreqAndDuty(FORWARD, PWMFreq, u_r);
			}
			else {
				motorRight.setFreqAndDuty(REVERSE, PWMFreq, u_r);
			}

			if (u_l_internal >= 0) {
				motorLeft.setFreqAndDuty(FORWARD, PWMFreq, u_l);
			}
			else {
				motorLeft.setFreqAndDuty(REVERSE, PWMFreq, u_l);
			}
			iter = 0;
		}
		
		if (millis() - lastprint > printdelay) {
			//log("Control commands: u_r: " + String(u_r) + ", u_l: " + String(u_l) + ", rho: " + String(rho) + ", alpha: " + String(alpha) + ", F: " + String(F) + ", T: " + String(T));
			//log("Td: " + String(Td) + ", a: " + String(alpha) + ", F: " + String(F) + ", T: " + String(T) + ", u_r: " + String(u_r_internal) + ", u_l: " + String(u_l_internal));
			log("T: " + String(T) + ", u_r: " + String(u_r_internal) + ", u_l: " + String(u_l_internal) + ", hding: " + String(heading) + ",  alpha: " + String(alpha));

			lastprint = millis();
		}
	}

	while (!central.connected()) {
		central = BLE.central();
		BLE.advertise();

		if (!wasPaused) {
			motorRight.pause();
			motorLeft.pause();
			wasPaused = true;
		}
	}
}