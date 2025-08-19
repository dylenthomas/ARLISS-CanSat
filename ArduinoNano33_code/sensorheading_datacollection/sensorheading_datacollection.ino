#include "Motor_PWM_Nano33.h"
#include "ArduinoBLE.h"
#include "BasicLinearAlgebra.h"
#include "Arduino_BMI270_BMM150.h"
#include "Neo6M_UBXParser.h"

#define DEBUGGING false

constexpr float data_samplerate = 10; // Hz

constexpr int mR_reverse_pin = 3;
constexpr int mR_forward_pin = 9;
constexpr int mL_reverse_pin = 6;
constexpr int mL_forward_pin = 5;
constexpr int PWMFreq = 25000; // Hz

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

GPSParser gps;
GPSParser::posllhData GPS_posllh = gps.getPOSLLH();
GPSParser::statusData GPS_status = gps.getSTATUS();
GPSParser::velnedData GPS_velned = gps.getVELNED();

float initial_heading;
float gyro_heading;
float gps_heading;

BLA::Matrix<1, 2> G;
BLA::Matrix<1, 1> xhat;
BLA::Matrix<2, 1> z;
BLA::Matrix<1, 1> P = { 0 };
BLA::Matrix<2,1> C = { // tuneable constants
  	1,
    1
};
double gps_variance = pow(0.5 * DEG_TO_RAD, 2);
double gyro_variance = pow(0.07 * DEG_TO_RAD, 2);
BLA::Matrix<2,2> R = { // sensor variance
    gps_variance, 0,
    0, gyro_variance
};
BLA::Matrix<1, 1> Q = { 0.1 }; // covariance of the process noise (this can help account for drift)
BLA::Matrix<1, 1> A = { 1 }; // state transition model

void predictFilter() {
    xhat = A * xhat;
    P = A * P * ~A + Q;
}

void updateFilter(BLA::Matrix<2,1> z) {
/*
z is a vector containing the gps heading and the calculated gyro heading
*/
    BLA::Matrix<1, 1> I = { 1 };

    G = P * ~C * Inverse(C * P * ~C + R);
    xhat = xhat + G * (z - C * xhat);
    P = (I - G * C) * P;
}

void findHeading() {
	const float acceptableHeadingAcc = 3.5;
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

void log(String data) {
	Serial.println(data);
	notifier.writeValue(data.c_str(), data.length());
}

void log(const char* data) {
	log(String(data));
}

float wrapToPi(float a) {
	//if (a > PI) { a -= 2 * PI; }
	//else if (a <= -PI) { a += 2 * PI; }

	if (a > 2 * PI) { while(a > 2 * PI) { a -= 2 * PI; } }
	else if (a < 0) { while(a < 0) { a += 2 * PI; } }
	return a;
}

void setup() {
  Serial.begin(115200);
  while(!Serial && DEBUGGING);

  Serial1.begin(9600);

  if (!BLE.begin()) {
    Serial.println("Bluetooth failed to initalize");
    while(1);
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

  log("Watiting for initial GPS position...");
	while (!gps.posllhChanged() || !GPS_status.gpsFixOk) {
		log("POSLLH Changed: " + String(gps.posllhChanged()) + ", gpsFixOk: " + String(GPS_status.gpsFixOk));
		while (Serial1.available()) {
			unsigned char c = Serial1.read();
			gps.addByte(c);
		}

		if (gps.statusChanged()) {
			GPS_status = gps.getSTATUS();
		}
	}
	GPS_posllh = gps.getPOSLLH();

  log("Initial GPS position found.");
	delay(250);
	log("Looking for current heading...");
	delay(2000);
	findHeading();

  log("<READY>");
}


void loop() {
  central = BLE.central();

  unsigned long lastprint = millis();
  unsigned long last_heading = micros();
  float gx, gy, gz;
	bool has_run = false;

  while (central.connected()) {
    predictFilter();

    while (Serial1.available()) {
      gps.addByte(Serial1.read());
    }

    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      double dt = (micros() - last_heading) * 1e-6;

			if (!has_run) {
				gyro_heading = initial_heading;
				has_run = true;
			}

      gyro_heading += (gz * DEG_TO_RAD * dt);
      gyro_heading = wrapToPi(gyro_heading);

      last_heading = micros();
    }

    if (gps.velnedChanged()) {
      GPS_velned = gps.getVELNED();

      //gps_heading = wrapToPi(GPS_velned.heading * DEG_TO_RAD);
      gps_heading = GPS_velned.heading * DEG_TO_RAD;

      z = {gps_heading, gyro_heading};
      updateFilter(z);
    }

    if (millis() - lastprint >= 1/data_samplerate * 1000) {
      float kalman_heading = wrapToPi(xhat(0));

      log(String(millis()) + "," + String(kalman_heading, 3) + "," + String(gps_heading, 2) + "," + String(gyro_heading,2));
      lastprint = millis();
    }
  }
}
