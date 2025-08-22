#include "BasicLinearAlgebra.h"
#include "Neo6M_UBXParser.h"
#include "Motor_PWM_Nano33.h"
#include "ArduinoBLE.h"
#include "Arduino_BMI270_BMM150.h"

constexpr bool debugging = false;

// Motor initialization ---------------------------------------------------------------------------
constexpr int mR_reverse_pin = 3;
constexpr int mR_forward_pin = 9;
constexpr int mL_reverse_pin = 6;
constexpr int mL_forward_pin = 5;
constexpr int PWMFreq = 25000; // Hz
const float minDutyCycle = 0.0; // *100%
const float maxDutyCycle = 1.0; // *100%

Motor_Control motorRight(mR_forward_pin, mR_reverse_pin);
Motor_Control motorLeft(mL_forward_pin, mL_reverse_pin);
// ------------------------------------------------------------------------------------------------

// Kalman filter matricies ------------------------------------------------------------------------
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
// ------------------------------------------------------------------------------------------------

// Controller variables ---------------------------------------------------------------------------
//const float Kp = /*1.0*/maxDutyCycle; // distance error gain
const float Vforward = 0.95;
const float KaP = 0.035; // heading proportional gain
const float KaD = 0.01; // heading derivative gain
const float KaI = 0.0; // heading integral gain
// ------------------------------------------------------------------------------------------------

// BLE Initalization ------------------------------------------------------------------------------
const char* uuidService = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* uuidNotify = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* uuidCommands = "19b10002-e8f2-537e-4f6c-d104768a1214";

const char* BLEname = "Nano33BLE";

BLEService nanoService(uuidService);
BLECharacteristic notifier(uuidNotify, BLENotify, 64);
BLEByteCharacteristic commands(uuidCommands, BLEWrite);
BLEDevice central;
// ------------------------------------------------------------------------------------------------

// GPS Initialization -----------------------------------------------------------------------------
GPSParser gps;
GPSParser::posllhData GPS_posllh = gps.getPOSLLH();
GPSParser::statusData GPS_status = gps.getSTATUS();
GPSParser::velnedData GPS_velned = gps.getVELNED();
// ------------------------------------------------------------------------------------------------

double ref_lat;
double ref_lon;
BLA::Matrix<3, 1> ref_pos;
BLA::Matrix<3, 3> NED_rotation;
int a = 6378137;
int b = 6356752;
double e = (double)(1 - pow((b / a), 2));
double N;

float initial_heading;
float gyro_heading;

/*
FOLLOWING LIST SHOULD BE EITHER:
		  { deg.decimal }
				  	or
		{ deg, min, second }
*/
double target_lon[] = {-84.40315};
double target_lat[] = {33.77907};

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

BLA::Matrix<3, 1> computeECEFpos(double lat, double lon, float height) {
	BLA::Matrix<3, 1> pos;
	lon = lon * DEG_TO_RAD;
	lat = lat * DEG_TO_RAD;

	N = (double)(a/sqrt(1 - pow(e, 2)*pow(sin(lat), 2)));
	pos(0) = (N + height)*cos(lat)*cos(lon);
	pos(1) = (N + height)*cos(lat)*sin(lon);
	pos(2) = (N * (1 - pow(e, 2)) + height)*sin(lat);

	return pos;
}

BLA::Matrix<3, 1> computeNEDpos(double lat, double lon, float height) {
	BLA::Matrix<3, 1> pos;
	BLA::Matrix<3, 1> ecef_pos = computeECEFpos(lat, lon, height);

	pos = NED_rotation * (ecef_pos - ref_pos);
	return pos;
}

void findHeading() {
	const float acceptableHeadingAcc = 2.5;
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

float wrapTo2Pi(float a) {
	if (a > 2 * PI) { a -= 2 * PI; }
	else if (a < 0) { a += 2 * PI; }
	return a;
}

float shortestRotation(float a) {
	if (a > PI) { a -= 2 * PI; }
	else if (a < -PI) { a += 2 * PI; }
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

double deg_minsec_to_dec(double* pos) {
	return (double)(pos[0] + pos[1] / 60.0 + pos[2] / 3600.0);
}

BLA::Matrix<3, 1> get_destination(double* lat, double* lon, float height) {
	double lon_dec;
	double lat_dec;

	if (sizeof(lon) / sizeof(lon[0]) == 1) {
		lon_dec = lon[0];
	}
	else {
		lon_dec = deg_minsec_to_dec(lon);
	}

	if (sizeof(lat) / sizeof(lat[0]) == 1) {
		lat_dec = lat[0];
	}
	else {
		lat_dec = deg_minsec_to_dec(lat);
	}

	return computeNEDpos(lat_dec, lon_dec, height);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && debugging);

    Serial1.begin(9600);

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

  gyro_heading = initial_heading;
	xhat = { initial_heading };

	ref_lat = GPS_posllh.lat;
	ref_lon = GPS_posllh.lon;
	ref_pos = computeECEFpos(ref_lat, ref_lon, GPS_posllh.height); // double check height value to be in meters
	log("Reference lat: " + String(ref_lat, 6) + ", reference lon: " + String(ref_lon, 6));
	log("Starting ECEF pos: x = " + String(ref_pos(0)) + ", y = " + String(ref_pos(1)) + ", z = " + String(ref_pos(2)));

	ref_lat = ref_lat * DEG_TO_RAD;
	ref_lon = ref_lon * DEG_TO_RAD;
	NED_rotation = {
		-sin(ref_lat)*cos(ref_lon), -sin(ref_lat)*sin(ref_lon), cos(ref_lat),
		-sin(ref_lon), cos(ref_lon), 0,
		-cos(ref_lat)*cos(ref_lat), -cos(ref_lat)*sin(ref_lon), -sin(ref_lat),
	};

	log("<READY>");
}

float stoppingDist = 5.0;

float gps_heading;
float speed;

float u_r = 0.0;
float u_l = 0.0;
float alpha;
float Tp;
float Td;
float Ti;
float T;
float u_r_internal = 0.0;
float u_l_internal = 0.0;

unsigned long last_time = micros();
float last_alpha = 0.0;
double dt_integral;
float de;

bool wasPaused = false;

int printdelay = 200;

BLA::Matrix<3, 1> position;
float target_heading = initial_heading;

BLA::Matrix<3, 1> destination = get_destination(target_lat, target_lon, GPS_posllh.height);

void loop() {
    central = BLE.central();

		int iter = 0;
		unsigned long lastprint = millis();
		unsigned long last_heading = micros();
		bool has_run = false;
		float gx, gy, gz;

		log("Current location: " + String(GPS_posllh.lat, 6) + " N, " + String(GPS_posllh.lon, 6) + " E.");
		position = computeNEDpos(GPS_posllh.lat, GPS_posllh.lon, GPS_posllh.height);
		log("Current local in m: x = " + String(position(0)) + ", y = " + String(position(1)));
		delay(250);
		if (sizeof(target_lat) / sizeof(target_lat[0]) == 3){
			log("Heading to position: " + String(target_lat[0]) + "," + String(target_lat[1]) + "," + String(target_lat[2]) + " N, " + String(target_lon[0]) +"," + String(target_lon[1]) + "," + String(target_lon[2]) + " E.");
		}
		else {
			log("Heading to position: " + String(target_lat[0], 6) + " N, " + String(target_lon[0], 6) +" E.");
		}
		delay(100);
		log("Which is " + String(destination(0)) + " m N and " + String(destination(1)) + " m E");
		delay(250);

    while (central.connected()) {
        if (wasPaused) {
            motorRight.resume();
            motorLeft.resume();
            wasPaused = false;
        }

        while (Serial1.available()) {
            gps.addByte(Serial1.read());
        }

        predictFilter();

        if (IMU.gyroscopeAvailable()) {
					IMU.readGyroscope(gx, gy, gz);
					double dt = (micros() - last_heading) * 1e-6;

					if (!has_run) {
						gyro_heading = initial_heading;
						has_run = true;
					}

					gyro_heading += (gz * DEG_TO_RAD * dt);
					gyro_heading = wrapTo2Pi(gyro_heading);

					last_heading = micros();
				}

        if (gps.velnedChanged()) {
          GPS_velned = gps.getVELNED();

          gps_heading = GPS_velned.heading * DEG_TO_RAD;
          speed = GPS_velned.gSpeed;
            
          z = {gps_heading, gyro_heading};
          updateFilter(z);
        }

				if (gps.posllhChanged()) {
					GPS_posllh = gps.getPOSLLH();
					position = computeNEDpos(GPS_posllh.lat, GPS_posllh.lon, GPS_posllh.height);
					float dy = destination(0) - position(0);
					float dx = destination(1) - position(1);

					if (sqrt(pow(dy, 2) + pow(dx, 2)) <= stoppingDist) {
						log("Reached destination");
						motorRight.pause();
						motorLeft.pause();
						while (1);
					}

					target_heading = wrapTo2Pi(atan2(dy, dx));
				}

				float kalman_heading = wrapTo2Pi(xhat(0));
				alpha = shortestRotation(target_heading - kalman_heading);

        dt_integral = max((micros() - last_time) * 1e-6, 1.0);
        de = alpha - last_alpha;

        last_alpha = alpha;
        last_time = micros();
        
        Tp = KaP * alpha;
        Td = KaD * de / dt_integral;
        Ti += KaI * alpha * dt_integral;

        Td = constrain(Td, -2, 2);
        Ti = constrain(Ti, -0.5, 0.5);

        T = Tp + Td + Ti;
				T = constrain(T, -0.5, 0.5);

        u_l_internal = Vforward - T;
        u_r_internal = Vforward + T;

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
				log("T: " + String(T) + ", speed: " + String(speed) + ", hding: " + String(kalman_heading) + ",  alpha: " + String(alpha));
				//log("FilterHding: " + String(kalman_heading) + ",  a: " + String(alpha) + ", gy_hding: " + String(gyro_heading) + ", gps_hding: " + String(gps_heading));

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
