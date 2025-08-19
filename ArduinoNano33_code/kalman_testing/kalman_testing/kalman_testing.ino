#include "Arduino_BMI270_BMM150.h"
#include "Neo6M_UBXParser.h"
#include "uNavINS.h"

#define GPS_DEBUG false
#define GPS_SAVE_DATA true

// initialize motor variables
constexpr int M1_IN1 = 6;
constexpr int M1_IN2 = 5;
constexpr int M2_IN1 = 9;
constexpr int M2_IN2 = 3;

// IMU variables
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;

GPSParser gps;
GPSParser::posllhData GPS_posllh = gps.getPOSLLH();
GPSParser::statusData GPS_status = gps.getSTATUS();
GPSParser::velnedData GPS_velned = gps.getVELNED();

unsigned long iTOW = 0;

void setup()
{
	Serial.begin(115200);
	while (!Serial);

	/* start gps serial */
	Serial1.begin(9600);

	if (!IMU.begin()) {
		Serial.println("Failed to initialize the IMU");
		while (1);
	}

	if (GPS_SAVE_DATA) {
		// Print csv header
		Serial.println("time,lat,lon,height,hMSL,velN,velE,velD,heading,speed,gSpeed,hACC,vACC,sACC,cACC,gpsFixOk");
	}
}

void loop()
{
	if (IMU.accelerationAvailable()) {
		IMU.readAcceleration(accX, accY, accZ); // measure acceleration in g
	}
	if (IMU.gyroscopeAvailable()) {
		IMU.readGyroscope(gyroX, gyroY, gyroZ); // measure angular velocity in dg/s
	}
	if (IMU.magneticFieldAvailable()) {
		IMU.readMagneticField(magX, magY, magZ); // measure external magnetic field in uT
	}

	while (Serial1.available()) {
		unsigned char c = Serial1.read();
		gps.addByte(c);
	}

	if (gps.posllhChanged()) {
		GPS_posllh = gps.getPOSLLH();

		iTOW = GPS_posllh.iTOW;
	}
	if (gps.statusChanged()) {
		GPS_status = gps.getSTATUS();

		iTOW = GPS_status.iTOW;
	}
	if (gps.velnedChanged()) {
		GPS_velned = gps.getVELNED();

		iTOW = GPS_velned.iTOW;
	}
	
	if (GPS_DEBUG) {
		Serial.print("TOW: ");
		Serial.print(iTOW);
		Serial.print(", ");

		Serial.print("Lon: ");
		Serial.print(GPS_posllh.lon);
		Serial.print(", ");
		Serial.print("Lat: ");
		Serial.print(GPS_posllh.lat);
		Serial.print(", ");
		Serial.print("Height: ");
		Serial.print(GPS_posllh.height);
		Serial.print(", ");
		Serial.print("Height above Median Sea Level: ");
		Serial.print(GPS_posllh.hMSL);
		Serial.print(", ");
		Serial.print("Heading: ");
		Serial.print(GPS_velned.heading);
		Serial.println();

	} else if (GPS_SAVE_DATA) {
		Serial.print(iTOW); Serial.print(",");
		Serial.print(GPS_posllh.lat, GPSParser::lat_dec); Serial.print(",");
		Serial.print(GPS_posllh.lon, GPSParser::lon_dec); Serial.print(",");
		Serial.print(GPS_posllh.height, GPSParser::height_dec); Serial.print(",");
		Serial.print(GPS_posllh.hMSL, GPSParser::hMSL_dec); Serial.print(",");
		Serial.print(GPS_velned.velN, GPSParser::velN_dec); Serial.print(",");
		Serial.print(GPS_velned.velE, GPSParser::velE_dec); Serial.print(",");
		Serial.print(GPS_velned.velD, GPSParser::velD_dec); Serial.print(",");
		Serial.print(GPS_velned.heading, GPSParser::heading_dec); Serial.print(",");
		Serial.print(GPS_velned.speed, GPSParser::speed_dec); Serial.print(",");
		Serial.print(GPS_velned.gSpeed, GPSParser::gSpeed_dec); Serial.print(",");
		Serial.print(GPS_posllh.hACC, GPSParser::hACC_dec); Serial.print(",");
		Serial.print(GPS_posllh.vACC, GPSParser::vACC_dec); Serial.print(",");
		Serial.print(GPS_velned.sAcc, GPSParser::sACC_dec); Serial.print(",");
		Serial.print(GPS_velned.cAcc, GPSParser::cACC_dec); Serial.print(",");
		Serial.print(GPS_status.gpsFixOk);

		Serial.print('\n');
	}
}