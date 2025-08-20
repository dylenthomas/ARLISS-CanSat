/*
Header file for library that can process and interpret specified messages from the UBX message format used by uBlox GNSS devices.

The folowing structs that are defined are the messages being recieved and store all the values being sent by the messages.
*/

#pragma once

#if defined(ARDUINO)
	#include "Arduino.h"
#else
	#include "stdint.h"
	#include "string.h"
	using namespace std;
#endif

class GPSParser {
public:
	GPSParser();

	struct posllhData {
		/* message info */
		unsigned char cls;
		unsigned char id;
		unsigned short len; /* 28 bytes */

		/* payload */
		unsigned long iTOW;
		double lon;
		double lat;
		float height;
		float hMSL;
		float hACC;
		float vACC;
	} posllh_data;

	struct statusData{
		/* message info */
		unsigned char cls;
		unsigned char id;
		unsigned short len; /* 16 bytes */

		/* payload */
		unsigned long iTOW;
		unsigned int gpsFix;
			/* for gpsFix an integer corresponds to fix type:
				0 - no fix
				1 - dead reckoning only
				2 - 2D-fix
				3 - 3D-fix 
				4 - GPS + dead reckoning
				5 - Time only fix

				this is only a type of fix though and does not qualify validity
			*/
		bool gpsFixOk;
		bool diffSoln;
		bool wknSet;
		bool towSet;
		unsigned char fixStat;
		unsigned char flags2;
		unsigned long ttff;
		unsigned long msss;
	} status_data;

	struct velnedData {
		/* message info */
		unsigned char cls;
		unsigned char id;
		unsigned short len; /* 36 bytes */

		/* payload */
		unsigned long iTOW;
		double velN;
		double velE;
		double velD;
		double speed;
		double gSpeed;
		float heading;
		float sAcc;
		float cAcc;
	} velned_data;

	bool posllhChanged();
	bool statusChanged();
	bool velnedChanged();

	posllhData getPOSLLH();
	statusData getSTATUS();
	velnedData getVELNED();

	void addByte(unsigned char c);

	static const int lon_dec = 7;
	static const int lat_dec = 7;
	static const int height_dec = 3;
	static const int hMSL_dec = 3;
	static const int hACC_dec = 3;
	static const int vACC_dec = 3;

	static const int velN_dec = 2;
	static const int velE_dec = 2;
	static const int velD_dec = 2;
	static const int speed_dec = 2;
	static const int gSpeed_dec = 2;
	static const int heading_dec = 5;
	static const int sACC_dec = 3;
	static const int cACC_dec = 6;


private:
	/* The following structs recieve the raw data from the gps message,
		the data is then converted to the correct format and returned in the form of the public structs.
	*/
	struct POSLLH {
		/* message info */
		unsigned char cls;
		unsigned char id;
		unsigned short len; /* 28 bytes */

		/* payload */
		unsigned long iTOW;
		long lon;
		long lat;
		long height;
		long hMSL;
		unsigned long hACC;
		unsigned long vACC;
	} posllhMessage;

	struct STATUS {
		/* message info */
		unsigned char cls;
		unsigned char id;
		unsigned short len; /* 16 bytes */

		/* payload */
		unsigned long iTOW;
		unsigned char gpsFix;
		unsigned char flags;
		unsigned char fixStat;
		unsigned char flags2;
		unsigned long ttff;
		unsigned long msss;
	} statusMessage;

	struct VELNED {
		/* message info */
		unsigned char cls;
		unsigned char id;
		unsigned short len; /* 36 bytes */

		/* payload */
		unsigned long iTOW;
		long velN;
		long velE;
		long velD;
		unsigned long speed;
		unsigned long gSpeed;
		long heading;
		unsigned long sAcc;
		unsigned long cAcc;
	} velnedMessage;

	/* Variables for reading incomming messages */
	const unsigned char ubx_header[2] = { 0xB5, 0x62 }; /* [181, 98] */
	const char cls_bytes[1] = { 0x01 }; /* NAV */
	const char id_bytes[3] = { 0x02, 0x03, 0x12 }; /* POSLLH, STATUS, VELNED */
	static const int assumed_buf_len = 100;

	int pos;
	unsigned char checksum[2];
	int msg_idx;
	int msg_len;
	uint16_t len;
	unsigned char buffer[assumed_buf_len];
	int is_valid;
	int num_flags;
	int flags;
	bool flag;

	bool posllh_assigned;
	bool status_assigned;
	bool velned_assigned;

	uint16_t lenBytesToInt(unsigned char& byte0, unsigned char& byte1);
	void computeChecksum(unsigned short len, unsigned char* buffer, unsigned char* CK);
	void zeroValues();
	int parseByte(
		int& pos,
		const unsigned char& c,
		const unsigned char* ubx_header,
		const char* cls_bytes,
		const char* id_bytes,
		int& msg_idx,
		uint16_t& len,
		unsigned char* buffer,
		unsigned char* checksum,
		int& is_valid);
	void storeValues(unsigned char *buffer, int &msg_len, int &msg_idx);
};