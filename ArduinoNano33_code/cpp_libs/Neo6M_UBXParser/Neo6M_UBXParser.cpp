#include "Neo6M_UBXParser.h"

GPSParser::GPSParser() {
	pos = 0;
	is_valid = 0;

	posllh_assigned = false;
	status_assigned = false;
	velned_assigned = false;

	num_flags = 4;
}

void GPSParser::addByte(unsigned char c) {
	msg_len = parseByte(
		pos,
		c,
		ubx_header,
		cls_bytes,
		id_bytes,
		msg_idx,
		len,
		buffer,
		checksum,
		is_valid
	);

	if (is_valid == 1) {
		storeValues(buffer, msg_len, msg_idx);
		zeroValues();
	}
	else if (pos > assumed_buf_len) {
		/* pos should never be this high */
		zeroValues();
	}
}

void GPSParser::zeroValues() {
	/* reset variables to zero */
	pos = 0;
	msg_idx = 0;
	msg_len = 0;
	len = 0;
	memset(buffer, 0, assumed_buf_len);
	is_valid = 0;
}

void GPSParser::computeChecksum(unsigned short len, unsigned char* buffer, unsigned char* CK) {
	/* validate message integrity with a checksum */
	memset(CK, 0, 2);
	for (int i; i < len; i++) {
		CK[0] += buffer[i];
		CK[1] += CK[0];
	}
}

uint16_t GPSParser::lenBytesToInt(unsigned char& byte0, unsigned char& byte1) {
	return (byte1 << 8) | byte0;
}

int GPSParser::parseByte(
	int& pos,
	const unsigned char& c,
	const unsigned char* ubx_header,
	const char* cls_bytes,
	const char* id_bytes,
	int& msg_idx,
	uint16_t& len,
	unsigned char* buffer,
	unsigned char* checksum,
	int& is_valid) {

	switch (pos) {
		case 0: /* verify header byte */
			if (c == ubx_header[0]) { pos++; }
			break;
		case 1: /* verify header byte */
			if (c == ubx_header[1]) { pos++; }
			break;
		case 2: /* verify class byte */
			for (int i = 0; i < strlen(cls_bytes); i++) {
				if (c == cls_bytes[i]) { 
					buffer[0] = c;
					break; 
				}
			}
			pos++;
			break;
		case 3: /* verify id */
			for (int i = 0; i < strlen(id_bytes); i++) {
				if (c == id_bytes[i]) {
					buffer[1] = c;
					msg_idx = i;
					break;
				}
			}
			pos++;
			break;
		case 4: /* store 1st length byte */
			buffer[2] = c;
			pos++;
			break;
		case 5: /* store second length byte and calculate total length */
			buffer[3] = c;
			len = lenBytesToInt(buffer[2], buffer[3]);
			pos++;
			break;
		default: /* parse the following payload */
			int msg_len = len + 4;
			if (pos - 2 < msg_len) {
				buffer[pos - 2] = c;
				if (pos - 2 == msg_len - 1) { /* reached last index in the buffer */
					computeChecksum(msg_len, buffer, checksum);
				}
				pos++;
			}
			else if (checksum[is_valid] == c) {
				if (is_valid == 1) { /* both checksum bytes are correct */
					return msg_len;
				}

				is_valid++;
			}
			else { /* checksum failed */
				zeroValues();
				return -1;
			}
			break;
	}
}

void GPSParser::storeValues(unsigned char* buffer, int& msg_len, int& msg_idx) {
	/* copy the buffer information to a buffer of the correct size */
	unsigned char trimed_buff[msg_len];
	for (int i = 0; i < msg_len; i++) {
		trimed_buff[i] = buffer[i];
	}
	memset(buffer, 0, assumed_buf_len); /* delete everything in the buffer */

	switch (msg_idx) {
		case 0:
			//posllh_data = parsePOSLLH(trimed_buff);
			memcpy(&posllhMessage, trimed_buff, sizeof(POSLLH));
			posllh_assigned = true;
			break;
		case 1:
			//status_data = parseSTATUS(trimed_buff);
			memcpy(&statusMessage, trimed_buff, sizeof(STATUS));
			status_assigned = true;
			break;
		case 2:
			//velned_data = parseVELNED(trimed_buff);
			memcpy(&velnedMessage, trimed_buff, sizeof(VELNED));
			velned_assigned = true;
			break;
	}
}

GPSParser::posllhData GPSParser::getPOSLLH() {
	posllh_data.cls = posllhMessage.cls;
	posllh_data.id = posllhMessage.id;
	posllh_data.len = posllhMessage.len;
	posllh_data.iTOW = posllhMessage.iTOW;
	/* divide by factor of 10 to get into proper decimal format since data is transmitted as an integer */
	posllh_data.lon = static_cast<double>(posllhMessage.lon) / pow(10, lon_dec);
	posllh_data.lat = static_cast<double>(posllhMessage.lat) / pow(10, lat_dec);
	posllh_data.height = static_cast<float>(posllhMessage.height) / pow(10, height_dec);
	posllh_data.hMSL = static_cast<float>(posllhMessage.hMSL) / pow(10, hMSL_dec);
	posllh_data.hACC = static_cast<float>(posllhMessage.hACC) / pow(10, hACC_dec);
	posllh_data.vACC = static_cast<float>(posllhMessage.vACC) / pow(10, vACC_dec);

	posllh_assigned = false; /* track that this data has been read */
	return posllh_data;
}

bool GPSParser::posllhChanged() {
	return posllh_assigned;
}

GPSParser::statusData GPSParser::getSTATUS() {
	flags = statusMessage.flags;

	status_data.cls = statusMessage.cls;
	status_data.id = statusMessage.id;
	status_data.len = statusMessage.len;
	status_data.iTOW = statusMessage.iTOW;
	status_data.gpsFix = static_cast<int>(statusMessage.gpsFix);

	/* process the flags bitfield */
	for (int i = 0; i < num_flags; i++) {
		flag = (flags & (0x01 << i)) != 0;

		switch (i) {
			case 0:
				status_data.gpsFixOk = flag;
				break;
			case 1:
				status_data.diffSoln = flag;
				break;
			case 2:
				status_data.wknSet = flag;
				break;
			case 3:
				status_data.towSet = flag;
				break;
		}
	}

	status_data.fixStat = statusMessage.fixStat;
	status_data.flags2 = statusMessage.flags2;
	status_data.ttff = statusMessage.ttff;
	status_data.msss = statusMessage.msss;

	status_assigned = false; /* track that this data has been read */
	return status_data;
}

bool GPSParser::statusChanged() {
	return status_assigned;
}

GPSParser::velnedData GPSParser::getVELNED() {
	velned_data.cls = velnedMessage.cls;
	velned_data.id = velnedMessage.id;
	velned_data.len = velnedMessage.len;
	velned_data.iTOW = velnedMessage.iTOW;
	/* divide by factor of 10 to conver to proper decimal format since data is transmitted as an int */
	velned_data.velN = static_cast<double>(velnedMessage.velN) / pow(10, velN_dec);
	velned_data.velE = static_cast<double>(velnedMessage.velE) / pow(10, velE_dec);
	velned_data.velD = static_cast<double>(velnedMessage.velD) / pow(10, velD_dec);
	velned_data.speed = static_cast<double>(velnedMessage.speed) / pow(10, speed_dec);
	velned_data.gSpeed = static_cast<double>(velnedMessage.gSpeed) / pow(10, gSpeed_dec);
	velned_data.heading = static_cast<float>(velnedMessage.heading) / pow(10, heading_dec);
	velned_data.sAcc = static_cast<float>(velnedMessage.sAcc) / pow(10, sACC_dec);
	velned_data.cAcc = static_cast<float>(velnedMessage.cAcc) / pow(10, cACC_dec);

	velned_assigned = false; /* track that this data has been read */
	return velned_data;
}

bool GPSParser::velnedChanged() {
	return velned_assigned;
}