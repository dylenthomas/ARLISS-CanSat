#include "Neo6M_UBXParser.h"
#include <math.h>

struct posllhData posllh_data;
struct statusData status_data;
struct velnedData velned_data;

static const unsigned char ubx_header[2] = { 0xB5, 0x62 };
static const char cls_bytes[1] = { 0x01 };
static const char id_bytes[3] = { 0x02, 0x03, 0x12 };

static int pos = 0;
static unsigned char checksum[2];
static int msg_idx;
static int msg_len;
//static uint16_t len;
static unsigned char buffer[assumed_buf_len];
static int is_valid = 0;
static int flags;
static bool flag;

static bool posllh_assigned = false;
static bool status_assigned = false;
static bool velned_assigned = false;

static struct POSLLH {
    unsigned char cls;
    unsigned char id;
    unsigned short len;
    unsigned long iTOW;
    long lon;
    long lat;
    long height;
    long hMSL;
    unsigned long hACC;
    unsigned long vACC;
} posllhMessage;

static struct STATUS {
    unsigned char cls;
    unsigned char id;
    unsigned short len;
    unsigned long iTOW;
    unsigned char gpsFix;
    unsigned char flags;
    unsigned char fixStat;
    unsigned char flags2;
    unsigned long ttff;
    unsigned long msss;
} statusMessage;

static struct VELNED {
    unsigned char cls;
    unsigned char id;
    unsigned short len;
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

void zeroValues() {
    pos = 0;
    msg_idx = 0;
    memset(buffer, 0, assumed_buf_len);
    memset(checksum, 0, 2);
    is_valid = 0;
}

void computeChecksum() {
    memset(checksum, 0, 2);
    for (int i = 0; i < msg_len; i++) {
        checksum[0] += buffer[i];
        checksum[1] += checksum[0];
    }
}

int lenBytesToInt(unsigned char byte0, unsigned char byte1) {
    return ((byte1 << 8) | byte0);
}

void parseByte(const unsigned char c) {
    switch (pos) {
        case 0: // verify header byte
            if (c == ubx_header[0]) { pos++; }
            break;
        case 1: // verify header byte
            if (c == ubx_header[1]) { pos++; }
            break;
        case 2: // verify class byte
            for (int i = 0; i < num_cls; i++) {
                if (c == cls_bytes[i]) {
                    buffer[0] = c;
                    break;
                }
            }
            pos++;
            break;
        case 3: // verify id byte
            for (int i = 0; i < num_ids; i++) {
                if (c == id_bytes[i]) {
                    buffer[1] = c;
                    msg_idx = i;
                    break;
                }
            }
            pos++;
            break;
        case 4:
            buffer[2] = c;
            pos++;
            break;
        case 5:
            buffer[3] = c;
            msg_len = lenBytesToInt(buffer[2], buffer[3]) + 4;
            pos++;
            break;
        default:
            if (pos < msg_len + 2) {
                buffer[pos - 2] = c;
                pos++;
                return;
            }
            else if(pos == msg_len + 2) {
                computeChecksum();
            }

            if (pos == msg_len + 2 && checksum[is_valid] == c) {
                is_valid++;
            }
            else {
                zeroValues();
                msg_len = -1;
            }
            break;
    }
}

int addByte(unsigned char c) {
    parseByte(c);

    if (is_valid == 1) {
        storeValues();
        zeroValues();
    }
    else if (pos > assumed_buf_len) {
        zeroValues();
    }
    return msg_len;
}

void storeValues() {
    unsigned char trimmed_buff[msg_len];
    for (int i = 0; i < msg_len; i++) {
        trimmed_buff[i] = buffer[i];
    }
    memset(buffer, 0, assumed_buf_len);

    switch (msg_idx) {
        case 0:
            memcpy(&posllhMessage, trimmed_buff, sizeof(posllhMessage));
            posllh_assigned = true;
            break;
        case 1:
            memcpy(&statusMessage, trimmed_buff, sizeof(statusMessage));
            status_assigned = true;
            break;
        case 2:
            memcpy(&velnedMessage, trimmed_buff, sizeof(velnedMessage));
            velned_assigned = true;
            break;
    }
}

struct posllhData getPOSLLH() {
    posllh_data.cls = posllhMessage.cls;
    posllh_data.id = posllhMessage.id;
    posllh_data.len = posllhMessage.len;
    posllh_data.iTOW = posllhMessage.iTOW;
    posllh_data.lon = ((float)posllhMessage.lon) / 1e7;
    posllh_data.lat = ((float)posllhMessage.lat) / 1e7;
    posllh_data.height = ((float)posllhMessage.height) / 1e3;
    posllh_data.hMSL = ((float)posllhMessage.hMSL) / 1e3;
    posllh_data.hACC = ((float)posllhMessage.hACC) / 1e3;
    posllh_data.vACC = ((float)posllhMessage.vACC) / 1e3;

    posllh_assigned = false;
    return posllh_data;
}

bool posllhChanged() {
    return posllh_assigned;
}

struct statusData getSTATUS() {
    flags = statusMessage.flags;

    status_data.cls = statusMessage.cls;
    status_data.id = statusMessage.id;
    status_data.len = statusMessage.len;
    status_data.iTOW = statusMessage.iTOW;
    status_data.gpsFix = (int)(statusMessage.gpsFix);

    for (int i = 0; i < 4; i++) {
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

    status_assigned = false;
    return status_data;
}

bool statusChanged() {
    return status_assigned;
}

struct velnedData getVELNED() {
    velned_data.cls = velnedMessage.cls;
    velned_data.id = velnedMessage.id;
    velned_data.len = velnedMessage.len;
    velned_data.iTOW = velnedMessage.iTOW;
    velned_data.velN = ((float)velnedMessage.velN) / 1e2;
    velned_data.velE = ((float)velnedMessage.velE) / 1e2;
    velned_data.velD = ((float)velnedMessage.velD) / 1e2;
    velned_data.speed = ((float)velnedMessage.speed) / 1e2;
    velned_data.gSpeed = ((float)velnedMessage.gSpeed) / 1e2;
    velned_data.heading = ((float)velnedMessage.heading) / 1e5;
    velned_data.sAcc = ((float)velnedMessage.sAcc) / 1e3;
    velned_data.cAcc = ((float)velnedMessage.cAcc) / 1e5;

    velned_assigned = false;
    return velned_data;
}

bool velnedChanged() {
    return velned_assigned;
}
