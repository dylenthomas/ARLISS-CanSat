#ifndef NEO6M_UBXPARSER_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define NEO6M_UBXPARSER_H
#define assumed_buf_len 100
#define num_ids 3
#define num_cls 1

bool posllhChanged();
bool statusChanged();
bool velnedChanged();

struct posllhData {
    /* message info */
    unsigned char cls;
    unsigned char id;
    unsigned short len;

    /* payload */
    unsigned long iTOW;
    float lat;
    float lon;
    float height;
    float hMSL;
    float hACC;
    float vACC;
};

struct statusData {
    /* message info */
    unsigned char cls;
    unsigned char id;
    unsigned short len;

    /* payload */
    unsigned long iTOW;
    /* for gpsFix type the integer corresponds to fix type:
     * 0 - no fix
     * 1 - dead reckoning only
     * 2 - 2D-fix
     * 3 - 3D-fix
     * 4 - GPS + dead reckoning
     * 5 - Time only fix
     */
    unsigned int gpsFix;
     bool gpsFixOk;
     bool diffSoln;
     bool wknSet;
     bool towSet;
     unsigned char fixStat;
     unsigned char flags2;
     unsigned long ttff;
     unsigned long msss;
};

struct velnedData {
    /* message info */
    unsigned char cls;
    unsigned char id;
    unsigned short len;

    /* payload */
    unsigned long iTOW;
    float velN;
    float velE;
    float velD;
    float speed;
    float gSpeed;
    float heading;
    float sAcc;
    float cAcc;
};

struct posllhData getPOSLLH();
struct statusData getSTATUS();
struct velnedData getVELNED();

int addByte(unsigned char c);

int lenBytesToInt(unsigned char byte0, unsigned char byte1);
void computeChecksum();
void zeroValues();
void parseByte(const unsigned char c);
void storeValues();

#endif
