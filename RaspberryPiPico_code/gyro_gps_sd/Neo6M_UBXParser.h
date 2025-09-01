#ifndef NEO6M_UBXPARSER_H

#define NEO6M_UBXPARSER_H

#include <stdbool.h>
#include <stdint.h>

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
    double lon;
    double lat;
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
    unsigned int gpsFix;
    /* for gpsFix type the integer corresponds to fix type:
     * 0 - no fix
     * 1 - dead reckoning only
     * 2 - 2D-fix
     * 3 - 3D-fix
     * 4 - GPS + dead reckoning
     * 5 - Time only fix
     */
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
    double velN;
    double velE;
    double velD;
    double speed;
    double gSpeed;
    float heading;
    float sAcc;
    float cAcc;
};

struct posllhData getPOSLLH();
struct statusData getSTATUS();
struct velnedData getVELNED();

void addByte(unsigned char c);

int lenBytesToInt(unsigned char byte0, unsigned char byte1);
void computeChecksum(unsigned short len, unsigned char* buffer, unsigned char* CK);
void zeroValues();
void storeValues(unsigned char* buffer, int msg_len, int msg_idx);


#endif
