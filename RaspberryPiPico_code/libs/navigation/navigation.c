#include "navigation.h"

// WGS-84 constants
static const float A  = 6378137.0f;          // semi-major (m)
static const float B  = 6356752.314245f;     // semi-minor (m)
static const float E2 = 1.0f - (B*B)/(A*A);  // eccentricity squared
static float N;

static float ref_lon;
static float ref_lat;
static float ref_height;

static float ref_pos[3];
static float ECEF_pos[3];
static float NED_pos[3];

void set_ref_pos(float lon, float lat, float height) {
    ref_lon = lon * DEG_TO_RAD;
    ref_lat = lat * DEG_TO_RAD;
    ref_height = height;

    LLH_to_ECEF(lon, lat, ref_height);
    for (int i = 0; i < 3; i++) { 
        ref_pos[i] = ECEF_pos[i]; 
    }
}

static void LLH_to_ECEF(float lon, float lat, float height) {
    lon = lon * DEG_TO_RAD;
    lat = lat * DEG_TO_RAD;

    N = (A / sqrt(1.0f - E2 * pow(sin(lat), 2)));
    ECEF_pos[0] = (N + height) * cos(lat) * cos(lon);
    ECEF_pos[1] = (N + height) * cos(lat) * sin(lon);
    ECEF_pos[2] = (N * (1 - E2) + height) * sin(lat);
}

void computeNEDpos(float lon, float lat, float height, float out[3]) {
    LLH_to_ECEF(lon, lat, height);
    float local_ecef[3];
    
    for (int i = 0; i < 3; i++ ) {
        local_ecef[i] = ECEF_pos[i] - ref_pos[i];
    }

    NED_pos[0] = local_ecef[0] * (-sin(ref_lat) * cos(ref_lon)) + local_ecef[1] * (-sin(ref_lat) * sin(ref_lon)) + local_ecef[2] * cos(ref_lat);
    NED_pos[1] = local_ecef[0] * (-sin(ref_lon)) + local_ecef[1] * (cos(ref_lon));
    NED_pos[2] = local_ecef[0] * (-cos(ref_lat) * cos(ref_lon)) + local_ecef[1] * (-cos(ref_lat) * sin(ref_lon)) + local_ecef[2] * (-sin(ref_lat));

    for (int i = 0; i < 3; i++) {
        out[i] = NED_pos[i];
    }
}