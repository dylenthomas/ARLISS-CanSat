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

void set_ref_pos(float lon, float lat, float height) {
    ref_lon = lon * DEG_TO_RAD;
    ref_lat = lat * DEG_TO_RAD;
    ref_height = height;
    LLH_to_ECEF_deg(lon, lat, ref_height, ref_pos);
}

static void LLH_to_ECEF_deg(float lon, float lat, float height, float out[3]) {
    lon = lon * DEG_TO_RAD;
    lat = lat * DEG_TO_RAD;

    N = (A / sqrt(1.0f - E2 * powf(sinf(lat), 2)));
    out[0] = (N + height) * cosf(lat) * cosf(lon);
    out[1] = (N + height) * cosf(lat) * sinf(lon);
    out[2] = (N * (1 - E2) + height) * sinf(lat);
}

void computeNEDpos_deg(float lon, float lat, float height, float out[3]) {
    float local_ecef[3];
    LLH_to_ECEF_deg(lon, lat, height, local_ecef);    
    for (int i = 0; i < 3; i++ ) { local_ecef[i] -= ref_pos[i]; }

    out[0] = local_ecef[0] * (-sinf(ref_lat) * cosf(ref_lon)) + local_ecef[1] * (-sinf(ref_lat) * sinf(ref_lon)) + local_ecef[2] * cosf(ref_lat);
    out[1] = local_ecef[0] * (-sinf(ref_lon)) + local_ecef[1] * (cosf(ref_lon));
    out[2] = local_ecef[0] * (-cosf(ref_lat) * cosf(ref_lon)) + local_ecef[1] * (-cosf(ref_lat) * sinf(ref_lon)) + local_ecef[2] * (-sinf(ref_lat));
}

float azimuth_deg(float lat1, float lon1, float lat2, float lon2) {
    lat1 *= DEG_TO_RAD;
    lon1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;
    lon2 *= DEG_TO_RAD;
    float y = sinf(lon2 - lon1) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(lon2 - lon1);
    float azimuth_angle = atan2(y, x);
    return azimuth_angle;
}