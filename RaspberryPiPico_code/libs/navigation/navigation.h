#ifndef NAVIGATION_H

#include "pico/stdlib.h"
#include <math.h>

#define NAVIGATION_H

#ifndef PI
#define PI 3.14159265359
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD PI / 180.0
#endif

void set_ref_pos(float ref_lon, float ref_lat, float ref_height);

static void LLH_to_ECEF_deg(float lon, float lat, float height, float out[3]);

/*
Convert LLH coordinate format to NED (local X, Y, Z)
    *pass the values in degrees
*/
void computeNEDpos_deg(float lon, float lat, float height, float out[3]);
/*
Compute the azimuth angle between current lat and lon and destination lat and lon
    *pass the values in degrees
*/
float azimuth_deg(float lat1, float lon1, float lat2, float lon2);

#endif
