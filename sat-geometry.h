#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include <math.h>

#define PI 3.1415926535897
#define MU 398601.2 // Greek Mu (km^3/s^2)
#define LIGHT 299793 // km/s
#define EARTH_PERIOD 86164 // seconds
#define EARTH_RADIUS 6378  // km
#define f_Delta 5

#define DEG_TO_RAD(x) ((x) * PI/180)
#define RAD_TO_DEG(x) ((x) * 180/PI)
#define DISTANCE(s_x, s_y, s_z, e_x, e_y, e_z) (sqrt((s_x - e_x) * (s_x - e_x) \
                + (s_y - e_y) * (s_y - e_y) + (s_z - e_z) * (s_z - e_z)))

using namespace ns3;

struct Coordinate
{
        double r;        // km
        double theta;    // radians
        double phi;      // radians
};

// Library of routines involving satellite geometry
class SatGeometry
{
public:
	SatGeometry () {}
        Vector SpheToCartCoord (Coordinate s);
        double distance_geo (Vector a, Vector b);            
	// static double propdelay(coordinate, coordinate);
	double get_latitude (Coordinate);
	double get_longitude (Coordinate);
	double get_radius (Coordinate a) { return a.r; }
	double get_altitude (Coordinate);
	// static double check_elevation(Coordinate, Coordinate, double);
	// static int are_satellites_mutually_visible(Coordinate, Coordinate);

protected: 
	// Define "command" appropriately if you want OTcl access to this class
        int command(/*int argc, const char*const* argv */) { return 0; }
};

Vector SatGeometry::SpheToCartCoord (Coordinate s)
{
        double x = s.r * sin(s.theta) * cos(s.phi);
        double y = s.r * sin(s.theta) * sin(s.phi);
        double z = s.r * cos(s.theta);
        return {x, y, z};
}

double SatGeometry::distance_geo (Vector a, Vector b)
{
        double dist_x = (a.x - b.x);
        dist_x = dist_x > 180.0 ? (360.0 - dist_x) : dist_x;
        double dist_y = (a.y - b.y);
        return sqrt(dist_x*dist_x + dist_y*dist_y);
}

double SatGeometry::get_altitude (Coordinate a)
{
        return (a.r - EARTH_RADIUS);
}

// Returns latitude in radians, in the range from -PI/2 to PI/2
double SatGeometry::get_latitude (Coordinate a)
{
        return (PI/2 - a.theta);
}

// Returns (earth-centric) longitude corresponding to the position of the node 
// (the input coordinate corresponds to fixed coordinate system, through
// which the Earth rotates, so we have to scale back the effects of rotation).
// The return value ranges from -PI to PI.
double SatGeometry::get_longitude (Coordinate coord_)
{
        double period = EARTH_PERIOD; // period of earth in seconds
        // adjust longitude so that it is earth-centric (i.e., account
        // for earth rotating beneath).   
        double earth_longitude = std::fmod((coord_.phi - (std::fmod(Simulator::Now().GetSeconds(), period)/period) * 2*PI), 2*PI);
	// Bring earth_longitude to be within (-PI, PI)
        if (earth_longitude < (-1*PI))
		earth_longitude = 2*PI + earth_longitude;
        if (earth_longitude > PI)
		earth_longitude = (-(2*PI - earth_longitude));
	if (std::fabs(earth_longitude) < 0.0001)
		return 0;   // To avoid trace output of "-0.00"
	else
		return (earth_longitude);
}       


//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------


