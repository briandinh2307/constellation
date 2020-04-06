#include "ns3/core-module.h"
#include "sat-geometry.h"

struct OrbitalSat
{
        double altitude;
        double inclination;
        double longitude;
        double alpha;
        uint32_t plane;
};  

class SatPosition
{
    public:
        std::vector<Vector> SatPos (std::vector<OrbitalSat> orbitalElements);
        Coordinate Coord ();
        std::vector<Vector> get_pos () { return satellite; }
                
    protected:
        double inc;         // constellation inclination
        double period;
        std::vector<Vector> satellite;
        std::vector<OrbitalSat> polarSat;
        SatGeometry geometry;

    private:
        void SetInit (uint32_t curSat);
        Coordinate init;
        Coordinate spheCoord;
        bool sat_init = 0;
};

void SatPosition::SetInit (uint32_t curSat)
{
    double satAlt = polarSat[curSat].altitude; 
    double satLon = polarSat[curSat].longitude;
    double satAlpha = polarSat[curSat].alpha;
    double satIncl = polarSat[curSat].inclination;

    init.r = satAlt + EARTH_RADIUS;     // Altitude in km above the earth
    if (satAlpha < 0)
        exit(1);
        
    if (satAlpha >= 360)
        init.theta = DEG_TO_RAD(satAlpha - 360);
    else
        init.theta = DEG_TO_RAD(satAlpha);

    if (satLon < -360 || satLon > 360)
        exit(1);

    if (satLon < 0)
        init.phi = DEG_TO_RAD(360 + satLon);
    else
        init.phi = DEG_TO_RAD(satLon);

    if (satIncl < 0 || satIncl > 180)
        exit(1);
    inc = DEG_TO_RAD (satIncl);

    double num = init.r * init.r * init.r;
    period = 2 * PI * sqrt(num/MU);     // [seconds] period of 1 satellite in this constellation
}

Coordinate SatPosition::Coord ()
{
	double partial;  // fraction of orbit period completed
	partial = (fmod(Simulator::Now().GetSeconds(), period)/period) * 2*PI; //rad
	double theta_cur, phi_cur, theta_new, phi_new;

	// Compute current orbit-centric coordinates:
	// theta_cur adds effects of time (orbital rotation) to init.theta
	theta_cur = fmod(init.theta + partial, 2*PI);
	phi_cur = init.phi;
	// Reminder:  theta_cur and phi_cur are temporal translations of 
	// initial parameters and are NOT true spherical coordinates.
	//
	// Now generate actual spherical coordinates,
	// with 0 < theta_new < PI and 0 < phi_new < 360

	// assert (inc < PI);

	// asin returns value between -PI/2 and PI/2, so    //     
	// theta_new guaranteed to be between 0 and PI
	theta_new = PI/2 - asin(sin(inc) * sin(theta_cur));
	// if theta_new is between PI/2 and 3*PI/2, must correct
	// for return value of atan()
	if ((theta_cur > PI/2 && theta_cur < 3*PI/2) || theta_cur == DEG_TO_RAD(270.0))
		phi_new = atan(cos(inc) * tan(theta_cur)) + phi_cur + PI;
	else
		phi_new = atan(cos(inc) * tan(theta_cur)) + phi_cur;
	phi_new = fmod(phi_new + 2*PI, 2*PI);
	
	spheCoord.r = init.r;
	spheCoord.theta = theta_new;
	spheCoord.phi = phi_new;
	return spheCoord;
}

// Update "sat_th" postion (long, lati, alti)
std::vector<Vector>
SatPosition::SatPos (std::vector<OrbitalSat> orbitalElements)
{
    polarSat = orbitalElements;
    double longitude;
    double latitude;
    double altitude;
    for (uint32_t i = 0; i < orbitalElements.size(); i++)
    {
        SetInit(i);
        longitude = RAD_TO_DEG(geometry.get_longitude(Coord()));
        latitude = RAD_TO_DEG(geometry.get_latitude(Coord()));
        altitude = geometry.get_altitude(Coord());

        // Time = 0s. Use push_back for syntax
        if (sat_init == 0)
            satellite.push_back({longitude, latitude, altitude});
        else
        {
            satellite[i].x = longitude;
            satellite[i].y = latitude;
            satellite[i].z = altitude;
        }        
    }
    sat_init = 1;
    return satellite;
}