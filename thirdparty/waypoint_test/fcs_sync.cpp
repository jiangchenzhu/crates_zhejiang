// Standard includes
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>

// For date parsing
#include <boost/date_time/local_time/local_time.hpp>

// For converting WGS84 <-> LTP coordinates
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

// Main entry point
int main(int argc, char* argv[])
{
	// We are in science here :)
	std::cout.precision(16);

	// Base station coordinates (Friday)
	double org_lat = 51.710979902;
	double org_lon = -0.210839049;   
	double org_alt = 141.1027;

	// Setup a converter
	GeographicLib::Geocentric wgs84_ecef(
		GeographicLib::Constants::WGS84_a(), 
		GeographicLib::Constants::WGS84_f()
	);
	GeographicLib::LocalCartesian   wgs84_enu(
		org_lat, 
		org_lon, 
		org_alt, 
		wgs84_ecef
	); 
	
	// Argument check
	if (argc < 2)
	{
		std::cout << "Usage: " << argv[0] << " <file> " << std::endl;
		return 1;
	}

	std::ifstream 	namefile(argv[1]);
	std::string 	input;
	int counter     = 0;
	int line        = 0;

	
  	boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1)); 
    
	// Now process the data
	std::string Date, Time, debug1, debug2, debug3, debug4;
	double x, y, z;
	double lat, lon, alt, en, ee, eu, ede, edn, edu, age, ratio;
	double AnglePitch, AngleRoll, AngleYaw,	PitchReference, RollReference, YawReference,
		AngVelPitch, AngVelRoll, AngVelYaw, TotalAccX, TotalAccY, TotalAccZ, TransAccX,	TransAccY,	
		TransAccZ,	TotalAcc,	 Height,	 dHeight,	HeightReference, dHeightReference, Latitude,	
		Longitude,	GPSHeight,	 SpeedX,	SpeedY, HorizontalAccuracy, Battery1, Battery2;
	long SystemTimestamp, Status, FlightMode, Pressure, TempGyro, TempADC, Pitch, Roll, Yaw, Thrust, GPSStatus;
	while(namefile
		>> Date	 
		>> Time	 
		>> SystemTimestamp
		>> AnglePitch
		>> AngleRoll
		>> AngleYaw	 
		>> PitchReference	
		>> RollReference
		>> YawReference	
		>> AngVelPitch
		>> AngVelRoll	
		>> AngVelYaw	
		>> TotalAccX	
		>> TotalAccY	
		>> TotalAccZ	
		>> TransAccX	
		>> TransAccY	
		>> TransAccZ	
		>> TotalAcc	 
		>> Height
		>> dHeight	
		>> HeightReference	
		>> dHeightReference
		>> Latitude	
		>> Longitude	
		>> GPSHeight	 	 
		>> SpeedX	
		>> SpeedY	
		>> HorizontalAccuracy	
		>> GPSStatus	
		>> Battery1
		>> Battery2	
		>> Status	
		>> FlightMode
		>> Pressure	
		>> TempGyro	
		>> TempADC	
		>> Pitch	 
		>> Roll	 
		>> Yaw	
		>> Thrust
		>> debug1
		>> debug2	
		>> debug3	
		>> debug4)
	{
		// Extract a meaningful value in seconds
	    std::stringstream ss;
	    boost::local_time::local_date_time ldt(boost::local_time::not_a_date_time);
	    boost::local_time::local_time_input_facet* input_facet = new boost::local_time::local_time_input_facet();
	    input_facet->format("%d.%m.%Y %H:%M:%s");
	    ss.imbue(std::locale(ss.getloc(), input_facet));
		ss << Date << " " << Time;
 		ss >> ldt;

 		boost::posix_time::ptime a = ldt.utc_time();
    	boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1));
    	double ms = (a - epoch).total_milliseconds();
    	ms /= 1e3;

 		// Convert value to LTP
		wgs84_enu.Forward(
			Latitude, 
			Longitude, 
			GPSHeight,
			x,
			y,
			z
		);

		// Print seconds, WGS84 and LTP
		std::cout << ms << "," << Latitude << "," << Longitude << "," << GPSHeight << "," << x << "," << y  << "," << z << std::endl;
	}

	// Success
	return 0;
}
