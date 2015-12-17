// Standard includes
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

// For converting WGS84 <-> LTP coordinates
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

// Main entry point
int main(int argc, char* argv[])
{
	// Base station coordinates
	double org_lat = 51.712419245;
	double org_lon = -0.209793490;   
	double org_alt = 152.3279;

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
	int counter = 0;
	int line    = 0;

	// Eat up the heading
	for (int i = 0; i < 15; i++)
		namefile >> input;

	// Now process the data
	std::string d, t;
	double x, y, z;
	double lat, lon, alt, en, ee, eu, ede, edn, edu, age, ratio;
	int q, ns;
	while(namefile >> d >> t >> lat >> lon >> alt >> q >> ns >> en >> ee >> eu >> edn >> ede >> edu >> age >> ratio)
	{
		wgs84_enu.Forward(
			lat, 
			lon, 
			alt,
			x,
			y,
			z
		);
		std::cout << x << "," << y  << "," << z << std::endl;
	}

	// Success
	return 0;
}
