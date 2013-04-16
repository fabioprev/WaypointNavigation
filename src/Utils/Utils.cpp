#include "Utils.h"
#include <cmath>
#include <iostream>

using namespace std;

namespace SSI
{
	map<Utils::ColorTypes,string> Utils::colorsTable(Utils::initColorsTable());
	
	Utils::Utils() {;}
	
	Utils::~Utils() {;}
	
	double Utils::deg2rad(double d)
	{
		return (d * M_PI) / 180.0;
	}
	
	map<Utils::ColorTypes,string> Utils::initColorsTable()
	{
		map<Utils::ColorTypes,string> table;
		
		colorsTable.insert(make_pair(Blue,"\033[22;34;1m"));
		colorsTable.insert(make_pair(Cyan,"\033[22;36;1m"));
		colorsTable.insert(make_pair(Green,"\033[22;38;1m"));
		colorsTable.insert(make_pair(Magenta,"\033[22;35;1m"));
		colorsTable.insert(make_pair(Red,"\033[22;31;1m"));
		colorsTable.insert(make_pair(White,"\033[22;37;1m"));
		colorsTable.insert(make_pair(Yellow,"\033[22;33;1m"));
		
		return table;
	}
	
	void Utils::print(const string& s, ColorTypes color)
	{
		const map<ColorTypes,string>::const_iterator& entryColorsTable = colorsTable.find(color);
		
		if (entryColorsTable != colorsTable.end()) cerr << entryColorsTable->second << s << "\033[0m" << flush;
		else cerr << s << flush;
	}
	
	void Utils::println(const string& s, ColorTypes color)
	{
		print(s,color);
		
		cerr << endl;
	}
	
	double Utils::rad2deg(double d)
	{
		return (d * 180.0) / M_PI;
	}
}
