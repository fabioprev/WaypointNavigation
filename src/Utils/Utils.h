#ifndef UTILS_H
#define UTILS_H

#include <map>
#include <string>

namespace SSI
{
	class Utils
	{
		public:
			enum ColorTypes
			{
				Blue = 0,
				Cyan,
				Green,
				Magenta,
				Red,
				White,
				Yellow
			};
			
			/**
			 *	Angle conversion from degrees to radiants.
			 *	@param d represents the angle value.
			 *	@return the angle converted.
			 */
			static double deg2rad(double d);
			
			/**
			 *	Colors table initialization.
			 *	@return returns a map with some colors inserted.
			 */
			static std::map<Utils::ColorTypes,std::string> initColorsTable();
			
			/**
			 *	In order to print a string with a specified color without endline.
			 *	@param s represents the string that have to be printed.
			 *	@param color represents the color used.
			 */
			static void print(const std::string& s, ColorTypes color);
			
			/**
			 *	In order to print a string with a specified color with endline.
			 *	@param s represents the string that have to be printed.
			 *	@param color represents the color used.
			 */
			static void println(const std::string& s, ColorTypes color);
			
			/**
			 *	Angle conversion from radiants to degrees.
			 *	@param d represents the angle value.
			 *	@return the angle converted.
			 */
			static double rad2deg(double d);
			
		private:
			static const int MAX_MESSAGE_SIZE = 1024;
			
			/**
			 *	Colors table map.
			 */
			static std::map<ColorTypes,std::string> colorsTable;
			
			Utils();
			
			~Utils();
	};
}

#endif
