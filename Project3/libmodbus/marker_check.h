#ifndef MARKER_CHECK
#define MARKER_CHECK

#include <algorithm>
#include <bitset>
#include <iostream>
#include <vector>

//------------------------------ANALOG FUNCTION-----------------------------------------------
double get_analog_position(std::vector <uint8_t> values);
/*
Detect the left and right(acceleration/deceleration) marker,using with has_only_1_surge_signal_marker()
*/
std::string detect_marker_shift(std::vector <uint8_t> previous, std::vector <uint8_t> current);

std::pair <bool, uint8_t> has_only_1_surge_signal(std::vector <uint8_t> arr);
//--------------------------------------------------------------------------------------------


//----------------------------DIGITAL FUNCTION------------------------------------------------
std::pair<double, int> get_digital_position(uint16_t value);
/*
Detect the intersection marker
*/
bool detect_intersection_marker(uint16_t prev, uint16_t curr);

//--------------------------------------------------------------------------------------------
#endif // !MARKER_CHECK

