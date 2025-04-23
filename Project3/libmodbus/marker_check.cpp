#include "marker_check.h"




//------------------------------ANALOG FUNCTION-----------------------------------------------
/*
Detect the center position of the line using analog sensor
*/
double get_analog_position(std::vector <uint8_t> values)
{
	int weight = 0;
	for (int i = 0; i < values.size(); i++)
	{
		weight += i * values[i];
	}
	int sum = 0;
	for (int i = 0; i < values.size(); i++)
	{
		sum += values[i];
	}
	if (sum == 0) //no line detected
	{
		return 100;
	}
	else
	{
		double position = static_cast<double> (weight) / static_cast<double> (sum);
		return position;
	}
}
/*
Detect the left and right(acceleration/deceleration) marker,using with has_only_1_surge_signal_marker()
*/
std::string detect_marker_shift(std::vector <uint8_t> previous, std::vector <uint8_t> current)
{
	double pre_position = get_analog_position(previous);
	double cur_position = get_analog_position(current);
	if (pre_position == 100 || cur_position == 100) return "NOT_LINE";
	if (cur_position < pre_position) return "LEFT";
	else if (cur_position > pre_position) return "RIGHT";
	else return "NOT_SHIFT";
}
/*
use for detect_marker_shift, return the max analog data and true (if on single line)
*/
std::pair <bool, uint8_t> has_only_1_surge_signal(std::vector <uint8_t> arr)
{
	auto it = std::max_element(arr.begin(), arr.end()); // Find the peak value
	uint8_t peak_index = std::distance(arr.begin(), it); // Find the index of the peak value
	for (int i = 0; i < peak_index; i++)
	{
		if (arr[i] > arr[i + 1]) return { false, arr[peak_index] };
	}
	for (int i = peak_index + 1; i < arr.size() - 1; i++)
	{
		if (arr[i] < arr[i + 1]) return { false, arr[peak_index] };
	}
	return { true, arr[peak_index] };
}
//--------------------------------------------------------------------------------------------

//----------------------------DIGITAL FUNCTION------------------------------------------------
/*
Detect the center position of the line using digital sensor
*/
std::pair<double, int> get_digital_position(uint16_t value)
{
	std::bitset<16> binary(value);  // Convert the value to binary
	std::vector<int> positions;

	// Store the positions of '1' bits in the binary number
	for (int i = 0; i < 16; i++) {
		if (binary[15 - i]) {
			positions.push_back(i);
		}
	}

	int len_pos = positions.size();
	//std::cout << "Length: " << len_pos << std::endl;
	if (len_pos == 0) return {(double) -1, 0};  // Return -1 if there are no '1' bits

	double sum_pos = 0;
	for (size_t i = 0; i < positions.size(); i++) {
		sum_pos += positions[i];
		//std::cout << "pos: " << positions[i] << std::endl;
	}
	double pos = sum_pos / len_pos;
	return { pos - 7.5, len_pos }; //target position for digital is 0
}
/*
Detect the intersection marker
*/
bool detect_intersection_marker(uint16_t prev, uint16_t curr)
{
	std::pair<double, int> prev_data = get_digital_position(prev);
	std::pair<double, int> curr_data = get_digital_position(curr);
	if ((curr_data.second - prev_data.second) <= 1) return false;
	if (curr_data.first == 0 || prev_data.first == 0) return false;
	if (abs(curr_data.first - prev_data.first) > 0.5) return false;
	return true;
}

//--------------------------------------------------------------------------------------------
