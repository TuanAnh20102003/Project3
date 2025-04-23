#include "handle_intersection.h"
#include "controller.h"
#include "robot_state.h"
#include "pid.h"
#include "mag_line.h"
#include "marker_check.h"
#include "read_file.h"

/*
*	Turn function
*	active_wheels = true: both wheels are active, false: only one wheel is active
*/
void turn_angle(Controller& motors, double angle, int16_t rpm, char direction, bool active_wheels)
{
	motors.set_accel_time(TURN_TIME, TURN_TIME);
	motors.set_decel_time(TURN_TIME, TURN_TIME);
	double turn_radius = (active_wheels) ? WHEEL_BASE / 2 : WHEEL_BASE;
	double turn_distance = 2 * 3.14159265359 * turn_radius * angle / 360;
	int temp = (direction == 'L') ? 1 : -1;
	if (active_wheels)
	{
		motors.set_rpm(rpm * temp, rpm * temp);
	}
	else
	{
		if(direction == 'L') motors.set_rpm(rpm * temp, 0);
		else motors.set_rpm(0, rpm * temp);
	}
	std::pair<int32_t, int32_t> start_pulse = motors.get_pulse_travelled();
	while (1)
	{
		std::pair<int32_t, int32_t> current_pulse = motors.get_pulse_travelled();
		std::pair<double, double> travelled_distance = motors.get_wheels_travelled(start_pulse.first, start_pulse.second, current_pulse.first, current_pulse.second);
		std::pair<double, double> rpm = motors.get_rpm();
		double stop_distance_R = std::abs(rpm.first) * 3.14159265359 * WHEEL_RADIUS * TURN_TIME / 60000;
		double stop_distance_L = std::abs(rpm.second) * 3.14159265359 * WHEEL_RADIUS * TURN_TIME / 60000;
		if (active_wheels)
		{
			if (travelled_distance.first + stop_distance_R >= std::abs(turn_distance) && travelled_distance.second + stop_distance_L >= std::abs(turn_distance)) break;
		}
		else
		{
			double moving_distance = (direction == 'L') ? travelled_distance.first : travelled_distance.second;
			double stop_distance = (direction == 'L') ? stop_distance_R : stop_distance_L;
			if (moving_distance + stop_distance >= std::abs(turn_distance)) break;
		}
	}
	motors.stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(1300));
	motors.enable_motor();
	motors.set_accel_time(50, 50);
	motors.set_decel_time(50, 50);
	std::cout << "Restart the vehicle after turn" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
/*
*	Handle the distance of intersection 
*/
void travel_distance(Controller& motors, char direction, std::shared_ptr<Pid> pid_object, std::shared_ptr<RobotState> state, Magnetic& mag)
{
	double stop_turn_distance;
	std::pair<double, double> rpm;
	std::pair<int32_t, int32_t> current_pulse;
	std::pair<double, double> travelled_distance;
	std::pair<int32_t, int32_t> start_pulse = motors.get_pulse_travelled();
	if (direction == 'L' || direction == 'R') //have to turn at current intersection
	{
		if (pid_object->get_speed_zone() == 60) stop_turn_distance = 0.37;
		else stop_turn_distance = 0.385;
		while (1)
		{
			{
				std::lock_guard<std::mutex> lock(state->getMotorMutex()); 
				rpm = motors.get_rpm();
				current_pulse = motors.get_pulse_travelled();
				travelled_distance = motors.get_wheels_travelled(start_pulse.first, start_pulse.second, current_pulse.first, current_pulse.second);
			}
			double average_rpm = (std::abs(rpm.second) + std::abs(rpm.first)) / 2;
			int stop_time = static_cast<int>((average_rpm / 60) * 1500);
			double linear_speed = (average_rpm * 2 * 3.14159265359 / 60000) * WHEEL_RADIUS;
			double stop_distance = linear_speed * stop_time / 2;
			double current_distance = (travelled_distance.first + travelled_distance.second) / 2;
			if (current_distance + stop_distance >= stop_turn_distance) //reach the center of intersection
			{
				std::scoped_lock lock(state->getMotorMutex(), state->getSensorMutex()); 
				std::cout << "Stop and turn" << std::endl;
				state->stop_with_time(motors, stop_time, pid_object);
				turn_angle(motors, 90, 15, direction, false);
				pid_object->set_speed_zone(25);
				pid_object->set_target_speed_in_zone();
				pid_object->set_i_value(0);
				pid_object->set_prev_error(get_analog_position(mag.get_analog_output()) - pid_object->target_analog_position);
				state->setRobotRunningAnalog(true);
				pid_object->startPidThreadAnalog(pid_object, motors, mag, state); //create new pid analog thread
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			
		}
	}
	else //do not have to turn so using pid digital
	{
		state->setRobotRunningAnalog(false);
		state->setRobotRunningDigital(true);
		pid_object->stopPidThread(state);
		pid_object->startPidThreadDigital(pid_object, motors, mag, state);

	}
	while (1) //while out of intersection area
	{
		current_pulse = motors.get_pulse_travelled();
		travelled_distance = motors.get_wheels_travelled(start_pulse.first, start_pulse.second, current_pulse.first, current_pulse.second);
		double current_distance = (travelled_distance.first + travelled_distance.second) / 2;
		if (current_distance >= RESUME_MARKER_DISTANCE) break;
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}
void handle_intersection(int& total_intersection_passed, int total_inrersection_required, int& single_intersection_marker_count, std::unordered_map<int, char > num_directions, std::shared_ptr<Pid> pid_object, std::shared_ptr<RobotState> state, Controller& motors, Magnetic& mag )
{
	if (!state->isStartPointPassed())
	{
		//std::cout << "Start point passed" << std::endl;
		total_intersection_passed -= 1;
		state->setStartPointPassed(true);
		single_intersection_marker_count = 0; //reset the number of marker travelled in one intersection
		pid_object->set_speed_zone(60);
		pid_object->set_target_speed_in_zone();
		return;
	}
	if (total_intersection_passed > total_inrersection_required)
	{
		total_intersection_passed = 0;
		pid_object->set_target_speed(0);
		single_intersection_marker_count = 0; //reset the number of marker travelled in one intersection
		state->setDestinationReached(true);
		return;
	}
	auto it = num_directions.find(total_intersection_passed);
	char direction = (it != num_directions.end()) ? it->second : '\0';  // '\0' if not found
	std::thread distance_thread([&]() {
		travel_distance(motors, direction, pid_object, state, mag);
		});   
	distance_thread.join();
 //   std::pair<int, std::unordered_map<int, char>> data = readDirectionFile(filename);
 //   std::unordered_map<int, char > num_directions = data.second;
 //   auto it = num_directions.find(intersection_index);
 //   char direction = (it != num_directions.end()) ? it->second : '\0';  // '\0' if not found
	//std::thread distance_thread([&]() {
	//	travel_distance(motors, direction, pid_object, state, mag);
	//	});   
	//distance_thread.join();
}

/*
*	Check marker when running
*/
void marker_check(std::shared_ptr<Pid> pid_object, std::shared_ptr<RobotState> state, Magnetic& mag, Controller& motors, int total_inrersection_required, std::unordered_map<int, char>& num_directions)
{
	int marker_check_count = 0; //when shift marker detected, it will increased by 1
	std::vector<uint8_t> analog_array;
	std::vector<uint8_t> prev_analog_array;
	std::vector<uint8_t> curr_analog_array;
	uint16_t digital_output = 0;
	uint16_t prev_digital_output = 0;
	uint16_t curr_digital_output = 0;
	int intersection_marker_check_count = 0; //when intersection marker is detected, it will be increased by 1
	int single_intersection_marker_count = 0; //number of marker travelled in one intersection (<= 2)
	int pin_count = -1;
	int total_intersection_passed = 0; //total intersection passed
	
	while ((!state->isOutOfLine()) && (!state->isDestinationReached()))
	{
		{
			std::lock_guard<std::mutex> lock(state->getSensorMutex());
			analog_array = mag.get_analog_output();
		}
		std::pair <bool, uint8_t> surge_signal = has_only_1_surge_signal(analog_array);
		if (surge_signal.first) //single line or intersection marker
		{
			prev_analog_array = analog_array;
			if ((marker_check_count < -13) && (pid_object->get_speed_zone() == 25))
			{
				std::cout << "Speed up" << std::endl;
				pid_object->set_speed_zone(60);
				pid_object->set_target_speed_in_zone(); 
				pid_object->set_i_value(0);
			}
			if ((marker_check_count > 3) && (pid_object->get_speed_zone() == 60)) //speed down
			{
				std::cout << "Speed down" << std::endl;
				pid_object->set_speed_zone(25);
				pid_object->set_target_speed_in_zone();
				pid_object->set_i_value(0);
			}
			marker_check_count = 0;
			{
				std::lock_guard<std::mutex> lock(state->getSensorMutex());
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				digital_output = mag.get_digital_output();
				pin_count = get_digital_position(digital_output).second;
			}
				
			if (pin_count < 7) //single line
			{	
				std::cout << "Single line detected" << std::endl;
				prev_digital_output = digital_output;
				std::cout << "Previous digital output: " << std::bitset<16>(prev_digital_output) << std::endl;
				if (pid_object->is_intersection_marker(intersection_marker_check_count)) 
				{
					single_intersection_marker_count += 1;
					if (single_intersection_marker_count >= 2) //out of intersection area
					{
						std::cout << "Out of intersection area" << std::endl;
						if (state->isRobotRunningDigital()) //switch to pid analog 
						{
							state->setRobotRunningDigital(false);
							state->setRobotRunningAnalog(true);
							pid_object->stopPidThread(state); 
							pid_object->startPidThreadAnalog(pid_object, motors, mag, state);
						}
						else if (state->isRobotRunningAnalog()) //speed up 
						{
							pid_object->set_speed_zone(60);
							pid_object->set_target_speed_in_zone();
							pid_object->set_i_value(0);
						}
						single_intersection_marker_count = 0;
					}
					else
					{
						std::cout << "In Intersection area" << std::endl;
						total_intersection_passed += 1;
						handle_intersection(total_intersection_passed, total_inrersection_required, single_intersection_marker_count, num_directions, pid_object, state, motors, mag);
					}
				}
				intersection_marker_check_count = 0;

			}
			else if (pin_count > 6 && surge_signal.second > 50) //intersection marker
			{
					
				curr_digital_output = digital_output; 
				std::cout << "Previous digital output: " << std::bitset<16>(prev_digital_output) << std::endl;
				std::cout << "Current digital output: " << std::bitset<16>(curr_digital_output) << std::endl;
				if (detect_intersection_marker(prev_digital_output, curr_digital_output))
				{
					intersection_marker_check_count += 1;
					std::cout << "Intersection marker detected" << std::endl;
				}
			}

		}
		else if (surge_signal.second > 20) //maker shift
		{
			curr_analog_array = analog_array;
			std::string marker_shift = detect_marker_shift(prev_analog_array, curr_analog_array);
			if (marker_shift == "LEFT")
			{
				std::cout << "Left shift" << std::endl;
				marker_check_count -= 1;
			}
			else if (marker_shift == "RIGHT")
			{
				std::cout << "Right shift" << std::endl;
				marker_check_count += 1;
			}
			else if (marker_shift == "NOT_SHIFT")
			{
				std::cout << "No shift" << std::endl;
			}
			else if (marker_shift == "NOT_LINE")
			{
				std::cout << "No line detected" << std::endl;
				//state->startLostLineThread(motors, pid_object); // Error

			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}