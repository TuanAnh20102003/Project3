// Project3.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <mutex>
#include "controller.h"
#include "pid.h"
#include "mag_line.h"
#include "robot_state.h"
#include "marker_check.h"
#include "handle_intersection.h"
#include "read_file.h"




int main()
{
	Controller motors("COM1");
	Magnetic mag("COM2");
	std::shared_ptr<Pid> pid_object = std::make_shared<Pid>();
	std::shared_ptr<RobotState> state = std::make_shared<RobotState>();

	std::string base_path = "C://Users//anhtu//Downloads//Project3//Map_Direction"; // Base path to the directory containing the files
	int start_table = 0; // Starting number for the file name
	int end_table[] = { 1 };
	int total_intersection_required = 0;
	std::unordered_map<int, char> num_directions;
	int num_tables = sizeof(end_table) / sizeof(end_table[0]);

	motors.disable_motor();
	motors.set_mode(3);
	motors.enable_motor();
	motors.set_accel_time(50, 50);
	motors.set_decel_time(50, 50);

	for (int i = 0; i < num_tables + 1; i++)
	{
		num_directions.clear();
		if (i == 0)
		{
			read_direction_file(base_path, start_table, end_table[i], total_intersection_required, num_directions);
		}
		else if (i == num_tables)
		{
			read_direction_file(base_path, end_table[i - 1], start_table, total_intersection_required, num_directions);
		}
		else
		{
			read_direction_file(base_path, end_table[i - 1], end_table[i], total_intersection_required, num_directions);

		}
		//print_directions(total_intersection_required, num_directions);
	
		/*
		*  Set AGV's initial state
		*/
		state->setRobotRunningAnalog(true);
		state->setRobotRunningDigital(false);
		pid_object->set_speed_zone(25);
		pid_object->set_target_speed_in_zone();
		state->setOutOfLine(false);

		/*
		*  Set initial flag of start point and destination
		*/
		state->setStartPointPassed(false);
		state->setDestinationReached(false);
		pid_object->set_i_value(0.0);
		pid_object->set_prev_error(get_analog_position(mag.get_analog_output()) - pid_object->target_analog_position);

		pid_object->startPidThreadAnalog(pid_object, motors, mag, state);
		//std::thread pid_thread(&pid::pid_controller_analog, pid_object, std::ref(motors), std::ref(mag), state);
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		std::thread marker_thread([pid_object, state, &mag, &motors, total_intersection_required, &num_directions]() {
			marker_check(pid_object, state, mag, motors, total_intersection_required, num_directions);
			});

		marker_thread.join();
		pid_object->stopPidThread(state);
		
	}

	
	
	/*uint16_t prev_digital_output = mag.get_digital_output();
	std::pair<double, int> digital_position = get_digital_position(prev_digital_output);
	std::cout << "Digital position: " << digital_position.first << std::endl;
	std::cout << "Pin count: " << digital_position.second << std::endl;
	std::vector<uint8_t> analog_array = mag.get_analog_output();
	std::cout << "Analog output: ";
	for (size_t i = 0; i < analog_array.size(); ++i)
	{
		std::cout << static_cast<int>(analog_array[i]) << " ";
	}
	std::pair <bool, uint8_t> surge_signal = has_only_1_surge_signal(analog_array);
	std::cout << "Surge signal: " << surge_signal.first << std::endl;
	std::cout << "Peak value: " << static_cast<int>(surge_signal.second) << std::endl;*/

}

//int main() {
//    Py_Initialize();  // ✅ Initialize Python interpreter
//
//    std::vector<double> x = { 1, 2, 3, 4 };
//    std::vector<double> y = { 1, 4, 9, 16 };
//
//    plt::plot(x, y);
//    plt::show();
//
//    Py_Finalize();  // ✅ Clean up Python interpreter
//    return 0;
//}