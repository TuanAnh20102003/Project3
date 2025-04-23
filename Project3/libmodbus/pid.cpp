#include "pid.h"
#include "robot_state.h"
#include "controller.h"
#include "mag_line.h"
#include "marker_check.h"
#include "handle_intersection.h"

/*
*  PID values map
*  format: {speed, {Kp, Ki, Kd}}
*/
const std::unordered_map<int, std::tuple<double, double, double>> Pid::pid_table = {
	{0,  {1.082, 0, 13.3571}},
	{5,  {1.0406, 0, 12.7857}},
	{10, {0.9992, 0, 12.2143}},
	{15, {0.9578, 0, 11.6429}},
	{20, {0.9164, 0, 11.0714}},
	{25, {0.875, 0.0066, 10.5}},
	{30, {0.8336, 0.0095, 9.9286}},
	{35, {0.7922, 0.0129, 9.3571}},
	{40, {0.7508, 0.0169, 8.7857}},
	{45, {0.7094, 0.0214, 8.2143}},
	{50, {0.668, 0.0264, 7.6429}},
	{55, {0.6266, 0.0319, 7.0714}},
	{60, {0.5852, 0.038, 6.5}}
};

/*
* PID controller for analog input
*/
void Pid::pid_controller_analog(Controller& motors, Magnetic& mag, std::shared_ptr<RobotState> state, std::shared_ptr<Pid> pid_object)
{
	std::cout << "PID analog" << std::endl;
	while (state->isRobotRunningAnalog())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500)); // remove after testing
		auto start = std::chrono::high_resolution_clock::now(); //count running time of pid analog

		/*
		* acceleration to the target speed and set pid values
		*/
		std::array<double, 2> current_speed = state->getCmds(); 
		std::cout << "Current speed: " << current_speed[0] << ", " << current_speed[1] << std::endl;
		if (current_speed[0] != get_target_speed())
		{
			double delta = (current_speed[0] < get_target_speed()) ? 2.5 : -2.5; //error
			current_speed[0] += delta;
			current_speed[1] -= delta;
			state->setCmds(current_speed[0], current_speed[1]);
			std::cout << "After speed: " << current_speed[0] << ", " << current_speed[1] << std::endl;
			std::cout << "Target speed: " << get_target_speed() << std::endl;
			int nearest_key = static_cast<int>((current_speed[0] + 2.5f) / 5) * 5;
			std::cout << "Nearest key: " << nearest_key << std::endl;
			setPID_values(nearest_key);
		}
		/*
		* Handle when robot reaches destination
		*/
		if (state->isDestinationReached() && state->getCmds()[0] == 0)
		{
			std::lock_guard<std::mutex> lock(state->getMotorMutex());
			motors.stop();
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			setPID_values(0);
			motors.enable_motor();
			turn_angle(motors, 180, 15, 'L', true);
			//target_speed.store(speed_zone.load());
			break;
		}
		//----------------------
		std::vector<uint8_t> analog_output;
		{
			std::lock_guard<std::mutex> lock(state->getSensorMutex());
			analog_output = mag.get_analog_output();
		}
		if (has_only_1_surge_signal(analog_output).first) //single line or intersection marker
		{
			this->position_analog = get_analog_position(analog_output);
			if (this->position_analog == 100) //out of line
			{
				std::cerr << "No line detected" << std::endl; ////////////////////////checked
				state->startLostLineThread(motors, pid_object); // thread handle when lost line
				this->correction = 0;
			}
			else //on line
			{
				state->cancelLostLineThread(); //cancel lost line thread
				if (state->isMarkerInZone())
				{
					this->prevError.store(get_analog_position(analog_output) - this->target_analog_position); //error
					state->setMarkerInZone(false);
				}
				
				this->error = this->position_analog - this->target_analog_position;
				this->d_value = this->error - this->prevError.load();
				{
					std::lock_guard<std::mutex> lock(state->getPidMutex());
					this->i_value += this->error;
					this->correction = this->Kp * this->error + this->Ki * this->i_value + this->Kd * this->d_value;
				}
				if (this->correction > 20) this->correction = 20;
				else if (this->correction < -20) this->correction = -20;
				this->prevError.store(this->error);
			}
		}
		else // marker shift
		{
			//std::cerr << "More than 1 surge signal detected" << std::endl;
			state->setMarkerInZone(true);
			this->correction = 0;
		}
		int16_t left_speed = static_cast<int16_t>(state->getCmds()[1] - this->correction);
		int16_t right_speed = static_cast<int16_t>(state->getCmds()[0] - this->correction);
		std::cout << "Left speed: " << left_speed << ", Right speed: " << right_speed << std::endl;
		{
			std::lock_guard<std::mutex> lock(state->getMotorMutex());
			motors.set_rpm(right_speed, left_speed);
		}
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		std::cout << "Elapsed time: " << elapsed.count() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(90));

	}
	std::cout << "PID analog has stopped" << std::endl;
}

/*
* PID controller for digital input
*/

void Pid::pid_controller_digital(Controller& motors, Magnetic& mag, std::shared_ptr<RobotState> state, std::shared_ptr<Pid> pid_object)
{
	std::cout << "PID digital" << std::endl;
	while (state->isRobotRunningDigital())
	{
		//auto start = std::chrono::high_resolution_clock::now();
		std::array<double, 2> current_speed = state->getCmds();
		if (current_speed[0] != get_target_speed())
		{
			double delta = (current_speed[0] < get_target_speed()) ? 2.5 : -2.5;
			current_speed[0] += delta;
			current_speed[1] -= delta;
			state->setCmds(current_speed[0], current_speed[1]);
			//std::tuple<double, double, double> pid_value = getPID_table(current_speed[0]);
		}

		if (state->isDestinationReached() && state->getCmds()[0] == 0)
		{
			std::lock_guard<std::mutex> lock(state->getMotorMutex());
			motors.stop();
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			setPID_values(0);
			motors.enable_motor();
			turn_angle(motors, 180, 15, 'L', true);
			//target_speed.store(speed_zone.load());
			break;
		}

		uint16_t digital_output;
		{
			std::lock_guard <std::mutex> lock(state->getSensorMutex());
			digital_output = mag.get_digital_output();
		}

		std::pair<double, int> digital_position = get_digital_position(digital_output);
		this->position_digital = digital_position.first;
		int pin_count = digital_position.second;
		if (this->position_digital == -1)
		{
			std::cerr << "No line detected" << std::endl;
			state->startLostLineThread(motors, pid_object);
			this->correction = 0;
		}
		else
		{
			state->cancelLostLineThread();
			if (pin_count < 7)
			{
				//std::lock_guard<std::mutex> lock(state->getPidMutex()); //could be removed
				this->error = this->position_digital;
				this->d_value = this->error - this->prevError;
				if (std::abs(this->error) < 1) this->correction = 0;
				else this->correction = this->Kp_d * this->error + this->Kd_d * this->d_value;
				if (this->correction > 20) this->correction = 20;
				else if (this->correction < -20) this->correction = -20;
				this->prevError.store(this->error);
			}
			else this->correction = 0;

		}
		int16_t left_speed = static_cast<int16_t>(state->getCmds()[1] - this->correction);
		int16_t right_speed = static_cast<int16_t>(state->getCmds()[0] - this->correction);
		{
			std::lock_guard<std::mutex> lock(state->getMotorMutex());
			motors.set_rpm(right_speed, left_speed); 
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(90));
	}
	std::cout << "PID digital has stopped" << std::endl;
}
/*
*  Check intersection marker
*/
bool Pid::is_intersection_marker(int intersect_marker_check_count)
{
	if (intersect_marker_check_count >= 3) {
		if (speed_zone.load() == 60 && intersect_marker_check_count > 2) return true;
		if (speed_zone.load() == 25 && intersect_marker_check_count > 8) return true;
	}
	return false;
}
void Pid::stopPidThread(std::shared_ptr<RobotState> state) {
	if (pid_thread.joinable()) {
		pid_thread.join();  // wait for thread to finish
	}
	state->setRobotRunningAnalog(false);
	pid_thread = std::thread();
}
void Pid::startPidThreadAnalog(std::shared_ptr<Pid> pid_object, Controller& motors, Magnetic& mag, std::shared_ptr<RobotState> state) {
	stopPidThread(state);  //guarantee that the previous thread is finished
	pid_thread = std::thread([pid_object, &motors, &mag, state]() {
		pid_object->pid_controller_analog(motors, mag, state, pid_object);
		state->setRobotRunningAnalog(false);
		});
}
void Pid::startPidThreadDigital(std::shared_ptr<Pid> pid_object, Controller& motors, Magnetic& mag, std::shared_ptr<RobotState> state) {
	stopPidThread(state);  //guarantee that the previous thread is finished
	pid_thread = std::thread([pid_object, &motors, &mag, state]() {
		pid_object->pid_controller_digital(motors, mag, state, pid_object);
		state->setRobotRunningDigital(false);
		});
}