#ifndef PID
#define PID
#include <iostream>
#include <thread>
#include <unordered_map>

class Controller;
class RobotState;
class Magnetic;



class Pid {
private:
	double Kp = 1.082, Ki = 0.0, Kd = 13.3571;
	double error = 0.0;
	std::atomic<double> prevError = 0.0;
	std::atomic<int> speed_zone = 25; //store target speed
	double d_value = 0.0, i_value = 0.0; //pid values analog
	double position_analog = 0; 
	double position_digital = 0;
	double correction = 0.0; 
	std::atomic<int> target_speed = speed_zone.load();
	std::thread pid_thread;
	
	static const std::unordered_map<int, std::tuple<double, double, double>> pid_table;
public:
	const double Kp_d = 0.2926, Kd_d = 7.5; //pid values for digital
	const double target_analog_position = 7.5;
	
	void setPID_values(int key) 
	{ //return pid_table.at(key); 
		std::tuple<double, double, double> pid_value = pid_table.at(key);
		this->Kp = std::get<0>(pid_value);
		this->Ki = std::get<1>(pid_value);
		this->Kd = std::get<2>(pid_value);
	}
	inline void set_speed_zone(int speed) { speed_zone.store(speed); }
	inline int get_speed_zone() { return speed_zone.load(); }
	inline void set_target_speed(int speed) { target_speed.store(speed); }
	inline void set_target_speed_in_zone() { target_speed.store(speed_zone.load()); }
	inline int get_target_speed() { return target_speed.load(); }
	inline void set_i_value(double value) { i_value = value; }
	inline double get_i_value() { return i_value; }
	inline void set_prev_error(double value) { prevError.store(value); }
	inline double get_prev_error() { return prevError.load(); }
	void stopPidThread(std::shared_ptr<RobotState> state);
	void startPidThreadAnalog(std::shared_ptr<Pid> pid_object, Controller& motors, Magnetic& mag, std::shared_ptr<RobotState> state);	
	void startPidThreadDigital(std::shared_ptr<Pid> pid_object, Controller& motors, Magnetic& mag, std::shared_ptr<RobotState> state);
	/*
	*	PID controller for analog input
	*/
	void pid_controller_analog(Controller & motors, Magnetic & mag, std::shared_ptr <RobotState> state, std::shared_ptr<Pid> pid_object);
	
	/*
	*	PID controller for digital input
	*/
	void pid_controller_digital(Controller& motors, Magnetic& mag, std::shared_ptr<RobotState> state, std::shared_ptr<Pid> pid_object);
	/*
	*	Check intersection marker
	*/
	bool is_intersection_marker(int intersect_marker_check_count);
};

#endif // !PID
