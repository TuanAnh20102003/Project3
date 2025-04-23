#include "robot_state.h"
#include "controller.h"
#include "pid.h"
#include "handle_intersection.h"

void RobotState::emergency_stop(Controller& motors, std::shared_ptr<Pid> pid_object)
{
    std::scoped_lock lock(stateMutex, motorMutex, pidMutex);
    if (isRobotRunningAnalog() || isRobotRunningDigital()) {
        std::cerr << "[EMERGENCY] Emergency stop !\n";
        setRobotRunningAnalog(false);
        setRobotRunningDigital(false);
        int stop_time = safetyStop(motors);
        stop_time += 500;
        std::this_thread::sleep_for(std::chrono::milliseconds(stop_time));
        setCmds(0, 0);
        pid_object->setPID_values(0);
		//setTargetSpeedReached(false);
        std::cout << "Stopped!" << std::endl;

        motors.enable_motor();
        motors.set_accel_time(50, 50);
        motors.set_decel_time(50, 50);
        std::cout << "Restart the vehicle" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
		cancelLostLineThread();
		motors.disable_motor();
		setOutOfLine(true);
        //not done. 
    }
}

void RobotState::stop_with_time(Controller& motor, int stop_time, std::shared_ptr<Pid> pid_object)
{
    std::scoped_lock lock(stateMutex, motorMutex, pidMutex);
    if (isRobotRunningAnalog() || isRobotRunningDigital())
    {
        std::cerr << "[STOP] Stop with time !\n";
        setRobotRunningAnalog(false);
        setRobotRunningDigital(false);
        motor.set_decel_time(stop_time, stop_time);
        motor.stop();
        stop_time += 500;
        std::this_thread::sleep_for(std::chrono::milliseconds(stop_time));
        setCmds(0, 0);
		pid_object->setPID_values(0);
		//setTargetSpeedReached(false);
        std::cout << "Stopped!" << std::endl;
        motor.enable_motor();
        motor.set_accel_time(50, 50);
        motor.set_decel_time(50, 50);
        std::cout << "Restart the vehicle" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void RobotState::startLostLineThread(Controller& motors, std::shared_ptr<Pid> pid_object)
{
    if (!lost_line_thread_active.exchange(true)) {  //If timer is not active
        std::cout << "Checking for line..." << std::endl;

        lost_line_thread = std::thread([&motors, this, pid_object]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(time_out));
            if (lost_line_thread_active.load())
            {
                this->emergency_stop(motors, pid_object);
            }
            this->lost_line_thread_active.store(false);
            });
    }
}

void RobotState::cancelLostLineThread()
{
    if (lost_line_thread_active.load()) {
        std::cout << "Line detected" << std::endl;
        lost_line_thread_active.store(false);
		if (lost_line_thread.joinable()) {
			lost_line_thread.join();
		}
    }
}

int RobotState::safetyStop(Controller& motors)
{
    auto [left_rpm, right_rpm] = motors.get_rpm();
    double average_rpm = (std::abs(left_rpm) + std::abs(right_rpm)) / 2;
	int stop_time = static_cast<int>((average_rpm / 60) * 1500); //1500 is the time to stop in ms for 60 rpm
    motors.set_decel_time(stop_time, stop_time);
    motors.stop();
    return stop_time;
}

//void RobotState::reachDestination(Controller& motors, std::shared_ptr<pid> pid_object)
//{
//	if (!destination_reached.exchange(true) && cmds[0] == 0) {
//		std::cout << "Destination reached!" << std::endl;
//        {
//			std::lock_guard<std::mutex> lock(motorMutex);
//	        motors.stop();
//            std::this_thread::sleep_for(std::chrono::milliseconds(500));
//			pid_object->setPID_values(0);
//	        motors.enable_motor();
//            turn_angle(motors, 180, 15, 'L', true);
//}
