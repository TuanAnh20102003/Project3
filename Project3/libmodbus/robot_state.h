#ifndef ROBOT_STATE
#define ROBOT_STATE

#include <array>
#include <atomic>
#include <mutex>
#include <iostream>
#include <memory>
#include <thread>

class Controller;
class Pid;

class RobotState {
private:
    mutable std::mutex stateMutex;
	mutable std::mutex motorMutex;
	mutable std::mutex sensorMutex;
	mutable std::mutex pidMutex;

    std::array<double, 2> cmds = { 0, 0 };
	std::thread lost_line_thread;
    std::atomic<bool> robot_running_analog = true;
    std::atomic<bool> robot_running_digital = false;
    std::atomic<bool> lost_line_thread_active = false;
	std::atomic<bool> marker_in_zone = false;
	std::atomic<bool> out_of_line = false;
	std::atomic<bool> start_point_passed = false;
    std::atomic<bool> destination_reached = false;

public:
    
    static constexpr int time_out = 500;

    // ======== Getter và Setter =========
    std::mutex& getMotorMutex() { return motorMutex; }
    std::mutex& getSensorMutex() { return sensorMutex; }
	std::mutex& getPidMutex() { return pidMutex; }
    inline void setRobotRunningAnalog(bool value) {robot_running_analog.store(value, std::memory_order_relaxed);}
    inline bool isRobotRunningAnalog() const {return robot_running_analog.load(std::memory_order_relaxed);}
    inline void setRobotRunningDigital(bool value) {robot_running_digital.store(value, std::memory_order_relaxed);}
    inline bool isRobotRunningDigital() const {return robot_running_digital.load(std::memory_order_relaxed);}
    inline void setDestinationReached(bool value) { destination_reached.store(value, std::memory_order_relaxed);}
    inline bool isDestinationReached() const {return destination_reached.load(std::memory_order_relaxed);}
    inline void setMarkerInZone(bool value) {marker_in_zone.store(value, std::memory_order_relaxed);}
    inline bool isMarkerInZone() const {return marker_in_zone.load(std::memory_order_relaxed);}
	inline void setOutOfLine(bool value) { out_of_line.store(value, std::memory_order_relaxed); }
	inline bool isOutOfLine() const { return out_of_line.load(std::memory_order_relaxed); }
	inline void setStartPointPassed(bool value) { start_point_passed.store(value, std::memory_order_relaxed); }
	inline bool isStartPointPassed() const { return start_point_passed.load(std::memory_order_relaxed); }
	
    
	// ===================================
    
	// ==========Handle stop==============
    void emergency_stop(Controller& motors, std::shared_ptr<Pid> pid_object);
    void stop_with_time(Controller& motor, int stop_time, std::shared_ptr<Pid> pid_object);
    void startLostLineThread(Controller& motors, std::shared_ptr<Pid> pid_object);
    void cancelLostLineThread();  
    int safetyStop(Controller& motors);
	//void reachDestination(Controller& motors, std::shared_ptr<pid> pid_object);
	// ===================================

    void setCmds(double left, double right) {
        //std::lock_guard<std::mutex> lock(stateMutex);
        cmds[0] = left;
        cmds[1] = right;
    }

    std::array<double, 2> getCmds() const {
        //std::lock_guard<std::mutex> lock(stateMutex);
        return cmds;
    }
};

#endif // !ROBOT_STATE
