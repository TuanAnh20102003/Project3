#ifndef HANDLE_INTERSECTION
#define HANDLE_INTERSECTION

#include <memory>
#include <string>
#include <iostream>
#include <unordered_map>

class Controller;
class RobotState;
class Magnetic;
class Pid;

#define WHEEL_BASE 0.255 //meter distance between two wheels
#define WHEEL_RADIUS 0.065 //meter radius of wheel
#define TURN_TIME 800 //ms
#define RESUME_MARKER_DISTANCE 0.75 //meter
#define PATH R"(C:\Users\anhtu\Downloads\Project3\Project3\command_direction.txt)"

void turn_angle(Controller& motors, double angle, int16_t rpm, char direction, bool active_wheels);
void travel_distance(Controller& motors, char direction, std::shared_ptr<Pid> pid_object, std::shared_ptr<RobotState> state, Magnetic& mag);
void handle_intersection(int& total_intersection_passed, int total_inrersection_required, int& single_intersection_marker_count, std::unordered_map<int, char > num_directions, std::shared_ptr<Pid> pid_object, std::shared_ptr<RobotState> state, Controller& motors, Magnetic& mag);
void marker_check(std::shared_ptr<Pid> pid_object, std::shared_ptr<RobotState> state, Magnetic& mag, Controller& motors, int total_inrersection_required, std::unordered_map<int, char>& num_directions);
#endif // !HANDLE_INTERSECTION
