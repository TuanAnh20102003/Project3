#ifndef READ_FILE
#define READ_FILE

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
//#include <filesystem>
#include <cctype>


bool read_direction_file(const std::string& base_path, int start_num, int end_num,
    int& total_intersection_required, std::unordered_map<int, char>& num_directions);
void print_directions(int total_intersection_required, const std::unordered_map<int, char>& num_directions);
void find(std::unordered_map<int, char> num_directions, int total_intersection_passed);
#endif // !READ_FILE