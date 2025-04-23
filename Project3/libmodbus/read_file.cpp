#include "read_file.h"



bool read_direction_file(const std::string& base_path, int start_num, int end_num,
    int& total_intersection_required, std::unordered_map<int, char>& num_directions) {
    std::string directory_path = base_path + "/" + std::to_string(start_num);
    std::string file_path = directory_path + "/" + std::to_string(start_num) + "_" + std::to_string(end_num) + ".txt";

    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "File not found: " << file_path << std::endl;
        return false;
    }
    else {
        std::cout << "File found: " << file_path << std::endl;
    }

    std::string line;

    // Read number of intersections
    if (std::getline(file, line)) {
        total_intersection_required = std::stoi(line);
    }
    else {
        std::cerr << "File format error: Missing number of intersections." << std::endl;
        return false;
    }

    // Read direction map (e.g., {1:'L', 2:'S', 3:'R'})
    if (std::getline(file, line)) {
        for (size_t i = 0; i < line.length(); ++i) {
            if (std::isdigit(line[i])) {
                int key = line[i] - '0';
                while (i < line.length() && line[i] != '\'') i++; // skip to first '
                if (i + 1 < line.length()) {
                    char value = line[i + 1];
                    num_directions[key] = value;
                    i += 2; // skip past closing '
                }
            }
        }
    }

    return true;

}
void print_directions(int total_intersection_required, const std::unordered_map<int, char>& num_directions) {
    std::cout << "Number of intersections: " << total_intersection_required << std::endl;
    std::cout << "Directions:" << std::endl;
    for (const auto& pair : num_directions) {
        std::cout << pair.first << ": " << pair.second << std::endl;
    }
    std::cout << std::endl;
}
void find(std::unordered_map<int, char> num_directions, int total_intersection_passed)
{
    auto it = num_directions.find(total_intersection_passed);
    char direction = (it != num_directions.end()) ? it->second : '\0';  // '\0' if not found
    if (direction != '\0') {
        std::cout << "Direction for intersection " << total_intersection_passed << ": " << direction << std::endl;
    }
    else {
        std::cout << "No direction found for intersection " << total_intersection_passed << std::endl;
    }
    std::cout << std::endl;
}