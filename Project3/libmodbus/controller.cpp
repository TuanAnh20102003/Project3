
#include "controller.h"
#include "modbus_helper.h"

#define pi 3.141592653589793
double travel_in_one_rev = 0.408407045; // meters
int cpr = 16385; // counts per revolution
double R_Wheel = 0.065; // meters

    Controller::Controller(const std::string &port) {
        // Initialize Modbus context
        ctx = modbus_new_rtu(port.c_str(), 115200, 'N', 8, 1);
        if (ctx == nullptr) {
            throw std::runtime_error("Unable to allocate modbus context"); 
        }
        modbus_set_slave(ctx, 1);  // Set slave ID to 1

        // Connect to the Modbus device
        if (modbus_connect(ctx) == -1) {
            throw std::runtime_error("Connection failed");
        }

        // Initialize fault list
        FAULT_LIST = {OVER_VOLT, UNDER_VOLT, OVER_CURR, OVER_LOAD, CURR_OUT_TOL,
                      ENCOD_OUT_TOL, MOTOR_BAD, REF_VOLT_ERROR, EEPROM_ERROR, WALL_ERROR, HIGH_TEMP};
    }

    Controller::~Controller() {
        modbus_close(ctx);
        modbus_free(ctx);
		std::cout << "Controller destructor called." << std::endl;
    }
    
    double rpm_to_radPerSec(double rpm)
    {
        return rpm * 2 * pi / 60;
    }
    double rpm_to_linear(double rpm)
    {
        double W_Wheel = rpm_to_radPerSec(rpm);
        return W_Wheel * R_Wheel;
    }
    //chon mode (1,2,3) 3: speed mode, 2: position mode, 1:
    void Controller::set_mode(const uint16_t mode) {
        
        if (mode < 1 || mode > 3) {
            std::cerr << "set_mode ERROR: set only 1, 2, or 3" << std::endl;
            return;
        }
        if(mode == 1) std::cout << "POS_REL_CONTROL" << std::endl;
        else if(mode == 2) std::cout << "POS_ABS_CONTROL" << std::endl;
        else if(mode == 3) std::cout << "VEL_CONTROL" << std::endl;
        write_register(ctx, OPR_MODE, mode);
    }

    //not fixed
    int Controller::get_mode() {
        
        uint16_t mode_register;
    
        // Read 1 holding register starting from OPR_MODE
        int rc = read_register(ctx, OPR_MODE, 1, &mode_register);
    
        // Error handling
        if (rc == -1) {
            std::cerr << "Error reading OPR_MODE register: " << modbus_strerror(errno) << std::endl;
            throw std::runtime_error("Failed to read OPR_MODE register");
        }
    
        // Return the mode value
        return mode_register;
    }


    void Controller::enable_motor() {
        
        write_register(ctx, CONTROL_REG, ENABLE);
    }

    void Controller::disable_motor() {
        
        write_register(ctx, CONTROL_REG, DOWN_TIME);
    }

    std::pair<bool, uint16_t> Controller::get_fault_code() {
        
        uint16_t fault_codes[2];
        read_register(ctx, L_FAULT, 2 , fault_codes);
        bool L_fault_flag = std::find(FAULT_LIST.begin(), FAULT_LIST.end(), fault_codes[0]) != FAULT_LIST.end();
        bool R_fault_flag = std::find(FAULT_LIST.begin(), FAULT_LIST.end(), fault_codes[1]) != FAULT_LIST.end();
        return {L_fault_flag, R_fault_flag};
    }

    void Controller::clear_alarm() {
        
        write_register(ctx, CONTROL_REG, ALRM_CLR);
    }
	//Mode 3 set acceleration time
    void Controller::set_accel_time(int L_ms, int R_ms) {
        
        if(L_ms >32767) L_ms = 32767;
        else if(L_ms < 0) L_ms = 0;
        if(R_ms >32767) R_ms = 32767;
        else if(R_ms < 0) R_ms = 0;
        uint16_t values[2] = {static_cast<uint16_t>(L_ms), static_cast<uint16_t>(R_ms)};
        write_register(ctx, L_ACL_TIME, 2, values);
    }
	//Mode3, set decel time
    void Controller::set_decel_time(int L_ms, int R_ms) {
        
        if(L_ms >32767) L_ms = 32767;
        else if(L_ms < 0) L_ms = 0;
        if(R_ms >32767) R_ms = 32767;
        else if(R_ms < 0) R_ms = 0;
        uint16_t values[2] = {static_cast<uint16_t>(L_ms), static_cast<uint16_t>(R_ms)};
        write_register(ctx, L_DCL_TIME, 2, values);
    }

    //set vel mode 3
    void Controller::set_rpm(int16_t L_rpm, int16_t R_rpm) {
        
        if(L_rpm > 3000) L_rpm = 3000;
        else if(L_rpm < -3000) L_rpm = -3000;
        if(R_rpm > 3000) R_rpm = 3000;
        else if(R_rpm < -3000) R_rpm = -3000;
        uint16_t left_bytes = static_cast<uint16_t>(L_rpm);
        uint16_t right_bytes = static_cast<uint16_t>(R_rpm);
        
        uint16_t values[2] = {left_bytes, right_bytes};
        write_register(ctx, L_CMD_RPM, 2, values);
    }
    //get vel mode 3
    std::pair<double, double> Controller::get_rpm() {
        
        uint16_t registers[2];
        read_register(ctx, L_FB_RPM, 2, registers);
        double fb_L_rpm = static_cast<int16_t>(registers[0]) / 10.0;
        double fb_R_rpm = static_cast<int16_t>(registers[1]) / 10.0;
        return {fb_L_rpm, fb_R_rpm};
    }

    //mode 3, tra van toc dai
    std::pair<double, double> Controller::get_linear_velocities() {
        
        std::pair<double, double> rpms = get_rpm();
        double rpmL = rpms.first;
        double rpmR = rpms.second;
        return std::make_pair(rpm_to_linear(rpmL), rpm_to_linear(-rpmR));
    }
    //mode 1,2 set max vel
    void Controller::set_maxRPM_pos(int max_L_rpm, int max_R_rpm) {
        
        if(max_L_rpm > 1000) max_L_rpm = 1000;
        else if(max_L_rpm < 1) max_L_rpm = 1;
        if(max_R_rpm > 1000) max_R_rpm = 1000;
        else if(max_R_rpm < 1) max_R_rpm = 1;
        uint16_t values[2] = {static_cast<uint16_t>(max_L_rpm), static_cast<uint16_t>(max_R_rpm)};
        write_register(ctx, L_MAX_RPM_POS, 2, values);
    }
    
     int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Function to convert degrees to a 32-bit array (HI_WORD and LO_WORD)
    std::vector<uint16_t> deg_to_32bitArray(int32_t deg) {
        int32_t dec = map(deg, -1440, 1440, -65536, 65536);
        uint16_t HI_WORD = (dec & 0xFFFF0000) >> 16;
        uint16_t LO_WORD = dec & 0x0000FFFF;
        return {HI_WORD, LO_WORD};
    }

    //mode 1,2 set relative angle
    void Controller::set_relative_angle(int32_t ang_L, int32_t ang_R) {
        
        std::vector<uint16_t> L_array = deg_to_32bitArray(ang_L);
        std::vector<uint16_t> R_array = deg_to_32bitArray(ang_R);

        std::vector<uint16_t> all_cmds_array;
        all_cmds_array.insert(all_cmds_array.end(), L_array.begin(), L_array.end());
        all_cmds_array.insert(all_cmds_array.end(), R_array.begin(), R_array.end());
        write_register(ctx, L_CMD_REL_POS_HI, all_cmds_array.size(), all_cmds_array.data());
    }

    
    
    // Mode 1,2Function to get the travelled distance of the wheels in meters
    std::pair<int32_t, int32_t> Controller::get_pulse_travelled()
    {
        
        uint16_t registers[4];
        read_register(ctx, L_FB_POS_HI, 4, registers);
        int32_t l_start_pulse = ((registers[0] & 0xFFFF) << 16) | (registers[1] & 0xFFFF);
        int32_t r_start_pulse = ((registers[2] & 0xFFFF) << 16) | (registers[3] & 0xFFFF);
        return { l_start_pulse, r_start_pulse };
    }
    
    int32_t handle_overflow_32bit(int32_t start_value, int32_t curr_value) {
        int64_t diff = static_cast<int64_t> (curr_value) - static_cast<int64_t> (start_value); 

        if (diff > static_cast < int64_t>(0x7FFFFFFF)) {
            diff -= static_cast < int64_t>(0x100000000);
        }
        else if (diff < static_cast<int64_t>(0x80000000) * -1) {
            diff += static_cast < int64_t>(0x100000000);
        }

        return static_cast<int32_t> (diff);
    }
    
    std::pair<double, double> Controller::get_wheels_travelled(int32_t l_start_pulse, int32_t r_start_pulse, int32_t l_current_pulse, int32_t r_current_pulse)
    {
        
        int32_t l_diff = handle_overflow_32bit(l_start_pulse, l_current_pulse);
        int32_t r_diff = handle_overflow_32bit(r_start_pulse, r_current_pulse);
        double L_meters = travel_in_one_rev * l_diff / cpr;
        double R_meters = travel_in_one_rev * r_diff / cpr;
        return std::pair<double, double>(std::abs(L_meters), std::abs(R_meters));
    }

	void Controller::stop() {
        
		write_register(ctx, CONTROL_REG, EMER_STOP);
	}

     