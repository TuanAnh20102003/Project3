// Purpose: Source file for the Magnetic class.


#include "mag_line.h"


//Handle the case when the modbus read fails
std::vector <uint16_t> Magnetic::modbusFailReadHandle(uint16_t addr, uint16_t nb)
{
	std::vector <uint16_t> reg(nb, 0);
	int rc;
	while (1)
	{
		rc = read_register(ctx2, addr, nb, reg.data());
		if (rc == -1)
		{
			std::cerr << "Error reading register of magnetic sensor: " << modbus_strerror(errno) << std::endl;
			Sleep(100);
		}
		else break;
	}
	return reg;
}
//Constructor
Magnetic::Magnetic(const std::string& port)
{
	ctx2 = modbus_new_rtu(port.c_str(), 115200, 'N', 8, 1);
	if (ctx2 == nullptr) {
		throw std::runtime_error("Unable to allocate modbus context");
	}
	int slave_flag = modbus_set_slave(ctx2, 2);  // Receive data from the device of ID 2
	if (slave_flag == -1) {
		std::cerr << "Error setting slave ID: " << modbus_strerror(errno) << std::endl;
		throw std::runtime_error("Failed to set slave ID");
	}
	// Connect to the Modbus device
	int connection = modbus_connect(ctx2);
	if (connection == -1) {
		throw std::runtime_error("Connection failed");
		modbus_free(ctx2);
	}
	if (connection == 0)
	{
		std::cout << "Connection successful" << std::endl;
	}
}
//Destructor
Magnetic::~Magnetic()
{
	modbus_close(ctx2);
	modbus_free(ctx2);
	//std::cout << "Magnetic sensor destructor called." << std::endl;
}

//Set the ID of the magnetic line sensor
void Magnetic::set_id(int id)
{
	uint16_t id_temp = static_cast<uint16_t>(id);
	write_register(ctx2, MAG_ID_ADDRESS, id_temp);
	uint16_t id_re;
	int rc = read_register(ctx2, MAG_ID_ADDRESS, 1, &id_re);
	if (rc == -1) {
		std::cerr << "Error reading MAG_ID_ADDRESS register: " << modbus_strerror(errno) << std::endl;
		throw std::runtime_error("Failed to read MAG_ID_ADDRESS register");
	}
	else std::cout << "ID of magnetic line sensor is: " << id_re << std::endl;
}

//-----------------------ANALOG FUNCTION--------------------------------
//Get the analog output of the magnetic line sensor
std::vector <uint8_t> Magnetic::get_analog_output()
{
	std::cout << "Get analog output" << std::endl;
	std::vector <uint8_t> reg(16, 0);
	std::vector <uint16_t> values = modbusFailReadHandle(HIGH_2_LOW_1, 8);
	
	for (size_t i = 0; i < values.size(); ++i)
	{
		reg[2 * i + 1] = static_cast<uint8_t>((values[i] & 0xFF00) >> 8);
		reg[2 * i] = static_cast<uint8_t>(values[i] & 0x00FF);
	}
	for (size_t i = 0; i < reg.size(); ++i) //if the value is less than 5, set it to 0
	{
		if (reg[i] <= 5)
		{
			reg[i] = 0;
		}
	}
	std::cout << "Analog output: " << std::endl;
	return reg;
}
//----------------------------------------------------------------------------


//----------------------------DIGITAL FUNCTION--------------------------------
//Reverse the bits of a 16-bit number
uint16_t reverse(uint16_t num)
{
	uint16_t reversed_num = 0;
	for (int i = 0; i < 16; i++)
	{
		reversed_num |= ((num >> i) & 1) << (15 - i); 
	}
	return reversed_num;
}
//get the digital output of the magnetic line sensor
uint16_t Magnetic::get_digital_output()
{
	std::cout << "Get digital output" << std::endl;
	std::vector <uint16_t> values = modbusFailReadHandle(SWITCH_OUTPUT_16_CHANNELS, 1);
	std::cout << "Digital output: " << std::endl;
	return reverse(values[0]);
}
//----------------------------------------------------------------------------
