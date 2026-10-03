#ifndef GPIO_ENCODER_LISTENER_H
#define GPIO_ENCODER_LISTENER_H


#include <gpiod.h>
#include <string.h>
#include <unistd.h>

#include <map>
#include <mutex>
#include <thread>


class GPIO_Encoder_Listener
{
private:
	// private variables
	static const int GPIO_CHIP_NAME_LENGTH_MAX = 20;
	static const int LINE_NUMBER_MOTOR_R_CH_A = 105;
	static const int LINE_NUMBER_MOTOR_R_CH_B = 106;
	static const int LINE_NUMBER_MOTOR_L_CH_A =  84;
	static const int LINE_NUMBER_MOTOR_L_CH_B = 130;
	const int line_numbers[4] = {
		LINE_NUMBER_MOTOR_R_CH_A,
		LINE_NUMBER_MOTOR_R_CH_B,
		LINE_NUMBER_MOTOR_L_CH_A,
		LINE_NUMBER_MOTOR_L_CH_B
	};
	std::map<int, int> line_number_to_index = {
		{LINE_NUMBER_MOTOR_R_CH_A, 0},
		{LINE_NUMBER_MOTOR_R_CH_B, 1},
		{LINE_NUMBER_MOTOR_L_CH_A, 2},
		{LINE_NUMBER_MOTOR_L_CH_B, 3}
	};
	char gpio_chip_name[GPIO_CHIP_NAME_LENGTH_MAX];
	struct gpiod_chip *gpio_chip;
	struct gpiod_line_bulk bulk;
	int disp1=0, disp2=0;
	bool continue_listening = false;
	// mutexes
	std::mutex disp_mutex;
	std::mutex fin_mutex;
	// the edge listener thread
	std::thread edge_listening_thread;

public:
	// public constructors
	GPIO_Encoder_Listener();

private:
	// private functions
	void listen();

public:
	// public functions
	const char *get_chip_name();
	void close_device();
	void start_listening_to_edges();
	void stop_listening();
	void get_displacements(int *disp1, int *disp2);
};

#endif // GPIO_ENCODER_LISTENER_H
