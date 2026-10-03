#include "gpio_encoder_listener/gpio_encoder_listener.h"


GPIO_Encoder_Listener::GPIO_Encoder_Listener()
{
	struct gpiod_line *line;
	strcpy(this->gpio_chip_name, "/dev/gpiochip1");
	this->gpio_chip = gpiod_chip_open(this->gpio_chip_name);
	gpiod_line_bulk_init(&(this->bulk));
	for (int i = 0; i < 4; i++)
	{
		line = gpiod_chip_get_line(this->gpio_chip, this->line_numbers[i]);
		gpiod_line_bulk_add(&(this->bulk), line);
	}
	gpiod_line_request_bulk_both_edges_events(&(this->bulk), "foobar");
}

const char *GPIO_Encoder_Listener::get_chip_name()
{
	return this->gpio_chip_name;
}

void GPIO_Encoder_Listener::close_device()
{
	gpiod_chip_close(this->gpio_chip);
}

void GPIO_Encoder_Listener::listen()
{
	struct gpiod_line_bulk event_bulk;
	struct gpiod_line_event event;
	uint8_t motor1_curr_state = 0, motor2_curr_state = 0;
	uint8_t motor1_prev_state = 0, motor2_prev_state = 0;
	struct timespec timeout = {0, 1000000};
	int event_status;
	int previous_event_types[4] = {0, 0, 0, 0};
	int event_types[4] = {0, 0, 0, 0};
	// uint8_t motor1_state = 0, motor2_state = 0;
	// struct timespec prev_time = {0, 0};
	while (this->continue_listening)
	{
		// Dummy for now
		// this->disp1 += 1;
		// this->disp2 += 1;
		// usleep(1000);
		// int event_detected[4] = {0, 0, 0, 0};
		// struct timespec event_ts[4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
		event_status = gpiod_line_event_wait_bulk(&(this->bulk), &timeout, &event_bulk);
		if (event_status == -1)
		{
			continue; // error occured, might want to handle this differently.
		}
		else if (event_status == 0)
		{
			continue; // timeout, go straight back to the beginning of the loop.
		}
		int num_events_waited = gpiod_line_bulk_num_lines(&event_bulk);
		// motor1_state = motor1_state & 3;
		// motor2_state = motor2_state & 3;
		// assume the current state is the same as the previous state (no movement)
		motor1_curr_state = motor1_prev_state;
		motor2_curr_state = motor2_prev_state;
		// printf("\n");
		for (int i_line = 0; i_line < num_events_waited; i_line++)
		{
			struct gpiod_line *line_handle =
				gpiod_line_bulk_get_line(&event_bulk, i_line);
			unsigned int this_line_number = gpiod_line_offset(line_handle);
			event_status = gpiod_line_event_read(line_handle, &event);
			int line_index = this->line_number_to_index[this_line_number];
			// printf("line index = %d, event type = %d\n", line_index, event.event_type);
			if (!event_status) // no error occured
			{
				// Just testing if the events comes in chronological order...
				// Turned out its not!
				// if (event.ts.tv_sec > prev_time.tv_sec ||
				// 	(event.ts.tv_sec == prev_time.tv_sec && event.ts.tv_nsec > prev_time.tv_nsec))
				// {
				// 	prev_time = event.ts;
				// 	printf("This is in chronological order!!!\n");
				// }
				// else
				// {
				// 	prev_time = event.ts;
				// 	printf("This is not in chronological order!!!\n");
				// }
				// printf("Motor1 curr state = %u\n", motor1_curr_state);
				// printf("Motor2 curr state = %u\n", motor2_curr_state);
				if (line_index == 0)
				{
					// we are looking at motor1 (a)
					this->disp1++;
				}
				else if (line_index == 1)
				{
					// we are looking at motor1 (b)
					this->disp1++;
				}
				else if (line_index == 2)
				{
					// we are looking at motor2 (a)
					this->disp2++;
				}
				else if (line_index == 3)
				{
					// we are looking at motor2 (b)
					this->disp2++;
				}
				// for debugging...
				/*
				if (event_detected[line_index])
				{
					printf("Two events detected from the same line in one iteration!\n");
				}
				*/
				// event_detected[line_index] = 1;
				// event_types[line_index] = event.event_type;
				// event_ts[line_index] = event.ts;
			}
		}
		// Do we get event from two channels at the same time?
		// Turns out we do! (not _very_ often though)
		/*
		if ((event_detected[0] && event_detected[1]) || (event_detected[2] && event_detected[3]))
		{
			printf("Yes, we do get two events at the same time!\n");
		}
		*/
	}
}

void GPIO_Encoder_Listener::start_listening_to_edges()
{
	this->disp1 = this->disp2 = 0;
	this->continue_listening = true;
	this->edge_listening_thread = std::thread { &GPIO_Encoder_Listener::listen, this };
}

void GPIO_Encoder_Listener::stop_listening()
{
	this->continue_listening = false;
	this->edge_listening_thread.join();
}

void GPIO_Encoder_Listener::get_displacements(int *disp1, int *disp2)
{
	this->disp_mutex.lock();
	*disp1 = this->disp1;
	*disp2 = this->disp2;
	this->disp1 = 0;
	this->disp2 = 0;
	this->disp_mutex.unlock();
}
