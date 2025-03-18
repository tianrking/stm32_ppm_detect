#include "ppm.h"
//extern volatile int filtered_channel_values[PPM_CHANNELS];
#define CHANNEL_MIN_PULSE_WIDTH  500   // Minimum pulse width 0.5ms
#define CHANNEL_MAX_PULSE_WIDTH  1500  // Maximum pulse width 1.5ms
#define FILTER_ALPHA  0.8             // Filter coefficient between 0 and 1
#define o_MAX 2000
#define o_MIN 1000
extern uint16_t    PPM_Databuf[8];
extern volatile int channel_values[8];
volatile int filtered_channel_values[8]; // Filtered channel values
	
static int map_value(int value, int in_min, int in_max, int out_min, int out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Apply filter to values
static int apply_filter(int new_value, int old_value) {
    return old_value + FILTER_ALPHA * (new_value - old_value);
}
int last_7,last_7_error;
// Process channel values in TIM5 interrupt handler
void process_channel_values() {
    for (int i = 0; i < 8 ; i++) {
			
		if(i == 7)
		{
			if(last_7 ==PPM_Databuf[7])
				continue;
			else
			{
				last_7 = PPM_Databuf[7];
				last_7_error++;
			}
		}
			
        // Map values to range from -100 to 100
        int mapped_value = map_value(PPM_Databuf[i], 
                                     o_MIN,
                                     o_MAX, 
                                     -100, 100);
        
        // Apply filter to smooth values
        filtered_channel_values[i] = apply_filter(mapped_value, filtered_channel_values[i]);
			
    }
    // Values are now available in filtered_channel_values[]
}
