/**
 * ESP32 PPM Signal Parser
 * Decodes RC PPM signals on a specified input pin
 */
 
#define PPM_INPUT_PIN D7     // GPIO pin connected to PPM signal
#define PPM_CHANNELS 8       // Number of channels in PPM signal
#define MIN_SYNC_GAP 3000    // Minimum sync pulse width (microseconds)
#define CHANNEL_MIN 1000     // Minimum valid channel value (microseconds)
#define CHANNEL_MAX 2000     // Maximum valid channel value (microseconds)

volatile uint32_t ppm_rising_edge = 0;
volatile uint32_t ppm_pulse_width = 0;
volatile uint16_t ppm_values[PPM_CHANNELS] = {0};
volatile uint8_t ppm_channel_index = 0;
volatile bool ppm_frame_complete = false;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/**
 * Interrupt handler for PPM signal edge detection
 */
void IRAM_ATTR ppmInterruptHandler() {
  uint32_t current_time = micros();
  
  // Calculate pulse width since last interrupt
  ppm_pulse_width = current_time - ppm_rising_edge;
  ppm_rising_edge = current_time;
  
  // Check if this is a sync pulse (larger gap between channels)
  if (ppm_pulse_width > MIN_SYNC_GAP) {
    ppm_channel_index = 0; // Reset channel counter for next frame
  } 
  // Ensure we have a valid pulse and haven't exceeded channel count
  else if (ppm_channel_index < PPM_CHANNELS) {
    // Store the pulse width for this channel
    ppm_values[ppm_channel_index] = ppm_pulse_width;
    ppm_channel_index++;
    
    // Check if we've received all channels
    if (ppm_channel_index >= PPM_CHANNELS) {
      ppm_frame_complete = true;
    }
  }
}

/**
 * Map PPM raw values to a specified output range
 */
int mapPpmToRange(uint16_t value, int outputMin, int outputMax) {
  return map(constrain(value, CHANNEL_MIN, CHANNEL_MAX), CHANNEL_MIN, CHANNEL_MAX, outputMin, outputMax);
}

void setup() {
  Serial.begin(115200);
  
  // Attach interrupt to PPM input pin
  pinMode(PPM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterruptHandler, RISING);
  
  Serial.println("PPM Signal Parser Initialized");
}

void loop() {
  if (ppm_frame_complete) {
    portENTER_CRITICAL(&timerMux);
    ppm_frame_complete = false;
    portEXIT_CRITICAL(&timerMux);
    
    // Print all channel values
    Serial.println("PPM Frame:");
    for (int i = 0; i < PPM_CHANNELS; i++) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(ppm_values[i]);
      Serial.print(" μs (");
      Serial.print(mapPpmToRange(ppm_values[i], 0, 100));
      Serial.println("%)");
    }
    Serial.println("---------------------");
    
    delay(100); // Small delay to make the output readable
  }
}