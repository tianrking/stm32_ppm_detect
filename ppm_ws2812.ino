/**
 * ESP32 PPM Signal Parser with FastLED WS2812 Control
 * Decodes RC PPM signals and controls WS2812 LEDs based on CH3 value using FastLED
 */
 
#include <Arduino.h>
#include <FastLED.h>

// PPM Configuration
#define PPM_INPUT_PIN D7     // GPIO pin connected to PPM signal
#define PPM_CHANNELS 8       // Number of channels in PPM signal
#define MIN_SYNC_GAP 3000    // Minimum sync pulse width (microseconds)
#define CHANNEL_MIN 1000     // Minimum valid channel value (microseconds)
#define CHANNEL_MAX 2000     // Maximum valid channel value (microseconds)

// FastLED Configuration
#define DATA_PIN D1          // WS2812 data pin
#define NUM_LEDS 40          // Total number of LEDs on the strip
#define BRIGHTNESS 10        // LED brightness (0-255)
#define SPEED 20             // Animation speed control, higher = slower

CRGB leds[NUM_LEDS];         // Define the array of leds

// PPM Variables
volatile uint32_t ppm_rising_edge = 0;
volatile uint32_t ppm_pulse_width = 0;
volatile uint16_t ppm_values[PPM_CHANNELS] = {0};
volatile uint8_t ppm_channel_index = 0;
volatile bool ppm_frame_complete = false;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// LED Control Variables
int current_led_count = -1;  // Initialize to -1 to ensure first update happens
uint8_t hue = 0;             // Used for color cycling

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

/**
 * Update WS2812 LEDs based on CH3 value
 */
void updateLEDs(uint16_t ch3_value) {
  // Map CH3 value (1000-2000) to number of LEDs (0-NUM_LEDS)
  int led_count = mapPpmToRange(ch3_value, 0, NUM_LEDS);
  
  // Only update the count if it has changed
  if (led_count != current_led_count) {
    current_led_count = led_count;
    Serial.print("LED Count: ");
    Serial.println(led_count);
  }
  
  // Update all LEDs with rainbow pattern
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < current_led_count) {
      // Create rainbow effect with color differences between LEDs
      leds[i] = CHSV(hue + (i * 255 / NUM_LEDS), 255, 255);
    } else {
      // Turn off LEDs beyond the current count
      leds[i] = CRGB::Black;
    }
  }
  
  // Slowly change the base color to create flowing effect
  EVERY_N_MILLISECONDS(SPEED) {
    hue++;  // Increase hue value every SPEED milliseconds
  }
  
  // Show the updated LED pattern
  FastLED.show();
}

void setup() {
  Serial.begin(115200);
  
  // Initialize FastLED
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();  // Start with all LEDs off
  FastLED.show();
  
  // Attach interrupt to PPM input pin
  pinMode(PPM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterruptHandler, RISING);
  
  Serial.println("PPM Signal Parser with FastLED WS2812 Control Initialized");
}

void loop() {
  if (ppm_frame_complete) {
    portENTER_CRITICAL(&timerMux);
    ppm_frame_complete = false;
    // Make a local copy of channel 3 value to use outside the critical section
    uint16_t ch3_value = ppm_values[2]; // Channel 3 is index 2 (zero-based)
    portEXIT_CRITICAL(&timerMux);
    
    // Print channel values every second for debugging
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime > 1000) {
      lastPrintTime = currentTime;
      
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
    }
  }
  
  // Update LEDs based on latest CH3 value
  // We do this outside the frame_complete check to keep animations smooth
  updateLEDs(ppm_values[2]);
  
  // Small delay to prevent overloading the CPU
  delay(5);
}
