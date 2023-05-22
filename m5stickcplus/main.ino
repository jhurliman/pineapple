/**
 * Firmware for the M5StickC+ LED light strip (WS2812b) controller on the
 * Pineapple Express art bike.
 */

// Enable ESP32 hardware SPI
#define FASTLED_ALL_PINS_HARDWARE_SPI 1
#define FASTLED_ESP32_SPI_BUS HSPI
// Fix flickering by disabling interrupts during FastLED operations
#define FASTLED_ALLOW_INTERRUPTS 0

#include <FastLED.h>
#include <M5StickCPlus.h>
#include <SPI.h>

#define LED_TYPE NEOPIXEL
#define LED_PIN 26
#define NUM_LEDS 300
#define COLOR_ORDER RGB
#define POWER_LIMIT_MW 90000

constexpr int IMU_WINDOW_SIZE = 300;  // Moving average window size

// Global state variables
CRGB leds[NUM_LEDS];
float xAccel = 0.0f;
float yAccel = 0.0f;
float zAccel = 0.0f;
float xAvgAccel = 0.0f;
float yAvgAccel = 0.0f;
float zAvgAccel = 0.0f;
float accelReadings[3][IMU_WINDOW_SIZE];
float accelSums[3] = {0, 0, 0};
int imuWindowIndex = 0;
int lowPowerLedIndex = 0;
float vinVoltage = 0.0f;

void render_pacman_game(CRGB* leds) {
  constexpr uint8_t NUM_GHOSTS = 4;
  constexpr uint8_t PELLET_SPACING = 10;
  const CRGB GHOST_COLORS[NUM_GHOSTS] = {0xFF0000, 0xFFB8FF, 0x00FFFF, 0xFFB852};
  const CRGB PACMAN_COLOR = 0xFFFF00;
  const CRGB GHOSTLY_COLOR = 0x0000FF;
  const CRGB PELLET_SMALL_COLOR = 0x080808;
  const CRGB PELLET_LARGE_COLOR = 0xFFFFFF;

  static int pacmanPosition = 0;
  static int pacmanDirection = 1;
  static int deadStartMs = 0;
  static int ghostPositions[NUM_GHOSTS] = {NUM_LEDS / 2 - NUM_LEDS / 4, NUM_LEDS / 2,
                                           NUM_LEDS / 2 + NUM_LEDS / 4, NUM_LEDS - 1};
  static int ghostDirections[NUM_GHOSTS] = {1, 1, -1, -1};
  static bool pelletStates[NUM_LEDS] = {false};
  static int pelletCount = NUM_LEDS;

  int now = millis();

  if (pelletCount == NUM_LEDS && pelletStates[0] == false) {
    // Initialize the pellets
    for (int i = 0; i < NUM_LEDS; i++) {
      pelletStates[i] = true;
    }
  }

  // Compute the small pellet color based on high frequency IMU reading
  constexpr float MAX_ACCEL = 3.0f;
  CRGB pelletSmallColor = CRGB(qadd8(8, fmin(1.0f, fabs(xAccel - xAvgAccel) / MAX_ACCEL) * 64.0f),
                               qadd8(8, fmin(1.0f, fabs(yAccel - xAvgAccel) / MAX_ACCEL) * 64.0f),
                               qadd8(8, fmin(1.0f, fabs(zAccel - xAvgAccel) / MAX_ACCEL) * 64.0f));

  // Draw pellets and background
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = pelletStates[i] ? (i % PELLET_SPACING == 0 ? PELLET_LARGE_COLOR : pelletSmallColor)
                              : CRGB::Black;
  }

  // Check if we're dead
  if (deadStartMs > 0) {
    if (now - deadStartMs >= 2000) {
      // Reset Pac-Man's state
      deadStartMs = 0;
      pacmanPosition = 0;
      pacmanDirection *= -1;
    }
    return;
  }

  // Ghosts
  for (int i = 0; i < NUM_GHOSTS; i++) {
    // 10% chance of changing direction
    if (random8(10) == 0) {
      ghostDirections[i] *= -1;
    }

    // Move and draw the ghosts
    ghostPositions[i] += ghostDirections[i];
    if (ghostPositions[i] <= 0 || ghostPositions[i] >= NUM_LEDS - 1) {
      ghostDirections[i] *= -1;
    }
    leds[ghostPositions[i]] = GHOST_COLORS[i];
  }

  // Pac-Man eats the curret pellet, if any
  if (pelletStates[pacmanPosition]) {
    pelletStates[pacmanPosition] = false;
    pelletCount--;
  }

  // Check for "win"
  if (pelletCount < NUM_LEDS / 4) {
    deadStartMs = now;

    // Reset the pellets
    for (int i = 0; i < NUM_LEDS; i++) {
      pelletStates[i] = true;
      pelletCount = NUM_LEDS;
    }

    return;
  }

  // Draw Pac-Man
  leds[pacmanPosition] = PACMAN_COLOR;

  // Move Pac-Man
  pacmanPosition += pacmanDirection;
  if (pacmanPosition >= NUM_LEDS) pacmanPosition = 0;
  if (pacmanPosition < 0) pacmanPosition = NUM_LEDS - 1;

  // Check for collisions with ghosts
  for (int i = 0; i < NUM_GHOSTS; i++) {
    if (ghostPositions[i] == pacmanPosition) {
      deadStartMs = now;
    }
  }
}

void run_lights_task(void* arg) {
  FastLED.addLeds<LED_TYPE, LED_PIN>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInMilliWatts(POWER_LIMIT_MW);

  // Init the NeoPixels library
  for (int p = 0; p < NUM_LEDS; p++) {
    leds[p] = CRGB(p % 256, (p + 86) % 256, (p + 172) % 256);
  }
  FastLED.show();
  vTaskDelay(200 / portTICK_RATE_MS);

  // Track the position of the power off warning light
  int powerOffWarningPosition = 0;

  while (true) {
    // Check if we're running on the M5Stick battery power (main power is off)
    while (vinVoltage < 3.0f) {
      // Just show one chasing red light
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      leds[powerOffWarningPosition] = CRGB::Red;
      powerOffWarningPosition = (powerOffWarningPosition + 1) % NUM_LEDS;
      FastLED.setBrightness(8);
      FastLED.show();
      vTaskDelay(50 / portTICK_RATE_MS);
    }

    render_pacman_game(leds);

    FastLED.setBrightness(255);
    FastLED.show();
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

void measure_imu_task(void* arg) {
  M5.IMU.Init();

  while (true) {
    // Measure the IMU
    M5.IMU.getAccelData(&xAccel, &yAccel, &zAccel);

    // Subtract the oldest sample and add the new one for each component
    accelSums[0] -= accelReadings[0][imuWindowIndex];
    accelSums[1] -= accelReadings[1][imuWindowIndex];
    accelSums[2] -= accelReadings[2][imuWindowIndex];

    // Store the new sample and update the sums
    accelSums[0] += accelReadings[0][imuWindowIndex] = xAccel;
    accelSums[1] += accelReadings[1][imuWindowIndex] = yAccel;
    accelSums[2] += accelReadings[2][imuWindowIndex] = zAccel;

    // Calculate the averages
    xAvgAccel = accelSums[0] / IMU_WINDOW_SIZE;
    yAvgAccel = accelSums[1] / IMU_WINDOW_SIZE;
    zAvgAccel = accelSums[2] / IMU_WINDOW_SIZE;

    // Increment the IMU window index with rollover
    imuWindowIndex = (imuWindowIndex + 1) % IMU_WINDOW_SIZE;

    // Read 5V input voltage (tells us if the LED strip is powered)
    vinVoltage = M5.Axp.GetVinVoltage();

    // Sleep for 50ms (20Hz)
    vTaskDelay(50 / portTICK_RATE_MS);
  }
}

void setup() {
  // Init the M5StickC+ LCD, power system, etc
  M5.begin();
  M5.Axp.ScreenBreath(12);  // Maximum brightness

  // Initial voltage reading
  vinVoltage = M5.Axp.GetVinVoltage();

  // Start the IMU measurement loop
  xTaskCreate(measure_imu_task, "measure_imu_task", 2048, nullptr, 1, nullptr);

  // Start the LED pixel strip controller
  xTaskCreate(run_lights_task, "run_lights_task", 2048, nullptr, 2, nullptr);
}

void loop() {
  // Check if the LED strip is powered
  if (vinVoltage < 3.0f) {
    // Turn off all LED output
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    while (vinVoltage < 3.0f) {
      // Flash the screen red every 5 seconds
      M5.Lcd.fillScreen(RED);
      delay(100);
      M5.Lcd.fillScreen(BLACK);
      delay(4900);
    }
  }

  // Print VIN voltage and IMU data to the screen every 500ms (2Hz)
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);

  M5.Lcd.printf("VinVoltage: %0.3f\n", vinVoltage);

  M5.Lcd.println("Accel:");
  M5.Lcd.setTextColor(RED);
  M5.Lcd.printf("  %0.4f\n", xAccel);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.printf("  %0.4f\n", yAccel);
  M5.Lcd.setTextColor(BLUE);
  M5.Lcd.printf("  %0.4f\n", zAccel);

  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.println("AvgAccel:");
  M5.Lcd.setTextColor(RED);
  M5.Lcd.printf("  %0.4f\n", xAvgAccel);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.printf("  %0.4f\n", yAvgAccel);
  M5.Lcd.setTextColor(BLUE);

  M5.Lcd.setTextColor(WHITE);
  delay(500);
}
