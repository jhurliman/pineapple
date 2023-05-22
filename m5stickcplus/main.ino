/**
 * Firmware for the M5StickC+ LED light strip controller on the Pineapple
 * Express art bike.
 */

#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#include <M5StickCPlus.h>
#include <esp_gap_ble_api.h>

// Enable ESP32 hardware SPI
#define FASTLED_ALL_PINS_HARDWARE_SPI 1
#define FASTLED_ESP32_SPI_BUS HSPI
// Fix flickering by disabling interrupts during FastLED operations
#define FASTLED_ALLOW_INTERRUPTS 0

#include <FastLED.h>
#include <SPI.h>

// NimBLE is a modern successor to bluedroid
// #define USE_NIMBLE 1
// #include "BleKeyboard.h"

#define LED_TYPE NEOPIXEL
#define LED_PIN 26
#define NUM_LEDS 300
#define COLOR_ORDER RGB
#define POWER_LIMIT_MW 90000

constexpr int IMU_WINDOW_SIZE = 300;  // Moving average window size

// #define MAX_RSSI -30
// #define MIN_RSSI -100

// State variables
CRGB leds[NUM_LEDS];
// BleKeyboard bleKeyboard{"Pineapple", "Cat Money Records", 100};
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
// uint8_t connectionScore = 0;  // [0, 100]
// float movementScore = 0.0f;
// bool isInRange = false;
// bool isInMotion = false;
// int lastMotionTime = 0;

// // Convert a value in a given range to a score in the range [0, 100] (clamped)
// static int score(int value, int min, int max) {
//   int output = int(float(value - min) / float(max - min) * 100.0f);
//   return constrain(output, 0, 100);
// }

// void gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
//   if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT) {
//     // Update the connection score and range boolean from the RSSI
//     int rssi = param->read_rssi_cmpl.rssi;
//     connectionScore = uint8_t(score(rssi, MIN_RSSI, MAX_RSSI));
//     isInRange = connectionScore >= 25;
//   } else if (event == ESP_GAP_BLE_SEC_REQ_EVT) {
//     // Grant security request access. This pairing seems to make the connection more stable
//     esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
//   } else if (event == ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT) {
//     // The following events were observed during pairing in this order
//     Serial.println("ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
//   } else if (event == ESP_GAP_BLE_AUTH_CMPL_EVT) {
//     Serial.println("ESP_GAP_BLE_AUTH_CMPL_EVT");
//   } else if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT) {
//     Serial.printf("ESP_GAP_BLE_ADV_START_COMPLETE_EVT status=%d\n",
//     param->adv_start_cmpl.status);
//   } else if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
//     Serial.printf("ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT status=%d\n",
//     param->adv_data_cmpl.status);
//   } else {
//     Serial.printf("Unhandled GAP event %d\n", event);
//   }
// }

// //////////////////////////////////////////////////////////////////////////
// // <https://www.youtube.com/watch?v=FgUqHP8ZkTc>
// void bouncyballs_loop() {
//   constexpr float GRAVITY = -1.0f;  // Downward (negative) acceleration of gravity in m/s^2
//   constexpr float h0 = 3.0f;        // Starting height, in meters, of the ball (strip length)
//   constexpr int NUM_BALLS = 6;      // Number of bouncing balls

//   static float h[NUM_BALLS];                        // An array of heights
//   static float vImpact0 = sqrt(-2 * GRAVITY * h0);  // Impact velocity of the ball when it hits
//   the
//                                                     // ground if "dropped" from the top of the
//                                                     strip
//   static float vImpact[NUM_BALLS];  // As time goes on the impact velocity will change, so make
//   an
//                                     // array to store those values
//   static float tCycle[NUM_BALLS];   // The time since the last time the ball struck the ground
//   static int pos[NUM_BALLS];        // The integer position of the dot on the strip (LED index)
//   static long tLast[NUM_BALLS];     // The clock time of the last ground strike
//   static float COR[NUM_BALLS];      // Coefficient of Restitution (bounce damping)
//   static CRGB palette[NUM_BALLS];   // Color palette for each ball
//   static bool initialized = false;  // A flag to indicate that setup() has run successfully

//   if (!initialized) {
//     for (int i = 0; i < NUM_BALLS; i++) {  // Initialize variables
//       tLast[i] = millis();
//       h[i] = h0;
//       pos[i] = 0;             // Balls start on the ground
//       vImpact[i] = vImpact0;  // And "pop" up at vImpact0
//       tCycle[i] = 0;
//       COR[i] = 0.90 - float(i) / pow(NUM_BALLS, 2);
//       palette[i] = CHSV(uint8_t(i * 40), 255, 255);
//     }
//     initialized = true;
//   }

//   // <https://www.color-hex.com/color-palette/10221>
//   palette[0] = CRGB(255, 113, 206);
//   palette[1] = CRGB(1, 205, 254);
//   palette[2] = CRGB(5, 255, 161);
//   palette[3] = CRGB(185, 103, 255);
//   palette[4] = CRGB(255, 251, 150);

//   // Initialize ball positions to black
//   for (int i = 0; i < NUM_BALLS; i++) {
//     leds[pos[i]] = CRGB::Black;
//   }

//   for (int i = 0; i < NUM_BALLS; i++) {
//     tCycle[i] =
//         millis() - tLast[i];  // Calculate the time since the last time the ball was on the
//         ground

//     // A little kinematics equation calculates positon as a function of time, acceleration
//     (gravity)
//     // and intial velocity
//     h[i] = 0.5 * GRAVITY * pow(tCycle[i] / 1000, 2.0) + vImpact[i] * tCycle[i] / 1000;

//     if (h[i] < 0) {
//       h[i] = 0;  // If the ball crossed the threshold of the "ground," put it back on the ground
//       vImpact[i] =
//           COR[i] *
//           vImpact[i];  // and recalculate its new upward velocity as it's old velocity * COR
//       tLast[i] = millis();

//       if (vImpact[i] < 0.01)
//         vImpact[i] = vImpact0;  // If the ball is barely moving, "pop" it back up at vImpact0
//     }
//     pos[i] = round(h[i] * (NUM_LEDS - 1) /
//                    h0);  // Map "h" to a "pos" integer index position on the LED strip
//   }

//   // Choose color of LEDs, then the "pos" LED on
//   for (int i = 0; i < NUM_BALLS; i++) {
//     leds[pos[i]] = palette[i];
//   }
// }

// //////////////////////////////////////////////////////////////////////////
// //
// void Fire2012WithPalette() {
//   // COOLING: How much does the air cool as it rises?
//   // Less cooling = taller flames.  More cooling = shorter flames.
//   constexpr uint8_t COOLING = 800;
//   // SPARKING: What chance (out of 255) is there that a new spark will be lit?
//   // Higher chance = more roaring fire. Lower chance = more flickery fire.
//   // Default 120, suggested range 50-200
//   constexpr uint8_t SPARKING = 50;

//   // Array of temperature readings at each simulation cell
//   static uint8_t heat[NUM_LEDS];

//   // Step 1.  Cool down every cell a little
//   for (int i = 0; i < NUM_LEDS; i++) {
//     heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
//   }

//   // Step 2.  Heat from each cell drifts 'up' and diffuses a little
//   for (int k = NUM_LEDS - 1; k >= 2; k--) {
//     heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
//   }

//   // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
//   if (random8() < SPARKING) {
//     int y = random8(7);
//     heat[y] = qadd8(heat[y], random8(160, 255));
//   }

//   // Step 4.  Map from heat cells to LED colors
//   const auto& palette = ForestColors_p;
//   for (int j = 0; j < NUM_LEDS; j++) {
//     // Scale the heat value from 0-255 down to 0-240
//     // for best results with color palettes.
//     uint8_t colorindex = scale8(heat[j], 240);
//     CRGB color = ColorFromPalette(palette, colorindex);
//     int pixelnumber;
//     pixelnumber = j;
//     leds[pixelnumber] = color;
//   }
// }

// //////////////////////////////////////////////////////////////////////////
// //
// // In this animation, there are four "layers" of waves of light.
// //
// // Each layer moves independently, and each is scaled separately.
// //
// // All four wave layers are added together on top of each other, and then
// // another filter is applied that adds "whitecaps" of brightness where the
// // waves line up with each other more.  Finally, another pass is taken
// // over the led array to 'deepen' (dim) the blues and greens.
// //
// // The speed and scale and motion each layer varies slowly within independent
// // hand-chosen ranges, which is why the code has a lot of low-speed 'beatsin8' functions
// // with a lot of oddly specific numeric ranges.
// //
// CRGBPalette16 pacifica_palette_1 = {0x000705, 0x000904, 0x000B03, 0x000D03, 0x001002, 0x001202,
//                                     0x001401, 0x001701, 0x001900, 0x001C00, 0x002600, 0x003100,
//                                     0x003B00, 0x004600, 0x14554B, 0x28AA50};

// CRGBPalette16 pacifica_palette_2 = {0x000705, 0x000904, 0x000B03, 0x000D03, 0x001002, 0x001202,
//                                     0x001401, 0x001701, 0x001900, 0x001C00, 0x002600, 0x003100,
//                                     0x003B00, 0x004600, 0x0C5F52, 0x19BE5F};

// CRGBPalette16 pacifica_palette_3 = {0x000802, 0x000E03, 0x001405, 0x001A06, 0x002008, 0x002709,
//                                     0x002D0B, 0x00330C, 0x00390E, 0x004010, 0x005014, 0x006018,
//                                     0x00701C, 0x008020, 0x10BF40, 0x20FF60};

// // Add one layer of waves into the led array
// void pacifica_one_layer(CRGBPalette16& p, uint16_t cistart, uint16_t wavescale, uint8_t bri,
//                         uint16_t ioff, CRGB* leds, int numLeds) {
//   uint16_t ci = cistart;
//   uint16_t waveangle = ioff;
//   uint16_t wavescale_half = (wavescale / 2) + 20;
//   for (uint16_t i = 0; i < numLeds; i++) {
//     waveangle += 250;
//     uint16_t s16 = sin16(waveangle) + 32768;
//     uint16_t cs = scale16(s16, wavescale_half) + wavescale_half;
//     ci += cs;
//     uint16_t sindex16 = sin16(ci) + 32768;
//     uint8_t sindex8 = scale16(sindex16, 240);
//     CRGB c = ColorFromPalette(p, sindex8, bri, LINEARBLEND);
//     leds[i] += c;
//   }
// }

// // Add extra 'white' to areas where the four layers of light have lined up brightly
// void pacifica_add_whitecaps(CRGB* leds, int numLeds) {
//   uint8_t basethreshold = beatsin8(9, 55, 65);
//   uint8_t wave = beat8(7);

//   for (uint16_t i = 0; i < numLeds; i++) {
//     uint8_t threshold = scale8(sin8(wave), 20) + basethreshold;
//     wave += 7;
//     uint8_t l = leds[i].getAverageLight();
//     if (l > threshold) {
//       uint8_t overage = l - threshold;
//       uint8_t overage2 = qadd8(overage, overage);
//       leds[i] += CRGB(overage, overage2, qadd8(overage2, overage2));
//     }
//   }
// }

// // Deepen the blues and greens
// void pacifica_deepen_colors(CRGB* leds, int numLeds) {
//   for (uint16_t i = 0; i < numLeds; i++) {
//     leds[i].blue = scale8(leds[i].blue, 145);
//     // leds[i].green = scale8(leds[i].green, 200);
//     leds[i] |= CRGB(2, 7, 2);
//   }
// }

// void pacifica_loop(CRGB* leds, int numLeds) {
//   // Increment the four "color index start" counters, one for each wave layer.
//   // Each is incremented at a different speed, and the speeds vary over time.
//   static uint16_t sCIStart1, sCIStart2, sCIStart3, sCIStart4;
//   static uint32_t sLastms = 0;
//   uint32_t ms = GET_MILLIS();
//   uint32_t deltams = ms - sLastms;
//   sLastms = ms;
//   uint16_t speedfactor1 = beatsin16(3, 179, 269);
//   uint16_t speedfactor2 = beatsin16(4, 179, 269);
//   uint32_t deltams1 = (deltams * speedfactor1) / 256;
//   uint32_t deltams2 = (deltams * speedfactor2) / 256;
//   uint32_t deltams21 = (deltams1 + deltams2) / 2;
//   sCIStart1 += (deltams1 * beatsin88(1011, 10, 13));
//   sCIStart2 -= (deltams21 * beatsin88(777, 8, 11));
//   sCIStart3 -= (deltams1 * beatsin88(501, 5, 7));
//   sCIStart4 -= (deltams2 * beatsin88(257, 4, 6));

//   // Clear out the LED array to a dim background blue-green
//   fill_solid(leds, numLeds, CRGB(2, 6, 10));

//   // Render each of four layers, with different scales and speeds, that vary over time
//   pacifica_one_layer(pacifica_palette_1, sCIStart1, beatsin16(3, 11 * 256, 14 * 256),
//                      beatsin8(10, 70, 130), 0 - beat16(301), leds, numLeds);
//   pacifica_one_layer(pacifica_palette_2, sCIStart2, beatsin16(4, 6 * 256, 9 * 256),
//                      beatsin8(17, 40, 80), beat16(401), leds, numLeds);
//   pacifica_one_layer(pacifica_palette_3, sCIStart3, 6 * 256, beatsin8(9, 10, 38), 0 -
//   beat16(503),
//                      leds, numLeds);
//   pacifica_one_layer(pacifica_palette_3, sCIStart4, 5 * 256, beatsin8(8, 10, 28), beat16(601),
//   leds,
//                      numLeds);

//   // Add brighter 'whitecaps' where the waves lines up more
//   pacifica_add_whitecaps(leds, numLeds);

//   // Deepen the blues and greens a bit
//   pacifica_deepen_colors(leds, numLeds);
// }

void renderPacmanGame(CRGB* leds) {
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
  CRGB pelletSmallColor = CRGB(
    qadd8(8, fmin(1.0f, fabs(xAccel - xAvgAccel) / MAX_ACCEL) * 64.0f),
    qadd8(8, fmin(1.0f, fabs(yAccel - xAvgAccel) / MAX_ACCEL) * 64.0f),
    qadd8(8, fmin(1.0f, fabs(zAccel - xAvgAccel) / MAX_ACCEL) * 64.0f)
  );

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

    renderPacmanGame(leds);

    FastLED.setBrightness(255);
    FastLED.show();
    vTaskDelay(100 / portTICK_RATE_MS);

    // Decide animation based on motion status
    // if (isInMotion) {
    //   if (isInRange) {
    //     bouncyballs_loop();
    //     FastLED.setBrightness(255);
    //     FastLED.show();
    //   } else {
    //     // Ocean animation
    //     pacifica_loop(leds, NUM_LEDS);
    //     FastLED.setBrightness(255);
    //     FastLED.show();
    //     // FastLED.delay(20);
    //     vTaskDelay(20 / portTICK_RATE_MS);
    //   }
    // } else {
    //   if (isInRange) {
    //     Fire2012WithPalette();

    //     // Scale brightness based on connection score (RSSI)
    //     uint8_t brightness = scale8(connectionScore, 650);
    //     FastLED.setBrightness(brightness);
    //     FastLED.show();
    //     FastLED.delay(16); // Using FastLED.delay() for temporal dithering
    //   } else {
    //     // Blink pink every 4 seconds
    //     for (int i = 0; i < NUM_LEDS; i++) {
    //       leds[i] = CRGB(255, 105, 180);
    //     }
    //     FastLED.setBrightness(20);
    //     FastLED.show();
    //     // Using the more efficient vTaskDelay()
    //     vTaskDelay(100 / portTICK_RATE_MS);

    //     for (int i = 0; i < NUM_LEDS; i++) {
    //       leds[i] = CRGB::Black;
    //     }
    //     FastLED.show();
    //     vTaskDelay(3900 / portTICK_RATE_MS);
    //   }
    // }
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

//   constexpr int samplesCount = 50;  // 50 samples/second * 1 seconds
//   constexpr float accelDiffThreshold = 0.35f;

//   static float samples[samplesCount];
//   static int sampleIndex = 0;
//   static float prevMagnitude = 0.0f;

//   M5.IMU.Init();

//   while (true) {
//     float accX, accY, accZ;
//     M5.IMU.getAccelData(&accX, &accY, &accZ);

//     // Compute the magnitude of the acceleration vector
//     float magnitude = sqrt(accX * accX + accY * accY + accZ * accZ);
//     float absDiff = abs(magnitude - prevMagnitude);
//     prevMagnitude = magnitude;

//     // Add the new sample to the array and update the index
//     samples[sampleIndex] = absDiff;
//     sampleIndex = (sampleIndex + 1) % samplesCount;

//     // Calculate the moving average of absolute differences
//     float sum = 0;
//     for (int i = 0; i < samplesCount; i++) {
//       sum += samples[i];
//     }
//     movementScore = sum / samplesCount;

//     // Threshold test to determine if the device is in motion
//     if (movementScore > accelDiffThreshold) {
//       isInMotion = true;
//       lastMotionTime = millis();
//     } else if (millis() - lastMotionTime > 10000) {
//       isInMotion = false;
//     }

//     // Sleep for 20ms (50Hz)
//     vTaskDelay(20 / portTICK_RATE_MS);
//   }
// }

void setup() {
  // Init the M5StickC+ LCD, power system, etc
  M5.begin();
  M5.Axp.ScreenBreath(12);  // Maximum brightness

  // Start the BLE keyboard service
  // bleKeyboard.begin();

  // Register GAP callback. Must be called after `BLEDevice::init()` which is called by
  // `bleKeyboard.begin()`
  // esp_ble_gap_re gister_callback(gap_callback);

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

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);

  // if (bleKeyboard.isConnected()) {
  //   M5.Lcd.printf("Connected: %d    \n", connectionScore);
  //   // Initiate a read RSSI request for the remote host
  //   esp_ble_gap_read_rssi(bleKeyboard.remoteAddress);
  // } else {
  //   M5.Lcd.println("Not connected    ");
  //   // Reset connection state
  //   connectionScore = 0;
  //   isInRange = false;
  // }

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
