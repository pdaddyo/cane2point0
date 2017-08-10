#include "FastLED.h"
FASTLED_USING_NAMESPACE
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>
#include "SerialFlash.h"
#include <U8g2lib.h>

NXPMotionSense imu;
NXPSensorFusion filter;

// text drawing with u8g2 library, set up a fake OLED screen so we can read framebuffer
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, 25, 26, 27, 16);
String povTextString = "Hi Boomtown!";

#define DATA_PIN    11
#define CLK_PIN   13
#define LED_TYPE    APA102
#define COLOR_ORDER BGR
#define NUM_LEDS    72
CRGB leds[NUM_LEDS];

int BRIGHTNESS = 15;
#define FRAMES_PER_SECOND  500

// List of patterns to cycle through.  Each is defined as a separate function below.

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

String blueToothIncomingString;
int currentSlice = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, heading;
float lastPitch = 0;
float pitchChange = 0;
float lastPitchChange = 0;

// bmp vars
char * bmpData = 0;
int bmpHeaderSize = 0;
int bmpWidth = 0;
int bmpHeight = 0;
int bmpPixelDataOffset = 0;

// multiple images
char fileNames[32][32];
int fileIndex = 0;
int numFiles = 0;
int eepromAddress = 0;

// interpolating the raw angle data
float angle = 0;
float lastAngle = 0;
float lastAngleChange = 0;
unsigned long lastAngleTime = 0;
unsigned long lastAngleDuration = 50;

// buttons
bool button1Down = false;
bool button2Down = false;

void setup() {
  // usb debug
  Serial.begin(9600);

  delay(500); // startup pause
  Serial.println("setup()");

  // init u8g2 for text
  u8g2.begin();
  u8g2.setFlipMode(0);
  u8g2.setAutoPageClear(0);
  u8g2.setFont(u8g2_font_helvB14_tf);

  // enable prop shield LED support
  pinMode(7, OUTPUT);

  // bluetooth
  Serial2.begin(9600);

  // enable memory module
  SerialFlash.begin(6);

  // load all filenames
  loadFileNames();

  // load which index we're meant to be loading from the eeprom
  byte value;
  value = EEPROM.read(eepromAddress);

  fileIndex = value % numFiles;
  Serial.print("loading the file at index ");
  Serial.println(fileIndex);
  Serial.println(fileNames[fileIndex]);

  //attempt to load bmp from prop shield flash
  loadBmp(fileNames[fileIndex]);

  // motion sensor
  imu.begin();
  filter.begin(100);

  // ENABLE led controller
  digitalWrite(7, HIGH);

  // button 1
  pinMode(22, INPUT_PULLDOWN);

  // button 2
  pinMode(20, INPUT_PULLDOWN);

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, CLK_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}


typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { pov3, Fire2012, povText, pov3Rainbow, Fire2012Reverse, beatFlash, angleRainbow, bpmPulsing, rainbow, rainbowWithGlitter, confetti, sinelon, juggle };

void loop()
{

  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);

  // Serial.println(mz);

  bool button1 = digitalRead(22);

  if (!button1Down) {
    if (button1) {
      nextPattern();
      button1Down = true;
    }
  } else {
    if (!button1) {
      button1Down = false;
    }
  }

  bool button2 = digitalRead(20);

  if (!button2Down) {
    if (button2) {
      prevPattern();
      button2Down = true;
    }
  } else {
    if (!button2) {
      button2Down = false;
    }
  }

  if (imu.available()) {
    lastAngle = mz;

    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    unsigned long currentTime = micros();
    lastAngleDuration = currentTime - lastAngleTime;
    lastAngleTime = currentTime;
    lastAngleChange = mz - lastAngle;
    angle = mz;

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    pitchChange = pitch - lastPitch;
    if (lastPitchChange < 0 && pitchChange > 0  && abs(pitchChange) > 0.08) {
      beat();
      //  Serial.print("up pitch = ");
      //  Serial.println(pitch);
    }

    if (lastPitchChange > 0 && pitchChange < 0 && abs(pitchChange) > 0.08) {

      // Serial.println("BEAT");
      // Serial.println(pitchChange);
    }
    lastPitchChange = pitchChange;
    lastPitch = pitch;

    // gHue = abs((int)pitch * 8);
  }

  EVERY_N_SECONDS( 3 ) {
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    Serial.print("mz = ");
    Serial.println(mz);
  }


  checkBlueToothSerialState();


  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) {
    gHue++;  // slowly cycle the "base color" through the rainbow
  }

  EVERY_N_SECONDS( 10 ) {
    // nextPattern();  // change patterns periodically
  }
}

void checkBlueToothSerialState() {
  if (Serial2.available()) {
    Serial.println("serial2 data:");
    blueToothIncomingString = Serial2.readString();

    if (blueToothIncomingString == "!B516!B507") {
      Serial.println("Up pushed!");
      // increase brightness
      BRIGHTNESS += 5;
      if (BRIGHTNESS > 128) {
        BRIGHTNESS = 128;
      }
      FastLED.setBrightness(BRIGHTNESS);
      return;
    }
    if (blueToothIncomingString == "!B615!B606") {
      Serial.println("Down pushed!");
      // reduce brightness
      BRIGHTNESS -= 5;
      if (BRIGHTNESS <= 0) {
        BRIGHTNESS = 1;
      }
      FastLED.setBrightness(BRIGHTNESS);
      return;
    }

    if (blueToothIncomingString == "!B813!B804") {
      // right arrow
      nextPattern();
      return;
    }
    if (blueToothIncomingString == "!B714!B705") {
      // left arrow
      prevPattern();
      return;
    }

    if (blueToothIncomingString == "!B11:!B10;") {
      // button 1 == next image
      fileIndex++;
      if (fileIndex >= numFiles) {
        fileIndex = 0;
      }

      EEPROM.write(eepromAddress, (byte)fileIndex);
      return;
    }

    // if we get here it's a random string, set text to it
    povTextString = blueToothIncomingString;
    // enter text mode
    gCurrentPatternNumber = 2;
    Serial.println(blueToothIncomingString);
  }
}



unsigned long lastBeatTime = 0;
unsigned long lastBeatDuration = 500000;

void beat() {
  unsigned long currentTime = micros();
  lastBeatDuration = currentTime - lastBeatTime;
  lastBeatTime = currentTime;
  Serial.print("BEAT! duration: ");
  Serial.println(lastBeatDuration);
}


// flash to the beat with the current hue
void beatFlash() {
  if ( micros() - lastBeatTime < 50000) {
    fill(CHSV( gHue, 255, 255));
  } else {
    black();
  }
}

// draw solid colour with hue based on mz (essentially raw pitch)
void angleRainbow() {
  fill(CHSV( abs(mz * 4), 255, 255));
}

/* mz - from -15 to 20 */
float povFromAngle = -15;
float povToAngle = 20;

bool waitingForForwardSwipe = true;
bool povRainbowMode = false;
bool povTextMode = true;

void pov3Internal(bool swipeBack) {
  if (waitingForForwardSwipe) {
    if (lastAngle < povFromAngle && angle >= povFromAngle) {
      Serial.println("GO FWD!");
      waitingForForwardSwipe = false;
      sweepThroughSlices(true);
      // we stalled the hue updating so push it forwards
      gHue += 20;
    }
  } else {
    if (lastAngle < povToAngle && angle >= povToAngle) {
      Serial.println("GO BACK!");
      waitingForForwardSwipe = true;
      if (swipeBack) {
        sweepThroughSlices(false);
      }
    }
  }
}

void pov3() {
  povRainbowMode = false;
  povTextMode = false;
  // pov2 was too jittery due to relying on absolute interpolated sensor data, skipping slices of the image etc
  // v3:
  // wait for cane to pass through trigger angle, measure velocity at that point and don't relinquish to main loop until we've drawn the perfect arc
  pov3Internal(false);
}

#define BIT(x,n) (((x) >> (n)) & 1)

void povText() {
  povTextMode = true;
  pov3Internal(false);
}

void beginPovText() {
  // draw it to buffer
  u8g2.clearBuffer();
  u8g2.drawStr(0, 20, povTextString.c_str());
}

void showPovTextSlice(int slice) {

  uint8_t *ptr;
  ptr = u8g2.getBufferPtr();
  uint8_t h = u8g2.getBufferTileHeight();
  uint8_t w = u8g2.getBufferTileWidth();

  int pixelX = 0;
  for (int y = 0; y < h * 8; y++) {
    pixelX = 0;
    for (int x = 0; x < w; x ++) {

      for (int b = 7; b >= 0; b--) {
        pixelX++;
        if (pixelX == slice) {
          if (BIT(((uint8_t) * (ptr + x + (w * y))), b) ) {
            leds[NUM_LEDS - y] = CHSV((gHue) % 255, 255, 255);
          } else {
            leds[NUM_LEDS - y] = CRGB::Black;
          }
        }
      }
    }
  }
}

void pov3Rainbow() {
  povRainbowMode = true;
  povTextMode = false;
  pov3Internal(false);
}

void sweepThroughSlices(bool forward) {
  float totalSweep = povFromAngle - povToAngle;
  float totalSweepTime = totalSweep / abs(lastAngleChange) * lastAngleDuration;
  float microsPerSlice = totalSweepTime / bmpWidth;
  if (totalSweepTime > 1000000) {
    totalSweepTime = 1000000;
  }
  float delayTimePerSlice = abs(microsPerSlice);
  Serial.print("delayTimePerSlice = ");
  Serial.println(delayTimePerSlice);
  if (povTextMode) {
    beginPovText();
  }

  if (forward) {
    for (int slice = 0; slice < bmpWidth; slice++) {
      if (povTextMode) {
        showPovTextSlice(slice);
      } else {
        showPovSlice(slice);
      }
      FastLED.show();
      delayMicroseconds(delayTimePerSlice);
    }
  } else {
    for (int slice = bmpWidth; slice > 0; slice--) {
      if (povTextMode) {
        showPovTextSlice(slice);
      } else {
        showPovSlice(slice);
      }
      FastLED.show();
      delayMicroseconds(delayTimePerSlice);
    }
  }
}
/*float povFromAngle = 40;
  float povToAngle = 0;
*/
void pov2Old() {
  float angle = mz + 25;

  if (angle > povFromAngle || angle < povToAngle) {
    // nothing to see here - clear pixels
    black();
  } else {
    float totalSweep = povFromAngle - povToAngle;
    float timeSinceLastAngle = micros() - lastAngleTime;
    float ratioToNextAngleReading = timeSinceLastAngle / lastAngleDuration;
    float interpolatedAngle = angle + (ratioToNextAngleReading * lastAngleChange);
    int sliceNumber = (totalSweep - interpolatedAngle) / totalSweep * bmpWidth;
    if (sliceNumber < 0 || sliceNumber >= bmpWidth) {
      // we're past the image, stop
      black();
      return;
    }
    // draw!
    showPovSlice( sliceNumber  );
  }
}

void black() {
  fill(CRGB::Black);
}

void fill(CRGB color) {
  for (int led = 0; led < NUM_LEDS; led++) {
    leds[led] = color;
  }
}

void povV1Old() {
  unsigned long timeSinceLastBeat = (micros() - lastBeatTime) % lastBeatDuration;
  float halfBeat = lastBeatDuration / 2;

  if (timeSinceLastBeat < halfBeat) {
    // first half: swipe up (backwards)
    float ratioComplete = timeSinceLastBeat / halfBeat;
    showPovSlice(bmpWidth - (ratioComplete * bmpWidth));
    /*for (int led = 0; led < NUM_LEDS; led++) {
      leds[led] = CRGB::Green;
      }*/
  } else {

    float ratioComplete = (timeSinceLastBeat / halfBeat) - 1.0;
    /*for (int led = 0; led < NUM_LEDS; led++) {
      leds[led] = CRGB::Red;
      }*/
    showPovSlice(ratioComplete * bmpWidth);
  }
}

// draw a vertical slice of the POV bitmap
void showPovSlice(int slice) {
  for (int led = 0; led < NUM_LEDS; led++) {
    if (povRainbowMode) {
      // basic mask - if it's not black, draw a moving rainbow
      if (!isPixBlack(led, slice) ) {
        leds[led] = CHSV((slice + gHue) % 255, 255, 255);
      }
      else {
        leds[led] = CRGB::Black;
      }

    } else {
      leds[led] = getPix(led, slice);
    }
  }
}


uint16_t row_bits, row_bytes;

long readLong(int offset) {
  long  l = bmpData[offset] | (bmpData[offset + 1] << 8) | (bmpData[offset + 2] << 16) | (bmpData[offset + 3] << 24);
  return l;
}


short readShort(int offset) {
  short  s = bmpData[offset] | (bmpData[offset + 1] << 8) ;
  return s;
}

void loadFileNames() {
  // load all filenames from the serialFlash
  //fileNames
  SerialFlash.opendir();
  fileIndex = 0;
  uint32_t filesize;
  while (SerialFlash.readdir(fileNames[numFiles], 32, filesize)) {
    Serial.print("found file: ");
    Serial.println(fileNames[numFiles]);
    numFiles++;
  }

  Serial.print("found files: ");
  Serial.println(numFiles);
}

void loadBmp(char *filename) {
  SerialFlashFile file;
  Serial.println("Waiting for serialFlash");

  while (SerialFlash.ready() == false) {
    delay(10);
  }

  Serial.print("serialFlash ready to load file ");
  Serial.println(filename);

  file = SerialFlash.open(filename);
  if (file) {
    Serial.print("opened file ");
    Serial.print(filename);
    Serial.print(" file size: ");
    uint32_t fileSize = file.size();
    Serial.println(fileSize);

    // 24 bit bmps only please!
    if (bmpData) {
      free(bmpData);
    }

    bmpData = (char*) malloc(fileSize);
    if (!bmpData) {
      Serial.println("failed to allocate memory for bmp file");
    }
    file.read(bmpData, fileSize);
    Serial.println("loaded file into memory.");
    bmpHeaderSize = readLong(0x0e);
    // check header for width and height
    bmpWidth = readShort(0x0e + 4);
    bmpHeight = readShort(0x0e + 8);
    bmpPixelDataOffset = readLong(0x0a);
    row_bits = (bmpWidth * 24 + 7) & ~7;
    // Calculate width in bytes (4-byte boundary aligned)
    row_bytes = (row_bits / 8 + 3) & ~3;

    Serial.print("bmpHeaderSize ");
    Serial.println(bmpHeaderSize);
    Serial.print("dimensions: ");
    Serial.print(bmpWidth);
    Serial.print("x");
    Serial.println(bmpHeight);
    Serial.print("pixel data offset is ");
    Serial.println(bmpPixelDataOffset);
    Serial.print("row_bytes ");
    Serial.println(row_bytes);
    Serial.print("first pixel: ");
    Serial.println(readLong(bmpPixelDataOffset), HEX);

  } else {
    Serial.println("failed to open file ");
  }
}

bool isPixBlack(int X, int Y) {
  uint16_t row = X % bmpHeight;
  uint16_t col = Y % bmpWidth;
  int offset = ((bmpPixelDataOffset + (row * row_bytes)) + (col * 3));
  return bmpData[offset] == 0 && bmpData[offset + 1] == 0 && bmpData[offset + 2] == 0;
}

CRGB getPix(int X, int Y) {
  uint16_t row = X % bmpHeight;
  uint16_t col = Y % bmpWidth;
  CRGB px;
  int offset = ((bmpPixelDataOffset + (row * row_bytes)) + (col * 3));
  px.blue = bmpData[offset];
  px.green = bmpData[offset + 1];
  px.red = bmpData[offset + 2];
  return px;
}



/// demo LED functions
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  fadeToBlackBy( leds, NUM_LEDS, 20);
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void prevPattern()
{
  fadeToBlackBy( leds, NUM_LEDS, 20);
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = abs((gCurrentPatternNumber - 1) % ARRAY_SIZE( gPatterns));
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS - 1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpmPulsing()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = (int)120;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for ( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++) {
    leds[beatsin16( i + 7, 0, NUM_LEDS - 1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

// Fire2012 by Mark Kriegsman, July 2012
// as part of "Five Elements" shown here: http://youtu.be/knWiGsmgycY
////
// This basic one-dimensional 'fire' simulation works roughly as follows:
// There's a underlying array of 'heat' cells, that model the temperature
// at each point along the line.  Every cycle through the simulation,
// four steps are performed:
//  1) All cells cool down a little bit, losing heat to the air
//  2) The heat from each cell drifts 'up' and diffuses a little
//  3) Sometimes randomly new 'sparks' of heat are added at the bottom
//  4) The heat from each cell is rendered as a color into the leds array
//     The heat-to-color mapping uses a black-body radiation approximation.
//
// Temperature is in arbitrary units from 0 (cold black) to 255 (white hot).
//
// This simulation scales it self a bit depending on NUM_LEDS; it should look
// "OK" on anywhere from 20 to 100 LEDs without too much tweaking.
//
// I recommend running this simulation at anywhere from 30-100 frames per second,
// meaning an interframe delay of about 10-35 milliseconds.
//
// Looks best on a high-density LED setup (60+ pixels/meter).
//
//
// There are two main parameters you can play with to control the look and
// feel of your fire: COOLING (used in step 1 above), and SPARKING (used
// in step 3 above).
//
// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 50, suggested range 20-100
#define COOLING  55

// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 70

bool gReverseDirection = false;

void Fire2012()
{
  // Array of temperature readings at each simulation cell
  static byte heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
  for ( int i = 0; i < NUM_LEDS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for ( int k = NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if ( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160, 255) );
  }

  // Step 4.  Map from heat cells to LED colors
  for ( int j = 0; j < NUM_LEDS; j++) {
    CRGB color = HeatColor( heat[j]);
    int pixelnumber;
    if ( false ) {
      pixelnumber = (NUM_LEDS - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
  }

  delay(4);
}

void Fire2012Reverse()
{
  // Array of temperature readings at each simulation cell
  static byte heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
  for ( int i = 0; i < NUM_LEDS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for ( int k = NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if ( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160, 255) );
  }

  // Step 4.  Map from heat cells to LED colors
  for ( int j = 0; j < NUM_LEDS; j++) {
    CRGB color = HeatColor( heat[j]);
    int pixelnumber;
    if ( true ) {
      pixelnumber = (NUM_LEDS - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
  }

  delay(4);
}


