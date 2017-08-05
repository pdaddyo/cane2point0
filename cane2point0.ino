#include "FastLED.h"
FASTLED_USING_NAMESPACE
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>
#include "SerialFlash.h"

NXPMotionSense imu;
NXPSensorFusion filter;

#define DATA_PIN    11
#define CLK_PIN   13
#define LED_TYPE    APA102
#define COLOR_ORDER BGR
#define NUM_LEDS    72
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          15
#define FRAMES_PER_SECOND  1000

void setup() {
  // usb debug
  Serial.begin(115200);

  delay(500); // startup pause
  Serial.println("setup()");

  // bluetooth
  Serial2.begin(9600);

  // enable memory module
  SerialFlash.begin(6);
  //attempt to load bmp from prop shield flash
  loadBmp("blythe.bmp");

  // motion sensor
  imu.begin();
  filter.begin(100);

  // enable prop shield LED support
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, CLK_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { pov, bpmPulsing, rainbow, rainbowWithGlitter, confetti, sinelon, juggle };

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

float lastAngle = 0;
float lastAngleChange = 0;
unsigned long lastAngleTime = 0;
unsigned long lastAngleDuration = 50;

void loop()
{

  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);


  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);


    unsigned long currentTime = micros();
    lastAngleDuration = currentTime - lastAngleTime;
    lastAngleTime = currentTime;
    lastAngleChange = mz - lastAngle;
    lastAngle = mz;

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    pitchChange = pitch - lastPitch;
    if (lastPitchChange < 0 && pitchChange > 0  && abs(pitchChange) > 0.08) {
      //  beat();
      //  Serial.print("up pitch = ");
      //  Serial.println(pitch);
    }

    if (lastPitchChange > 0 && pitchChange < 0 && abs(pitchChange) > 0.08) {
      // Serial.println("BEAT");
      // Serial.println(pitchChange);
    }
    lastPitchChange = pitchChange;
    lastPitch = pitch;

    gHue = abs((int)pitch * 8);
  }

  EVERY_N_SECONDS( 3 ) {
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
  }

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) {
    if (Serial2.available()) {
      Serial.println("serial2 data!");
      blueToothIncomingString = Serial2.readString();
      // send a byte to the software serial port
      Serial.println(blueToothIncomingString);
      // if we hear anything nudge to next pattern - todo: proper control
      nextPattern();
    }

    //gHue++;  // slowly cycle the "base color" through the rainbow
  }

  EVERY_N_SECONDS( 10 ) {
    // nextPattern();  // change patterns periodically
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

/* mz - from 15 to -25 */
float povFromAngle = 40;
float povToAngle = 0;

void pov() {
  float angle = mz + 25;
  
  if (angle > povFromAngle || angle < povToAngle) {
    // nothing to see here - clear pixels
    black();
  } else {
    float totalSweep = povFromAngle - povToAngle;
    float timeSinceLastAngle = micros() - lastAngleTime;
    float ratioToNextAngleReading = timeSinceLastAngle / lastAngleDuration;
    float interpolatedAngle = angle + (ratioToNextAngleReading * lastAngleChange);
/*
    Serial.print("angle, inter, ratio = ");
    Serial.print(angle);
    Serial.print(", ");
    Serial.print(interpolatedAngle);
    Serial.print(", ");
    Serial.println(ratioToNextAngleReading);
*/
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
  for (int led = 0; led < NUM_LEDS; led++) {
    leds[led] = CRGB::Black;
  }
}

void povBeatDeprecated() {
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
    showPovSlice((ratioComplete * bmpWidth));
  }
}

void showPovSlice(int slice) {
  for (int led = 0; led < NUM_LEDS; led++) {
    leds[led] = getPix(led, slice);
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

void loadBmp(const char *filename) {
  SerialFlashFile file;
  Serial.println("Waiting for serialFlash");

  while (SerialFlash.ready() == false) {
    delay(10);
  }

  Serial.println("serialFlash ready..");


  file = SerialFlash.open(filename);
  if (file) {
    Serial.print("opened file ");
    Serial.print(filename);
    Serial.print(" file size: ");
    //  uint32_t address = file.getFlashAddress();
    uint32_t fileSize = file.size();
    Serial.println(fileSize);

    // 24 bit bmps only please!

    bmpData = (char*) malloc(fileSize);
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
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
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

