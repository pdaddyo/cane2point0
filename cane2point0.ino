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

#define BRIGHTNESS          3
#define FRAMES_PER_SECOND  400

void setup() {
  // usb debug
  Serial.begin(115200);
  delay(500); // startup pause

  SerialFlash.begin(6); // enable memory module
  //attempt to load bmp from prop shield flash
  loadBmp("boomtown.bmp");

  // bluetooth
  Serial2.begin(9600);

  Serial.println("setup()");
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
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

String blueToothIncomingString;
int currentSlice = 0;

void loop()
{
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  // Call the current pattern function once, updating the 'leds' array
  //  gPatterns[gCurrentPatternNumber]();

  showPovSlice(currentSlice++);
  
  // send the 'leds' array out to the actual LED strip
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);


  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    gHue = abs((int)pitch * 2);
  }


  EVERY_N_SECONDS( 3 ) {
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
  }

  if (Serial2.available()) {
    Serial.println("serial2 data!");
    blueToothIncomingString = Serial2.readString();
    // send a byte to the software serial port
    Serial.println(blueToothIncomingString);
    nextPattern();
  }


  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) {
    //gHue++;  // slowly cycle the "base color" through the rainbow
  }
  EVERY_N_SECONDS( 10 ) {
    nextPattern();  // change patterns periodically
  }
}


char * bmpData = 0;
int bmpHeaderSize = 0;
int bmpWidth = 0;
int bmpHeight = 0;
int bmpPixelDataOffset = 0;
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


void showPovSlice(int slice) {
  for (int led = 0; led < NUM_LEDS; led++) {
    leds[led] = getPix(led, slice);
  }
}


CRGB getPix(int X, int Y) {
  uint16_t row = X % bmpHeight;
  uint16_t col = Y % bmpWidth;
  CRGB px;
  int offset = ((bmpPixelDataOffset + (row * row_bytes)) + (Y * 3));
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

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
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

