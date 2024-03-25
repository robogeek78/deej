#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "icons12.h"
#include "icons16.h"

#define SCREEN_WIDTH (128)  // OLED display width, in pixels
#define SCREEN_HEIGHT (64)  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET (-1)        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS (0x3C)  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* This is the number of sliders across the top of the display */
#define NUM_SLIDERS (5)

/* This allows the sliders to control multiple layers of volumes */
#define NUM_OF_LAYERS (2)
const uint8_t analogInputs[NUM_SLIDERS] = { A0, A1, A2, A3, A6 };

/* This is the primary volume control that is not effected by layers */
const uint8_t analogInputPrimary = A7;

/* this is the button that switches between layers and resumes communication after 
 * a layer switch
 */
const uint8_t buttonInput = D8;

/* If you change this you need to make sure to change your icon slider definitions */
#define NUM_OF_LAYERS (2)

/* this can be anything from 1-6 to not overflow a uint16_t and 3 should be sufficient
 * if you feel the channels are sluggish reduce this number
 */
#define IIR_FILTER_SHIFT (1)

uint16_t analogSliderIIRAccumulator[NUM_OF_LAYERS][NUM_SLIDERS];
uint16_t analogSliderValues[NUM_OF_LAYERS][NUM_SLIDERS];
uint16_t analogPrimaryIIRAccumulator;
uint16_t analogPrimaryValue;

const uint8_t* iconSliderBmps[NUM_OF_LAYERS][NUM_SLIDERS] = {
  { bmp12_chrome, bmp12_halflife, bmp12_spot, bmp12_quest, bmp12_mic },
  { bmp12_0, bmp12_1, bmp12_2, bmp12_3, bmp12_mic },
};

uint8_t currentLayer = 0;
bool enableReads = true;
bool valueChangeFlag = false;

void setup() {
  for (uint8_t slider = 0; slider < NUM_SLIDERS; slider++) {
    pinMode(analogInputs[slider], INPUT);
    analogSliderIIRAccumulator[0][slider] = 0;
    analogSliderIIRAccumulator[1][slider] = 0;
  }
  pinMode(analogInputPrimary, INPUT);
  pinMode(buttonInput, INPUT_PULLUP);

  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.setRotation(3);
}

void loop() {
  updateSliderValues();
  updateButtonInput();
  sendSliderValues();  // Actually send data (all the time)
  displaySliderValuesPortraitPrimary();
  delay(10);
}

void updateSliderValues() {
  if (enableReads) {
    for (uint8_t slider = 0; slider < NUM_SLIDERS; slider++) {
      analogSliderIIRAccumulator[currentLayer][slider] -= analogSliderIIRAccumulator[currentLayer][slider] >> IIR_FILTER_SHIFT;
      analogSliderIIRAccumulator[currentLayer][slider] += (analogRead(analogInputs[slider]) & 0x03F8 );
      uint16_t tempNewValue = analogSliderIIRAccumulator[currentLayer][slider] >> IIR_FILTER_SHIFT;
      if (tempNewValue != analogSliderValues[currentLayer][slider]) {
        analogSliderValues[currentLayer][slider] = tempNewValue;
        valueChangeFlag = true;
      }
    }
  }

  analogPrimaryIIRAccumulator -= analogPrimaryIIRAccumulator >> IIR_FILTER_SHIFT;
  analogPrimaryIIRAccumulator += 0;//(analogRead(analogInputPrimary)  & 0x03FC );
  uint16_t tempNewValue = analogPrimaryIIRAccumulator >> IIR_FILTER_SHIFT;
  if (tempNewValue != analogPrimaryValue) {
    analogPrimaryValue = tempNewValue;
    valueChangeFlag = true;
  }
}

void sendSliderValues() {
  /*No need to build a string then send it, you can send while you are building it */
  if (valueChangeFlag) {
    valueChangeFlag = false;
    for (uint8_t layer = 0; layer < NUM_OF_LAYERS; layer++) {
      for (uint8_t slider = 0; slider < NUM_SLIDERS; slider++) {
        Serial.print(analogSliderValues[layer][slider]);
        Serial.print("|");
      }
    }
    Serial.println(analogPrimaryValue);
  }
}

#define BAR_WIDTH (8)
#define BAR_HEIGHT (96)
#define CHAN_WIDTH (13)
#define BAR_OFFSET ((CHAN_WIDTH - BAR_WIDTH) / 2)

#define PRIMARY_BAR_OFFSET (BAR_HEIGHT + 2 + ICON12_SIZE)
#define PRIMARY_BAR_HEIGHT (12)
#define PRIMARY_BAR_WIDTH (SCREEN_HEIGHT - (ICON16_SIZE + 1))

void displaySliderValuesPortraitPrimary() {
  display.clearDisplay();

  for (uint8_t slider = 0; slider < NUM_SLIDERS; slider++) {
    uint8_t fillBar = map(analogSliderValues[currentLayer][slider], 0, 1016, 0, BAR_HEIGHT);
    uint8_t localX = BAR_OFFSET + CHAN_WIDTH * slider;
    display.drawRect(localX, 0, BAR_WIDTH, BAR_HEIGHT, SSD1306_WHITE);
    display.fillRect(localX + 1, BAR_HEIGHT - fillBar + 1, BAR_WIDTH - 2, fillBar, SSD1306_WHITE);

    display.drawBitmap(localX - 2, BAR_HEIGHT + 1, iconSliderBmps[currentLayer][slider], ICON12_SIZE, ICON12_SIZE, SSD1306_WHITE);
  }
  display.drawBitmap(0, PRIMARY_BAR_OFFSET, bmp16_speaker, ICON16_SIZE, ICON16_SIZE, SSD1306_WHITE);

  uint8_t fillPrimaryBar = map(analogPrimaryValue, 0, 1016, 0, PRIMARY_BAR_WIDTH);
  display.drawRect(ICON16_SIZE + 1, PRIMARY_BAR_OFFSET + 2, PRIMARY_BAR_WIDTH, PRIMARY_BAR_HEIGHT, SSD1306_WHITE);
  display.fillRect(ICON16_SIZE + 1, PRIMARY_BAR_OFFSET + 3, fillPrimaryBar, PRIMARY_BAR_HEIGHT - 2, SSD1306_WHITE);


  display.display();
}

void updateButtonInput() {
  static uint8_t oldButton = 0;
  uint8_t newButton = digitalRead(buttonInput);

  if ((newButton == 0) & (oldButton == 1)) {
    if (enableReads) {
      currentLayer++;
      enableReads = false;
      if (currentLayer >= NUM_OF_LAYERS) {
        currentLayer = 0;
      }
    } else {
      enableReads = true;
    }
  }

  oldButton = newButton;
}