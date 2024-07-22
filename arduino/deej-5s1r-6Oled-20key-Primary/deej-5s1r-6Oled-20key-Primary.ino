#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Tiny4kOLED.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_NeoPixel.h>
#include "icon24.h"
#include "Keyboard.h"

/**********************************************************************************************************************/
/* Display definitions                                                                                                */
/**********************************************************************************************************************/
#define OLED_RESET     ( -1 )    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS ( 0x3C )  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SH110X_NO_SPLASH
#define SSD1306_NO_SPLASH
#define ICON_CANVAS_SIZE ( 16u )

uint8_t width  = 64;
uint8_t height = 32;

#define ICON_SIZE  ( 24u )
#define BYTE_WIDTH ( ICON_SIZE / 8u )
#define ARRAY_SIZE ( ICON_SIZE * BYTE_WIDTH )

Adafruit_SH1107   displayBig( 64, 128, &Wire, OLED_RESET );
Adafruit_NeoPixel strip( 20, 7, NEO_GRB + NEO_KHZ800 );
GFXcanvas1        iconPrep( ICON_CANVAS_SIZE, ICON_CANVAS_SIZE );
GFXcanvas1        bargraphPrep( 64, 6 );

/**********************************************************************************************************************/
/* Slider channel stuff                                                                                               */
/**********************************************************************************************************************/
typedef struct
{
    uint8_t analogInput;
    uint8_t oledChannel;
    bool    bigDisplayFlag : 1;
} sliderConstType;

#define SLEEP_CHANGE_VAL ( 32 )
#define SLEEP_TIME_MS    ( 2000u )
#define NUM_SLIDERS      ( 6u )
const sliderConstType analogStruct[NUM_SLIDERS] = {
    { A7, 7, true },  /* For this board this is the master Knob */
    { A0, 0, false }, /* Far Left Slider */
    { A1, 1, false },
    { A2, 2, false },
    { A3, 3, false },
    { A6, 4, false }, /* Far Right Slider */
};

uint16_t analogValues[NUM_SLIDERS];
uint16_t analogValuesOld[NUM_SLIDERS];
bool     analogMuteState[NUM_SLIDERS];
uint8_t  analogIconVal[NUM_SLIDERS] = { 0, ICON_FALLOUTBOY, ICON_HALFLIFE, ICON_CHROME, ICON_SPOTIFY, ICON_MICROPHONE };

/**********************************************************************************************************************/
/* Keypad Stuff                                                                                                       */
/**********************************************************************************************************************/
typedef enum
{
    KEYM_FNUM = 0,  // Value = F# to send
    KEYM_MUTE,      // Value = Slider # to control
    KEYM_MENU,      // Value = ignored
    KEYM_BACK       // Value = ignored
} keyTypeMine;

typedef struct
{
    keyTypeMine type  : 3;
    uint8_t     value : 5;
} keypadConstType;

typedef struct
{
    bool     value     : 1;
    bool     processed : 1;
    uint32_t time;
} keypadVariableType;

#define NUM_COLS ( 5u )
#define NUM_ROWS ( 4u )
const keypadConstType keyPadStruct[NUM_COLS][NUM_ROWS] = {
    { { KEYM_MUTE, 1 }, { KEYM_MENU, 0 }, { KEYM_FNUM, 22 }, { KEYM_FNUM, 17 } },
    { { KEYM_MUTE, 2 }, { KEYM_BACK, 0 }, { KEYM_FNUM, 21 }, { KEYM_FNUM, 16 } },
    { { KEYM_MUTE, 3 }, { KEYM_FNUM, 24 }, { KEYM_FNUM, 20 }, { KEYM_FNUM, 15 } },
    { { KEYM_MUTE, 4 }, { KEYM_FNUM, 23 }, { KEYM_FNUM, 19 }, { KEYM_FNUM, 14 } },
    { { KEYM_MUTE, 5 }, { KEYM_MUTE, 0 }, { KEYM_FNUM, 18 }, { KEYM_FNUM, 13 } },
};
#define MASTER_MUTE_LED ( 9u )

const uint8_t keypadRowPins[NUM_ROWS] = { 1, 0, 5, 10 };
const uint8_t keypadColPins[NUM_COLS] = { 15, 16, 14, 8, 9 };  // B1,B2,B3,B4,B5

keypadVariableType keypadValues[NUM_COLS][NUM_ROWS];

bool    uiChangeFlag     = true;
bool    uiSleepingFlag   = false;
bool    pixelChangeFlag  = true;
bool    menuDisplayFlag  = false;
uint8_t menuSliderSelect = 0;

/**********************************************************************************************************************/
/* Arduino Setup & Loop                                                                                               */
/**********************************************************************************************************************/
void setup( )
{
    volatile uint8_t slider;
    Serial.begin( 115200 );

    Keyboard.begin( );

    strip.begin( );                             // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.setBrightness( 100 );                 // Set BRIGHTNESS to about 1/5 (max = 255)
    colorWipe( strip.Color( 255, 0, 0 ), 10 );  // Red
    strip.show( );                              // Turn On all pixels ASAP // for testing

    Wire.begin( );
    for ( slider = 0; slider < NUM_SLIDERS; slider++ )
    {
        pinMode( analogStruct[slider].analogInput, INPUT );
        TCA9548A_setBus( analogStruct[slider].oledChannel );
        if ( analogStruct[slider].bigDisplayFlag )
        {
            displayBig.begin( SCREEN_ADDRESS, true );
            displayBig.setRotation( 4 );
            displayBig.clearDisplay( );
            displayBig.setFont( );
            displayBig.display( );
        }
        else
        {
            oled.begin( width, height, sizeof( tiny4koled_init_64x32br ), tiny4koled_init_64x32br );
            oled.clear( );
            oled.on( );
            //oled.setFont(&cp_437_box_drawing_font);
            drawSmallDisplay( slider );
        }
        setMuteLedColor( slider );
    }

    for ( uint8_t row = 0; row < NUM_ROWS; row++ )
    {
        pinMode( keypadRowPins[row], INPUT_PULLUP );
    }

    for ( uint8_t col = 0; col < NUM_COLS; col++ )
    {
        pinMode( keypadColPins[col], OUTPUT );
        digitalWrite( keypadColPins[col], HIGH );
    }

    for ( uint8_t row = 0; row < NUM_ROWS; row++ )
    {
        for ( uint8_t col = 0; col < NUM_COLS; col++ )
        {
            keypadValues[col][row].processed = true;
            keypadValues[col][row].value     = true;
        }
    }

    drawMainScreen( );
}

void loop( )
{
    static uint32_t nextUpdateTime  = 0;
    static uint32_t nextDisplayTime = 0;
    uint32_t        currentTime     = millis( );

    scanKeypad( );
    if ( pixelChangeFlag )
    {
        pixelChangeFlag == false;
        strip.show( );
    }

    if ( ( (int32_t)( nextUpdateTime - currentTime ) ) <= 0 )
    {
        nextUpdateTime = currentTime + 10;  // this allows for even sampling
        updateSliderValues( );
        sendSliderValues( );
        displaySleepProcess( );
    }

    //this allows for more responsive LEDs
    if ( ( (int32_t)( nextDisplayTime - currentTime ) ) <= 0 )
    {
        nextDisplayTime = currentTime + 100;
        for ( int slider = 0; slider < NUM_SLIDERS; slider++ )
        {
            TCA9548A_setBus( analogStruct[slider].oledChannel );
            if ( analogStruct[slider].bigDisplayFlag )
            {
                bargraphPrepValue( analogValues[slider] );
                displayBig.drawBitmap( 0, 120, bargraphPrep.getBuffer( ), 64, 6, 1, 0 );
                displayBig.display( );
            }
            else
            {
                drawSmallDisplay( slider );
            }
        }
    }
}

/**********************************************************************************************************************/
/* Keypad functions                                                                                                   */
/**********************************************************************************************************************/
/******************************************************************************
 * scanKeypad
 *
 * This function scans the keypad by:
 *  1. Setting a column pin low, 
 *  2. Reading the value of each of the rows
 *  3. Sees if it is different than the previous value
 *  4. waits for next call to process if it is still the same
 *  5. only processes once for each high/low value change
 *****************************************************************************/
void scanKeypad( void )
{
    uint8_t row, col;

    for ( col = 0; col < NUM_COLS; col++ )
    {
        digitalWrite( keypadColPins[col], LOW );
        delayMicroseconds( 10 );
        for ( row = 0; row < NUM_ROWS; row++ )
        {
            uint8_t locKey = digitalRead( keypadRowPins[row] );
            if ( locKey != keypadValues[col][row].value )
            {
                uiChangeFlag                     = true;
                keypadValues[col][row].value     = locKey;
                keypadValues[col][row].processed = false;
            }
            else
            {
                if ( keypadValues[col][row].processed == false )
                {
                    processKeypad( col, row );
                }
            }
        }
        digitalWrite( keypadColPins[col], HIGH );
    }
}

/******************************************************************************
 * processKeypad
 *
 * Processes a keypad Press/Release based on key pressed, and if we are in
 * menu mode or not. this function manages RGB colors for respective keys
 *
 * KEYM_FNUM:
 *     Calls Keyboard press/release depending on press/release value
 *     [menuDisplayFlag] Sets active selected slider to equivelent icon 
 *
 * KEYM_MUTE:
 *     Toggles slider mute flag on a press
 *     [menuDisplayFlag] Changes Selected slider to allow for icon change 
 *
 * KEYM_MENU:
 *     Sets menuDisplayFlag and turns on default slider select
 *
 * KEYM_BACK:
 *     Clears menuDisplayFlag and turns off standard slider select
 *
 * @param  col Column of key to process
 * @param  row Row of key to process
 *****************************************************************************/
void processKeypad( uint8_t col, uint8_t row )
{
    keypadValues[col][row].processed = true;

    switch ( keyPadStruct[col][row].type )
    {
        case KEYM_FNUM:
            {
                if ( keypadValues[col][row].value == 1 )
                {
                    strip.setPixelColor( row * NUM_COLS + col, strip.Color( 255, 0, 0 ) );
                    pixelChangeFlag = true;

                    if ( menuDisplayFlag )
                    {
                        //
                        analogIconVal[menuSliderSelect] = keyPadStruct[col][row].value - 13;
                    }
                    else
                    {
                        //send fnum value
                        Keyboard.release( keyPadStruct[col][row].value + 227 );
                    }
                }
                else
                {
                    strip.setPixelColor( row * NUM_COLS + col, strip.Color( 0, 0, 255 ) );
                    pixelChangeFlag = true;
                    if ( menuDisplayFlag )
                    {
                        //
                        analogIconVal[menuSliderSelect] = keyPadStruct[col][row].value - 13;
                    }
                    else
                    {
                        //send fnum value
                        Keyboard.press( keyPadStruct[col][row].value + 227 );
                    }
                }
            }
            break;

        case KEYM_MUTE:
            {
                if ( menuDisplayFlag & ( keyPadStruct[col][row].value != 0 ) )
                {
                    setMuteLedColor( menuSliderSelect );
                    menuSliderSelect = keyPadStruct[col][row].value;
                    strip.setPixelColor( menuSliderSelect - 1, strip.Color( 0, 0, 255 ) );
                }
                else
                {
                    if ( keypadValues[col][row].value == 0 )
                    {
                        analogMuteState[keyPadStruct[col][row].value] = !analogMuteState[keyPadStruct[col][row].value];
                        setMuteLedColor( keyPadStruct[col][row].value );
                    }
                }
            }
            break;

        case KEYM_MENU:
            {
                menuDisplayFlag  = true;
                menuSliderSelect = 1;
                strip.setPixelColor( menuSliderSelect - 1, strip.Color( 0, 0, 255 ) );
                pixelChangeFlag = true;
                drawMainScreen( );
            }
            break;

        case KEYM_BACK:
            {
                menuDisplayFlag = false;
                setMuteLedColor( menuSliderSelect );
                drawMainScreen( );
            }
            break;
    }
}

void displaySleepProcess( void )
{
    static uint32_t nextSleepTime = 0;

    if ( uiChangeFlag )
    {
        nextSleepTime = millis( ) + SLEEP_TIME_MS;
        uiChangeFlag  = false;

        if ( uiSleepingFlag )
        {
            uiSleepingFlag = false;
            for ( uint8_t slider = 0; slider < NUM_SLIDERS; slider++ )
            {
                TCA9548A_setBus( analogStruct[slider].oledChannel );
                if ( analogStruct[slider].bigDisplayFlag )
                {
                    displayBig.oled_command( SH110X_DISPLAYON );
                }
                else
                {
                    oled.on( );
                }
            }
        }
    }

    if ( ( (int32_t)( nextSleepTime - millis( ) ) ) <= 0 )
    {
        uiSleepingFlag = true;

        for ( uint8_t slider = 0; slider < NUM_SLIDERS; slider++ )
        {
            TCA9548A_setBus( analogStruct[slider].oledChannel );
            if ( analogStruct[slider].bigDisplayFlag )
            {
                displayBig.oled_command( SH110X_DISPLAYOFF );
            }
            else
            {
                oled.off( );
            }
        }
    }
}

/**********************************************************************************************************************/
/* LED Functions                                                                                                      */
/**********************************************************************************************************************/
/******************************************************************************
 * setMuteLedColor
 *
 * Changes desired Slider Mute Button LED to:
 *    Mute ON:  RED
 *    Mute OFF: GREEN
 *
 * @param  slider slider designation to change it's LED
 *****************************************************************************/
void setMuteLedColor( uint8_t slider )
{
    uint8_t pixel = slider - 1;
    if ( slider == 0 )
    {
        pixel = MASTER_MUTE_LED;
    }
    if ( analogMuteState[slider] )
    {
        strip.setPixelColor( pixel, strip.Color( 255, 0, 0 ) );
    }
    else
    {
        strip.setPixelColor( pixel, strip.Color( 0, 255, 0 ) );
    }
    pixelChangeFlag = true;
}

/**********************************************************************************************************************/
/* Get the analog sliders                                                                                             */
/**********************************************************************************************************************/
/******************************************************************************
 * updateSliderValues
 *
 * Vanilla update slider method,
 * With added IIR response smoothing
 *    Alpha = 0.5 so we can just divide by 2
 *
 * Also checks for changes to wakeup from sleep mode
 *****************************************************************************/
void updateSliderValues( void )
{
    for ( int slider = 0; slider < NUM_SLIDERS; slider++ )
    {
        analogValues[slider] = analogRead( analogStruct[slider].analogInput ) / 2 + analogValues[slider] / 2;

        int16_t changedValue = analogValues[slider] - analogValuesOld[slider];
        if ( ( changedValue > SLEEP_CHANGE_VAL ) || ( changedValue < -SLEEP_CHANGE_VAL ) )
        {
            analogValuesOld[slider] = analogValues[slider];
            uiChangeFlag            = true;
        }
    }
}

/**********************************************************************************************************************/
/* Slider Values to DEEJ app                                                                                          */
/**********************************************************************************************************************/
/******************************************************************************
 * sendSliderValues
 *
 * Vanilla send slider method, modified to send as building, muted sliders send 0
 *****************************************************************************/
void sendSliderValues( void )
{
    for ( uint8_t slider = 0; slider < NUM_SLIDERS; slider++ )
    {
        if ( analogMuteState[slider] )
        {
            Serial.print( "0" );
        }
        else
        {
            Serial.print( analogValues[slider] );
        }

        if ( slider < NUM_SLIDERS - 1 )
        {
            Serial.print( "|" );
        }
    }

    Serial.println( );
}

/**********************************************************************************************************************/
/* Display Functions                                                                                                  */
/**********************************************************************************************************************/
/******************************************************************************
 * drawMainScreen
 *
 * draws the main screen table and lines
 *
 * KEYM_FNUM:
 *     Draws a procedurally generated icon that says Fnum depending on Value
 *     [menuDisplayFlag] Draws the icon associated 0-11 array depending on 13-24 Fnum
 *
 * KEYM_MUTE:
 *     Draws the mute icon
 *
 * KEYM_MENU:
 *     Draws the Menu Icon
 *
 * KEYM_BACK:
 *     Draws the Back Icon
 *****************************************************************************/
void drawMainScreen( void )
{
    uint8_t row, col;

    displayBig.clearDisplay( );

    for ( col = 0; col < NUM_COLS; col++ )
    {
        for ( row = 1; row < NUM_ROWS; row++ )
        {
            switch ( keyPadStruct[col][row].type )
            {
                case KEYM_FNUM:
                    {
                        if ( menuDisplayFlag )
                        {
                            uint8_t tempIcon[ARRAY_SIZE];
                            uint8_t dispIcon[32];
                            uint8_t index16 = 0;
                            uint8_t index24 = 0;

                            memcpy_P( tempIcon, ico24_allArray[keyPadStruct[col][row].value - 13], ARRAY_SIZE );
                            while ( index16 < 32 )
                            {
                                dispIcon[index16]     = tempIcon[index24];
                                dispIcon[index16 + 1] = tempIcon[index24 + 1];
                                index16 += 2;
                                index24 += 3;
                            }
                            iconPrep.drawBitmap( 0, 0, dispIcon, ICON_CANVAS_SIZE, ICON_CANVAS_SIZE, 1, 0 );
                        }
                        else
                        {
                            iconPrep.fillScreen( 0 );
                            iconPrepFnum( keyPadStruct[col][row].value );
                        }
                    }
                    break;

                case KEYM_MUTE:
                    {
                        iconPrep.drawBitmap( 0, 0, ico16_mute, ICON_CANVAS_SIZE, ICON_CANVAS_SIZE, 1, 0 );
                    }
                    break;

                case KEYM_MENU:
                    {
                        iconPrep.drawBitmap( 0, 0, ico16_menu, ICON_CANVAS_SIZE, ICON_CANVAS_SIZE, 1, 0 );
                    }
                    break;

                case KEYM_BACK:
                    {
                        iconPrep.drawBitmap( 0, 0, ico16_back, ICON_CANVAS_SIZE, ICON_CANVAS_SIZE, 1, 0 );
                    }
                    break;
            }

            displayBig.drawBitmap( ( row - 1 ) * 21 + 2, col * 21 + 2, iconPrep.getBuffer( ),
                                   ICON_CANVAS_SIZE, ICON_CANVAS_SIZE, 1, 0 );
            displayBig.drawRect( ( row - 1 ) * 21, col * 21, 20, 20, 1 );
        }
    }
    displayBig.drawBitmap( 0, 120, bargraphPrep.getBuffer( ), 64, 6, 1, 0 );
}

/******************************************************************************
 * iconPrepFnum
 *
 * Procedurally generates the Fnumber Icon
 *
 * @param  num Fnumber to print in the procedural icon
 *****************************************************************************/
void iconPrepFnum( uint8_t num )
{
    iconPrep.fillScreen( 0 );
    iconPrep.setTextColor( 1 );
    iconPrep.setCursor( 1, 1 );
    iconPrep.print( 'F' );
    iconPrep.setCursor( 4, 8 );
    iconPrep.print( num );
}

/******************************************************************************
 * bargraphPrepValue
 *
 * Procedurally generates the Bargraph icon
 *
 * @param  value Value to fill the bargraph in (0-1023)
 *****************************************************************************/
void bargraphPrepValue( uint16_t value )
{
    uint8_t localWidth = value / 16;
    bargraphPrep.fillScreen( 0 );
    bargraphPrep.drawRect( 0, 0, 64, 6, 1 );
    bargraphPrep.fillRect( 0, 0, localWidth, 6, 1 );
}

/******************************************************************************
 * drawSmallDisplay
 *
 * Creates and sends the Small display for each display
 *
 * @param  slider Value to Create the display for Icon/Bargraph size
 *****************************************************************************/
void drawSmallDisplay( uint8_t slider )
{
    uint8_t localWidth = analogValues[slider] / 16;
    uint8_t localPercent = map(analogValues[slider],0,1023,0,100);
    uint8_t tempIcon[ARRAY_SIZE];
    uint8_t dispIcon[ARRAY_SIZE];
    char tempString[4];
    memset( dispIcon, 0, ARRAY_SIZE );
    memcpy_P( tempIcon, ico24_allArray[analogIconVal[slider]], ARRAY_SIZE );

    //remap icon for tinyOled Library
    for ( uint8_t row = 0; row < ICON_SIZE; row++ )
    {
        uint8_t rowByte = row >> 3;
        uint8_t rowBit  = 1 << ( row & 0x7 );

        for ( uint8_t col = 0; col < ICON_SIZE; col++ )
        {
            uint8_t colByte = col >> 3;
            uint8_t colBit  = 1 << ( 7 - col & 0x7 );

            if ( tempIcon[row * BYTE_WIDTH + colByte] & colBit )
            {
                // Set the corresponding bit in the output array
                dispIcon[col + rowByte * ICON_SIZE] |= rowBit;
            }
        }
    }

    RamBitmap( 20, 0, 20 + 24, 3, dispIcon );  // send the icon to the screen

    // get the bargraph ready
    if ( analogMuteState[slider] )
    {
        localWidth = 0;  // if muted make the bar show 0
        localPercent = 0;
    }
    memset( dispIcon, 0x84, width );       // draw the top and bottom bars
    memset( dispIcon, 0xFC, localWidth );  // draw the solid portion
    dispIcon[0]         = 0xFC;            // draw the near end cap
    dispIcon[width - 1] = 0xFC;            // draw the far end cap
    RamBitmap( 0, 3, 64, 4, dispIcon );    // send the bargraph to the screen
    oled.setFont(FONT6X8);
    oled.setCursor(0,0);
    if(localPercent < 10)
    {
      oled.print(' ');
      oled.print(localPercent);
    }
    else
    {
      oled.print(localPercent);
    }
    
}

/**********************************************************************************************************************/
/* Helper Functions                                                                                                   */
/**********************************************************************************************************************/

/******************************************************************************
 * TCA9548A_setBus
 *
 * Sets the I2C Mux to a channel
 *
 * @param  argBus Which I2C bus to select
 *****************************************************************************/
void TCA9548A_setBus( uint8_t argBus )
{
    Wire.beginTransmission( 0x70 );
    Wire.write( 1 << argBus );
    Wire.endTransmission( );
}

/******************************************************************************
 * colorWipe
 *
 * Wipes a color across the LED strip
 *
 * @param  color what color to use
 * @param  wait a mS delay to wait between each subsequent LED shown
 *****************************************************************************/
void colorWipe( uint32_t color, int wait )
{
    for ( int i = 0; i < strip.numPixels( ); i++ )
    {                                     // For each pixel in strip...
        strip.setPixelColor( i, color );  //  Set pixel's color (in RAM)
        strip.show( );                    //  Update strip to match
        delay( wait );                    //  Pause for a moment
    }
}

/******************************************************************************
 * RamBitmap
 *
 * Uses Ram to display a bitmap to the small screen Oled.Bitmap uses Rom and
 * didn't work with rotated bitmaps
 *
 * @param  x0 Start X bank
 * @param  y0 Start Y Pixel
 * @param  x1 End X bank
 * @param  y2 End Y Pixel
 * @param  bitmap bitmap data
 *****************************************************************************/
void RamBitmap( uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t *bitmap )
{
    uint16_t j = 0;
    for ( uint8_t y = y0; y < y1; y++ )
    {
        oled.setCursor( x0, y );
        oled.startData( );
        for ( uint8_t x = x0; x < x1; x++ )
        {
            oled.sendData( *bitmap );
            bitmap++;
        }
        oled.endData( );
    }
    oled.setCursor( 0, 0 );
}
