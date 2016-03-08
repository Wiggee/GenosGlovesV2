#include <SPI.h>                // To talk to the SD card and MP3 chip
#include <SdFat.h>              // SD card file system
#include <SFEMP3Shield.h>       // MP3 decoder chip
#include <Adafruit_NeoPixel.h>  // NeoPixel library

boolean  debugging = true; // Set serial debugging



/*******************************************************************************/
/* Begin NeoPatterns                                                           */
/*******************************************************************************/
// NeoPatterns from https://learn.adafruit.com/multi-tasking-the-arduino-part-3/
// Pattern types supported:
enum  pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE, FADEOUT };
// Patern directions supported:
enum  direction { FORWARD, REVERSE };

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
    public:

    // Member Variables:  
    pattern  ActivePattern;  // which pattern is running
    direction Direction;     // direction to run the pattern
    
    unsigned long Interval;   // milliseconds between updates
    unsigned long lastUpdate; // last update of position
    
    uint32_t Color1, Color2;  // What colors are in use
    uint32_t ColorLast, ColorCurrent;  // What colors are in use
    uint16_t TotalSteps;  // total number of steps in the pattern
    uint16_t Index;  // current step within the pattern
    
    void (*OnComplete)();  // Callback on completion of pattern
    
    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
    :Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }
    
    // Update the pattern
    void Update()
    {
        if((millis() - lastUpdate) > Interval) // time to update
        {
            lastUpdate = millis();
            switch(ActivePattern)
            {
                case RAINBOW_CYCLE:
                    RainbowCycleUpdate();
                    break;
                case THEATER_CHASE:
                    TheaterChaseUpdate();
                    break;
                case COLOR_WIPE:
                    ColorWipeUpdate();
                    break;
                case SCANNER:
                    ScannerUpdate();
                    break;
                case FADE:
                    FadeUpdate();
                    break;
                case FADEOUT:
                    FadeOutUpdate();
                    break;
                default:
                    break;
            }
        }
    }
	
    // Increment the Index and reset at the end
    void Increment()
    {
        if (Direction == FORWARD)
        {
           Index++;
           if (Index >= TotalSteps)
            {
                Index = 0;
                ColorCurrent = ColorLast;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
        else // Direction == REVERSE
        {
            --Index;
            if (Index <= 0)
            {
                Index = TotalSteps-1;
                ColorCurrent = ColorLast;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
    }
    
    // Reverse pattern direction
    void Reverse()
    {
        if (Direction == FORWARD)
        {
            Direction = REVERSE;
            Index = TotalSteps-1;
        }
        else
        {
            Direction = FORWARD;
            Index = 0;
        }
    }
    
    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = RAINBOW_CYCLE;
        Interval = interval;
        TotalSteps = 255;
        Index = 0;
        Direction = dir;
    }
    
    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
        for(int i=0; i< numPixels(); i++)
        {
            ColorLast = Wheel(((i * 256 / numPixels()) + Index) & 255);
            setPixelColor(i, ColorLast);
        }
        show();
        Increment();
    }

    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = THEATER_CHASE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
   }
    
    // Update the Theater Chase Pattern
    void TheaterChaseUpdate()
    {
        for(int i=0; i< numPixels(); i++)
        {
            if ((i + Index) % 3 == 0)
            {
                setPixelColor(i, Color1);
                ColorLast = Color1;
            }
            else
            {
                setPixelColor(i, Color2);
                ColorLast = Color2;
            }
        }
        show();
        Increment();
    }

    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = COLOR_WIPE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color;
        Index = 0;
        Direction = dir;
    }
    
    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
        setPixelColor(Index, Color1);
        ColorLast = Color1;
        show();
        Increment();
    }
    
    // Initialize for a SCANNNER
    void Scanner(uint32_t color1, uint8_t interval)
    {
        ActivePattern = SCANNER;
        Interval = interval;
        TotalSteps = (numPixels() - 1) * 2;
        Color1 = color1;
        Index = 0;
    }

    // Update the Scanner Pattern
    void ScannerUpdate()
    { 
        for (int i = 0; i < numPixels(); i++)
        {
            if (i == Index)  // Scan Pixel to the right
            {
                 setPixelColor(i, Color1);
                 ColorLast = Color1;
            }
            else if (i == TotalSteps - Index) // Scan Pixel to the left
            {
                 setPixelColor(i, Color1);
                 ColorLast = Color1;
            }
            else // Fading tail
            {
                 setPixelColor(i, DimColor(getPixelColor(i)));
                 ColorLast = DimColor(getPixelColor(i));
            }
        }
        show();
        Increment();
    }
    
    // Initialize for a Fade
    void Fade(uint32_t colorNew, uint16_t steps, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = FADE;
        Interval = interval;
        TotalSteps = steps;
        Color1 = ColorCurrent;
        Color2 = colorNew;
        Index = 0;
        Direction = dir;
    }
    
    // Update the Fade Pattern
    void FadeUpdate()
    {
        // Calculate linear interpolation between Color1 and Color2
        // Optimise order of operations to minimize truncation error
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;
        
        //if (debugging) { Serial.print("Fade update: R "); Serial.print(red); Serial.print(", G "); Serial.print(green); Serial.print(", B "); Serial.print(blue); Serial.println("."); } 
        
        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }
    
    // Initialize for a single color Fade
    void FadeOut(uint16_t steps, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = FADEOUT;
        Interval = interval;
        TotalSteps = steps;
        Color1 = ColorLast;
        Color2 = Color(0,0,0);
        Index = 0;
        Direction = dir;
    }
    
    // Update the Fade Pattern
    void FadeOutUpdate()
    {
        // Calculate linear interpolation between Color1 and Color2
        // Optimise order of operations to minimize truncation error
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;
        
        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }
    
     // Initialize for Stop
    void Stop()
    {
        ActivePattern = NONE;
    }
   
    // Calculate 50% dimmed version of a color (used by ScannerUpdate)
    uint32_t DimColor(uint32_t color)
    {
        // Shift R, G and B components one bit to the right
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        return dimColor;
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
            ColorLast = color;
        }
        show();
    }
    
    // Returns the current 32-bit color
    uint32_t getCurrentColor()
    {
        return ColorLast;
    }
    
    // Returns the current 32-bit color
    int getCurrentPattern()
    {
        return ActivePattern;
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }
    
    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if(WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if(WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        else
        {
            WheelPos -= 170;
            return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }
    }
};


/*******************************************************************************/
/* End NeoPatterns                                                             */
/*******************************************************************************/








/*******************************************************************************/
/* Begin Main code                                                             */
/*******************************************************************************/

// Constants for the trigger input pins, which we'll place
// in an array for convenience:
const int TRIG1 = A0;
const int TRIG2 = A4;
const int TRIG3 = A5;
const int TRIG4 = 1;
const int TRIG5 = 0;
int trigger[5] = {TRIG1,TRIG2,TRIG3,TRIG4,TRIG5};
NeoPatterns Ring (16, 5, NEO_GRB + NEO_KHZ800, &RingComplete);
NeoPatterns Strip(60, 7, NEO_GRB + NEO_KHZ800, &StripComplete);
uint32_t colorCurrent = Ring.Color ( 128, 0, 0 );
uint32_t colorDefault = Ring.Color ( 128, 0, 0 );
uint32_t colorFire =    Ring.Color ( 0, 128, 0 );
uint32_t colorFade =    Ring.Color ( 0, 0, 0 );
int brightnessNormal = 100;
int brightnessDim = 20;
int tempNum = 0;

// And a few outputs we'll be using:
const int ROT_LEDR = 10; // Red LED in rotary encoder (optional)
const int EN_GPIO1 = A2; // Amp enable + MIDI/MP3 mode select
const int SD_CS = 9;     // Chip Select for SD card

// Create library objects:
SFEMP3Shield MP3player;
SdFat sd;
boolean  interrupt = true; // set triggered file to be able to be interrupted.
boolean  interruptself = true; // set triggered file to interrupt itself.


// We'll store the five filenames as arrays of characters.
// "Short" (8.3) filenames are used, followed by a null character.
char filename[9][13];
//***************************************************************************

//0 PWRDOWN / Muscle Sensor Trigger
//1 Trigger 2  - Toggle backlight
//2 Trigger 3  - Theme song
//3 Trigger 4
//4 Trigger 5  - MyoWare Sensor
//5 IMPORT
//6 ONLINE
//7 PWRUP
//8 FIRE

// We need to setup a muscle sensor reading value which will be used to tell 
// at what level of muscle flexion we want the repulsor to be triggered.
// Adjust higher to make less sensative, adjust lower to make more sensitive
int iThreshold			= 250;    // Value to trigger repulsor power up
const int NeoPixelPin           = 5;      // pin connected to the Glove
const int NeoPixelStripPin      = -1;      // pin connected to the Glove
const boolean useStrip         = false;
boolean bPreviousState		= false;
boolean firstLoop               = true;
boolean keepLights              = true;
boolean isLit                   = false;
long POWERUP_SFX_LENGTH = 1080; //ms
long POWERDWN_SFX_LENGTH = 1250; //ms
long FIRE_SFX_LENGTH = 1828; //ms
long LED_MAX = 255;
long brightnessValue = brightnessNormal;


void RingComplete()
{
    Ring.Stop();
}


// Strip Completion Callback
void StripComplete()
{
    Strip.Stop();
}

//----------------------------------------------------------------------------
//
///	powerUp
///
///	@desc	Plays the repulsor power up sound effect while reading the muscle sensor.
///             The sound effect will be interupted if the muscle sensor signal drops below 
///             the threshold. This allows for rapid firing of the repulsor.
///
//----------------------------------------------------------------------------
void powerUp() 
{
  if (debugging) { Serial.println("Running powerUp");  }
  if(firstLoop || !isLit) {  brightnessValue = 1; }
  MP3player.playMP3(filename[7]);

  // setup fade in variables  
  long currTime = millis();
  long prevTime = currTime;
  long brightnessStart = brightnessNormal;
  long brightnessEnd = brightnessDim;
  long timeDivision = POWERUP_SFX_LENGTH/(abs(brightnessEnd-brightnessStart)); // clip length in milliseconds divided by the desired LED value (25% brightness)
  uint32_t  colorOld = colorCurrent;
  uint32_t  colorNew = colorDefault;
  int  delayVal = 60;
  int  numChanges = 40;
  int  currentChange = 0;

  if (debugging) { Serial.println("powerUp fade 1");  }
  Ring.Fade(colorDefault, POWERUP_SFX_LENGTH, 1);
  while (MP3player.isPlaying()) 
  {
    // fade in from min to max over length of clip:
    currTime = millis();
    if(currTime-prevTime >= timeDivision)
    {
      brightnessValue +=1;

      // update glove LEDs
      if(brightnessValue <= brightnessEnd)
      {
        Ring.setBrightness(brightnessValue);
        if(useStrip) { Strip.setBrightness(brightnessValue); }
      }
    }  	  
    Ring.Update();

    if(interrupt)
    {
      boolean  bState = ReadMuscleSensor(TRIG1);
      if(!bState)
        MP3player.stopTrack();
        
      //ReadMP3PlayerTriggers();
      readTriggers();
    }
  }
}

//----------------------------------------------------------------------------
//
///	fire
///
///	@desc	Plays the repulsor firing sound effect while reading the muscle sensor.
///             The sound effect will be interupted if the muscle sensor signal goes above 
///             the threshold (meaning another fire sequence is being initiated. If the firing
///             sound effect is not interupted, the repulsor power down sound effect is played.
///             The power down sound effect can also be interupted. This allows for rapid firing 
///             of the repulsor.
///
//----------------------------------------------------------------------------
void fire() 
{
  if (debugging) { Serial.println("Fire!"); }
  uint32_t colorOld = colorCurrent;
  uint32_t colorNew = colorFire;
  uint32_t colorTemp = colorDefault;
  int  delayVal = 20;
  uint32_t colorChange = colorOld;
  //int  colorChange[3] = {round((colorOld[0]-colorNew[0])/delayVal),round((colorOld[1]-colorNew[1])/delayVal),round((colorOld[2]-colorNew[2])/delayVal)};
  //long brightnessValue = LED_MAX; // full brightness
  boolean  bState = false;
  int  numChanges = 120;
  int  currentChange = 0;

  MP3player.playMP3(filename[8]);
  
  int colorValue = 0;
  // turn on the glove LEDs
  int brightnessChange = round((LED_MAX-brightnessValue)/delayVal);
  if (debugging) { Serial.print("Fire fade 1"); }
  Ring.Fade(colorFire, 1828, 1);
  while(brightnessValue < LED_MAX)
  {
    brightnessValue+=brightnessChange;
    Ring.setBrightness(brightnessValue);
    if(useStrip) { Strip.setBrightness(brightnessValue); }
    currentChange++;
    Ring.Update();
  }
  
  if (debugging) { Serial.print("Fire fading out"); }
  // fade out over length of clip:
  if(keepLights) { Ring.Fade(colorDefault, POWERDWN_SFX_LENGTH, 1); }
  else { Ring.Fade(colorFade, POWERDWN_SFX_LENGTH, 1); }
  while (MP3player.isPlaying()) 
  {    
    if(brightnessValue > brightnessNormal)
      brightnessValue--;
    Ring.setBrightness(brightnessValue);
    if(useStrip) { Strip.setBrightness(brightnessValue); }
    Ring.Update();
    //setAllPixels(colorCurrent);

    if(interrupt)
    {
      bState = ReadMuscleSensor(TRIG1);    
      if(bState)
        MP3player.stopTrack();
        
      //ReadMP3PlayerTriggers();
      readTriggers();
    }      
  }

  if(!bState)
  { 
    powerDown();
  }     
}

void powerDown()
{
    if (debugging) { Serial.println("Powering down"); }
    uint32_t colorOld = colorCurrent;
    uint32_t colorNew = colorDefault;
    if(!keepLights) {
      colorNew = colorFade;
    }
    
    int  numChanges = 40;
    int  currentChange = 0;
    
    int  delayVal = 20;
    boolean  bState = false;
   // now power down
    MP3player.playMP3(filename[0]);

    //setup fade out variables
    long currTime = millis();
    long prevTime = currTime;
    long brightnessStart = brightnessValue;
    long brightnessEnd = brightnessNormal;
    if(!keepLights) { brightnessEnd = brightnessDim; }
    long timeDivision = POWERDWN_SFX_LENGTH/(abs(brightnessEnd-brightnessStart)); // clip length in milliseconds divided by the desired LED brightness value

    
    Ring.Fade(colorDefault, POWERDWN_SFX_LENGTH, 1);
    while (MP3player.isPlaying()) 
    {
      // fade out from max to min over length of clip:
      currTime = millis();
      if(currTime-prevTime >= timeDivision)
      {
        if(brightnessValue > brightnessEnd)
          brightnessValue--;

        // update glove LEDs
        if(brightnessValue >= brightnessEnd)
        {
          Ring.setBrightness(brightnessValue);
          if(useStrip) { Strip.setBrightness(brightnessValue); }
        }
        currentChange++;
        prevTime = currTime;
        delay(delayVal);
      }

      if(interrupt)
      {
        bState = ReadMuscleSensor(TRIG1);
        if(bState)
          MP3player.stopTrack();
        
        //ReadMP3PlayerTriggers();
        readTriggers();
      }
      
      delay(delayVal);
    }
}

void setAllPixels(uint32_t colorNew)
//-----------------------------------------------------------------------------------------------------------------------------------
//
///	setAllPixels
///
///	@desc		This method sets all pixels of the ring and strip (if necessary) to one color.
///
///	@param	        colorArray A 3-int array, each 0-255
///
//-----------------------------------------------------------------------------------------------------------------------------------
{
    //setAllPixels(colorArray[0], colorArray[1], colorArray[2]);
    Ring.Fade(colorNew, 40, 1);
}


void killAllPixels()
//-----------------------------------------------------------------------------------------------------------------------------------
//
///	setAllPixels
///
///	@desc		This method turns all pixels of the ring and strip (if necessary) off
///
///
//-----------------------------------------------------------------------------------------------------------------------------------
{
    Ring.Fade(colorFade, 40, 1);
    isLit = false;
    Ring.show();
}

void toggleLights()
//-----------------------------------------------------------------------------------------------------------------------------------
//
///	toggleLights
///
///	@desc		This method turns all pixels of the ring and strip (if necessary) off
///
///
//-----------------------------------------------------------------------------------------------------------------------------------
{
    if(isLit) {
      keepLights = false;
      powerDown();
    }
    else {
      keepLights = true;
      powerUp();
    }
}



//-----------------------------------------------------------------------------------------------------------------------------------
//
///	ReadMuscleSensor
///
///	@desc		This method reads each game button's state. If a button is pressed, it will change the button's
///		        state to true. If not, it will change the button's state to false.
///
///	@param	        iMuscleSensorPin  // Analog pin reading muscle sensor output
///
///	@return 	true if the muscle sensor value is greater than the threshold
//-----------------------------------------------------------------------------------------------------------------------------------
boolean ReadMuscleSensor(int iMuscleSensorPin)
{
  int val = analogRead(iMuscleSensorPin);    

  if(val >= iThreshold)
    return true;
  else
    return false;
}

//***************************************************************************
// Example code from Trigger.ino a sketch for Lilypad MP3 Player
//***************************************************************************
void SetupMP3Player()
{
  byte result;
  // The board uses a single I/O pin to select the
  // mode the MP3 chip will start up in (MP3 or MIDI),
  // and to enable/disable the amplifier chip:  
  pinMode(EN_GPIO1,OUTPUT);
  digitalWrite(EN_GPIO1,LOW);  // MP3 mode / amp off

  result = sd.begin(SD_CS, SPI_HALF_SPEED); // 1 for success
  
  if (result != 1) // Problem initializing the SD card
    errorBlink(1); // Halt forever, blink LED if present.
  
  // Start up the MP3 library
  result = MP3player.begin(); // 0 or 6 for success

  // Check the result, see the library readme for error codes.
  if ((result != 0) && (result != 6)) // Problem starting up
    errorBlink(result); // Halt forever, blink red LED if present.

  FindAudioFilenames();

  // Set the VS1053 volume. 0 is loudest, 255 is lowest (off):
  MP3player.setVolume(0,255);
  
  // Turn on the amplifier chip:  
  digitalWrite(EN_GPIO1,HIGH);
}

void FindAudioFilenames()
{
  SdFile file;
  char tempfilename[13];
  int index;
  // Now we'll access the SD card to look for any (audio) files
  // starting with the characters '1' to '5':

  // Start at the first file in root and step through all of them:
  sd.chdir("/",true);
  while (file.openNext(sd.vwd(),O_READ))
  {
    // get filename
    file.getFilename(tempfilename);

    // Does the filename start with char '1' through '10'?
    if (tempfilename[0] >= '1' && tempfilename[0] <= '10')
    {
      // Yes! subtract char '1' to get an index of 0 through 4.
      index = tempfilename[0] - '1';
      
      // Copy the data to our filename array.
      strcpy(filename[index],tempfilename);
    }
    file.close();
  }
}

void readTriggers()
{
  int t;              // current trigger
  static int last_t;  // previous (playing) trigger
  int x;
  byte result;
  
  // Step through the trigger inputs, looking for LOW signals.
  // The internal pullup resistors will keep them HIGH when
  // there is no connection to the input. Only check triggers 2-5.
  for(t = 2; t <= 5; t++)
  {
    if (digitalRead(trigger[t-1]) == LOW)
    {
      // Wait for trigger to return high for a solid 50ms
      // (necessary to avoid switch bounce on T2 and T3
      // since we need those free for I2C control of the
      // amplifier)      
      x = 0;
      while(x < 50)
      {
        if (digitalRead(trigger[t-1]) == HIGH)        {x++;}
        else                                          {x = 0;}      
        
        delay(1);
      } 
      
      switch(t) {
        case 2:
              powerUp();
              //testFade();
          break;
        case 3:
              fire();
              //testFade();
          break;
        case 4:
              toggleLights();
              //testFade();
          break;
        default:
              // If a file is already playing, and we've chosen to
              // allow playback to be interrupted by a new trigger,
              // stop the playback before playing the new file.
              if (interrupt && MP3player.isPlaying() && ((t != last_t) || interruptself))
              {
                MP3player.stopTrack();
              }
        
              // Play the filename associated with the trigger number.
              // (If a file is already playing, this command will fail
              //  with error #2).
              result = MP3player.playMP3(filename[t-1]);
              if (result == 0) last_t = t;  // Save playing trigger  
          break;
      }
      delay(50);
    }  
  }
}



void testFade() {
  if (debugging) { Serial.print("Testing fade #"); Serial.print(tempNum); Serial.print(": "); }
  tempNum++;
  if(tempNum > 4) { tempNum = 0; }
  uint32_t color1 = Ring.Color(255, 0, 0);
  uint32_t color2 = Ring.Color(0, 255, 0);
  uint32_t color3 = Ring.Color(0, 0, 255);
  uint32_t color4 = Ring.Color(0, 255, 255);
  uint32_t color5 = Ring.Color(255, 255, 0);
  uint32_t colorNew = color1;
  if(tempNum == 2)      { colorNew = color2;  }
  else if(tempNum == 3) { colorNew = color3;  }
  else if(tempNum == 4) { colorNew = color4;  }
  else if(tempNum == 5) { colorNew = color5;  }
  int numChanges = 50;
  int currentChange = 0;
  int delayNum = 250;
  if (debugging) { Serial.print("Fading to color "); Serial.print(String(colorNew,HEX)); Serial.print(" from color "); Serial.println(String(Ring.getCurrentColor(),HEX)); }
  Ring.Fade(colorNew, numChanges, 1);
  delay(delayNum);
}


void ReadMP3PlayerTriggers()
{
  int t;              // current trigger
  static int last_t;  // previous (playing) trigger
  int x;
  byte result;
  
  // Step through the trigger inputs, looking for LOW signals.
  // The internal pullup resistors will keep them HIGH when
  // there is no connection to the input. Only check triggers 2-5.
  for(t = 2; t <= 5; t++)
  {
    // The trigger pins are stored in the inputs[] array.
    // Read the pin and check if it is LOW (triggered).
    if (digitalRead(trigger[t-1]) == LOW)
    {
      // Wait for trigger to return high for a solid 50ms
      // (necessary to avoid switch bounce on T2 and T3
      // since we need those free for I2C control of the
      // amplifier)      
      x = 0;
      while(x < 50)
      {
        if (digitalRead(trigger[t-1]) == HIGH)        {x++;}
        else                                          {x = 0;}      
        
        delay(1);
      } 
      
      // If a file is already playing, and we've chosen to
      // allow playback to be interrupted by a new trigger,
      // stop the playback before playing the new file.
      if (interrupt && MP3player.isPlaying() && ((t != last_t) || interruptself))
      {
        MP3player.stopTrack();
      }

      // Play the filename associated with the trigger number.
      // (If a file is already playing, this command will fail
      //  with error #2).
      result = MP3player.playMP3(filename[t-1]);
      if (result == 0) last_t = t;  // Save playing trigger  
      
    }
  }
}

void errorBlink(int blinks)
{
  // The following function will blink the repulsor a given number 
  // of times and repeat forever. This is so you can see any startup
  // error codes without having to use the serial monitor window.

  int x;

  while(true) // Loop forever
  {
    for (x=0; x < blinks; x++) // Blink the given number of times
    {
      for(int i=0; i<Ring.numPixels(); i++) {Ring.setPixelColor(i, Ring.Color(127, 0, 0));}
      Ring.show(); // Turn LED ON
      delay(250);
      for(int i=0; i<Ring.numPixels(); i++) {Ring.setPixelColor(i, 0);}
      Ring.show(); // Turn LED OFF
      delay(250);
    }
    delay(1500); // Longer pause between blink-groups
  }
}


void IncrementAndDirection(int &value, const int minValue, const int maxValue, 
                const int incValue, boolean &forward )
{
  if(forward && value >= maxValue)
    {
      value = maxValue;
      forward = !forward;
    }    
    else if(!forward && value <= minValue)
    {
      value = minValue;
      forward = !forward;
    }
    
    if(forward)
      value = value + incValue;
    else
      value = value - incValue; 
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return Ring.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return Ring.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return Ring.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}


void setup()
{
  //***************************************************************************
  // Example code from Trigger.ino a sketch for Lilypad MP3 Player
  //***************************************************************************
  int x, index;
  SdFile file;
  byte result;
  char tempfilename[13];
  
  if (debugging)
  {
    Serial.begin(9600);
    Serial.println(F("Lilypad MP3 Player trigger sketch"));
  }
  

  // Setup the five trigger pins 
  if (debugging) { Serial.print(F("Initial pins...")); }
  for (x = 0; x <= 4; x++)
  {
    
    pinMode(trigger[x],INPUT);
    if(x > 0)
      digitalWrite(trigger[x],HIGH);
  }
  //***************************************************************************
  
  // Setup the repulsor LED PWM pin 
  pinMode(NeoPixelPin, OUTPUT); 
  if(useStrip && NeoPixelStripPin > 0) {
    pinMode(NeoPixelStripPin, OUTPUT); 
  }
  if (debugging) Serial.println(F("success!"));
  
  
  //
  SetupMP3Player();
  
  delay(20);
}

void loop()
{  
  if(firstLoop)
  {
    // First time through we want the JARVIS SFXs to play
    // Play the JARVIS "Importing Preferences" SFX
    MP3player.playMP3(filename[5]);
    
    for(int i=0; i<Ring.numPixels(); i++) {Ring.setPixelColor(i, 0);}
    int pixelIndex = 0;
    int colorIndex = 0;
    while (MP3player.isPlaying())
    {
      // This code creates a fluctuating comet that travels around
      // the NeoPixel ring varying brightness and color
      int numCluster = 3;        //change this value to set the comet tail length
      boolean fwdColor = true;     
      const int colorInc = 5;  
      
      // turn on leading NeoPixel
      Ring.setBrightness(brightnessNormal);
      Ring.setPixelColor(pixelIndex, Wheel(colorIndex % 255));      
      // turn off trailing NeoPixels
      if(pixelIndex-numCluster >= 0)
        Ring.setPixelColor(pixelIndex-numCluster, 0);       
      else
        Ring.setPixelColor(Ring.numPixels() + (pixelIndex-numCluster), 0);         
      Ring.show();
  
      IncrementAndDirection(colorIndex, 0, 255, 5, fwdColor);     // color: min=0, max=255, increment=5
      pixelIndex++;  
  
      if(pixelIndex > Ring.numPixels())
        pixelIndex = 0;
  
      delay(50);  
    }  
    
    // turn all NeoPixels off
    for(int i=0; i<Ring.numPixels(); i++) {Ring.setPixelColor(i, 0);}
    
    //Play the JARVIS "Online and Ready" SFX
    MP3player.playMP3(filename[6]);
    
    // color wipe green
    int brightIndex = 20;
    Ring.setBrightness(brightnessNormal);
    for(int i=0; i<Ring.numPixels(); i++) {Ring.setPixelColor(i, Ring.Color(0, 127, 0));Ring.show();delay(50);}          
    
    boolean fwdBright = false;         
    while (MP3player.isPlaying())
    {
      Ring.setBrightness(brightnessNormal);
      for(int i=0; i<Ring.numPixels(); i++) {Ring.setPixelColor(i, Ring.Color(0, 127, 0));}    
      Ring.show();
  
      IncrementAndDirection(brightnessNormal, 3, 20, 1, fwdBright);   // brightness: min=1, max=255
      delay(25);  
    }
    if(!keepLights) { killAllPixels(); }
    else { setAllPixels(colorDefault); }    
    firstLoop = false;
  }
  
  
  
  // Read the muscle sensor value
  boolean  bState = ReadMuscleSensor(TRIG1);

  // If the muscle sensor value is over the threshold (bState = true)
  // and the previous state was below the threshold, then play the
  // power up SFX. If the muscle sensor goes back below the threshold,
  // then play the fire SFX and turn the repulsor on while the SFX is
  // playing and the power down SFX afterwards.
  if(bState && !bPreviousState)
  {
    //digitalWrite(NeoPixelPin, HIGH);  // Turn the repulsor on when flexing
    powerUp();                    // Play the Repulsor Power Up SFX
  }
  else if(!bState && bPreviousState)
  {
    //digitalWrite(NeoPixelPin, LOW); // Turn the repulsor off when relaxed    
    fire();       // Play the Repulsor Firing and Power Down SFXs    
  }

  // Store state for next loop
  bPreviousState = bState;  
  
  // Check to see
  //ReadMP3PlayerTriggers();
  readTriggers();
  
  Ring.Update();
  
}
//***************************************************************************



/*******************************************************************************/
/* End Main code                                                               */
/*******************************************************************************/
