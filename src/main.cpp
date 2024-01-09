#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include "thingProperties.h"
#include <ezButton.h>
#include <ESP32Encoder.h>

/* =================== Version Changelog ==============================================================

    Version 1.1
      Seperate data pins for each temperature sensor
      Implementing LCD screen
      blinking status led
      rotary encoder button interaction

   ====================================================================================================
*/

// ==================== Settings ===================================================================

// Temperature sesnor precision e.g. 10 == 0.25
#define TEMPERATURE_PRECISION 10

// pins for each sensor
const int sensorTempIn = 25;
const int sensorTempOut = 33;
const int sensorTempPool = 32;
const int sensorPressure = 35;

// pins for the rotary encoder
const int pinRotarySW = 36;
const int pinRotaryCLK = 34;
const int pinRotaryDT = 39;

// pin for status led
const int pinStatusLed = 27; // the number of the LED pin

// =================================================================================================

// Array of OneWires for TemperatureIn, TemperatureOut, TemperaturePool
OneWire Wires[] = {OneWire(sensorTempIn), OneWire(sensorTempOut), OneWire(sensorTempPool)};

// Array of DallasTemperature sensors for TemperatureIn, TemperatureOut, TemperaturePool
DallasTemperature sensorsTemperature[] = {DallasTemperature(&Wires[0]), DallasTemperature(&Wires[1]), DallasTemperature(&Wires[2])};

// Measurements for each sensor Current/Min/Max
float ReadingsMatrix[4][3];

// Threshold matrix with layout {min, max}:   {{0, 100}, ...}
float LimitMatrix[4][3] = {
    {-255, 0, 40},
    {-255, 0, 85},
    {-255, 0, 95},
    {-255, 0.5, 3.5}};

// Names for each sensor; used for warning messages:  {"Sensor1", ...}
String SensorNames[4] = {
    "Kaltwassersensor",
    "Heizwassersensor",
    "Poolwassersensor",
    "Drucksensor"};

bool ErrorMatrix[4][3]; // {defect, min, max} for each sensor

int ledState = LOW;
unsigned long prevMillisStatusOn = 0; // will store last time LED was updated
unsigned long prevMillisStatusOff = 0;
unsigned long prevMillisLcdRefresh = 0; // will store last time LED was updated
unsigned long prevMillisMain = 0;
unsigned long prevMillisTemp = 0;

unsigned long start;
unsigned long end;
bool ErrorState;

long intervalStatusOn = 250; // interval at which to blink (milliseconds)
long intervalStatusOff = 10000;
const long intervalLcdRefresh = 2500;
const long intervalMain = 1000;

// Menu skipping
volatile int buttonState = 0;
volatile int displayOn = 0;
volatile bool rotaryEncoder = false;

volatile long unsigned int lastMillis = 0;
volatile long unsigned int currentMillis = 0;
volatile long unsigned int lastMillisLCDOn = 0;
volatile int TurnOnDisplay = 0;

long int intervalTemp = 750 / (1 << (12 - TEMPERATURE_PRECISION)); // TemperatureConversionTime
int rotationCounter = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

ezButton button(pinRotarySW);

// =================================================================================================

// Interrupt routine just sets a flag when rotation is detected
void IRAM_ATTR rotary()
{
  rotaryEncoder = true;
}

// Round Float to specified decimal places
float roundFloat(float number, int decimals)
{
  return (float)((int)((float)number * pow(10, decimals) + .5) / pow(10, decimals));
}

// Read DallasTemperature sensors by object address
float readTemperatureSensor(DallasTemperature *sensor)
{
  float temp = sensor->getTempCByIndex(0);
  if (temp == DEVICE_DISCONNECTED_C)
    return -255;
  else
    return temp;
}

// Read analog pressure sensor and convert it into [bar]
float readPressureSensor(int pin)
{
  float analogReading = analogRead(pin);
  if (analogReading < 400)
    return -255;                                                   // set pressure to -255 if device is defect
  float convPressure = roundFloat((analogReading - 400) / 510, 2); // 1k/2k-pcb: roundFloat((analogReading - 485) / 745, 2)
  return convPressure;
}

void readSensors()
{
  for (int i = 0; i < 3; i++)
  {
    ReadingsMatrix[i][0] = readTemperatureSensor(&sensorsTemperature[i]);
  }

  ReadingsMatrix[3][0] = readPressureSensor(sensorPressure);
}

// read sensors and fill min/max readings with current readings
void initReadings()
{
  readSensors();

  for (int i = 0; i < 4; i++)
  {
    for (int k = 1; k < 3; k++)
    {
      ReadingsMatrix[i][k] = ReadingsMatrix[i][0];
    }
  }
}

// Update all sensor readings including minimum and maximum values
void updateReadings()
{
  readSensors();

  for (int i = 0; i < 4; i++)
  {
    // update minimum if necessary
    if (ReadingsMatrix[i][0] < ReadingsMatrix[i][1] && ReadingsMatrix[i][0] != LimitMatrix[i][0])
      ReadingsMatrix[i][1] = ReadingsMatrix[i][0];

    // update maximum if necessary
    if (ReadingsMatrix[i][0] > ReadingsMatrix[i][2] && ReadingsMatrix[i][0] != LimitMatrix[i][0])
      ReadingsMatrix[i][2] = ReadingsMatrix[i][0];
  }
}

// Check each Sensor for new Defect, Min and Max Error; defect values are -255; return true if new error occurred
bool UpdateErrors()
{
  bool newError = false;

  for (int i = 0; i < 4; i++)
  {
    if (ReadingsMatrix[i][0] == LimitMatrix[i][0])
    {
      if (!ErrorMatrix[i][0])
      {
        ErrorMatrix[i][0] = true;
        newError = true;
      }
    }
    else
    {
      if (!ErrorMatrix[i][1] && ReadingsMatrix[i][0] <= LimitMatrix[i][1])
      {
        ErrorMatrix[i][1] = true;
        newError = true;
      }

      if (!ErrorMatrix[i][2] && ReadingsMatrix[i][0] >= LimitMatrix[i][2])
      {
        ErrorMatrix[i][2] = true;
        newError = true;
      }
    }
  }

  return newError;
}

// Set all errors to false
void ResetErrors()
{
  for (int i = 0; i < 4; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      ErrorMatrix[i][k] = false;
    }
  }
  warning = "Fehlermeldungen zurückgesetzt";
  ErrorState = 0;
  initReadings();
}

// Creates a Warning-Message with Information for all sensors
void CreateMessage()
{
  String Message = "Fehler protokolliert:\n";
  String Einheit;

  for (int i = 0; i < 4; i++)
  {
    if (ErrorMatrix[i][0] || ErrorMatrix[i][1] || ErrorMatrix[i][2])
    {
      if (i < 3)
      {
        Message += "\n\n" + SensorNames[i] + ":";
        Einheit = " °C";
      }
      else
      {
        Message += "\n\n" + SensorNames[i] + ":";
        Einheit = " bar";
      }

      if (ErrorMatrix[i][0])
        Message += "\n - defekt";
      if (ErrorMatrix[i][1])
        Message += "\n - Minimalwert mit " + String(ReadingsMatrix[i][1], 2) + Einheit + " unterschritten";
      if (ErrorMatrix[i][2])
        Message += "\n - Maximalwert mit " + String(ReadingsMatrix[i][2], 2) + Einheit + " überschritten";
    }
  }
  warning = Message;
  // onWarningChange(); //Aufruf nötig?
}

void onWarningChange()
{
  // Serial.println("Changed Text to");
  // Serial.println(warning);
  if (warning == "Reset")
    ResetErrors();
}

int8_t checkRotaryEncoder()
{
  // Reset the flag that brought us here (from ISR)
  rotaryEncoder = false;

  static uint8_t lrmem = 3;
  static int lrsum = 0;
  static int8_t TRANS[] = {0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0};

  // Read BOTH pin states to deterimine validity of rotation (ie not just switch bounce)
  int8_t l = digitalRead(pinRotaryCLK);
  int8_t r = digitalRead(pinRotaryDT);

  // Move previous value 2 bits to the left and add in our new values
  lrmem = ((lrmem & 0x03) << 2) + 2 * l + r;

  // Convert the bit pattern to a movement indicator (14 = impossible, ie switch bounce)
  lrsum += TRANS[lrmem];

  /* encoder not in the neutral (detent) state */
  if (lrsum % 4 != 0)
  {
    return 0;
  }

  /* encoder in the neutral state - clockwise rotation*/
  if (lrsum == 4)
  {
    lrsum = 0;
    return 1;
  }

  /* encoder in the neutral state - anti-clockwise rotation*/
  if (lrsum == -4)
  {
    lrsum = 0;
    return -1;
  }

  // An impossible rotation has been detected - ignore the movement
  lrsum = 0;
  return 0;
}

void setup()
{
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(pinRotaryCLK), rotary, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinRotaryDT), rotary, CHANGE);

  // initialize the temperature sensors
  for (int i = 0; i < 3; i++)
  {
    sensorsTemperature[i].begin();
    sensorsTemperature[i].setResolution(TEMPERATURE_PRECISION);
    sensorsTemperature[i].setWaitForConversion(false);
    sensorsTemperature[i].requestTemperatures();
  }

  // Wait for Temperature request
  delay(1000);

  // initialize the lcd display
  lcd.init();
  lcd.noBacklight();

  // set debounce time for the rotay switch
  // button.setDebounceTime(50);

  // set the status led pin as output
  pinMode(pinStatusLed, OUTPUT);
  pinMode(pinRotaryCLK, INPUT);
  pinMode(pinRotaryDT, INPUT);

  initReadings();
  prevMillisTemp = millis();

  // Setup Arduino IoT Cloud
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop()
{
  // Update sensor readings
  // ====================================================================================================
  if (millis() - prevMillisTemp >= 2500 && millis() - prevMillisTemp >= intervalTemp)
  {
    updateReadings();
    for (int i = 0; i < 3; i++)
      sensorsTemperature[i].requestTemperatures();
    prevMillisTemp = millis();

    // Upload readings to IoT Cloud
    temperatureIn = ReadingsMatrix[0][0];
    temperatureOut = ReadingsMatrix[1][0];
    temperaturePool = ReadingsMatrix[2][0];
    pressure = ReadingsMatrix[3][0];
    ArduinoCloud.update();
    // for (int i = 0; i < 4; i++)
    // {
    //   for (int k = 0; k < 3; k++)
    //   {
    //     Serial.print(ErrorMatrix[i][k]);
    //     Serial.print(" ");
    //   }
    //   Serial.println();
    // }
  }
  // ====================================================================================================

  // check for rotary encoder rotation
  // ====================================================================================================
  if (rotaryEncoder)
  {
    // Get the movement (if valid)
    int8_t rotationValue = checkRotaryEncoder();

    // If valid movement, do something
    if (rotationValue != 0)
    {
      rotationCounter += rotationValue;
      lcd.backlight();
      lcd.display();
      TurnOnDisplay = 1;
      lastMillisLCDOn = millis();
      prevMillisLcdRefresh = prevMillisLcdRefresh - intervalLcdRefresh;
    }
  }
  // ====================================================================================================

  button.loop();
  if (button.isPressed())
    ResetErrors();

  // Turn off display after 60s without input
  if (TurnOnDisplay == 1 && (millis() - lastMillisLCDOn >= 60000))
  {
    lcd.noBacklight();
    lcd.noDisplay();
    TurnOnDisplay = 0;
    rotationCounter = 0;
  }

  if (TurnOnDisplay && (millis() - prevMillisLcdRefresh >= intervalLcdRefresh))
  {
    if (rotationCounter <= 0)
      rotationCounter = 1;

    if (rotationCounter == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print(String("Druck: ") + String(ReadingsMatrix[3][0], 2) + String(" bar "));
      lcd.setCursor(17, 0);
      lcd.print("Now");

      lcd.setCursor(0, 1);
      lcd.print(String("Kaltwasser: ") + String(ReadingsMatrix[0][0], 1) + (char)223 + String("C "));

      lcd.setCursor(0, 2);
      lcd.print(String("Warmwasser: ") + String(ReadingsMatrix[1][0], 1) + (char)223 + String("C "));

      lcd.setCursor(0, 3);
      lcd.print(String("Poolwasser: ") + String(ReadingsMatrix[2][0], 1) + (char)223 + String("C "));

      prevMillisLcdRefresh = millis();
    }

    if (rotationCounter == 2)
    {
      lcd.setCursor(0, 0);
      lcd.print(String("Druck: ") + String(ReadingsMatrix[3][1], 2) + String(" bar "));
      lcd.setCursor(17, 0);
      lcd.print("Min");

      lcd.setCursor(0, 1);
      lcd.print(String("Kaltwasser: ") + String(ReadingsMatrix[0][1], 1) + (char)223 + String("C "));

      lcd.setCursor(0, 2);
      lcd.print(String("Warmwasser: ") + String(ReadingsMatrix[1][1], 1) + (char)223 + String("C "));

      lcd.setCursor(0, 3);
      lcd.print(String("Poolwasser: ") + String(ReadingsMatrix[2][1], 1) + (char)223 + String("C "));

      prevMillisLcdRefresh = millis();
    }
    if (rotationCounter == 3)
    {
      lcd.setCursor(0, 0);
      lcd.print(String("Druck: ") + String(ReadingsMatrix[3][2], 2) + String(" bar "));
      lcd.setCursor(17, 0);
      lcd.print("Max");
      lcd.setCursor(0, 1);
      lcd.print(String("Kaltwasser: ") + String(ReadingsMatrix[0][2], 1) + (char)223 + String("C "));

      lcd.setCursor(0, 2);
      lcd.print(String("Warmwasser: ") + String(ReadingsMatrix[1][2], 1) + (char)223 + String("C "));

      lcd.setCursor(0, 3);
      lcd.print(String("Poolwasser: ") + String(ReadingsMatrix[2][2], 1) + (char)223 + String("C "));

      prevMillisLcdRefresh = millis();
    }

    if (rotationCounter > 3)
      rotationCounter = 3;
  }

  if (UpdateErrors())
  {
    CreateMessage();
    prevMillisStatusOff -= intervalStatusOff;
    ErrorState = 1;
    digitalWrite(pinStatusLed, HIGH);
    // Serial.println("Error occurred");
  }

  if (!ErrorState)
  {
    if (WiFi.status() != WL_CONNECTED && intervalStatusOn != 500)
    {
      intervalStatusOn = 1000; // interval at which to blink (milliseconds)
      intervalStatusOff = 1000;
    }
    else if (!ArduinoCloud.connected() && intervalStatusOn != 250)
    {
      intervalStatusOn = 500; // interval at which to blink (milliseconds)
      intervalStatusOff = 1500;
    }
    else if (ArduinoCloud.connected() && intervalStatusOn != 250)
    {
      intervalStatusOn = 250; // interval at which to blink (milliseconds)
      intervalStatusOff = 9750;
    }

    if (millis() - prevMillisStatusOff >= intervalStatusOff && ledState == LOW)
    {
      prevMillisStatusOn = millis();
      ledState = HIGH;
    }

    if (millis() - prevMillisStatusOn >= intervalStatusOn && ledState == HIGH)
    {
      prevMillisStatusOff = millis();
      ledState = LOW;
    }

    digitalWrite(pinStatusLed, ledState);
  }
}