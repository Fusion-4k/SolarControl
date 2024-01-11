#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include "thingProperties.h"
#include <ezButton.h>
#include <ESP32Encoder.h>
#include <lcdUpdate.h>

/* =================== Version Changelog ==============================================================

    Version 1.1
      Improved formatting text for the lcd
      Status LED indicates cloud connection
      Improved rotary encoder algorithm
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

// pin for the status led
const int pinStatusLed = 27;

// =================================================================================================

// Array of OneWire-objects for TemperatureIn, TemperatureOut, TemperaturePool
OneWire Wires[] = {OneWire(sensorTempIn), OneWire(sensorTempOut), OneWire(sensorTempPool)};

// Array of DallasTemperature-objects for sensors TemperatureIn, TemperatureOut, TemperaturePool
DallasTemperature sensorsTemperature[] = {DallasTemperature(&Wires[0]), DallasTemperature(&Wires[1]), DallasTemperature(&Wires[2])};

// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Create button object
ezButton button(pinRotarySW);

// Measurements for each sensor Current/Min/Max
float ReadingsMatrix[4][3];

// Threshold matrix with layout {min, max}:   {{0, 100}, ...}
float LimitMatrix[4][3] = {
    {-255, 0, 40},
    {-255, 0, 85},
    {-255, 0, 95},
    {-255, 0.5, 3.5}};

// Name for each sensor (used to creating warning messages)
String SensorNames[4] = {
    "Kaltwassersensor",
    "Heizwassersensor",
    "Poolwassersensor",
    "Drucksensor"};

// Contains errors for each sensor defect/min/max
bool ErrorMatrix[4][3]; //

// Used for formatting the lcd text
char LCDMessage[21];

// Time required to convert temperature
long int intervalTemp = 750 / (1 << (12 - TEMPERATURE_PRECISION));

// Timer variables
unsigned long lastMillisStatusLedOn = 0;
unsigned long lastMillisStatusLedOff = 0;
unsigned long lastMillisLcdRefresh = 0;
unsigned long lastMillisTempReq = 0;
long unsigned int lastMillisLCDOn = 0;

long intervalStatusLedOn = 250;
long intervalStatusLedOff = 10000;
long intervalLcdRefresh = 2500;

// State variables
bool lcdOn = false;
int lcdPageNumber = 0;
bool ErrorState = false;
int ledState = LOW;

// Set to true if any data pin of the rotary encoder caused an Interrupt
volatile bool rotaryEncoder = false;

// Interrupt routine sets flag to true when a rotation is detected
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
    return -255;
  float convPressure = roundFloat((analogReading - 400) / 510, 2);
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
void CreateWarningMessage()
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

// Update Status-LED
void updateStatusLed()
{
  if (ErrorState && ledState != HIGH)
  {
    ledState = HIGH;
  }
  else
  {
    if (WiFi.status() != WL_CONNECTED && intervalStatusLedOn != 500)
    {
      intervalStatusLedOn = 1000; // interval at which to blink (milliseconds)
      intervalStatusLedOff = 1000;
    }
    else if (!ArduinoCloud.connected() && intervalStatusLedOn != 250)
    {
      intervalStatusLedOn = 500; // interval at which to blink (milliseconds)
      intervalStatusLedOff = 1500;
    }
    else if (ArduinoCloud.connected() && intervalStatusLedOn != 250)
    {
      intervalStatusLedOn = 250; // interval at which to blink (milliseconds)
      intervalStatusLedOff = 9750;
    }

    if (millis() - lastMillisStatusLedOff >= intervalStatusLedOff && ledState == LOW)
    {
      lastMillisStatusLedOn = millis();
      ledState = HIGH;
    }

    if (millis() - lastMillisStatusLedOn >= intervalStatusLedOn && ledState == HIGH)
    {
      lastMillisStatusLedOff = millis();
      ledState = LOW;
    }
  }
  digitalWrite(pinStatusLed, ledState);
}

void setup()
{
  Serial.begin(115200);

  pinMode(pinStatusLed, OUTPUT);
  pinMode(pinRotaryCLK, INPUT);
  pinMode(pinRotaryDT, INPUT);

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

  // Wait for temperature conversion
  delay(1000);

  // initialize the lcd display
  lcd.init();
  lcd.noBacklight();

  // set debounce time for the rotay switch button
  button.setDebounceTime(50);

  // initialize ReadingsMatrix
  initReadings();

  lastMillisTempReq = millis();

  // Setup Arduino IoT Cloud
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop()
{
  ArduinoCloud.update();

  // Update sensor readings
  if (millis() - lastMillisTempReq >= intervalTemp)
  {
    updateReadings();
    for (int i = 0; i < 3; i++)
      sensorsTemperature[i].requestTemperatures();
    lastMillisTempReq = millis();

    // Update cloud variables for ArduinoIoTCloud
    temperatureIn = ReadingsMatrix[0][0];
    temperatureOut = ReadingsMatrix[1][0];
    temperaturePool = ReadingsMatrix[2][0];
    pressure = ReadingsMatrix[3][0];
  }

  // check for rotary encoder rotation
  if (rotaryEncoder)
  {
    // Get the movement (if valid)
    int8_t rotationValue = checkRotaryEncoder();

    // If valid movement, enable display
    if (rotationValue != 0)
    {
      lcdPageNumber += rotationValue;
      lcdOn = true;
      lcd.backlight();
      lcd.display();
      lastMillisLCDOn = millis();
      lastMillisLcdRefresh = lastMillisLcdRefresh - intervalLcdRefresh;
    }
  }

  // Check for rotary encoder button press
  button.loop();
  if (button.isPressed())
    ResetErrors();

  // Turn off display after 60s without input
  if (lcdOn == 1 && (millis() - lastMillisLCDOn >= 60000))
  {
    lcd.noBacklight();
    lcd.noDisplay();
    lcdOn = 0;
    lcdPageNumber = 0;
  }

  if (lcdOn && (millis() - lastMillisLcdRefresh >= intervalLcdRefresh))
  {
    if (lcdPageNumber <= 0)
      lcdPageNumber = 1;

    if (lcdPageNumber == 1)
    {
      lcd.setCursor(0, 0);
      if (ErrorMatrix[3][0])
        sprintf(LCDMessage, "Druck: defekt    NOW");
      else
        sprintf(LCDMessage, "Druck: %5.2f bar NOW", ReadingsMatrix[3][0]);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 1);
      if (ErrorMatrix[0][0])
        sprintf(LCDMessage, "Kaltwasser: defekt  ");
      else
        sprintf(LCDMessage, "Kaltwasser: % 5.1f %cC", ReadingsMatrix[0][0], (char)223);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 2);
      if (ErrorMatrix[1][0])
        sprintf(LCDMessage, "Warmwasser: defekt  ");
      else
        sprintf(LCDMessage, "Warmwasser: % 5.1f %cC", ReadingsMatrix[1][0], (char)223);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 3);
      if (ErrorMatrix[2][0])
        sprintf(LCDMessage, "Poolwasser: defekt  ");
      else
        sprintf(LCDMessage, "Poolwasser: % 5.1f %cC", ReadingsMatrix[2][0], (char)223);
      lcd.print(LCDMessage);
    }

    if (lcdPageNumber == 2)
    {
      lcd.setCursor(0, 0);
      sprintf(LCDMessage, "Druck: %5.2f bar MIN", ReadingsMatrix[3][1]);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 1);
      sprintf(LCDMessage, "Kaltwasser: % 5.1f %cC", ReadingsMatrix[0][1], (char)223);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 2);
      sprintf(LCDMessage, "Warmwasser: % 5.1f %cC", ReadingsMatrix[1][1], (char)223);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 3);
      sprintf(LCDMessage, "Poolwasser: % 5.1f %cC", ReadingsMatrix[2][1], (char)223);
      lcd.print(LCDMessage);
    }

    if (lcdPageNumber == 3)
    {
      lcd.setCursor(0, 0);
      sprintf(LCDMessage, "Druck: %5.2f bar MAX", ReadingsMatrix[3][2]);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 1);
      sprintf(LCDMessage, "Kaltwasser: % 5.1f %cC", ReadingsMatrix[0][2], (char)223);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 2);
      sprintf(LCDMessage, "Warmwasser: % 5.1f %cC", ReadingsMatrix[1][2], (char)223);
      lcd.print(LCDMessage);

      lcd.setCursor(0, 3);
      sprintf(LCDMessage, "Poolwasser: % 5.1f %cC", ReadingsMatrix[2][2], (char)223);
      lcd.print(LCDMessage);
    }

    if (lcdPageNumber > 3)
      lcdPageNumber = 3;

    lastMillisLcdRefresh = millis();
  }

  if (UpdateErrors())
  {
    CreateWarningMessage();
    lastMillisStatusLedOff -= intervalStatusLedOff;
    ErrorState = 1;
  }

  // Update Status-LED
  if (ErrorState && ledState != HIGH)
  {
    ledState = HIGH;
  }
  else
  {
    if (WiFi.status() != WL_CONNECTED && intervalStatusLedOn != 500)
    {
      intervalStatusLedOn = 1000; // interval at which to blink (milliseconds)
      intervalStatusLedOff = 1000;
    }
    else if (!ArduinoCloud.connected() && intervalStatusLedOn != 250)
    {
      intervalStatusLedOn = 500; // interval at which to blink (milliseconds)
      intervalStatusLedOff = 1500;
    }
    else if (ArduinoCloud.connected() && intervalStatusLedOn != 250)
    {
      intervalStatusLedOn = 250; // interval at which to blink (milliseconds)
      intervalStatusLedOff = 9750;
    }

    if (millis() - lastMillisStatusLedOff >= intervalStatusLedOff && ledState == LOW)
    {
      lastMillisStatusLedOn = millis();
      ledState = HIGH;
    }

    if (millis() - lastMillisStatusLedOn >= intervalStatusLedOn && ledState == HIGH)
    {
      lastMillisStatusLedOff = millis();
      ledState = LOW;
    }
  }
  digitalWrite(pinStatusLed, ledState);
}