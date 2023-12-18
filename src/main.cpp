#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include "thingProperties.h"
#include <ezButton.h>
#include <ESP32Encoder.h>
// #include "myfunctions.h"

/* =================== Version Changelog ===========================================================

// Version 1.1
Seperate data pins for each temperature sensor
Implementing LCD screen
blinking status led
rotary encoder button interaction

// ==================== Settings =================================================================== */

#define TEMPERATURE_PRECISION 10 // Set float precision e.g. 10 == 0.25

const int pinStatusLed = 27; // the number of the LED pin
const int pinRotarySW = 33;
const int pinRotaryCLK = 26;
const int pinRotaryDT = 25;

// Array of OneWires for TemperatureIn, TemperatureOut, TemperaturePool
OneWire Wires[] = {OneWire(18), OneWire(19), OneWire(23)};

// Array of DallasTemperature sensors for TemperatureIn, TemperatureOut, TemperaturePool
DallasTemperature sensorsTemperature[] = {DallasTemperature(&Wires[0]), DallasTemperature(&Wires[1]), DallasTemperature(&Wires[2])};

const int sensorPressure = 35;

float ReadingsMatrix[4][3];         // measurements for each sensor Current/Min/Max
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

int ledState = LOW;
unsigned long prevMillisStatusOn = 0; // will store last time LED was updated
unsigned long prevMillisStatusOff = 0;
unsigned long prevMillisLcdRefresh = 0; // will store last time LED was updated

const long intervalStatusOn = 250; // interval at which to blink (milliseconds)
const long intervalStatusOff = 10000;
const long intervalLcdRefresh = 1000;

// Menu skipping
volatile int buttonState = 0;
volatile int displayOn = 0;

volatile long unsigned int lastMillis = 0;
volatile long unsigned int currentMillis = 0;
volatile long unsigned int lastMillisLCDOn = 0;
volatile int displayActive = 0;

// Threshold matrix with layout {defect, min, max}:   {{-255, 0, 100}, ...}
float LimitMatrix[4][3] = {
    {-255, 0, 40},
    {-255, 0, 85},
    {-255, 0, 85},
    {-255, 1.5, 2.5}};

// Names for each sensor; used for warning messages:  {"Sensor1", ...}
String SensorNames[4] = {
    "Kaltwasser",
    "Heizwasser",
    "Poolwasser",
    "Allgemein"};

bool ErrorMatrix[4][3]; // {defect, min, max} for each sensor

ezButton button(pinRotarySW);

// =================================================================================================

void setup()
{
  button.setDebounceTime(50);

  // initialize temperature sensors
  for (int i = 0; i < 3; i++)
    sensorsTemperature[i].begin();
  // initialize the lcd
  lcd.init();
  lcd.noBacklight();

  pinMode(pinStatusLed, OUTPUT);
  pinMode(pinRotarySW, INPUT);

  Serial.begin(115200);
  delay(1000);
  // Setup Arduino IoT Cloud
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  for (int i = 0; i < 3; i++)
    sensorsTemperature[i].setResolution(TEMPERATURE_PRECISION);
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
  if (analogReading <= 0)
    return -255; // set pressure to -255 if device is defect
  float convpressure = roundFloat((analogReading - 380) / 620, 2);
  return convpressure;
}

// Update all sensor readings including minimum and maximum values
void updateReadings()
{
  for (int i = 0; i < 3; i++)
  {
    ReadingsMatrix[i][0] = readTemperatureSensor(&sensorsTemperature[i]);
  }

  ReadingsMatrix[3][0] = readPressureSensor(sensorPressure);

  for (int i = 0; i < 4; i++)
  {
    if (ReadingsMatrix[i][0] < ReadingsMatrix[i][1])
      ReadingsMatrix[i][1] = ReadingsMatrix[i][0]; // update minimum if necessary
    if (ReadingsMatrix[i][0] > ReadingsMatrix[i][2])
      ReadingsMatrix[i][2] = ReadingsMatrix[i][0]; // update maximum if necessary
  }
}

// Check each Sensor for new Defect, Min and Max Error; defect values are -255 and below
bool CheckErrors()
{
  bool result = false;

  for (int i = 0; i < 4; i++)
  {
    if (!ErrorMatrix[i][0] && ReadingsMatrix[i][0] == LimitMatrix[i][0])
    { // Check for new defect
      ErrorMatrix[i][0] = true;
      result = true;
    }
    if (!ErrorMatrix[i][1] && ReadingsMatrix[i][0] <= LimitMatrix[i][1] && ReadingsMatrix[i][0] > LimitMatrix[i][0])
    {
      ErrorMatrix[i][1] = true;
      result = true;
    }
    if (!ErrorMatrix[i][2] && ReadingsMatrix[i][0] >= LimitMatrix[i][2])
    { // Check for reaching maximum value
      ErrorMatrix[i][2] = true;
      result = true;
    }
  }
  // if (result)
  // state = false;
  return result;
}

/*
// Creates a Warning-Message with Information for all sensors
void CreateMessage()
{
  String Message = "Fehler protokolliert:\n";
  String Einheiten[] = {" °C", " bar"};
  String Einheit;

  for (int i = 0; i < TEMPSENSOR_COUNT + PRESSURESENSOR_COUNT; i++)
  {
    if (ErrorMatrix[i][0] || ErrorMatrix[i][1] || ErrorMatrix[i][2])
    {
      if (i < TEMPSENSOR_COUNT)
      {
        // Message += "Temperatursensor-" + (i + 1);
        Message += "Temperatursensor-" + SensorNames[i] + ":";
        Einheit = Einheiten[0];
      }
      else
      {
        Message += "Drucksensor-" + SensorNames[i] + ":";
        Einheit = Einheiten[0];
      }

      if (ErrorMatrix[i][0])
        Message += "\n - defekt";
      if (ErrorMatrix[i][1])
        Message += "\n - Minimalwert (" + String(LimitMatrix[i][1], 1) + Einheit + ") unterschritten";
      if (ErrorMatrix[i][2])
        Message += "\n - Maximalwert (" + String(LimitMatrix[i][1], 1) + Einheit + ") überschritten";

      Message += "\n\n";
    }
  }
  warning = Message;
  // onWarningChange(); //Aufruf nötig?
}

// Resets warning
void ResetErrors()
{
  for (int i = 0; i < TEMPSENSOR_COUNT + PRESSURESENSOR_COUNT; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      ErrorMatrix[i][k] = false;
    }
  }
  state = true;
  warning = "Fehlermeldungen zurückgesetzt";
} */

void onWarningChange()
{
  /* if (warning == "Reset")
    ResetErrors(); */
}

void ReadingsInit()
{
}

void loop()
{
  button.loop();
  ArduinoCloud.update();
  for (int i = 0; i < 3; i++)
    sensorsTemperature[i].requestTemperatures();

  delay(100);

  updateReadings();

  // Upload readings to IoT Cloud ------------------------------------------------------------------------------
  temperatureIn = ReadingsMatrix[0][0];
  temperatureOut = ReadingsMatrix[1][0];
  temperaturePool = ReadingsMatrix[2][0];
  pressure = ReadingsMatrix[3][0];

  // Show data on lcd ------------------------------------------------------------------------------------------
  currentMillis = millis();

  if (button.isPressed())
  {
    if (buttonState > 2)
      buttonState = 0;
    buttonState++;
    Serial.println(buttonState);
    lcd.backlight();
    lcd.display();
    // lcd.clear();
    displayActive = 1;
    displayOn = 0;
    lastMillisLCDOn = currentMillis;
  }

  if (displayActive == 1 && (currentMillis - lastMillisLCDOn >= 60000))
  {
    Serial.println("Turned off Display");
    lcd.noBacklight();
    lcd.noDisplay();
    displayActive = 0;
    buttonState = 0;
  }

  if (displayActive)
  {
    if (currentMillis - prevMillisLcdRefresh >= intervalLcdRefresh)
    {
      if (buttonState == 1)
      {
        // lcd.clear();
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

        prevMillisLcdRefresh = currentMillis;
      }

      if (buttonState == 2)
      {
        // lcd.clear();
        Serial.println("Page2");
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

        prevMillisLcdRefresh = currentMillis;
      }
      if (buttonState == 3)
      {
        // lcd.clear();
        Serial.println("Page3");
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

        prevMillisLcdRefresh = currentMillis;
      }
    }
  }

  if (currentMillis - prevMillisStatusOff >= intervalStatusOff && ledState == LOW)
  {
    // save the last time you blinked the LED
    prevMillisStatusOn = currentMillis;
    ledState = HIGH;
  }

  if (currentMillis - prevMillisStatusOn >= intervalStatusOn && ledState == HIGH)
  {
    // save the last time you blinked the LED
    prevMillisStatusOff = currentMillis;
    ledState = LOW;
  }

  digitalWrite(pinStatusLed, ledState);

  /* if (CheckErrors())
    CreateMessage();
  Serial.print(warning); */
}