/*  Author: Jan Schöfer


    Changelog:

    Version 0.00a: 26.04.2018
	  -tested and working
      -initial version
      -derrived from Version EnvironemtSensorBME280 0.21a of 26.04.2018

    Setup:
    Arduino UNO
    LCD Keypad Shield and 
    BME680 connected to +5V (red), NGD (black), A4 (SDA, green), A5 (SCL, yellow), 
*/


#define VERSION "0.00a"

#define SERIAL_INTERVAL 1000 //interval for sending serial [ms]
#define LCD_INTERVAL 1000 //interval for refreshing LCD [ms] 
#define BME_INTERVAL 2000 //interval for reading BME280 (may collide with sensor standby)
#define BLINK_INTERVAL 1000 //interval for blinking (LED, LCD heart)
//#define NO_VALUE_FLOAT 0.00f //dummy value for error data
//#define SENSOR_ERROR_STRING "/SENSOR ERROR/" //string to print on error													 

//for Sensor BME680
#include <Wire.h> //I2C library
#include <Adafruit_Sensor.h> //not really needed here (Adafruit_BME280.h needs this)
#include <Adafruit_BME680.h>
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; //I2C

float humidity = 0.0;
float temperature = 0.0;
float pressure = 0.0;
float gas = 0.0;
byte sensorError = 0;

//for LCD Keypad Shield
#include <Wire.h> //I2C library
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // pins for LCD panel

int btnID     = 0;
int btnAnalogValue  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//for time handling
long t_lastDisplay = 0;
long t_lastSerial = 0;
long t_lastSensor = 0;
long t_lastBlink = 0;

bool blinkState = false;

//additional LCD characters
uint8_t heart[8] = {0x00, 0x0A, 0x1F, 0x1F, 0x0E, 0x04, 0x00, 0x00}; //heart symbol
uint8_t degree[8] = {0x02, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00}; //degree symbol
uint8_t arrowup[8] = {0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04, 0x00}; //arrow-up symbol
uint8_t ohm[8] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x1B, 0x00}; //ohm symbol

// read buttons
int read_buttons()
{
  btnAnalogValue = analogRead(0);      // read the value from analog button pin
  // analog button values (right:0; up:132; down:306; left:480; select:721; none:1023)

  if (btnAnalogValue > 1000) return btnNONE;
  if (btnAnalogValue < 50)   return btnRIGHT;
  if (btnAnalogValue < 250)  return btnUP;
  if (btnAnalogValue < 400)  return btnDOWN;
  if (btnAnalogValue < 650)  return btnLEFT;
  if (btnAnalogValue < 850)  return btnSELECT;

  return btnNONE;  // when all others fail, return this...
}

void displayHandler()
{
  if (t_lastDisplay < (millis() - LCD_INTERVAL)) {
    t_lastDisplay = millis();
    btnID = read_buttons();         // read the buttons
    switch (btnID) {               // depending on which button was pushed, we perform an action
        
      case btnUP:
        {
        //show up time
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Up-Time:");
        lcd.setCursor(0, 1);
        lcd.print(String(millis()/1000) + "s ~ ");
        lcd.print(String(float(millis())/3600000.0, 3));
        lcd.print("h");
        break;
      }
      case btnRIGHT:
      {
        //break;
      }
      case btnLEFT:
      {
        //break;
      }
      case btnDOWN:
      {
        //break;
      }
      case btnSELECT:
      {
        //break;
      }
      case btnNONE:
      {
        // standard state
        lcd.clear();
        lcd.setCursor(0, 0);

        if (!blinkState) {
          lcd.write(byte(0)); //heart
          //lcd.print("X ");
        } else {
          lcd.print("  "); //delete heart
        }
        lcd.setCursor(1, 0);
        lcd.print(" " + String(temperature));
        lcd.write(byte(1)); //degree symbol
        lcd.print("C " + String(humidity) + "%");

        // lcd.setCursor(row, line);
        lcd.setCursor(0, 1); // move to the begining of the second LCD line
        lcd.print(String(pressure, 0) + " " + String(gas) + "k");
        lcd.write(byte(3)); //ohm symbol
      }
      break;
    }
  }
}

void serialWriteHandler()
{
  if (t_lastSerial < (millis() - SERIAL_INTERVAL)) {
    t_lastSerial = millis();
    Serial.println(String(millis()/1000) + "; " + String(temperature) + "; " + String(humidity) + "; " + String(pressure) + "; " + String(gas) + "; ");
  }
}

void serialReadHandler()
{
  if (Serial.available() > 0) {
    String tmp = Serial.readString();
  }
}

void setup()
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // initialise pins
  pinMode(LED_BUILTIN, OUTPUT);

  // initialise LCD
  lcd.createChar(0, heart);
  lcd.createChar(1, degree);
  lcd.createChar(2, arrowup);
  lcd.createChar(3, ohm);
  lcd.begin(16, 2);              // initialise LCD
  lcd.clear();
  
  // print welcome message
  lcd.setCursor(0, 0);           // set cursor to beginning
  lcd.print("Welcome!");         // print a simple message
  lcd.setCursor(0, 1);           // cursor to second line
  lcd.print("EnviroSens ");      // print Version
  lcd.print(VERSION);

  delay(2000); // wait so someone can read the display

  // initialse sensor
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    lcd.setCursor(0, 1);
    lcd.print("No BME280 found...");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop()
{
  //blink LED and Display
  if (t_lastBlink < (millis() - BLINK_INTERVAL)) {
    t_lastBlink = millis();
    if (!blinkState) {
      digitalWrite(LED_BUILTIN, HIGH);
      blinkState = true;
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      blinkState = false;
    }
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // read BME680
  if (t_lastSensor < (millis() - BME_INTERVAL)) {
    t_lastSensor = millis();
    sensorError = 0;
    if (! bme.performReading()) {
      sensorError = 1;
    } else {
    humidity = bme.readHumidity();
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0f;
    gas = bme.gas_resistance / 1000.0f;
    }
  }

  // serialReadHandler();
  serialWriteHandler();
  displayHandler();
  


/*
    switch (lcd_key)               // depending on which button was pushed, we perform an action
    {
      case btnRIGHT:
        {
          lcd.print("RIGHT ");
          break;
        }
      case btnLEFT:
        {
          lcd.print("LEFT   ");
          break;
        }
      case btnUP:
        {
          lcd.print("UP    ");
          break;
        }
      case btnDOWN:
        {
          lcd.print("DOWN  ");
          break;
        }
      case btnSELECT:
        {
          lcd.print("SELECT");
          break;
        }
      case btnNONE:
        {
          lcd.print("NONE  ");
          break;
        }
    }
    */
}
