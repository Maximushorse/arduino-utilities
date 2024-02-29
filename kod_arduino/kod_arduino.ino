#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Program variables ----------------------------------------------------------
//const long interval = 5;
//const int pin = A1;
//const int numReadings = 10;
unsigned long previousMillis = 0;
const int numberOfADConversions = 200;
int conversionNumber = 0;

const int currentSensorPin = A0;
int* currentValues[numberOfADConversions];
int currentSensor = 0;
float currentSensorV = 0;
float currentSensorI = 0;

const int voltageSensorPin = A1;
int* voltageValues[numberOfADConversions];
int voltageSensor = 0;
float voltageSensorV = 0;

const int pressureSensorPin = A2;
int* pressureValues[numberOfADConversions];
int pressureSensor = 0;
float pressureSensorV = 0;
float pressureSensorI = 0;
float pressureSensorBar = 0;

float power = 0;

const int button1Pin = 10;
const int button2Pin = 9;

const int purgeValvePin = 11;
const int valvePin = 12;

bool valve = 0;
bool purgeValve = 0;

const int fanPin = 3;
const int fanReadPin = A3;
int fanPWM = 0;
int fanReadPWM = 0;

// Serial data variables ------------------------------------------------------
//Incoming Serial Data Array
const byte kNumberOfChannelsFromExcel = 6; 

// Comma delimiter to separate consecutive data if using more than 1 sensor
const char kDelimiter = ',';    
// Interval between serial writes
const int kSerialInterval = 500;   
// Timestamp to track serial interval
unsigned long serialPreviousTime; 

char* arr[kNumberOfChannelsFromExcel];

// SETUP ----------------------------------------------------------------------
void setup() {
  // Initialize Serial Communication
  Serial.begin(9600); 
 
  pinMode(button1Pin, INPUT_PULLUP);  
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(valvePin, OUTPUT);  
  pinMode(purgeValvePin, OUTPUT);

  pinMode(fanReadPin, INPUT);
  pinMode(fanPin, OUTPUT);

  digitalWrite(purgeValvePin, HIGH);
  digitalWrite(valvePin, LOW);

  lcd.init();                      // initialize the lcd 
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2,2);
  lcd.print("Hydrogreen 2024");
  lcd.setCursor(3,1);
  lcd.print("Fuel Cell Test"); 
  delay(750);

  //test uśredniania
  //averager.begin();
}

// START OF MAIN LOOP --------------------------------------------------------- 
void loop()
{
   //Valves
  valveControl();
  // Gather and process sensor data
  processSensors();
  

  //test uśredniania
  /*
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    float averageVoltage = averager.getAverageVoltage();
  }
*/


  // Read Excel variables from serial port (Data Streamer)
  processIncomingSerial();

  // Process and send data to Excel via serial port (Data Streamer)
  processOutgoingSerial();

  digitalWrite(fanPin, fanPWM);
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// SENSOR INPUT CODE-----------------------------------------------------------
void processSensors() 
{
  if (conversionNumber < numberOfADConversions) {
    currentValues[conversionNumber] = analogRead(currentSensorPin);
    voltageValues[conversionNumber] = analogRead(voltageSensorPin);
    pressureValues[conversionNumber] = analogRead(pressureSensorPin);
    conversionNumber++;
  } 
  else 
  {
    conversionNumber = 0;

    // Calculate mean values
    for (int i = 0; i < numberOfADConversions; i++) {
      currentSensor += currentValues[i];
      voltageSensor += voltageValues[i];
      pressureSensor += pressureValues[i];
    }
    currentSensor /= numberOfADConversions;
    voltageSensor /= numberOfADConversions;
    pressureSensor /= numberOfADConversions;
    
    // Current sensor
    currentSensor = analogRead(currentSensorPin);
    currentSensorV = mapf((float)currentSensor, 0.0, 1023.0, 0.5, 4.5);   //currentSensorV = currentSensor * 5.0/1023;
    currentSensorI = mapf(currentSensorV, 0.5, 4.5, -30.0, 30.0);

    //Voltage 
    voltageSensor = analogRead(voltageSensorPin);
    voltageSensorV = mapf((float)voltageSensor, 0.0, 1023.0, 0.5, 4.5) * 20;

    //Power
    power = currentSensorI * voltageSensorV;

    //Presure
    pressureSensor = analogRead(pressureSensorPin);
    pressureSensorV = mapf((float)currentSensor, 0.0, 1023.0, 0.5, 4.5);
    pressureSensorI = mapf(pressureSensorV, 0.5, 4.5, 4, 20);
    pressureSensorBar = mapf(pressureSensorI, 4, 20, 0, 4);  //4-20mA 0-4bar

    //Fan PWM
    fanReadPWM = analogRead(fanReadPin);
    fanPWM = map(fanReadPWM, 0, 1023, 0, 255);
  }
}

// Add any specialized methods and processing code below


// OUTGOING SERIAL DATA PROCESSING CODE----------------------------------------
void sendDataToSerial()
{
  // Send data out separated by a comma (kDelimiter)
  // Repeat next 2 lines of code for each variable sent:

  Serial.print(currentSensorI); Serial.print(kDelimiter);
  Serial.print(voltageSensorV); Serial.print(kDelimiter);
  Serial.print(currentSensor); Serial.print(kDelimiter);
  Serial.print(power); Serial.print(kDelimiter);
  Serial.print(pressureSensorBar); Serial.print(kDelimiter);  
  Serial.print(pressureSensor); Serial.print(kDelimiter);
  Serial.print(valve); Serial.print(kDelimiter);
  Serial.print(purgeValve); Serial.print(kDelimiter);
  Serial.print(fanPWM); Serial.print(kDelimiter);
  Serial.print(fanReadPWM);

  Serial.println(); // Add final line ending character only once
}

//-----------------------------------------------------------------------------
// DO NOT EDIT ANYTHING BELOW THIS LINE
//-----------------------------------------------------------------------------

// OUTGOING SERIAL DATA PROCESSING CODE----------------------------------------
void processOutgoingSerial()
{
   // Enter into this only when serial interval has elapsed
  if((millis() - serialPreviousTime) > kSerialInterval) 
  {
    // Reset serial interval timestamp
    serialPreviousTime = millis(); 
    sendDataToSerial(); 
    
    //LCD
    lcdDisplay();
  }

}

// INCOMING SERIAL DATA PROCESSING CODE----------------------------------------
void processIncomingSerial()
{
  if(Serial.available()){
    parseData(GetSerialData());
  }
}

// Gathers bytes from serial port to build inputString
char* GetSerialData()
{
  static char inputString[64]; // Create a char array to store incoming data
  memset(inputString, 0, sizeof(inputString)); // Clear the memory from a pervious reading
  while (Serial.available()){
    Serial.readBytesUntil('\n', inputString, 64); //Read every byte in Serial buffer until line end or 64 bytes
  }
  return inputString;
}

// Seperate the data at each delimeter
void parseData(char data[])
{
    char *token = strtok(data, ","); // Find the first delimeter and return the token before it
    int index = 0; // Index to track storage in the array
    while (token != NULL){ // Char* strings terminate w/ a Null character. We'll keep running the command until we hit it
      arr[index] = token; // Assign the token to an array
      token = strtok(NULL, ","); // Conintue to the next delimeter
      index++; // incremenet index to store next value
    }
}

void valveControl()
{
  purgeValve = digitalRead(button1Pin);
  valve = digitalRead(button2Pin);    //---------------dodać wyłącznik bezpieczeństwa

  digitalWrite(purgeValvePin, purgeValve);
  digitalWrite(valvePin, valve);
}

void lcdDisplay()
{
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(currentSensorI); lcd.print(" A");
  lcd.setCursor(10,0); lcd.print("Valve: "); lcd.print(valve);
  lcd.setCursor(0,1); lcd.print(voltageSensorV); lcd.print(" V");
  lcd.setCursor(10,1); lcd.print("Purge: "); lcd.print(purgeValve);
  lcd.setCursor(0,2); lcd.print(power); lcd.print(" W");
  lcd.setCursor(0,3); lcd.print(pressureSensorBar); lcd.print(" bar");
 //lcd.setCursor(10,4); lcd.print("PWM: "); lcd.print(fanReadPWM); 

  lcd.setCursor(10,2); lcd.print(pressureSensorI); lcd.print(" mA");
  lcd.setCursor(10,3); lcd.print(pressureSensorV); lcd.print(" V"); 
}