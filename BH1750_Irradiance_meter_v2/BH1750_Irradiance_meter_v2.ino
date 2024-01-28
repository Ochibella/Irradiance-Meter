//include libraries
#include "Wire.h" // library for I2C Communication 
#include "LiquidCrystal_I2C.h" // Library for I2C LCD

// Create an lcd object, default address is 0x27
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

//declare and initialize variables
#define buttonPin 2 //Reading select push button attached to this pin
#define timerPin 3 // // set the timer button pin

// Power
#define BH1750_POWER_DOWN 0x00  // No active state
#define BH1750_POWER_ON 0x01  // Waiting for measurement command
#define BH1750_RESET 0x07  // Reset data register value - not accepted in POWER_DOWN mode

// Measurement Mode
#define CONTINUOUS_HIGH_RES_MODE 0x10  // Measurement at 1 lux resolution. Measurement time is approx 120ms
#define CONTINUOUS_HIGH_RES_MODE_2 0x11  // Measurement at 0.5 lux resolution. Measurement time is approx 120ms
#define CONTINUOUS_LOW_RES_MODE 0x13  // Measurement at 4 lux resolution. Measurement time is approx 16ms
#define ONE_TIME_HIGH_RES_MODE 0x20  // Measurement at 1 lux resolution. Measurement time is approx 120ms
#define ONE_TIME_HIGH_RES_MODE_2 0x21  // Measurement at 0.5 lux resolution. Measurement time is approx 120ms
#define ONE_TIME_LOW_RES_MODE 0x23  // Measurement at 4 lux resolution. Measurement time is approx 16ms

// I2C Address
#define BH1750_1_ADDRESS 0x23  // Sensor  connected to GND
// Definition of Variable
unsigned int RawData;
double SensorValue;

bool buttonState = 0; //this variable tracks the state of the button, low if not pressed, high if pressed
int stateHolder = 0; //this variable holds the state of the button untill the next press (toggles on and off)

long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 250;    // the debounce time; increase if the output flickers
long timerResolution = 965; // timer resolution 
long previousTime = 0;
long resetTime = 0;

// Timer variables
int hours = 0; int minutes = 0; int seconds = 0;

void timer(){
   // Timer operation 
  if( (millis() - previousTime) > timerResolution){
    seconds += 1;
    previousTime = millis();
    if(seconds > 59){
      seconds = 0;
      minutes += 1;
    }
    if(minutes > 59){
      minutes = 0;
      hours += 1;
    }
  }
}

void setup() {
  Wire.begin();
  pinMode(buttonPin, INPUT);
  pinMode(timerPin, INPUT);
  lcd.begin();
  lcd.backlight();
  Serial.begin(9600); // Baud Rate
}

void loop() {
  timer(); // Perform timing operation 
  init_BH1750(BH1750_1_ADDRESS, CONTINUOUS_HIGH_RES_MODE);
  delay(120);
  RawData_BH1750(BH1750_1_ADDRESS);
  SensorValue = RawData / 1.2;  
  float irradianceValue = 0.0;

  // ++++++++++++++++++++ CALIBARTION OF THE SENSOR +++++++++++++++++++++++++++++++
  //float Irradiance = 5.0478*pow(2.718281828,0.0022*SensorValue);
  float cal_val1 = 0.06211*SensorValue + 2.6697-8.1; // low reading calibration
  float cal_val2 = -2.4765*pow(10,-7)*pow(SensorValue,3) + 6.1467*pow(10,-4)*pow(SensorValue,2) - 0.4153*SensorValue + 126.79; // high values calibrations 
  if(SensorValue > 720 ){ irradianceValue = cal_val2;} // low irradince 
  else if (SensorValue < 549){irradianceValue = cal_val1;} // high irradiance 
  else {irradianceValue = (cal_val1 + cal_val2)/2.0;} // get average where the high and low readings curves share a common region
  if(irradianceValue < 0){irradianceValue = 0.0;} // TO PREVENT READING NEGATIVE VALUES
  //float cal_val2_high =  0.07127*SensorValue+3.1525;  
  //if(Irradiance >= 277.4){Irradiance = 277.4;}

  int press = pressButton();
  bool resetState = digitalRead(timerPin);
  // timer reset function
  if( resetState and (millis() - resetTime) > 2000){
    hours = 0; minutes = 0; seconds = 0;
    resetTime = millis();
  }
  // else 
  if(press == 0){
    lcd.clear(); // clear any content on the lcd
    lcd.setCursor(0,0); // set the lcd cursor position 
    lcd.print("Irradiance");
    lcd.setCursor(0,1); // set lcd cursor position 
    lcd.print(irradianceValue); 
    //lcd.setCursor(5,1);
    lcd.print(" uW/cm2/nm");
    delay(250);
  }

  else if (press == 1){
    lcd.clear(); // clear any content on the lcd
    lcd.setCursor(0,0);
    lcd.print("TIME: H : M : S");
    lcd.setCursor(5,1);
    lcd.print(hours);
    lcd.setCursor(9,1);
    lcd.print(minutes);
    lcd.setCursor(13,1);
    lcd.print(seconds);
    delay(250);
  }
}

void init_BH1750(int ADDRESS, int MODE){
  //BH1750 Initializing & Reset
  Wire.beginTransmission(ADDRESS);
  Wire.write(MODE);  // PWR_MGMT_1 register
  Wire.endTransmission(true);
}

void RawData_BH1750(int ADDRESS){
  Wire.beginTransmission(ADDRESS);
  Wire.requestFrom(ADDRESS,(int)2,true);  // request the address register of the sensor
  RawData = Wire.read() << 8  | Wire.read();  // Read Raw Data of BH1750
  Wire.endTransmission(true);
}

int pressButton(){
//sample the state of the button - is it pressed or not?
  buttonState = digitalRead(buttonPin);
  //filter out any noise by setting a time buffer
  if ( (millis() - lastDebounceTime) > debounceDelay) {
    //if the button has been pressed, lets toggle the stateHolder
    if(stateHolder > 1){stateHolder = 0;}
    else{
      if (buttonState) {
      stateHolder += 1; //change the state of stateHolder
      lastDebounceTime = millis(); //set the current time
    }   
    }
  }//close if(time buffer)  
  return stateHolder;
}