
int tdssensorValue = 0;
float tdsValue = 0;
float tdsVoltage = 0;
//#define SERIAL Serial
#define sensorPin A1


#define VOLTAGE 3.37   //vcc voltage(unit: V)
#define LED 13         //operating instructions
#define ArrayLenth  40 //times of collection
#define orpPin A4       //orp meter output,connect to Arduino controller ADC pin
#define calPin A5       //orp cal control pin, get a offset by set it to low


#define SensorPin A2            //pH meter Analog output to Arduino Analog Input 0
#define pHOffset 29.52 //41.02740741      //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define uart  Serial
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;
#include <LiquidCrystal.h>          //the liquid crystal library contains commands for printing to the display

LiquidCrystal lcd(13, 12, 11, 10, 9, 8);   // tell the RedBoard what pins are connected to the display



double orpValue;
// double offset=0.0;
int offset=0;
bool is_calibrated = false;
int wait_count = 5;
int orpArray[ArrayLenth];
int orpArrayIndex=0;

float phval;
int orpval;
float turbididyval;
int tdsval;

float time;

double avergearray(int* arr, int number);

int i;
// for the data logging shield, we use digital pin 10 for the SD cs line
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
RTC_DS1307 rtc;
String filename;
String tfilename;

File myFile;

void setup(void) {  
  Serial.begin(9600);
   Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  i = 0;
  while(!filename) {
      tfilename = "data";
      tfilename = tfilename + i;
      tfilename = tfilename + ".csv";
    if(!SD.exists(tfilename)) {
      filename = tfilename;
    }
    i++;
  }
  myFile = SD.open(filename, FILE_WRITE);
 
  myFile.println("time since start,ph,orp,turbididy,tds");
  myFile.close();
  pinMode(LED,OUTPUT);
  pinMode(calPin,OUTPUT);
  // digitalWrite(calPin, LOW);
  digitalWrite(calPin, HIGH);
  pinMode(LED, OUTPUT);
  uart.begin(9600);
  uart.println("pH meter experiment!");    //Test the uart monitor
  lcd.begin(16, 2);                 //tell the lcd library that we are using a display that is 16 characters wide and 2 characters high
  lcd.clear();        
  //lcd.setCursor(0, 0);              //set the cursor to the 0,0 position (top left corner)
  //lcd.print("pH:   ORP:");       //print hello, world! starting at that position
}

void loop(void) {
 
  static unsigned long samplingTime = millis();
  static unsigned long pHprintTime = millis();
  static float pHValue, pHvoltage;
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    pHvoltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = -19.18518519 * pHvoltage + pHOffset;
    samplingTime = millis();
  }
  if(millis() >= pHprintTime)   //Every 800 milliseconds, print a numerical
  {
        //uart.print("Voltage:");
    //uart.print(voltage, 2);
    uart.print("    pH value: ");
    uart.println(pHValue, 2);
    digitalWrite(LED, digitalRead(LED) ^ 1);
    phval = pHValue;
    pHprintTime = millis();
   
 
 
 
 
 
   
  //int sensorValue = analogRead(A0);// read the input on analog pin 0:
  //float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //float percent = voltage * 20;
  // WRITE DATA TO LCD
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //lcd.setCursor(0, 1);
  //lcd.print(percent);
 
  static unsigned long orpTimer=millis();   //analog sampling interval
  static unsigned long printTime=millis();
  if(millis() >= orpTimer)
  {
    orpTimer=millis()+20;
    orpArray[orpArrayIndex++]=analogRead(orpPin);    //read an analog value every 20ms
    if (orpArrayIndex==ArrayLenth) {
      orpArrayIndex=0;
    }  
    orpValue=((30*(double)VOLTAGE*1000)-(75*avergearray(orpArray, ArrayLenth)*VOLTAGE*1000/1024))/75-offset;
  //}
  //if(millis() >= printTime)   //Every 800 milliseconds, print a numerical
  //{
    if(!is_calibrated) {
      if(wait_count==0){
        offset += (int)orpValue;
        is_calibrated = true;
        digitalWrite(calPin, LOW);
        Serial.print("offset: ");
        Serial.print((int)offset);
        Serial.println(" mV");
      }
      wait_count--;
    }
    else {
      Serial.print("ORP: ");
      Serial.print((int)orpValue);
     
      orpval = ((int)orpValue);
     
      Serial.println(" mV");
      digitalWrite(LED,1-digitalRead(LED)); // convert the state of the LED indicator
    //lcd.setCursor(0, 1);              //move the cursor to the first space of the bottom row
    //lcd.print((int)orpValue);       //print the number of seconds that have passed since the last reset
    }
    printTime=millis()+800;  
  }
}
{
    //Elio_Code
 
  int turbsensorValue = analogRead(A0);// read the input on analog pin 0:
  float turbvoltage = turbsensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  Serial.println(turbvoltage); // print out the value you read:
 
  turbididyval = turbvoltage;
 
  tdssensorValue = analogRead(sensorPin);
  tdsVoltage = tdssensorValue*5/1024.0; //Convert analog reading to Voltage
  tdsValue=(133.42/tdsVoltage*tdsVoltage*tdsVoltage - 255.86*tdsVoltage*tdsVoltage + 857.39*tdsVoltage)*0.5; //Convert voltage value to TDS value
    Serial.print("TDS Value = ");
    Serial.print(tdsValue);
    Serial.println(" ppm");
    tdsval = tdsValue;
  }
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(phval);
  lcd.setCursor(6, 1);
  lcd.print(orpval);
  lcd.setCursor(0,0);
  lcd.print(turbididyval);
  lcd.setCursor(6,0);
  lcd.print(tdsval);
  time = millis();
 
  myFile = SD.open("data.txt", FILE_WRITE);
if(myFile) {
    myFile.print(time);
    myFile.print(",");
    myFile.print(phval);
    myFile.print(",");
    myFile.print(orpval);
    myFile.print(",");
    myFile.print(turbididyval);
    myFile.print(",");
    myFile.println(tdsval);
    myFile.close();
}
  delay(1000);
 
}

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    printf("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
 
 
 
}
