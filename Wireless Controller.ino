#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
RF24 radio(7, 8); // CE, CSN Pins
const uint64_t sendAddress = 0xF0F0F0F0E1FF;
const uint64_t receiveAddress = 0xF0F0F0F0E1LL;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
const int incTemp = 2;
const int decTemp = 3;
const int enTemp = 4;
const int enUnit = 5;
int incTempState = 0;
int decTempState = 0;
int enTempState = 0;
int unitState = 0;
double currTemp = 273;
double currTemp2 = 273;
double cTemp = currTemp;
double desTemp = 273;
double dTemp = desTemp;
int cUnit = 0;
String tUnit [3] = {"C   ", "K   ", "F   "};

void setup() {
  Serial.begin(9600);
  lcd.begin(20,4);
  pinMode(incTemp, INPUT);
  pinMode(decTemp, INPUT);
  pinMode(enTemp, INPUT);
  pinMode(enUnit, INPUT);
  radio.begin();
  radio.openWritingPipe(sendAddress); // 00002
  radio.openReadingPipe(1, receiveAddress); // 00001
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
}

void loop() {
  incTempState = digitalRead(incTemp);
  decTempState = digitalRead(decTemp);
  enTempState = digitalRead(enTemp);
  unitState = digitalRead(enUnit);

  double dcTemp = desTemp - 273;
  double dkTemp = desTemp;
  double dfTemp = ((desTemp - 273) * 9 / 5) + 32;
  double ccTemp = currTemp2 - 273;
  double ckTemp = currTemp2;
  double cfTemp = ((currTemp2 - 273) * 9 / 5) + 32;

  radio.stopListening();
  radio.write(&desTemp, sizeof(desTemp));
  delay(30);
  radio.startListening();
  radio.read(&currTemp, sizeof(currTemp));
  if(currTemp2 != currTemp && currTemp != 0){
    currTemp2 = currTemp;
  }
  if(cUnit > 2){
      cUnit = 0;
    }
  if(cUnit == 0){
    cTemp = ccTemp;
  }
  else if(cUnit == 1){
    cTemp = ckTemp;
  }
  else if(cUnit == 2){
    cTemp = cfTemp;
  }

  Serial.print("desTemp: ");
   Serial.println(desTemp);
  Serial.print("currTemp: ");
  Serial.println(currTemp);
    
  lcd.setCursor(0, 0);
  lcd.print("Curr. Temp: ");
  lcd.print(cTemp);
  lcd.print(tUnit[cUnit]);
  lcd.setCursor(0, 1);
  lcd.print("Set Temp: ");
  lcd.print(dTemp);
  lcd.print(tUnit[cUnit]);
  lcd.setCursor(0, 2);
  lcd.setCursor(0, 3);
  lcd.print("+Temp -Temp En. Unit");

  if(incTempState == HIGH){
    dTemp = dTemp + 0.5;
  }
  if(decTempState == HIGH){
    dTemp = dTemp - 0.5;
  }
  if(enTempState == HIGH){
    if(cUnit == 0){
      desTemp = dTemp;
      desTemp = desTemp + 273;
    }
    else if(cUnit == 1){
      desTemp = dTemp;
    }
    else{
      desTemp = dTemp;
      desTemp = ((dTemp - 32) * 5 / 9) + 273.15;
    }
  }
  if(unitState == HIGH){
    cUnit++;
    if(cUnit > 2){
      cUnit = 0;
    }
    if(cUnit == 0){
      cTemp = ccTemp;
      dTemp = dcTemp;
    }
    else if(cUnit == 1){
      cTemp = ckTemp;
      dTemp = dkTemp;
    }
    else if(cUnit == 2){
      cTemp = cfTemp;
      dTemp = dfTemp;
    }
    delay(50);
  }
  delay(100);
}

