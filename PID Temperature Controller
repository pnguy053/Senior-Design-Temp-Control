#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
RF24 radio(7,8);
const uint64_t sendAddress = 0xF0F0F0F0E1LL;
const uint64_t receiveAddress = 0xF0F0F0F0E1FF; 

const int n = 101;
double PT100[n] = {10000,10039,10078,10117,10156,10195,10234,10273,10312,10351   //PT100 Datatable (can be replace with any)
,10390,10429,10468,10507,10546,10585,10624,10663,10702,10740
,10779,10818,10857,10896,10935,10973,11012,11051,11090,11128
,11167,11206,11245,11283,11322,11361,11399,11438,11477,11515
,11554,11593,11631,11670,11708,11747,11785,11824,11862,11901
,11940,11978,12016,12055,12093,12132,12170,12209,12247,12286
,12324,12362,12401,12439,12477,12517,12555,12593,12632,12670
,12708,12746,12785,12823,12861,12899,12938,12976,13014,13052
,13090,13128,13167,13205,13243,13281,13319,13357,13395,13433
,13471,13509,13547,13585,13623,13661,13699,13737,13775,13813
,13851};

  //PID constants
  //int kp = 17;   int ki = .075;   int kd = 3.9;
  int kp = 17;   int ki = 1;   int kd = 0;
  int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

  //Pins

  int PWM_pin = 3;
  double kTemptot = 0;
  
  //Variables
  float temperature_read = 0;
  float set_temperature = 21;
  double set_temperature_k = 0;
  double set_temperature_k2 = 0;
  double temptot = 0;
  float PID_error = 0;
  float previous_error = 0;
  float elapsedTime, Time, timePrev;
  int PID_value = 0;
  int m = 12.6;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(sendAddress);
  radio.openReadingPipe(1, receiveAddress);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  pinMode(3,OUTPUT);
  //TCCR1B = TCCR1B & B11111000 | B00000101;;
  analogReference(INTERNAL);
  Time = millis(); 
  }


void loop() {
  radio.stopListening();
  kTemptot = temptot + 273;
  radio.write(&kTemptot, sizeof(kTemptot));
  delay(30);
  radio.startListening();
  radio.read(&set_temperature_k, sizeof(set_temperature_k));
  if(set_temperature_k != set_temperature_k2 && set_temperature_k != 0){
    set_temperature_k2 = set_temperature_k;
  }
  set_temperature = set_temperature_k2-273;

  float I = analogRead(A0);
  float V = analogRead(A1);

  float curavg = 0;
  float volavg = 0;

  //Averaging of ADC values
  
  for (int i = 0; i < 100; i++){
    curavg += analogRead(A0);
    volavg += analogRead(A1);
  }

  
  I = curavg/100;
  V = volavg/100;

  //ADC reading of 1k and PT100
  
  Serial.print("ADC 1k = ");
  Serial.println(I);

  Serial.print("ADC PT100 = ");
  Serial.println(V);
  
  I = atv(I);
  V = atv(V);

  I = I/(12);      //voltage in divided by gain of op amp 1
  Serial.print("Voltage to opamp1 = "); 
  Serial.print(I*1000); //conversion of voltage to mV
  Serial.println("mV");
  I = I/984;         //voltage divided by resistance == current going to pt100
  
  Serial.print("Current to PT100 = "); 
  Serial.print(I*1000*1000);  //conversion of current to uA
  Serial.println("uA");

  V = V/(92);     //voltage divided by gain of op amp 2 == voltage going through pt100
  
  Serial.print("  Voltage of PT100 = "); 
  Serial.print(V*1000); //conversion of voltage to mV
  Serial.println("mV");

  
  double r;

  r = V/I;           //voltage divided by current = resistance of pt100
  Serial.print("Resistance of PT100= ");
 Serial.println((r+m)/100);

  //binary sorting algorithm

  int h; //higher
  int l; //lower
  int mid; //middle
  bool flag = 0; 
  int res = (r+m)*100;

 
  if (res >= PT100[n/2]){
    h = n;
    l = n/2;
  }
  else{
    h = n/2;
    l = 0;
  }
  
  mid = h/2;

  //SPECIAL CASE: Perfect resistance measurement point
  while (flag == 0){
      if (res == PT100[mid]){
    flag = 1;
  }
    if (res > PT100[mid]){
      h = h;
      l = mid;
      mid = (h+l)/2;
      
    }
    else if (res < PT100[mid]){
      h = mid;
      l = 0;
      mid = (h+l)/2;
    }
    if (PT100[l+1] == PT100[h]){
      flag = 1;
    }
  }
  /*
  Serial.print("High = ");
  Serial.println(h);
  Serial.print(   "Low = ");
  Serial.println(l);
  */
  
  double slope;
  double temp;

  slope = (h-l)/(PT100[h]-PT100[l]);
  temp = (slope*(res-PT100[l]))+l;

  

  for (int i = 0; i<1000; i++){
    temptot += temp;
  }

  temptot = temptot/1000;

Serial.print("Temperature(C) = ");
  Serial.println(temptot);

Serial.print("Temperature(F) = ");
Serial.println((temptot*(1.8))+32);

  //PID
  // First we read the real value of temperature
  temperature_read = temptot;
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
 Serial.print("PID Error = ");
 Serial.println(PID_error);
 //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if(-10 < PID_error < 10)
  {
    PID_i = PID_i + (ki * PID_error);
  }

   //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

    //We define PWM range between 0 and 255
  if(PID_value < -255)
  {    PID_value = -255;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  
  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;
  
  Serial.println("-----------");
  delay(1000);
}

float atv(float adc){

  float voltage;
  voltage = adc*((1.1)/1023);
  return voltage;
}
