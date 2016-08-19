//Program reads data from different sensors: 
// water, vibration, sound, UV Index, accelerometer, Compass, Digital Light, gas, dust, light
// and prints results out

//Connect with SD card
#include <SD.h>

//---for sd card using fOpen()---
#include <stdio.h>
#include <stdlib.h>
 
#define SOUND_SENSOR A0
#define WATER_SENSOR 8
#define THRESHOLD_VALUE 400//The threshold to turn the led on 400.00*5/1024 = 1.95v

void pins_init();

//libraries for the GPS 
//#include <SoftwareSerial.h>
//#include "TinyGPS++.h"
//SoftwareSerial SoftSerial(2, 3);
//TinyGPSPlus gps;

// buffer array for data recieve over myFile port
unsigned char buffer[256]; 
// counter for buffer array
int count=0;      

//libraries for the Digitial Accelometer Sensor
#include "MMA7660.h"
MMA7660 accelemeter;

//Reference the I2C Library for Compass
#include <Wire.h>

//Reference the HMC5883L Compass Library
//#include <HMC5883L.h>

//library for Digital Light sensor
#include <Wire.h>
#include <Digital_Light_TSL2561.h>

//library for light sensor
#include <math.h>
float Rsensor; //Resistance of sensor in K

// Dust sensor variables
int pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 2000;//sampe 30s&nbsp;;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

//Tempreture variables
int a;
float temperature;
int B=3975;    //B value of the thermistor
float resistance;

//Compass Variables
#define HMC5883_WriteAddress 0x1E // i.e 0x3C >> 1
#define HMC5883_ModeRegisterAddress 0x02
#define HMC5883_ContinuousModeCommand 0x00
#define HMC5883_DataOutputXMSBAddress 0x03

int regb=0x01;
int regbdata=0x40;
int outputData[6];

//-------------------------------------SET UP-------------------------------------------------------
void setup()
{
//    SoftSerial.begin(9600);
    Serial.begin(9600);
    
    accelemeter.init();
    pins_init();

   //------Digital Light Init---------
   Wire.begin();
   TSL2561.init();

   // Dust sensor init
    pinMode(8,INPUT);
    starttime = millis();//get the current time;

   //GPS sesnor init
   pinMode(10, OUTPUT);
   
}

 //--------------------------------------LOOP-------------------------------------------------------
void loop()
{
   FILE * fp;

   fp = fopen("/media/sdcard/sensor-data.txt", "a");
   
     
    /*  ROW ID  */
    fprintf(fp, "a");
    fprintf(fp, ", ");
    
     
   //--------------------------Sound sensor-----------------
    int sensorValue = analogRead(SOUND_SENSOR);//use A0 to read the electrical signal
    //fprintf(fp,"Sound: ");
    fprintf(fp, sensorValue);
    fprintf(fp, ", ");
    
    
    //------------------------Water Sensor--------------------
    //fprintf(fp,"Water: ");
      if(isExposedToWater())
         fprintf(fp,fp,"True");
      else 
        fprintf(fp,"False");
     fprintf(fp,", ");
    
    //--------------------------Vibration Sensor-------------------
    int sensorValue1 = analogRead(A1);
    //fprintf(fp,"Vibration: ");
    fprintf(fp,sensorValue1);
    //delay(1000);
    //if (sensorValue1 ==1023)
    //  fprintf(fp,"High");
    //else 
    //  fprintf(fp,"Low");
    fprintf(fp,", ");    
    
    //---------------------------UV Index---------------------
    
    int sensorValue3;
    long  sum=0;
    for(int i=0;i<1024;i++)// accumulate readings for 1024 times
     {  
        sensorValue3=analogRead(A0);
        sum=sensorValue3+sum;
  //      delay(2);
     }   
     long meanVal = sum/1024;  // get mean value
     //fprintf(fp,"UV index :");
     fprintf(fp,(meanVal/4.3-83)/21);//get a detailed calculating expression for UV index in schematic files. 
     fprintf(fp,", ");
     //delay(20); 
  
      
    //-----------------------------Digital Light---------------------------
    //fprintf(fp,"Digital Light value: ");
    fprintf(fp,TSL2561.readVisibleLux());
    fprintf(fp,", ");
   
    //------------------------------Gas Sensor--------------------------------
    float sensor_volt; 
    float sensorValue4;
   
    sensorValue4 = analogRead(A3);
    sensor_volt = sensorValue4/1024*5.0;
    
    //fprintf(fp,"Gas sensor_volt = ");
    fprintf(fp,sensor_volt);
    
    //fprintf(fp,"V");
    fprintf(fp,", ");
  
    //---------------------------Dust Sensor --------------------------------
     duration = pulseIn(pin, LOW);
     lowpulseoccupancy = lowpulseoccupancy+duration;
   
      if ((millis()-starttime) >= sampletime_ms)//if the sampel time = = 30s
      {
        ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=&gt;100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
        //fprintf(fp,"Dust concentration = ");
        fprintf(fp,concentration);
        //fprintf(fp," pcs/0.01cf");
        fprintf(fp,", ");
        lowpulseoccupancy = 0;
        starttime = millis();
      } else {
        fprintf(fp,concentration);
        fprintf(fp,", ");
      }
  
    //-----------------------------Tempreture Sensor --------------------------------
      a=analogRead(1);
      resistance=(float)(1023-a)*10000/a; //get the resistance of the sensor;
      temperature=1/(log(resistance/10000)/B+1/298.15)-273.15;//convert to temperature via datasheet&nbsp;;
      //delay(1000);
      //fprintf(fp,"Current temperature is ");
      fprintf(fp, temperature);
      fprintf(fp, "\r\n");
      
  
    //-------------------------Accelometer Sensor-------------------(newline)
      
    int8_t x;
    int8_t y;
    int8_t z;
    float ax,ay,az;
    accelemeter.getXYZ(&x,&y,&z);
    //fprintf(fp,"\n"); //start a new line
  
    /*  ROW ID  */
    fprintf(fp,"b");
    fprintf(fp,", ");
      
    //fprintf(fp,"Accelometer: ");  
    //fprintf(fp,"x = ");
    fprintf(fp,x); 
    fprintf(fp,", ");
    //fprintf(fp,"y = ");
    fprintf(fp,y); 
    fprintf(fp,", ");  
    //fprintf(fp,"z = ");
    fprintf(fp,z);
    fprintf(fp,", ");
    
    accelemeter.getAcceleration(&ax,&ay,&az);
    //fprintf(fp," Accleration of X/Y/Z: ");
    fprintf(fp,ax);
    fprintf(fp,", ");
    //fprintf(fp," g ");
    fprintf(fp,ay);
    fprintf(fp,", ");
    //fprintf(fp," g ");
    fprintf(az);
    
    
    fprintf(fp, "\r\n");
     
    
    //----------------------------Digital Compass (I2C)----------------------(newline)
      /*  ROW ID  */
      fprintf(fp,"c");
      fprintf(fp,", ");
    
     int i_,x_,y_,z_;
     double angle;
     Wire.beginTransmission(HMC5883_WriteAddress);
     Wire.write(regb);
     Wire.write(regbdata);
     Wire.endTransmission();
     //delay(100);
     Wire.beginTransmission(HMC5883_WriteAddress); // Initiate a transmission with HMC5883 (Write address).
     Wire.write(HMC5883_ModeRegisterAddress); //Place the Mode Register Address in send-buffer.
     Wire.write(HMC5883_ContinuousModeCommand); //Place the command for Continuous operation Mode in send-buffer.
     Wire.endTransmission(); //Send the send-buffer to HMC5883 and end the I2C transmission.
     //delay(100);
     Wire.beginTransmission(HMC5883_WriteAddress); // Initiate a transmission with HMC5883 (Write address).
     Wire.requestFrom(HMC5883_WriteAddress,6); // Request 6 bytes of data from the address specified.
     //delay(100);
     
     //Read the value of magnetic components X,Y and Z Future Electronics Egypt Ltd. (Arduino Egypt).
     if(6 <= Wire.available()) // If the number of bytes available for reading be <=6.
     {
       for(i_=0;i_<6;i_++)
       {
          outputData[i_]=Wire.read(); //Store the data in outputData buffer
       }
     }
     x_=outputData[0] << 8 | outputData[1]; //Combine MSB and LSB of X Data output register
     z_=outputData[2] << 8 | outputData[3]; //Combine MSB and LSB of Z Data output register
     y_=outputData[4] << 8 | outputData[5]; //Combine MSB and LSB of Y Data output register
     angle= atan2((double)y_,(double)x_) * (180 / 3.14159265) + 180; // angle in degrees
  
    //Print the approximate direction
     //fprintf(fp,"You are heading ");
     if((angle < 22.5) || (angle > 337.5 ))
       fprintf(fp,"South, ");
       
     if((angle > 22.5) && (angle < 67.5 ))
       fprintf(fp,"South-West, ");
       
     if((angle > 67.5) && (angle < 112.5 ))
       fprintf(fp,"West, ");
       
     if((angle > 112.5) && (angle < 157.5 ))
       fprintf(fp,"North-West, ");
       
     if((angle > 157.5) && (angle < 202.5 ))
       fprintf(fp,"North, ");
       
     if((angle > 202.5) && (angle < 247.5 ))
       fprintf(fp,"NorthEast, ");
       
     if((angle > 247.5) && (angle < 292.5 ))
       fprintf(fp,"East, ");
       
     if((angle > 292.5) && (angle < 337.5 ))
       fprintf(fp,"SouthEast, ");
       
     //fprintf(fp,": Angle between X-axis and the South direction ");
      
     if((0 < angle) && (angle < 180) )
     {
     angle=angle;
     }
     else
     {
     angle=360-angle;
     }
     
     fprintf(angle,2);
     //fprintfln(" Deg");
     fprintf(fp,"\r\n");

  
   
    
 
   
   /*********/
   delay(1000);
   /*********/

  
} // end of Void Loop
 
void pins_init()
{
    pinMode(WATER_SENSOR, INPUT);
    pinMode(SOUND_SENSOR, INPUT);
}


/*Function: Determine whether the sensor is exposed to the water    */
/*Parameter:-void                                 */
/*Return: -boolean,if it is exposed to the water,it will return true. */
boolean isExposedToWater()
{
  if(digitalRead(WATER_SENSOR) == LOW)
    return true;
  else return false;
}


