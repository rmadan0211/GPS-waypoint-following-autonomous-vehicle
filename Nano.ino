#include <I2C_Anything.h>

#include <RPLidar.h>

#include <Wire.h>
#include <RPLidar.h>
RPLidar lidar;          // define lidar as RPLIDAR Object
#define RPLIDAR_MOTOR 3 // motor pin for lidar speed control

float distance;
float angle;
float dr=0,ar=0;
int lob=0,rob=0;

void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT); // set lidar speed control pin as output
  lidar.begin(Serial);            // begin communication with lidar
  Wire.begin(8);                  // join i2c bus with address 8 as SLAVE
  Wire.onRequest(requestEvent);   // call "requestEvent" function when receives a request
  //Wire.onReceive(receiveEvent);   // call "receiveEvent" function when receives a byte
  lidar.startScan();                                    // start scan                           // Dont change this......
  analogWrite(RPLIDAR_MOTOR, 255); 
}
int left = 0;                     // variable for detected points on left hand side
int right = 0;                    // variable for detected points on right hand side
unsigned long time = millis();    // time variable for reseting variables
unsigned long lasttime=0,t=0;

//int c1;                           // variable for received integer

/*void receiveEvent(int bytes)
{
  char c;
       // read the received byte as integer. This indicates what data to send back when master is requesting data
       while (1 < Wire.available()) { // loop through all but the last
     c = Wire.read(); // receive byte as a character
    //Serial.print(c);         // print the character
  }
  c1 = Wire.read();    // receive byte as an integer
 
    
    
}*/

void requestEvent() 
{
   // receive message byte as a character
   // if master's request is right side data, ("1"), send back the right side data
   // if master's request is left side data, ("2"), send back the left side data

  
    
   // Wire.write(left);
    //Wire.write(" Distance=");
   //Wire.write(int(dr));
   //Wire.write(int(ar));
    //Wire.write(" Angle= ");
   // Wire.write(int(angle));
    I2C_writeAnything(left); 
   I2C_writeAnything(right);

   
    
    
   /*else if(c1==2){*/
    
  // Wire.write(right);
   
     
  
}

void loop() 
{
  if (IS_OK(lidar.waitPoint())) { // if lidar is working properly (waiting time less than timeout)
    // read angle and distance of the obstacle
    // filter data (keep only the data in desired range and with desired angle) 
    // COUNT the number of obstacles on LEF and RIGHT side
    // reset obstacle variables every 1 second
     lob=0;rob=0;
     //lasttime=millis();
     //for(int i=0;i<100;i++){
     while(millis()%2!=0){
     distance = lidar.getCurrentPoint().distance; //distance value in mm unit
     angle    = lidar.getCurrentPoint().angle; //anglue value in degree
     //Wire.write(3);
      
    
   //if((angle>0 && angle<90)||(angle>270 && angle<360)){dr=distance;ar=angle;}
  
    if(distance<=300 && distance>50.0){
      if(angle>0 && angle<90){
          rob++;
        }
      else if(angle>270 && angle<360){lob++;}  
      }
      //t=millis();
      //left=lob;
      //right=rob;
      //}
      left=lob;
      right=rob;
  
    
    /* if(distance<300 && distance>0.000){
    if(a>=270 && a<=360){lob++;}
    else if(a>=0 && a<=90){rob++;}*/
     
     
    /*if(distance<150){
     if(angle>0 && angle<90){left++;}
     else if(angle>270 && angle<360){right++;}*/
     
     

    
  } else {                                                  // if lidar is not responding           // Dont change this......
  // Wire.write(4);
    analogWrite(RPLIDAR_MOTOR, 0);                          //stop the rplidar motor                // Dont change this......
    rplidar_response_device_info_t info;                    // try to detect RPLIDAR...             // Dont change this......
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {            // if detected,                         // Dont change this......
      lidar.startScan();                                    // start scan                           // Dont change this......
      analogWrite(RPLIDAR_MOTOR, 255);                      // start motor rotating at max speed    // Dont change this......
      delay(1000);                                                                                  // Dont change this......
    }
  }
}


