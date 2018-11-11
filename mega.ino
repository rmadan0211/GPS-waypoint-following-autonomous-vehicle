#include <I2C_Anything.h>

#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"

Adafruit_GPS GPS(&Serial3);                   // define GPS object DO NOT USE Serial0
DFR_Key keypad;                               // define keypad object
Servo myServo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object

#define GPSECHO  false                        // echo GPS Sentence 0
#define Threshold 5                           // Threshold for Obstacle avoidance (number of obstacles)

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

// Global variables that change across functions
int STEERANGLE = 90;        // servo initial angle (range is 0:180)
float HEADING = 0;          // Heading in degree
int LidarRight;             // LIDAR left
int LidarLeft;              // LIDAR right
boolean usingInterrupt = false;
int carSpeedPin = 2;              // pin for DC motor (PWM for motor driver). don't use other pins....
float errorHeadingRef = 0;        // error
double lat;                     // GPS latitude in degree decimal * 100000   |     we multiply decimal degree by 100000 to convert it to meter  https://en.wikipedia.org/wiki/Decimal_degrees
double lon;                     // GPS latitude in degree decimal * 100000   |     0.00001 decimal degree is equal to 1.0247 m at 23 degree N/S
double latDestination = 33.425891 * 100000;       // define an initial reference Latitude of destination
double lonDestination =  -111.940458 * 100000;    // define an initial reference Longitude of destination
float Bearing = 0;                                  // initialize Bearing
int localkey = 0;                                   // var

float ldistance, langle, distance=0;

int la[10],ra[10];

boolean obdet=false;
int rightcluster=0,leftcluster=0;//rightfrontcluster=0, leftfrontcluster;
//farrightcluster: bw 45 to 90
//farleftcluster: bw 270 to 315
//rightfrontcluster: bw 0 to 45
//leftfrontcluster: bw 315 to 360

//long int time=millis();
//long int lasttime;
imu::Vector<3> euler;


SIGNAL(TIMER0_COMPA_vect) {       // don't change this !!
  char c = GPS.read();            // interrupt for reading GPS sentences char by char....
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {    // enable inttrupt for GPS, don't change this!!
  if (v) {
    OCR0A = 0xAF;               // Timer0 is already used for millis() - we'll just interrupt somewhere
    TIMSK0 |= _BV(OCIE0A);      // in the middle by Output Compare Register A (OCR0A) and "TIMER0_COMPA_vect" function is called
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A); // do not call the interrupt function COMPA anymore
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) {  // Timer interrupt for reading GPS DATA
  sei();        //   reset interrupt flag
  TCNT4  = 336; //   re-initialize timer value
  GPSRead();    //   read GPS data
}

void GPSRead() {
  // read GPS data
   if (GPS.newNMEAreceived()) {
    
    
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return; }
    lat=GPS.latitudeDegrees;
    lon=GPS.longitudeDegrees;
}


void setup() {/*
  myServo.attach(44);     // servo is connected to pin 44     (All pins are used by LCD except 2. Pin 2 is used for DC motor)
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)

  ///Setting the reference (Lat and Lon)///
  localkey = 0;
  while (localkey != 1) {
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to save dest.");
    delay(100);
  }
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // ask GPS to send RMC & GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate, don't use higher rates
  GPS.sendCommand(PGCMD_ANTENNA);               // notify if antenna is detected
  useInterrupt(true);                           // use interrupt for reading chars of GPS sentences (From Serial Port)



  delay(2000);
  
  GPSRead();
  latDestination = lat;     // saving the destiantion point
  lonDestination = lon;     // saving the destiantion point
  localkey = 0;
  while (localkey != 1) {
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to drive!");
    delay(100);
  }


 
  byte c_data[22] = {254, 255, 4, 0, 28, 0, 38, 255, 36, 2, 234, 1, 254, 255, 253, 255, 0, 0, 232, 3, 124, 3};               // Use your CALIBRATION DATA
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);

  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;               // initialize timer1
  TCCR1B = 0;               // initialize timer1
  TCNT1  = 59016;           // interrupt is generated every 0.1 second ( ISR(TIMER1_OVF_vect) is called)
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer compare interrupt

  TCCR4A = 0;               // initialize timer4
  TCCR4B = 0;               // initialize timer4
  TCNT4  = 336;             // interrupt is generated every 1 second  ( ISR(TIMER4_OVF_vect) is called)
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);   // enable timer compare interrupt
  interrupts();             // enable intrrupt flag again
*/

  myServo.attach(44);     // servo is connected to pin 44
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)
  Serial.begin(9600);     // serial for monitoring
  Serial.println("Orientation Sensor Calibration"); Serial.println("");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) { //if you want to calibrate using another mode, set it here. OPERATION_MODE_COMPASS for a precise tilt compensated compass (Section 3.3.2 / 3.3.3)
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  ///Setting the reference (Lat and Lon)///
  localkey = 0;
  while (localkey != 1) {    // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to save dest.");
    delay(100);               // delay to make display visible
  }
GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate // it's more stable than 10Hz
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);

  delay(2000);
  
  GPSRead();

  /*
  if (GPS.newNMEAreceived()) {
    
    
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return; }*/
    latDestination=lat;
    lonDestination=lon;

    




  
  //latDestination = lat;     // saving the destiantion point
  //lonDestination = lon;     // saving the destiantion point
  localkey = 0;
  while (localkey != 1) {   // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to drive!");
    delay(100);             // delay to make display visible
  }


  byte c_data[22] = {254, 255, 4, 0, 28, 0, 38, 255, 36, 2, 234, 1, 254, 255, 253, 255, 0, 0, 232, 3, 124, 3};
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);
  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);  // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);  // enable timer compare interrupt
  interrupts();



}



void ReadHeading()
{
  // calculate HEADING
  euler=bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  HEADING=euler.x();
}

void CalculateBearing() {
  // Calculate Bearing
  double dx,dy;
  float theta;
  dy=(latDestination)-(lat);
  dx=(lonDestination)-(lon);
  theta=(atan2(dy,dx)*(180/3.1415926));
  if(theta>90){Bearing=90-theta+360;}
  else
  {Bearing=90-theta;}
}

void CalculateSteer() {
  // Calculate Steer angle based on GPS data and IMU
  //if(rightcluster<30 && leftcluster<20){
  if(0<=(Bearing)<=90)//Bearing in 1st quadrant
  {
    if(Bearing>HEADING && 0<=(Bearing-HEADING)<90){STEERANGLE=90+(Bearing-HEADING);}//soft right
    else if(HEADING>Bearing && 0<=(HEADING-Bearing)<=90){STEERANGLE=90-(HEADING-Bearing);}//soft left
    else if(HEADING>Bearing && 270<=(HEADING-Bearing)){STEERANGLE=90+(360-(HEADING-Bearing));}//soft right
    else if(HEADING>Bearing && 90<(HEADING-Bearing)<=180){STEERANGLE=0;}//hard left
    else if(HEADING>Bearing && 180<(HEADING-Bearing)<270){STEERANGLE=180;}//hard left
    }
  else if(270<(Bearing)<360){//Bearing in 2nd quadrant
      if(Bearing>HEADING && 0<=(Bearing-HEADING)<=90){STEERANGLE=90+(Bearing-HEADING);}//soft right
      else if(Bearing>HEADING && 270<=(Bearing-HEADING)){STEERANGLE=90-(360-(Bearing-HEADING));}//soft left
      else if(HEADING>Bearing && 0<=(HEADING-Bearing)<=90){STEERANGLE=90-(HEADING-Bearing);}//soft left
      else if(Bearing>HEADING && 90<(Bearing-HEADING)<=180){STEERANGLE=180;}//hard right
      else if(Bearing>HEADING && 180<(Bearing-HEADING)<270){STEERANGLE=0;}//hard left
      
    }
  else if(180<(Bearing)<=270)//Bearing in third quadrant
  {
    if(Bearing>HEADING && 0<=(Bearing-HEADING)<=90){STEERANGLE=90+(Bearing-HEADING);}//soft right
    else if(HEADING>Bearing && 0<=(HEADING-Bearing)<=90){STEERANGLE=90-(HEADING-Bearing);}//soft left
    else if(Bearing>HEADING && 90<(Bearing-HEADING)<=180){STEERANGLE=180;}//hard right
    else if(Bearing>HEADING && 180<(Bearing-HEADING)<270){STEERANGLE=0;}//hard left
    else if(HEADING>Bearing && 90<(HEADING-Bearing)<=180){STEERANGLE=0;}//hard left
    
    }
  else if(90<(Bearing)<=180)//Bearing in the 4th quadrant
  {
    if(Bearing>HEADING && 0<=(Bearing-HEADING)<90){STEERANGLE=90+(Bearing-HEADING);}//soft right
    else if(HEADING>Bearing && 0<=(HEADING-Bearing)<=90){STEERANGLE=90-(HEADING-Bearing);}//soft left
    else if(HEADING>Bearing && 90<(HEADING-Bearing)<=180){STEERANGLE=0;}//hard left
    else if(HEADING>Bearing && 180<(HEADING-Bearing)<270){STEERANGLE=180;}//hard right
    else if(Bearing>HEADING && 90<(Bearing-HEADING)<=180){STEERANGLE=180;}//hard right
    }
  //}
  /*else if(rightcluster>leftcluster)
  {
    STEERANGLE=45;
    Serial.println("Turning left");}
  else{
    STEERANGLE=135;
    Serial.println("Turning right");
    }*/
}

void CalculateDistance(){
  double dx,dy;
  dy=(latDestination*100000)-(lat*100000);
  dx=(lonDestination*100000)-(lon*100000);
  distance=sqrt((dy*dy)+(dx*dx));
  
  }

void SetCarDirection() {    // Input: Lidar data
  // Set Steering angle,
  // If any obstacle is detected by Lidar, Ignore steering angle and turn left or right based on observation
  /*if(ldistance<300 && ldistance>0){
    if(langle>=0 && langle<=90)//obstacle on the right
    {myServo.write(0);
    //delay(1500);
    Serial.println("Turning left!");}
    else if(langle>=270 && langle<=360){myServo.write(180);
    //delay(1500);
    Serial.println("Turning right!");}
    }
    else{myServo.write(STEERANGLE);}*/

   /* if(farrightcluster==0 && farleftcluster==0 && leftfrontcluster==0 && rightfrontcluster==0){myServo.write(STEERANGLE);}
    else{
      if(farrightcluster>farleftcluster && farrightcluster>rightfrontcluster && farrightcluster>leftfrontcluster)//turn left
      {
        myServo.write(0);Serial.println("Turning 0");}//turn hard left
        else if(farleftcluster>farrightcluster && farleftcluster>rightfrontcluster && farleftcluster>leftfrontcluster){
          myServo.write(180);Serial.println("Turning 180");}//turn hard right
        else if(leftfrontcluster>farleftcluster && leftfrontcluster>farrightcluster && leftfrontcluster>rightfrontcluster){
          myServo.write(135);Serial.println("Turning 135");}//turn soft left
        else if(rightfrontcluster>farleftcluster && rightfrontcluster>farrightcluster && rightfrontcluster>leftfrontcluster){
          myServo.write(45);Serial.println("Turning 45");}
          }*/
          /*if(rightcluster<30 && leftcluster<30){myServo.write(STEERANGLE);}
          else if(rightcluster>leftcluster){
           
            
            lasttime=millis();
            //noInterrupts();
            myServo.write(0);
            analogWrite(carSpeedPin,20);
            Serial.println("Turning Left");
           // delay(2000);
            time=millis();
            while((time-lasttime)<=2000){time=millis();}
            //interrupts();
            myServo.write(90);
            }
          else if(leftcluster>rightcluster){
            lasttime=millis();
            // noInterrupts();
            myServo.write(180);
            analogWrite(carSpeedPin,20);
            Serial.println("Turning Right");
            //delay(2000);
            time=millis();
            while((time-lasttime)<=2000){time=millis();}
            //interrupts();
            myServo.write(90);}*/
            myServo.write(STEERANGLE);
            }
            


void SetCarSpeed() {  // Input: GPS data
  // set speed,
  // if destination is within 5 meters of current position, set speed to zero.
  if(distance<5){
    analogWrite(carSpeedPin,0);}
    else{
      analogWrite(carSpeedPin,20);
      
      }
}

void ReadLidar() {    // Output: Lidar Data
  // read Lidar Data from Nano Board (I2C)
  // you should request data from Nano and read the number of obstacle (within the range) on your rightside and leftside
  // Then, you can decide to either do nothing, turn left or turn right based on threshold. For instance, 0 = do nothing, 1= right and 2 = left
  //int x=1;
  //float d,a;
  int l,r;
  /*Wire.beginTransmission(8); 
  Wire.write(x);
   Wire.endTransmission();  */
   int i;
  //rightcluster=0;
  //leftcluster=0;
  //rightfrontcluster=0;
  //leftfrontcluster=0;
  for(i=0;i<10;i++){
  Wire.requestFrom(8, 8);
  
  //l=Wire.read();
  //l=map(l,0,255,0,6000);
  //r=Wire.read();
  //r=map(r,0,255,0,6000);
  //Serial.println("Distance:");
  I2C_readAnything(l);
  I2C_readAnything(r);
  //ldistance=d;
  //langle=a;
  la[i]=l;
  ra[i]=r;
  }
  int rsum=0,lsum=0;
  for(i=0;i<10;i++){
    lsum+=la[i];
    rsum+=ra[i];}
  
  leftcluster=lsum/10;
  rightcluster=rsum/10;

  /*Serial.println("Distance:");
  Serial.println(d);
  Serial.println("Angle:");
  Serial.println(a);
  //Serial.println("Angle:");*/
  //Serial.println(a);

  /*if(d<300 && d>50.000){
    if(a>=270 && a<=360){leftcluster++;}
    else if(a>=0 && a<=90){rightcluster++;}
    //else if(a>=0 && a<=45){rightfrontcluster++;}
    //else if(a>45 && a<=90){farrightcluster++;}
    }*/
  /*Serial.println("left:");
  Serial.println(l);
  Serial.println("right:");
  Serial.println(r);*/
  
  
  /*while(Wire.available()){
    r=Wire.read();
    Serial.println(r);}*/
  
  Serial.println("Rightcluster:");
  Serial.println(rightcluster);
  Serial.println("LeftCluster");
  Serial.println(leftcluster);
  /*Serial.println("Rightfrontcluster:");
  Serial.println(rightfrontcluster);
  Serial.println("LeftfrontCluster");
  Serial.println(leftfrontcluster);*/
  
  

}

ISR(TIMER1_OVF_vect) {        // function will be call every 0.1 seconds
  sei();                  // reset interrupt flag
  TCNT1  = 59016;
  ReadHeading();
  ReadLidar();
  CalculateBearing();
  CalculateSteer();
  CalculateDistance();
  SetCarDirection();
  SetCarSpeed();
}


void printHeadingOnLCD() {
  lcd.print("h=");
 lcd.print(HEADING);
 lcd.print("b=");
 lcd.print(Bearing);
 lcd.setCursor(0,1);
 lcd.print(STEERANGLE);
}

void printLocationOnLCD() {
  lcd.setCursor(0,0);
  lcd.print("lat=");
  lcd.print(latDestination,6);
  lcd.setCursor(0,1);
  lcd.print("lon=");
  lcd.print(lonDestination,6);

}

void printDistanceOnLCD() {
 lcd.setCursor(0,0);
 lcd.print("d=");
 lcd.print(distance);
}

void printObstacleOnLCD() {

}

void loop() {
  lcd.clear();      // clear lcd
  // Pring data to LCD in order to debug your program!
  printDistanceOnLCD();
  delay(1500);
  //printLocationOnLCD();
  //Serial.println("hi");
  printHeadingOnLCD();
  delay(1500);
}
