//----------------------------
//| Push-To telescope system |
//----------------------------
#include <Arduino.h>
#include "wiring_private.h" // Required for pinPeripheral
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <string.h>
#include <EasyNextionLibrary.h>



#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// --- NEW SERIAL FOR NEXTION ON PINS 5 & 6 ---
// Create a new Hardware Serial port (SERCOM0) on Pins 5 (RX) and 6 (TX)
Uart nextionSerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Interrupt handler for the new Serial port
void SERCOM0_Handler() {
  nextionSerial.IrqHandler();
}

EasyNex myNex(nextionSerial); // Tell Nextion library to use our new port

static const uint32_t GPSBaud = 9600;  

TinyGPSPlus gps;
Adafruit_SSD1306 d(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

//location variables
double lat;
double lng;
//target variables
double targetRA;
double targetDec;
double targetAz;
double targetAlt;
String targetName;
// time variables
double JD;
double LST;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(GPSBaud);

  // --- INITIALIZE NEW NEXTION SERIAL ---
  nextionSerial.begin(115200); // Most Nextion displays default to 9600
  
  // Assign Pins 5 and 6 to the SERCOM function
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  
  //setting up display
  Serial.println("Initialising"); Serial.println("");
 
  d.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  d.clearDisplay();
  d.setTextSize(1);
  d.setTextColor(WHITE);
  d.setCursor(0,0);
  d.println("Initialising");
  d.display();
  delay(1000);
  // gyroscope sense check
  if(!bno.begin())
  {
    Serial.print("No BNO055 detected...");
    d.println("NO BNO055 CONNECTION");
    d.display();
    while(1);
  }
  else {
    Serial.print("Working!");
    d.println("CONNECTED");
    d.display();
  }
  bno.setExtCrystalUse(true);
  delay(100);
  // Switch to gyro-only
  bno.setMode(OPERATION_MODE_GYRONLY);
  delay(20);

}
void matrixMultiplication(float A[3][3], float B[3][3], float R[3][3]) // matrix multiplication converted to code
{
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            R[i][j] = 0;
            for(int k=0; k<3; k++) {
                R[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}
void readDataAndDisplay()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // BNO055 euler values
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag); // not neccessary

  Serial.print("CALIBRATION: "); // calibration to serial 
  Serial.print(sys); Serial.print(" ");
  Serial.print(gyro); Serial.print(" ");
  Serial.print(accel); Serial.print(" ");
  Serial.println(mag);
  
  while (Serial1.available() > 0) {  
    gps.encode(Serial1.read());
  }
  // gps data
  
  lat = 52.02707; // gps.location.lat();
  lng = -0.4465455; // gps.location.lng();
  int year = gps.date.year();
  int month = gps.date.month();
  int day = gps.date.day();
  int gpsHour = gps.time.hour();
  int gpsMin = gps.time.minute();
  int gpsSec = gps.time.second();
  
  bool gpsTimeReady = gps.time.isValid() &&
                gps.date.isValid();

  if (!gpsTimeReady) {
    d.clearDisplay();
    d.setCursor(0,0);
    d.print("Waiting for GPS time...");
    d.display();
    return;   // skip calculations this cycle
  }
  
  JD = getJulianDate(year,month,day,gpsHour,gpsMin,gpsSec);
  LST = JDtoLST(JD,gpsHour,gpsMin,gpsSec); 
  // gyro data
  float yaw   = euler.x() * DEG_TO_RAD;   // heading
  float roll  = euler.y() * DEG_TO_RAD;   // roll
  float pitch = euler.z() * DEG_TO_RAD;   // pitch
  
  float Rx[3][3] = {
    {1, 0, 0},
    {0, cos(roll), -sin(roll)},
    {0, sin(roll),  cos(roll)}
  };

  float Ry[3][3] = {
    { cos(pitch), 0, sin(pitch)},
    {0, 1, 0},
    {-sin(pitch), 0, cos(pitch)}
  };

  float Rz[3][3] = {
    {cos(yaw), -sin(yaw), 0},
    {sin(yaw),  cos(yaw), 0},
    {0, 0, 1}
  };

  float Rzy[3][3];
  float R[3][3];

  matrixMultiplication(Rz, Ry, Rzy); // Rz *Ry
  matrixMultiplication(Rzy, Rx, R); //Rzy * Rx

  float East  = R[0][0];
  float North = R[1][0];
  float Up    = R[2][0];

  float rawazimuth  = atan2(-East, North) * RAD_TO_DEG +90;

  float azimuthOffset =0;
  float calculatedAltitude = asin(Up) * RAD_TO_DEG + 1.87;
  float calculatedAzimuth = rawazimuth + azimuthOffset;
  if (calculatedAzimuth < 0) calculatedAzimuth += 360;
  if (calculatedAzimuth >= 360) calculatedAzimuth -= 360;
  
  
  myNex.NextionListen();
  

  // display to OLED screen
  targetAzAlt();
  d.clearDisplay();
  d.setTextSize(1);
  d.setTextColor(WHITE);
  d.setCursor(0,0);
  d.print("\nAz: ");
  d.print(calculatedAzimuth);
  d.print("\nAlt: ");
  d.print(calculatedAltitude);
  d.print("\n---------------------");
  d.print("\nTarget:");
  d.print(targetName);
  d.print("\n---------------------");

  if (!gps.location.isValid()){
    d.setCursor(67, 5);
    d.print("Finding");
    d.setCursor(67, 13);
    d.print("satellite");
  }
  
  // --- SEND TO NEXTION ---
  // Multiplied by 100 for decimals (requires vvs1 attribute = 2 in Nextion Editor)
  myNex.writeNum("tAz.val", (targetAz*100));
  myNex.writeNum("tAlt.val", (targetAlt*100));
  myNex.writeNum("cAz.val", (calculatedAzimuth*100));
  myNex.writeNum("cAlt.val", (calculatedAltitude*100));
  

  d.setCursor(0,48);
  d.print("T't Az: ");
  d.print(targetAz);
  d.print("\nT't Alt: ");
  d.print(targetAlt);
  d.display();
  /*
  Serial.print("X: "); Serial.print(yaw);
  Serial.print("\tY: "); Serial.print(pitch);
  Serial.print("\tZ: "); Serial.print(roll);
  Serial.print("\t");
  
  Serial.print("lat: "); Serial.print(lat,6);
  Serial.print(" lng: "); Serial.print(lng,6);
  Serial.print(" LST(h): "); Serial.print(LST,6);
  Serial.print(" RA(h): "); Serial.print(targetRA,6);
  double ha_h = LST - targetRA; Serial.print(" HA(h): "); Serial.print(ha_h,6);
  Serial.print(" HA(deg): "); Serial.print(ha_h*15.0,6);
  Serial.print(" targetDec: "); Serial.println(targetDec,6);
  */
// trigger functions that are called when Nextion display sends hex values in response to a button click 
} 
void trigger1() {
  objectData(1); // Item 0 was picked
  targetAzAlt();
}

void trigger2() {
  objectData(2); // Item 1 was picked
  targetAzAlt();
}

void trigger3() {
  objectData(3); // Item 2 was picked
  targetAzAlt();
}
void trigger4() {
  objectData(4); // Item 3 was picked
  targetAzAlt();
}
void trigger5() {
  objectData(5); // Item 4 was picked
  targetAzAlt();
}
void trigger6() {
  objectData(6); // Item 5 was picked
  targetAzAlt();
}
double getJulianDate(int y, int m, int d, int hr, int mn, int sc){ // formula converts UTC to JD - Practical Astronomy with your Calculator or Spreadsheet
  double dy = d + (hr*3600 + mn*60 +sc)/86400.0;
  if (m <=2){
    y = y-1;
    m = m+12;
  }
  int A = floor(y/100.0);
  int B = 2- A + floor(A/4.0);
  int C;
  if (y<0){
     C = floor((365.25*y)-0.75);
  }
  else{
     C = floor(365.25*y);
  }
  int D = floor(30.6001*(m+1));
  double JD = (B+C+D+dy+1720994.5);
  return JD;
}
double JDtoLST(double JD,int hour, int min, int sec){
  double S = JD - 2451545.0;
  double T = S/36525.0;
  double TZero =  fmod((6.697374558 +(2400.051336*T)+(0.000025862*T*T)),24);
  double UT = (hour + (min/60.0)+(sec/3600.0))*1.002737909;
  double GST = fmod((TZero + UT),24);
  // Now GST to LST
  double longitudeHours = lng/15.0;
  
  LST = fmod((GST + longitudeHours),24);
  
  return LST;
  
}
void targetAzAlt(){
  double hourAngle = (LST - targetRA); // Convert degrees to hours
  
  if (hourAngle < 0) hourAngle += 24;
  hourAngle *= 15 * DEG_TO_RAD; 
  
  double RA = targetRA *DEG_TO_RAD;
  double tD= targetDec *DEG_TO_RAD;
  double Lat = lat *DEG_TO_RAD;
  
  targetAz = atan2((sin(hourAngle)),((cos(hourAngle)*sin(Lat))-(tan(tD)*cos(Lat))))*RAD_TO_DEG; 
  
  targetAz += 180;
  
  targetAlt = asin(sin(Lat)*sin(tD) +cos(Lat)*cos(tD)*cos(hourAngle))*RAD_TO_DEG;
  
}
void onObjectIDChange() {
  
}
void onLocationChange() {}
void loop() {
  // put your main code here, to run repeatedly:
  readDataAndDisplay();
  //ArduinoCloud.update();
}
void objectData(int ObjectID){
  if (ObjectID==1){ //crab nebula: M1
    targetRA = 5.575833;
    targetDec = 22.0015;
    targetName = "Crab Nebula";
  }
  else if (ObjectID == 2){ // Plaides cluster: M45
    targetRA = 3.7833;
    targetDec = 24.0117;
    targetName = "Plaides Clstr";
  }
  else if (ObjectID == 3){ //andromeda galaxy: M31
    targetRA = 0.736;
    targetDec = 41.41503;
    targetName = "Androm' Glxy";
  }
  else if (ObjectID == 4){ //orion nebula : M42
    targetRA = 5.609917;
    targetDec = -5.375;
    targetName = "Orion Nebula";
  }
  else if (ObjectID == 5){ //whirlpool galaxy: M51
    targetRA = 13.5166;
    targetDec = 47.05483;
    targetName = "Whirlp' Glxy";
  }
  else if (ObjectID == 6) {//beehive cluster: M44
    targetRA = 8.7488;
    targetDec = 19.5727;
    targetName= "Beehive Clstr";
  }
}