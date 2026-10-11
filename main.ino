#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h> // 1.3" OLED Library
#include <TinyGPS++.h>

// --- PIN DEFINITIONS ---
#define SDA_PIN 8
#define SCL_PIN 9
#define GPS_RX 20
#define GPS_TX 21

// --- DISPLAY SETUP (1.3" OLED) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- SENSOR SETUP ---
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
const uint8_t MPU_ADDR = 0x68;

// --- STRICT TOILET DATABASE ---
struct Coord { 
  double lat; 
  double lon; 
  String label;
};

const int NUM_TOILETS = 2;
Coord toilets[NUM_TOILETS] = {
  {11.278417, 75.844318, "Upper Washroom"},
  {11.278558, 75.844197, "Lower Washroom"}
};

// --- SYSTEM STATES ---
enum State { FIND_NEAREST, CALIBRATE, NAVIGATE, ARRIVED };
State currentState = FIND_NEAREST;

// --- NAVIGATION & KALMAN VARIABLES ---
double targetLat = 0.0;
double targetLon = 0.0;
String targetName = "";
const double ARRIVAL_RADIUS_KM = 0.015; // 15 meters

double yawAngle = 0.0;      
double P = 1.0;             
double Q = 0.01;            
double R = 5.0;             
unsigned long lastTime = 0; 

void setup() {
  Serial.begin(115200);
  delay(3000); 

  // Initialize I2C Bus
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Initialize GPS UART
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize 1.3" OLED
  if (!display.begin(0x3C, true)) { 
    Serial.println("SH1106 allocation failed");
    while(1);
  }
  
  // Wake up MPU6050 (Using raw I2C logic)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0x00);  // Wake up sensor
  byte error = Wire.endTransmission();

  if (error != 0) {
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 20);
    display.println("MPU6050 ERROR!");
    display.display();
    while (true) delay(1000);
  }
}

void loop() {
  // 1. Feed GPS data continuously
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. Prepare Display
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  // --- STATE MACHINE LOGIC ---
  
  if (currentState == FIND_NEAREST) {
    display.setCursor(0, 20);
    
    if (!gps.location.isValid()) {
      display.println("Waiting for 3D Fix...");
      display.setCursor(0, 40);
      display.println("Locating Washrooms");
    } else {
      double minDistance = 999999.0;
      
      for (int i = 0; i < NUM_TOILETS; i++) {
        double dist = TinyGPSPlus::distanceBetween(
          gps.location.lat(), gps.location.lng(),
          toilets[i].lat, toilets[i].lon
        );
        
        if (dist < minDistance) {
          minDistance = dist;
          targetLat = toilets[i].lat;
          targetLon = toilets[i].lon;
          targetName = toilets[i].label;
        }
      }
      currentState = CALIBRATE;
    }
  } 
  
  else if (currentState == CALIBRATE) {
    display.setCursor(0, 10);
    display.println(targetName + " locked.");
    display.setTextSize(2);
    display.setCursor(0, 30);
    display.println("START WALKING");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.println("To calibrate compass");

    if (gps.speed.isValid() && gps.speed.kmph() > 2.0 && gps.course.isValid()) {
      yawAngle = gps.course.deg(); 
      lastTime = millis();         
      currentState = NAVIGATE;
    }
  } 
  
  else if (currentState == NAVIGATE) {
    double distKm = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), targetLat, targetLon
    ) / 1000.0;

    if (distKm <= ARRIVAL_RADIUS_KM) {
      currentState = ARRIVED;
    } else {
      
      // --- KALMAN FILTER MATH ---
      unsigned long currentTime = millis();
      double dt = (currentTime - lastTime) / 1000.0; 
      lastTime = currentTime;

      // 1. Predict (Gyroscope using raw I2C)
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x47); // Register for GYRO_ZOUT_H
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, (uint8_t)2);
      
      int16_t gz = (Wire.read() << 8) | Wire.read();
      float gyroZ_deg = gz / 131.0; 
      
      // Add the rotation to our heading. 
      // If arrow rotates the wrong way when you turn, remove the minus sign.
      yawAngle += (-gyroZ_deg * dt);
      
      if (yawAngle >= 360.0) yawAngle -= 360.0;
      if (yawAngle < 0.0) yawAngle += 360.0;
      
      P = P + Q;

      // 2. Update (GPS) 
      if (gps.speed.isValid() && gps.speed.kmph() > 2.0 && gps.course.isValid()) {
        double gpsCOG = gps.course.deg();
        double Y = gpsCOG - yawAngle;
        
        if (Y > 180.0) Y -= 360.0;
        if (Y < -180.0) Y += 360.0;

        double K = P / (P + R); 
        yawAngle = yawAngle + (K * Y);
        P = (1.0 - K) * P;
      }

      // --- DRAWING LOGIC ---
      double targetBearing = TinyGPSPlus::courseTo(
        gps.location.lat(), gps.location.lng(), targetLat, targetLon
      );

      double arrowAngle = targetBearing - yawAngle;
      if (arrowAngle < 0) arrowAngle += 360.0;

      display.setCursor(0, 0);
      display.print("-> "); display.println(targetName);
      display.setCursor(0, 15);
      display.print("Dist: "); display.print(distKm * 1000, 0); display.println("m");
      display.setCursor(0, 25);
      display.print("Spd: "); display.print(gps.speed.kmph(), 1); display.println("km/h");
      
      drawArrow(arrowAngle, 96, 32, 16);
    }
  } 
  
  else if (currentState == ARRIVED) {
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println("ARRIVED!");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.println("Press RST button to");
    display.setCursor(0, 60);
    display.println("find another.");
  }

  display.display();
}

// --- ARROW DRAWING HELPER FUNCTION ---
void drawArrow(double angleDeg, int cx, int cy, int radius) {
  double angleRad = angleDeg * PI / 180.0;
  
  int x1 = cx + radius * sin(angleRad);
  int y1 = cy - radius * cos(angleRad);
  
  int x2 = cx + (radius * 0.7) * sin(angleRad - (140 * PI / 180.0));
  int y2 = cy - (radius * 0.7) * cos(angleRad - (140 * PI / 180.0));
  
  int x3 = cx + (radius * 0.7) * sin(angleRad + (140 * PI / 180.0));
  int y3 = cy - (radius * 0.7) * cos(angleRad + (140 * PI / 180.0));
  
  display.fillTriangle(x1, y1, x2, y2, x3, y3, SH110X_WHITE);
}
