#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

const int chipSelect = 10;  // Adjust if needed

File dataFile;

uint8_t detectedAddress = 0;
unsigned long lastTime = 0;

float posX = 0, posY = 0, posZ = 0;
float velX = 0, velY = 0, velZ = 0;

float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
const float alpha = 0.9;

void scanI2CForBNO08x() {
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (bno08x.begin_I2C(addr)) {
        detectedAddress = addr;
        Serial.print(" BNO08x found at 0x");
        Serial.println(addr, HEX);
        return;
      }
    }
  }
  Serial.println(" BNO08x not found.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card init failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Open CSV file
  dataFile = SD.open("imu_data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("timestamp_ms,posX,posY,posZ,qx,qy,qz,qw");
    dataFile.close();
  } else {
    Serial.println("Error opening imu_data.csv");
    while (1);
  }

  scanI2CForBNO08x();
  if (detectedAddress == 0) while (1) delay(10);

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Failed to enable rotation vector!");
    while (1) delay(10);
  }

  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Failed to enable linear acceleration!");
    while (1) delay(10);
  }

  lastTime = millis();
  Serial.println("🚀 Logging started...");
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  while (bno08x.getSensorEvent(&sensorValue)) {

    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      float ax = sensorValue.un.linearAcceleration.x;
      float ay = sensorValue.un.linearAcceleration.y;
      float az = sensorValue.un.linearAcceleration.z;

      if (fabs(ax) < 0.03) ax = 0;
      if (fabs(ay) < 0.03) ay = 0;
      if (fabs(az) < 0.03) az = 0;

      ax_filtered = alpha * ax_filtered + (1 - alpha) * ax;
      ay_filtered = alpha * ay_filtered + (1 - alpha) * ay;
      az_filtered = alpha * az_filtered + (1 - alpha) * az;

      ax = ax_filtered;
      ay = ay_filtered;
      az = az_filtered;

      velX += ax * deltaTime;
      velY += ay * deltaTime;
      velZ += az * deltaTime;

      if (ax == 0 && ay == 0 && az == 0) {
        velX *= 0.9;
        velY *= 0.9;
        velZ *= 0.9;
      }

      posX += velX * deltaTime;
      posY += velY * deltaTime;
      posZ += velZ * deltaTime;
    }

    else if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;

      // Serial output
      Serial.print(currentTime); Serial.print(", ");
      Serial.print(posX, 6); Serial.print(", ");
      Serial.print(posY, 6); Serial.print(", ");
      Serial.print(posZ, 6); Serial.print(", ");
      Serial.print(qx, 6); Serial.print(", ");
      Serial.print(qy, 6); Serial.print(", ");
      Serial.print(qz, 6); Serial.print(", ");
      Serial.println(qw, 6);

      // CSV output
      dataFile = SD.open("imu_data.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.print(currentTime); dataFile.print(",");
        dataFile.print(posX, 6); dataFile.print(",");
        dataFile.print(posY, 6); dataFile.print(",");
        dataFile.print(posZ, 6); dataFile.print(",");
        dataFile.print(qx, 6); dataFile.print(",");
        dataFile.print(qy, 6); dataFile.print(",");
        dataFile.print(qz, 6); dataFile.print(",");
        dataFile.println(qw, 6);
        dataFile.close();
      }
    }
  }
}