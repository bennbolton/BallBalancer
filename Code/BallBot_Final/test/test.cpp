#include <Wire.h>
#include <Arduino.h>

void setup() {
  Wire.begin();  // Use default SDA/SCL, or specify pins: Wire.begin(SDA, SCL);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Scanning for I2C devices...");
}

void loop() {
  byte error, address;
  int count = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  ✓");
      count++;
    } else if (error == 4) {
      Serial.print("Unknown error at 0x");
      Serial.println(address, HEX);
    }
  }

  if (count == 0)
    Serial.println("No I2C devices found.\n");
  else
    Serial.println("Scan complete.\n");

  delay(5000);  // Repeat scan every 5 seconds
}