#include <SPI.h>
#include <AD9833.h>

#define FSYNC1 16  // Chip Select / FSYNC pin
#define FSYNC2 17  // Chip Select / FSYNC pin
#define FSYNC3 5  // Chip Select / FSYNC pin


AD9833 AD[3] = {
  AD9833(FSYNC1),
  AD9833(FSYNC2),
  AD9833(FSYNC3),
};

void setup() {
  Serial.begin(115200);
  SPI.begin();        // Initialise SPI

  for (int i = 0; i < 2; i++)
  {
    AD[i].begin();
    AD[i].reset();
    AD[i].setWave(AD9833_SQUARE1);
  };

  AD[0].setFrequency(500);           // Frequency in Hz
  AD[1].setFrequency(10);           // Frequency in Hz
  AD[2].setFrequency(500);           // Frequency in Hz

  //ENABLE
  pinMode(21, OUTPUT); 
  digitalWrite(21, LOW);

  //DIRECTION
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
}

void loop() {
}