#include <Arduino.h>

#include <SPI.h>

#define CS A3

// Plotted variables must be declared as globals
double x;

// Define encoder variables
float lsb=360.0/pow(2,14);
uint16_t receivedVal16;

void setup() {
  // Setup SPI communications with AS5047D
  pinMode(CS, OUTPUT);
  SPI.begin();

  // Setup serial interface
  Serial.begin(9600);
}

void loop() {
  // Perform SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS, LOW);
  receivedVal16 = SPI.transfer16(0xFFFF);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();

  // Convert counts to degrees and assign to plot variable
  x = int (receivedVal16 & 0x3FFF) * lsb;

  // Print value to screen
  Serial.println(x,HEX);

  // Wait before next loop
  delay(100);
}
