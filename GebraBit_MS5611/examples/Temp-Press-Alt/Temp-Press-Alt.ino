#include "GebraBit_MS5611.h"

GebraBit_MS5611 MS5611;

void setup() {

  Serial.begin(9600);
  
  SPI.begin();
  pinMode(SPI_CS_Pin, OUTPUT);
  digitalWrite(SPI_CS_Pin, HIGH);  
  
  GB_MS5611_initialize(&MS5611);
  
  Serial.println("MS5611 Initialized");
}

void loop() {

  GB_MS5611_Calculate_Temperature(&MS5611);
  Serial.print("Temperature: ");
  Serial.print(MS5611.TEMPERATURE);
  Serial.println(" °C");
  
  GB_MS5611_Calculate_Temperature_Compensated_Pressure(&MS5611);
  Serial.print("Pressure: ");
  Serial.print(MS5611.PRESSURE);
  Serial.println(" hPa");
  
  GB_MS5611_Altitude(&MS5611);
  Serial.print("Altitude: ");
  Serial.print(MS5611.ALTITUDE);
  Serial.println(" meters");
  
  delay(1000);
}
