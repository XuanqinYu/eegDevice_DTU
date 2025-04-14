
#include "BluetoothSerial.h"
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

Adafruit_ADS1115 ads;
BluetoothSerial SerialBT;
         
#define SDA_PIN  33  
#define SCL_PIN  32 

void setup() {
  setCpuFrequencyMhz(80); 
  btStart();    
  Serial.begin(115200);             
  SerialBT.begin("EEG_Classic_BT");  
  Serial.println("Bluetooth started, ready to transmit EEG data...");
  Wire.begin(SDA_PIN, SCL_PIN);
  ads.begin(0x48);
  ads.setGain(GAIN_ONE);                 // ±6.144V 
  ads.setDataRate(RATE_ADS1115_250SPS);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, /*continuous=*/true); // Continuous mode
}

void loop() {
  int16_t adc0 =  ads.getLastConversionResults();  // AIN0 = Channel 1
  // float voltage = adc0 * 4.096 / 32767.0; 
  float voltage = ads.computeVolts(adc0);  //
  SerialBT.println(voltage);
  delay(10);  // 1000/10=100 Hz
}
