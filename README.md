#include <Wire.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <Arduino.h>
#include "SHTSensor.h"

// Forward declaration
void sendCAN(uint8_t idx, float tempC, float rh);

// Config
#define NUM_MUX     4
#define PCA_ADDR    0x70
#define CAN_ID_BASE 0x000
uint32_t sendIntervalMs = 1000;

// SHT85 sensors via multiplexer
SHTSensor shtMux[NUM_MUX];

// CAN bus instance on CAN3
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;
CAN_message_t msgBuf;

// Select PCA9546A channel
void pcaSelect(uint8_t ch) {
  if (ch >= NUM_MUX) return;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(5);
}

// Magnus formula for dew point
float dewPoint(float t, float h) {
  const float a = 17.27, b = 237.7;
  float alpha = (a * t) / (b + t) + log(h / 100.0);
  return (b * alpha) / (a - alpha);
}

void setup() {
  Serial.begin(115200);
  delay(4000);
  Serial.println("SHT85 CAN Bus Example");

  Wire.begin();
  Wire.setClock(100000); // 100 kHz for stability

  // Initialize sensors behind multiplexer
  for (uint8_t i = 0; i < NUM_MUX; i++) {
    pcaSelect(i);
    delay(50);
    shtMux[i] = SHTSensor(SHTSensor::SHT85);
    if (!shtMux[i].init()) {
      Serial.print("SHT85 not found on mux channel "); Serial.println(i);
    } else {
      Serial.print("SHT85 found on mux channel "); Serial.println(i);
    }
  }

  // Initialize CAN
  CANbus.begin();
  CANbus.setClock(CLK_24MHz);
  CANbus.setBaudRate(500000);
  CANbus.setMaxMB(16);

  msgBuf.len = 6;

  Serial.println("Setup complete.");
}

void loop() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();

  if (now - lastSend >= sendIntervalMs) {
    lastSend = now;

    for (uint8_t i = 0; i < NUM_MUX; i++) {
      pcaSelect(i);
      delay(10);
      if (shtMux[i].readSample()) {
        float temp = shtMux[i].getTemperature();
        float hum = shtMux[i].getHumidity();
        sendCAN(i, temp, hum);
      } else {
        Serial.print("Error reading sensor "); Serial.println(i);
      }
    }
  }
}

void sendCAN(uint8_t idx, float tempC, float rh) {
  float dp = dewPoint(tempC, rh);

  Serial.println("SENDING MSG");

  // Clamp into representable window
  if (tempC < -40.0f) tempC = -40.0f;
  if (tempC > 215.0f) tempC = 215.0f;
  if (dp < -40.0f) dp = -40.0f;
  if (dp > 215.0f) dp = 215.0f;

  uint16_t Ts = uint16_t((tempC + 40.0f) * 100);  // biased +40
  uint16_t Hs = uint16_t(rh * 100);
  uint16_t Ds = uint16_t((dp + 40.0f) * 100);     // biased +40

  msgBuf.id = CAN_ID_BASE + idx;
  msgBuf.buf[0] = Ts & 0xFF;  msgBuf.buf[1] = Ts >> 8;
  msgBuf.buf[2] = Hs & 0xFF;  msgBuf.buf[3] = Hs >> 8;
  msgBuf.buf[4] = Ds & 0xFF;  msgBuf.buf[5] = Ds >> 8;

  CANbus.write(msgBuf);

  Serial.print("Sensor ");
  Serial.print(idx);
  Serial.print(" → Temp=");
  Serial.print(tempC);
  Serial.print(" C, RH=");
  Serial.print(rh);
  Serial.print(" %, DewPt=");
  Serial.print(dp);
  Serial.println(" C");
}# SHT85
Uses SH85 THS  with Teensy and reads through PCAN
