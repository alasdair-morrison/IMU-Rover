#include <Wire.h>
#include "Adafruit_HUSB238.h"

Adafruit_HUSB238 husb238;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial Monitor

  Serial.println("HUSB238 & Arduino Due Diagnostic Boot...");

  // Initialize the primary Wire bus (Pins 20 and 21)
  if (!husb238.begin(HUSB238_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Error: Could not locate HUSB238 board. Check I2C wiring!");
    while (1) delay(10);
  }
  Serial.println("HUSB238 communication established.");
}

void loop() {
  Serial.println("\n--- Interrogating Power Bank Profiles ---");

  // 1. Check what the Power Bank is actually broadcasting right now
  for (int i = PD_SRC_5V; i <= PD_SRC_20V; i++) {
    bool available = husb238.isVoltageDetected((HUSB238_PDSelection)i);
    
    switch (i) {
      case PD_SRC_5V:  Serial.print("5V Rail:  "); break;
      case PD_SRC_9V:  Serial.print("9V Rail:  "); break;
      case PD_SRC_12V: Serial.print("12V Rail: "); break;
      case PD_SRC_15V: Serial.print("15V Rail: "); break;
      case PD_SRC_18V: Serial.print("18V Rail: "); break;
      case PD_SRC_20V: Serial.print("20V Rail: "); break;
    }
    
    if (available) {
      Serial.print("AVAILABLE");
      HUSB238_CurrentSetting maxCurrent = husb238.currentDetected((HUSB238_PDSelection)i);
      Serial.print(" | Reported Max Current Capacity Code: ");
      Serial.println(maxCurrent); // Reads out the index value of max current available
    } else {
      Serial.println("UNAVAILABLE");
    }
  }

  // 2. Programmatically force a 9V request, bypassing the hardware jumpers entirely
  Serial.println("\nSending explicit I2C request override for 9V...");
  husb238.selectPD(PD_SRC_9V);
  husb238.requestPD();
  
  delay(1000); // Allow handshake to settle

  // 3. Read back the finalized voltage contract
  HUSB238_VoltageSetting currentContract = husb238.getPDSrcVoltage();
  Serial.print("Finalized Negotiated Voltage: ");
  if(currentContract == PD_9V) {
    Serial.println("SUCCESS! Operating at 9V via software selection.");
  } else if(currentContract == PD_5V) {
    Serial.println("FAILED. Handshake rejected; defaulted back to 5V safety rail.");
  } else {
    Serial.println("Unknown/Other Voltage negotiated.");
  }

  delay(5000); // Repeat every 5 seconds for observation
}