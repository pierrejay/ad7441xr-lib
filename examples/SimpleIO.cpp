/*
 * File: SimpleIO.cpp
 * Description: Simple library application example
 * Author: Pierre Jay
 * Date: 2024/09/22
 */

#include <Arduino.h>
#include <SPI.h>
#include "ad7441xr.h"

#define SPI_MISO    D2
#define SPI_MOSI    D4
#define SPI_SCK     D3

#define AD7441XR_CHIP_ID    ad74412r
#define AD7441XR_CS_PIN     D1
#define AD7441XR_RST_PIN    D0
#define AD7441XR_ALERT_PIN  -1

AD7441XR swio(AD7441XR_CS_PIN, SPI,
              AD7441XR_CHIP_ID, AD7441XR_RST_PIN, AD7441XR_ALERT_PIN);

unsigned long lastReading = 0;
unsigned long readingInterval = 500;

// Formatting function for Serial output
String chToStr(int ch) {
  return "Channel " + String((char)('A' + ch));
}

// Formatting function for Serial output
String unitToStr(int unit) {
    switch (unit) {
        case 0:     return "";      break;
        case 1:     return "V";     break;
        case 2:     return "mA";    break;
        case 3:     return "ohm";   break;
        default:    return "U_ERR"; break;
    }
}

// Formatting function for Serial output
String funcToStr(int func) {
    switch (func) {
        case 0:     return "HighZ";     break;
        case 1:     return "AO 10V";    break;
        case 2:     return "AO 20mA";   break;
        case 3:     return "AI 10V";    break;
        case 4:     // Fallthrough
        case 5:     return "AI 20mA";   break;
        case 6:     return "AI RTD";    break;
        case 7:     // Fallthrough
        case 8:     return "DI 24V";    break;
        default:    return "U_ERR";     break;
    }
}

void displayAlerts() {
    ad7441xr_alert_info AlertList[16];
    if (ad7441xr.getAlertList(alertList)) {
        Serial.println("!!! Active alerts:");
        for (int i = 0; i < 16; i++) {
            if (alertList[i].status) {
                Serial.print(" ");
                Serial.print(alertList[i].name);
            }
        }
        Serial.println();
    }
}

void displayIOs() {
    for (int i = 0; i < 4; i++) {
        if (swio.isEnabled(i)) {
            Serial.print(chToStr(i));
            Serial.print(": Mode ");
            Serial.print(funcToStr(swio.getChannelFunc(i)));
            Serial.print(", ADC reading: ");
            Serial.print(swio.getAdc(i));
            Serial.print(" ");
            Serial.print(unitToStr(swio.getAdcUnit(i)));
            Serial.print(", ADC code: ");
            Serial.print(swio.getAdcRaw(i));
            Serial.print(", DAC code: ");
            Serial.println(swio.getDacRaw(i));
        }
    }
}

void setup()
{
    Serial.begin(9600);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    digitalWrite(AD7441XR_CS_PIN, HIGH);

    int ret = swio.begin();
    if (ret)
        Serial.println("AD7441XR init failed...");
    else
        Serial.println("AD7441XR init success!");

    swio.enableChannel(AD7441XR_CH_A, true);
    swio.enableChannel(AD7441XR_CH_B, true);
    swio.enableChannel(AD7441XR_CH_C, true);
    swio.enableChannel(AD7441XR_CH_D, true);

    swio.setChannelFunc(AD7441XR_CH_A, AD7441XR_VOLTAGE_IN);
    swio.setChannelFunc(AD7441XR_CH_B, AD7441XR_VOLTAGE_IN);
    swio.setChannelFunc(AD7441XR_CH_C, AD7441XR_VOLTAGE_IN);
    swio.setChannelFunc(AD7441XR_CH_D, AD7441XR_VOLTAGE_OUT);

    // Start ADC continuous reading
    swio.setAdcMode(AD7441XR_START_CONT);

    // Set DAC to 3.14V on channel 4 (index 3)
    swio.setDac(3, 3.14);
}

void loop()
{
    swio.poll();

    if (millis() - lastReading >= readingInterval)
    {
        displayIOs();               // Display IO values on Serial port
        displayAlerts();            // Display active alerts on Serial port
        Serial.println("---");
        lastReading = millis();
    }
}