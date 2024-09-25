/*
 * File: SimpleIO.cpp
 * Description: Simple library application example
 * Author: Pierre Jay
 * Date: 2024/09/22
 */


#include <Arduino.h>
#include <SPI.h>
#include "ad7441xr.h"

#define SPI_MISO D2
#define SPI_MOSI D4
#define SPI_SCK D3

#define AD7441XR_CHIP_ID ad74412r
#define AD7441XR_CS_PIN D1
#define AD7441XR_RST_PIN D0
#define AD7441XR_ALERT_PIN -1

AD7441XR swio(AD7441XR_CS_PIN, SPI,
              AD7441XR_CHIP_ID, AD7441XR_RST_PIN, AD7441XR_ALERT_PIN);

unsigned long lastReading = 0;
unsigned long readingInterval = 500;

int checkAlerts(long alerts)
{
    if (alerts == 0)
    {
        return 0;
    }
    Serial.print("ALERTS DETECTED:");
    if (alerts & 0x0001)
        Serial.print(" VI_ERR_A");
    if (alerts & 0x0002)
        Serial.print(" VI_ERR_B");
    if (alerts & 0x0004)
        Serial.print(" VI_ERR_C");
    if (alerts & 0x0008)
        Serial.print(" VI_ERR_D");
    if (alerts & 0x0010)
        Serial.print(" HI_TEMP_ERR");
    if (alerts & 0x0020)
        Serial.print(" CHARGE_PUMP_ERR");
    if (alerts & 0x0040)
        Serial.print(" ALDO5V_ERR");
    if (alerts & 0x0080)
        Serial.print(" AVDD_ERR");
    if (alerts & 0x0100)
        Serial.print(" DVCC_ERR");
    if (alerts & 0x0200)
        Serial.print(" ALDO1V8_ERR");
    if (alerts & 0x0400)
        Serial.print(" ADC_CONV_ERR");
    if (alerts & 0x0800)
        Serial.print(" ADC_SAT_ERR");
    if (alerts & 0x1000)
        Serial.print(" SPI_SCLK_CNT_ERR");
    if (alerts & 0x2000)
        Serial.print(" SPI_CRC_ERR");
    if (alerts & 0x4000)
        Serial.print(" CAL_MEM_ERR");
    if (alerts & 0x8000)
        Serial.print(" RESERVED");
    Serial.println();
    return 1;
}

// Formatting function to convert channel no to channel name
String chToStr(int ch) {
  return "Channel " + String((char)('A' + ch));
}

// Formatting function to convert unit code to unit name
String unitToStr(int unit) {
    switch (unit) {
        case 0:     return "";      break;
        case 1:     return "V";     break;
        case 2:     return "mA";    break;
        case 3:     return "ohm";   break;
        default:    return "U_ERR"; break;
    }
}

// Formatting function to convert function code to function name
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

void setup()
{
    Serial.begin(9600);
    delay(3000);

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

    // Set DAC to 5.5V on channel 4 (index 3)
    swio.setDac(3, 5.5);
}

void loop()
{

    swio.poll();

    // Affichage des valeurs sur port série
    if (millis() - lastReading >= readingInterval)
    {
        lastReading = millis();
        checkAlerts(swio.getAlerts()); // Display alerts if detected
        for (int i = 0; i < 4; i++)
        {
            if (swio.isEnabled(i))
            {
                Serial.print(chToStr(i));
                Serial.print(": ");
                Serial.print(swio.getAdc(i));
                Serial.print(" ");
                Serial.print(unitToStr(swio.getAdcUnit(i)));
                Serial.print(", ADC code: ");
                Serial.print(swio.getAdcRaw(i));
                Serial.print(", DAC code: ");
                Serial.println(swio.getDacRaw(i));
            }
        }
        Serial.println("---");
    }
}