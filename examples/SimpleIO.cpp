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

// Pulse input/output pins for testing
#define INPUT_PIN   D5
#define OUTPUT_PIN  D6

#define AD7441XR_CHIP_ID        ad74412r
#define AD7441XR_CS_PIN         D1
#define AD7441XR_RST_PIN        D0
#define AD7441XR_ALERT_PIN      -1

AD7441XR swio(AD7441XR_CS_PIN, SPI, 
    AD7441XR_CHIP_ID, AD7441XR_RST_PIN, AD7441XR_ALERT_PIN);

int ret;
struct ad7441xr_adc_value result;
union ad7441xr_alert_status alerts;
float resultf;
int setDiThresholdError;

// Poll AD7441XR every 100ms
unsigned long lastPolling = 0;
unsigned long pollingInterval = 100;

// Update readings every 500ms
unsigned long lastReading = 0;
unsigned long readingInterval = 500;

// Switch output every 50ms
unsigned long lastOutputSwitch = 0;
unsigned long outputSwitchInterval = 50;
bool lastInputState = false;
unsigned long lastInputSwitch = 0;

// Helper functions for display

String chToLetter(int ch) {
  return "Channel " + String((char)('A' + ch));
}

String unitToLetter(int ch) {
    int unit = swio.getAdcUnit(ch);
    switch (unit) {
        case 0: return "";
        case 1: return "V";
        case 2: return "mA";
        case 3: return "ohm";
        default: return "U_ERR";
    }
}

int checkAlerts(long alerts) {
    if (alerts == 0) {
        return 0;
    }
    Serial.println("ALERTS DETECTED:");
    if (alerts & 0x0001) Serial.println("VI_ERR_A");
    if (alerts & 0x0002) Serial.println("VI_ERR_B");
    if (alerts & 0x0004) Serial.println("VI_ERR_C");
    if (alerts & 0x0008) Serial.println("VI_ERR_D");
    if (alerts & 0x0010) Serial.println("HI_TEMP_ERR");
    if (alerts & 0x0020) Serial.println("CHARGE_PUMP_ERR");
    if (alerts & 0x0040) Serial.println("ALDO5V_ERR");
    if (alerts & 0x0080) Serial.println("AVDD_ERR");
    if (alerts & 0x0100) Serial.println("DVCC_ERR");
    if (alerts & 0x0200) Serial.println("ALDO1V8_ERR");
    if (alerts & 0x0400) Serial.println("ADC_CONV_ERR");
    if (alerts & 0x0800) Serial.println("ADC_SAT_ERR");
    if (alerts & 0x1000) Serial.println("SPI_SCLK_CNT_ERR");
    if (alerts & 0x2000) Serial.println("SPI_CRC_ERR");
    if (alerts & 0x4000) Serial.println("CAL_MEM_ERR");
    if (alerts & 0x8000) Serial.println("RESERVED");
    return 1;
}

void setup() 
{
    pinMode(INPUT_PIN, INPUT);
    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, LOW);

    Serial.begin(9600);
    delay(3000);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
	  digitalWrite(AD7441XR_CS_PIN, HIGH);

    int ret = swio.begin();
    if (ret) Serial.println("AD7441XR init failed...");
    else Serial.println("AD7441XR init success!");

    swio.enableChannel(AD7441XR_CH_A, true);
    swio.enableChannel(AD7441XR_CH_B, true);
    swio.enableChannel(AD7441XR_CH_C, true);
    swio.enableChannel(AD7441XR_CH_D, true);

    // Set CH A to digital input + set DI threshold to 1V
    swio.setChannelFunc(AD7441XR_CH_A, AD7441XR_DIGITAL_INPUT);
    setDiThresholdError = swio.setDiThreshold(AD7441XR_CH_A, 1000); 
    // Set CH B to voltage input
    swio.setChannelFunc(AD7441XR_CH_B, AD7441XR_VOLTAGE_IN);
    // Set CH C to current input
    swio.setChannelFunc(AD7441XR_CH_C, AD7441XR_CURRENT_IN_EXT);
    // Set CH D to current output
    swio.setChannelFunc(AD7441XR_CH_D, AD7441XR_CURRENT_OUT);

    // Start ADC continuous reading + set DAC output to 5.5mA
    swio.setAdcMode(AD7441XR_START_CONT);
    swio.setDac(3, 5.5);

    // Configure GPO_A as comparator output (trigger on DI change)
    swio.setGpoMode(AD7441XR_CH_A, AD7441XR_GPO_COMP_OUT);
}

void loop() 
{
    // Switch output periodically
    if (millis() - lastOutputSwitch >= outputSwitchInterval) {
        lastOutputSwitch = millis();
        digitalWrite(OUTPUT_PIN, !digitalRead(OUTPUT_PIN));
    }
    
    // Check if input changed state and display message
    bool inputState = digitalRead(INPUT_PIN);
    if (inputState != lastInputState) {
        unsigned long inputSwitchElapsed = millis() - lastInputSwitch;
        Serial.printf("Input switch detected after %lu ms\n", inputSwitchElapsed);
        Serial.printf("DI state: %d\n", swio.getDi(AD7441XR_CH_A));
        Serial.printf("DI voltage: %.3f V\n", swio.getAdc(AD7441XR_CH_A));
        Serial.println("---");
        lastInputSwitch = millis();
        lastInputState = inputState;
    }

    // Poll AD7441XR
    if (millis() - lastPolling >= pollingInterval) {
        lastPolling = millis();
        swio.poll();
    }

    // Read and display AD7441XR values periodically
    if (millis() - lastReading >= readingInterval) {
	    lastReading = millis();
        checkAlerts(swio.getAlerts()); // Display alerts if detected
        for (int i=0 ; i<4 ; i++) {
            if (swio.isEnabled(i)) {
                Serial.print(chToLetter(i));
                Serial.print(": ");
                Serial.print(swio.getAdc(i));
                Serial.print(" ");
                Serial.print(unitToLetter(i));
                Serial.print(", ADC code: ");
                Serial.print(swio.getAdcRaw(i));
                Serial.print(", DAC code: ");
                Serial.println(swio.getDacRaw(i));
            }
        }
        Serial.println("---");
            if (setDiThresholdError) {
            Serial.printf("Error setting DI threshold: %d\n", setDiThresholdError);
        }
    }
}
