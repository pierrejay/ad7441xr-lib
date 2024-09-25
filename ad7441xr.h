/***************************************************************************//**
 *   @file   ad7441xr.h
 *   @brief  Header file of AD7441xR Driver.
 *   @author Original author Ciprian Regus (ciprian.regus@analog.com), modified by Pierre Jay (pierre.jay@gmail.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef _AD7441XR_H_
#define _AD7441XR_H_

#include "stdint.h"
#include "stdbool.h"
#include "ad7441xr_dfs.h"

/**
 * @brief Channel configuration structure.
 */
struct channel_cfg
{
	bool enabled;
	enum ad7441xr_op_mode func;
	long adc_raw;							  // Latest ADC raw value (raw adc code 0-65535 or -EINOP if channel unavailable)
	float adc_real;							  // Latest ADC real value (value depending on current mode or -EINOP if channel unavailable)
    enum ad7441xr_adc_unit adc_unit;          // ADC unit
	unsigned long adc_timestamp;			  // Timestamp of latest ADC reading based on millis() function (0 after a device start/reset)
	enum ad7441xr_adc_sample adc_sample_rate; // ADC sample rate
	long dac_raw;							  // Raw DAC code 0-65535 or -EINOP if unavailable
    float dac_real;                           // Real DAC setpoint (value depending on current mode or -EINOP if channel unavailable)
	long dac_clr;							  // Raw DAC clear code 0-65535 or -EINOP if unavailable
    int din_state;                            // DIN state 0-1
	int din_threshold;						  // DIN threshold 0-16000
    int din_debounce;						  // DIN threshold 0-31 (see datasheet for debounce time settings)
};

/**
 * @brief AD7441XR configuration structure.
 */
struct ad7441xr_cfg
{
	enum ad7441xr_chip_id chip_id;
	int cs_pin;
	int rst_pin;
	enum ad7441xr_conv_seq adc_conv;				 // ADC conversion mode (idle, single, continuous, off)
    int adc_rdy;
    int adc_busy;
	struct channel_cfg channel[AD7441XR_N_CHANNELS]; // Channel configuration (enabled, op mode, raw value...)
    union ad7441xr_live_status live_status;
    union ad7441xr_alert_status alert_status;
};

class AD7441XR {
    private:
        int _cs;
        SPIClass &spi;
        enum ad7441xr_chip_id chipId;
        int _rstPin;
        int _alertPin;
        boolean usingResetPin;
        boolean usingAlertPin;
        uint8_t _txBuffer[4];
        uint8_t _rxBuffer[4];
        struct ad7441xr_cfg cfg;
    
        /** Register read/write functions */
        static void _formatRegWrite(uint8_t, uint16_t, uint8_t *buff);
        int _readRegister(uint32_t, uint16_t *);
        int _readRegisterRaw(uint32_t, uint8_t *);
        int _updateRegister(uint32_t, uint16_t, uint16_t);
        int _writeRegister(uint32_t, uint16_t);

        /** Clear the ALERT_STATUS register */
        int _clearErrors();
        
        /** Get the number of active channels */
        int _getActiveChannels(uint8_t *);
        
        /** Get the rejection setting for a specific channel */
        int _getAdcRejection(uint32_t, enum ad7441xr_rejection *);
        
        /** Get a single ADC raw value for a specific channel, then power down the ADC */
        int _getAdcSingle(uint32_t, uint16_t *);
        int _getAdcSingle(uint32_t, struct ad7441xr_adc_value *);

        /** Read the raw ADC raw conversion value */
        int _getRawAdcResult(uint32_t, uint16_t *);
        
        /** Comm test function */
        int _scratchTest();

        /** Start or stop ADC conversions */
        int _setAdcConversions(enum ad7441xr_conv_seq);

        /** Update the alert status bits */
        int _updateAlertStatus();
        
        /** Update the live status bits & busy/rdy state */
        int _updateLiveStatus();
        int _updateBusyRdy(union ad7441xr_live_status);

        /** Convert raw ADC code to floating point measurement */
        int _adcRawToReal(uint16_t, ad7441xr_op_mode, float *, ad7441xr_adc_unit *);

        /** Set DI threshold & debounce */
        int _setThreshold(uint32_t, uint32_t);
        int _setDebounce(uint32_t, uint16_t);

        /** Convert DAC real setpoint to code and vice versa */
        int _dacVoltageToCode(uint32_t, uint32_t *);
        int _setDacCode(uint32_t, uint16_t);

        /** Poll ADC reading & update status */
        int _pollAdc(uint32_t);

        /** Additional chip features */
        int _getDiag(uint32_t, uint16_t *);
        int _getTemp(uint32_t, uint16_t *);
        int _setMux(uint32_t, enum ad7441xr_adc_mux);
        int _setRange(uint32_t, enum ad7441xr_adc_range);
        int _getDiState(uint32_t, uint16_t *);
    
    public:
        AD7441XR(int, SPIClass &spi,  enum ad7441xr_chip_id, int rstPin = -1, int alertPin = -1);

        int begin();					 						// Initialize AD7441XR chip (method to be called in setup)
	    int poll();					 						    // Fetch ADC value and status bits
	    int enableChannel(int ch, bool enable); 				// Enable ADC channel
	    int isEnabled(int ch);                                  // Get enabled channel state
        int setChannelFunc(int ch, enum ad7441xr_op_mode func);	// Set channel function
        int getChannelFunc(int ch);	                            // Get channel function
	    int setAdcMode(enum ad7441xr_conv_seq mode);			// Set ADC conversion mode (idle, single, continuous, off)
	    int requestAdc();			 	 						// Request a new ADC conversion (only works in single mode)
        long getAlerts();                                       // Get alerts bits
        bool getAlertList(ad7441xr_alert_info alertList[16]);   // Get an array of "ad7441xr_alert_info" objects containing name and status for each alert type
	    int isAdcBusy();				 						// Checks if ADC is busy (busy state is updated by the pollADC function)
	    float getAdc(int ch);			 						// Gets latest real ADC value (in V, mA or ohm depending on mode). Will return an error code if channel disabled.
	    long getAdcRaw(int ch);			 					    // Gets latest raw ADC value (0-65535). Will return an error code if channel disabled.
	    float getSingleAdc(int ch);		 						// Get one shot clear ADC value (clear value in V, mA or ohm depending on mode)
        int getAdcUnit(int ch);                                 // Get ADC unit (0, V, mA, ohm)
	    int setDac(int ch, float val);	 	    				// Set DAC value (0-11 V in voltage output mode or 0-25 mA in current output mode). Only works in DAC mode.
        float getDac(int ch);	 	    					    // Get real DAC value (in V or mA depending on mode). Will return an error code if channel not in output mode.
	    long getDacRaw(int ch);	 	    					    // Get raw DAC code (0-65535). Only works in DAC mode.
        int getDi(int ch);                                      // Get DI status. Only works in DIN mode.
	    int setDiThreshold(int ch, int mv); 					// Set DI threshold (in mV 0-16000) - by default a DI is initiated to 4V
	    int getTemp(int ch);									// Get chip temperature
        ad7441xr_cfg getCfg();                                  // Get configuration structure to access chip parameters & DAC/ADC values directly
        int getAlertPinState();                                 // Monitor ALERT pin
        int softReset();                                        // Perform a soft reset
};

#endif