/***************************************************************************//**
 *   @file   ad7441xr.cpp
 *   @brief  Source file of AD7441xR Driver.
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

#include "ad7441xr.h"
#include "crc8.h"
#include "util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7441XR_FRAME_SIZE 		4
#define AD7441XR_CRC_POLYNOMIAL 	0x7
#define AD7441XR_DIN_DEBOUNCE_LEN 	NO_OS_BIT(5)

/******************************************************************************/
/************************ Variable Declarations ******************************/
/******************************************************************************/
DECLARE_CRC8_TABLE(_crc_table);

static const unsigned int ad7441xr_debounce_map[AD7441XR_DIN_DEBOUNCE_LEN] = {
	0,     13,    18,    24,    32,    42,    56,    75,
	100,   130,   180,   240,   320,   420,   560,   750,
	1000,  1300,  1800,  2400,  3200,  4200,  5600,  7500,
	10000, 13000, 18000, 24000, 32000, 42000, 56000, 75000,
};

/** The time required for an ADC conversion by rejection (us) */
static const uint32_t conv_times_ad74413r[] = { 208, 833, 50000, 100000 };
static const uint32_t conv_times_ad74412r[] = { 50000, 208};

/**
 * @brief Constructor
 * @param cs - Chip CS pin
 * @param spi - Pointer to user SPI class
 * @param id - Chip type : AD7441{2,3}R
 * @param rst - Chip RST pin
 * @param alrt - Chip ALERT pin
 * @return 0 in case of success, negative error code otherwise.
 */
AD7441XR::AD7441XR(int cs, SPIClass &spi, enum ad7441xr_chip_id id, int rst, int alrt) :
	_cs(cs), spi(spi), chipId(id), _rstPin(rst), _alertPin(alrt)	
{
    pinMode(_cs, OUTPUT);

	if (_rstPin != -1)
	{
		pinMode(_rstPin, OUTPUT);
		usingResetPin = true;
	}

	if (_alertPin != -1)
	{
		pinMode(_alertPin, INPUT_PULLUP);
		usingAlertPin = true;
	}

	cfg.chip_id = id;
   	cfg.cs_pin = cs;
   	cfg.rst_pin = rst;
}

/**
 * @brief Enable channel
 * @param ch - ADC channel
 * @param en - Channel target state
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::enableChannel(int ch, bool en)
{
    int ret;

	// Stop conversions before channel enable/disable
	ad7441xr_conv_seq prec_conv = cfg.adc_conv;
	ret = _setAdcConversions(AD7441XR_STOP_PWR_DOWN);
	if (ret) return ret;

	ret = _updateRegister(AD7441XR_ADC_CONV_CTRL, AD7441XR_CH_EN_MASK(ch), en);
	if (ret) return ret;

	cfg.channel[ch].enabled = en;

	// Restore conversion mode
	ret = _setAdcConversions(prec_conv);
	if (ret) return ret;

	return AD7441XR_SUCCESS;
}

/**
 * @brief Check if channel is enabled
 * @param ch - ADC channel
 * @return Channel enable state
 */
int AD7441XR::isEnabled(int ch)
{
    return cfg.channel[ch].enabled;
}

/**
 * @brief Clear chip errors
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_clearErrors()
{
	return _writeRegister(AD7441XR_ALERT_STATUS, AD7441XR_ERR_CLR_MASK);
}

/**
 * @brief Get active channels
 * @param nb_channels - Pointer to active channels buffer
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_getActiveChannels(uint8_t *nb_channels)
{
	int ret;
	uint16_t reg_val;

	ret = _readRegister(AD7441XR_ADC_CONV_CTRL, &reg_val);
	if (ret)
		return ret;

	reg_val = no_os_field_get(NO_OS_GENMASK(3, 0), reg_val);
	*nb_channels = no_os_hweight8((uint8_t)reg_val);

	return 0;
}

/**
 * @brief Get raw ADC result
 * @param ch - ADC channel
 * @param val - Pointer to value buffer
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_getRawAdcResult(uint32_t ch, uint16_t *val)
{
	_readRegister(AD7441XR_ADC_RESULT(ch), val);
	return 0;
}

/**
 * @brief Get ADC rejection mode
 * @param ch - Channel
 * @param val - Pointer to rejection mode buffer
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_getAdcRejection(uint32_t ch, enum ad7441xr_rejection *val)
{
	int ret;
	uint16_t rejection_val;

	ret = _readRegister(AD7441XR_ADC_CONFIG(ch), &rejection_val);
	if (ret)
		return ret;

	// CHECK
	//*val = no_os_field_get(AD7441XR_ADC_REJECTION_MASK, rejection_val);
	*val = static_cast<enum ad7441xr_rejection>(no_os_field_get(AD7441XR_ADC_REJECTION_MASK, rejection_val));

	return 0;
}

/**
 * @brief Get ADC single value (uint16)
 * @param ch - ADC channel
 * @param val - Pointer to ADC value buffer
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_getAdcSingle(uint32_t ch, uint16_t *val)
{
	int ret;
	uint32_t del;
	uint8_t nb_active_channels;
	enum ad7441xr_rejection rejection;

	ret = enableChannel(ch, true);
	if (ret)
		return ret;

	ret = _getActiveChannels(&nb_active_channels);
	if (ret)
		return ret;

	ret = _setAdcConversions(AD7441XR_START_SINGLE);
	if (ret)
		return ret;

	delayMicroseconds(AD7441XR_ADC_CONVERSION_DELAY_US); // The user must wait 100us before ADC starts doing conversions if the ADC was powered down

	ret = _getAdcRejection(ch, &rejection);
	if (ret)
		return ret;

	if (chipId == ad74413r)
		del = conv_times_ad74413r[rejection];
	else
		del = conv_times_ad74412r[rejection];

	/** Wait for all channels to complete the conversion. */
	if (del < 1000)
		delayMicroseconds(del * nb_active_channels);
	else
		delay(del * nb_active_channels / 1000);

	ret = _getRawAdcResult(ch, val);
	if (ret)
		return ret;

	ret = _setAdcConversions(AD7441XR_STOP_PWR_DOWN);
	if (ret)
		return ret;

	ret = enableChannel(ch, false);
	if (ret)
		return ret;

	return 0;
}

/**
 * @brief Get single ADC value (structure)
 * @param ch - ADC channel
 * @param val - Pointer to ADC value structure
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_getAdcSingle(uint32_t ch, struct ad7441xr_adc_value *val)
{
	int ret;
	uint16_t adc_code;

	ret = _getAdcSingle(ch, &adc_code);
	if (ret)
		return ret;

	switch (cfg.channel[ch].func) {
	case AD7441XR_HIGH_Z:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RANGE_10V_SCALE,
						 AD7441XR_RANGE_10V_SCALE_DIV,
						 &val->decimal);
		break;
	case AD7441XR_VOLTAGE_OUT:
		/**
		 * I_Rsense = (Vmin + (ADC_CODE/65535) * range) / Rsense
		 */
		val->integer = no_os_div_s64_rem((adc_code + AD7441XR_RANGE_5V_OFFSET) *
						 AD7441XR_RANGE_5V_SCALE,
						 AD7441XR_RSENSE * AD7441XR_RANGE_5V_SCALE_DIV,
						 (int32_t *)&val->decimal);
		break;
	case AD7441XR_CURRENT_OUT:
	case AD7441XR_VOLTAGE_IN:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RANGE_10V_SCALE,
						 AD7441XR_RANGE_10V_SCALE_DIV,
						 &val->decimal);
		break;
	case AD7441XR_CURRENT_IN_EXT_HART:
		if (chipId == ad74412r)
			return AD7441XR_ERR_NOT_SUPPORTED;
	case AD7441XR_CURRENT_IN_LOOP_HART:
		if (chipId == ad74412r)
			return AD7441XR_ERR_NOT_SUPPORTED;
	case AD7441XR_CURRENT_IN_EXT:
	case AD7441XR_CURRENT_IN_LOOP:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RANGE_2V5_SCALE,
						 AD7441XR_RANGE_2V5_SCALE_DIV * AD7441XR_RSENSE,
						 &val->decimal);
		break;
	case AD7441XR_RESISTANCE:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RTD_PULL_UP,
						 AD7441XR_ADC_MAX_VALUE - adc_code,
						 &val->decimal);
		break;
	case AD7441XR_DIGITAL_INPUT:
	case AD7441XR_DIGITAL_INPUT_LOOP:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RANGE_10V_SCALE,
						 AD7441XR_RANGE_10V_SCALE_DIV,
						 &val->decimal);
		break;
	default:
		return AD7441XR_ERR_INVALID_PARAM;
	}

	cfg.channel[ch].adc_raw = adc_code;
	cfg.channel[ch].adc_timestamp = millis();
	float fval;
	ad7441xr_adc_unit unit;
	_adcRawToReal(adc_code, cfg.channel[ch].func, &fval, &unit);
	cfg.channel[ch].adc_real = fval;
	cfg.channel[ch].adc_unit = unit;

	return 0;
}

/**
 * @brief Update alert status bits
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_updateAlertStatus()
{
	int ret;
	uint16_t val;

	ret = _readRegister(AD7441XR_ALERT_STATUS, &val);
	if (ret) return ret;
	cfg.alert_status.value = val;

	return _clearErrors();	
}

/**
 * @brief Update live status bits
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_updateLiveStatus()
{
	int ret;
	uint16_t val;

	ret = _readRegister(AD7441XR_LIVE_STATUS, &val);
	if (ret) return ret;
	cfg.live_status.value = val;

	ret = _updateBusyRdy(cfg.live_status);
	if (ret) return ret;

	return 0;
}

/**
 * @brief Get alert pin state
 * @return Alert pin state
 */
int AD7441XR::getAlertPinState()
{
	return digitalRead(_alertPin); 
}

/**
 * @brief Set ADC conversion mode
 * @param status - ADC conversion mode to set
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_setAdcConversions(enum ad7441xr_conv_seq status)
{
	int ret;

	ret = _updateRegister(AD7441XR_ADC_CONV_CTRL, AD7441XR_CONV_SEQ_MASK, status);
	if (ret)
		return ret;
	cfg.adc_conv = status;

	return AD7441XR_SUCCESS;
}

/**
 * @brief Set ADC channel function
 * @param addr - ADC channel
 * @param func - ADC function code to set
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::setChannelFunc(int ch, enum ad7441xr_op_mode func) {
    // Validation des paramètres
    if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
        return AD7441XR_ERR_INVALID_PARAM;
    }

    // Vérification de la compatibilité du mode avec le modèle de chip
    if ((chipId == ad74412r) && 
        (func == AD7441XR_CURRENT_IN_EXT_HART || func == AD7441XR_CURRENT_IN_LOOP_HART)) {
        return AD7441XR_ERR_NOT_SUPPORTED;
    }

    int ret;

	// Stop conversions before mode change
	ad7441xr_conv_seq prec_conv = cfg.adc_conv;
	ret = _setAdcConversions(AD7441XR_STOP_PWR_DOWN);
	if (ret) return ret;

	// Reset DAC code
	ret = _setDacCode(ch, 0);
	if (ret) return ret;

	// Enable High-Z function before changing mode
	ret = _updateRegister(AD7441XR_CH_FUNC_SETUP(ch), AD7441XR_CH_FUNC_SETUP_MASK, AD7441XR_HIGH_Z);
	if (ret) return ret;
	cfg.channel[ch].func = AD7441XR_HIGH_Z;

	delayMicroseconds(AD7441XR_MODE_CHANGE_DELAY_US); // Datasheet requirement between mode changes

	// Enable requested function
	ret = _updateRegister(AD7441XR_CH_FUNC_SETUP(ch), AD7441XR_CH_FUNC_SETUP_MASK, func);
	if (ret) return ret;
	cfg.channel[ch].func = func;

	// Override MUX & range settings + set 200K pull down resistor depending on requested function
	switch (cfg.channel[ch].func) {
		// Enable 200K PD resistor for High-Z & input functions
		case AD7441XR_HIGH_Z:
		case AD7441XR_VOLTAGE_IN:
		case AD7441XR_CURRENT_IN_EXT:
		case AD7441XR_CURRENT_IN_LOOP:
		case AD7441XR_CURRENT_IN_EXT_HART:
		case AD7441XR_CURRENT_IN_LOOP_HART:
		case AD7441XR_DIGITAL_INPUT:
		case AD7441XR_DIGITAL_INPUT_LOOP: {
			ret = _updateRegister(AD7441XR_ADC_CONFIG(ch), AD7441XR_CH_200K_TO_GND_MASK, 1);
			if (ret) return ret;
			break;}
		// Enable 200K PD resistor + measure 0-10V voltage across terminals for voltage output function
		case AD7441XR_VOLTAGE_OUT: {
			ret = _updateRegister(AD7441XR_ADC_CONFIG(ch), AD7441XR_CH_200K_TO_GND_MASK, 1);
			if (ret) return ret;
			ret = _setMux(ch, AD7441XR_ADC_MUX_IO);
			if (ret) return ret;
			ret = _setRange(ch, AD7441XR_ADC_RANGE_10V);
			if (ret) return ret;
			break;}
		// Measure -2.5-0V voltage across Rsense for current output function
		case AD7441XR_CURRENT_OUT: {
			ret = _setMux(ch, AD7441XR_ADC_MUX_RSENSE);
			if (ret) return ret;
			ret = _setRange(ch, AD7441XR_ADC_RANGE_2P5V_INT_POW);
			if (ret) return ret;
			break;}
		// Do nothing for Resistance function
		case AD7441XR_RESISTANCE:
		default:
			break;
	}

	// Restore conversion mode
	ret = _setAdcConversions(prec_conv);
	if (ret) return ret;

	delayMicroseconds(AD7441XR_DAC_WRITE_DELAY_US); // Datasheet requirement before new DAC code write

	return AD7441XR_SUCCESS;
}

/**
 * @brief Get ADC channel function
 * @param ch - ADC channel
 * @return Channel function code in case of success, negative error code otherwise.
 */
int AD7441XR::getChannelFunc(int ch)
{
	if (cfg.channel[ch].enabled) return cfg.channel[ch].func;
   	else return AD7441XR_ERR_CHANNEL_DISABLED;	
}

/**
 * @brief Format SPI message
 * @param addr - SPI register address
 * @param mask - Write value
 * @param val - Formatted message buffer
 * @return 0 in case of success, negative error code otherwise.
 */
void AD7441XR::_formatRegWrite(uint8_t reg, uint16_t val, uint8_t *buff)
{
    buff[0] = reg;
	no_os_put_unaligned_be16(val, &buff[1]);
	buff[3] = crc8(_crc_table, buff, 3, 0);
}

/**
 * @brief Read register (w/ CRC check)
 * @param addr - SPI register address
 * @param val - Pointer to read buffer
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_readRegister(uint32_t addr, uint16_t *val)
{
    int ret;
	uint8_t expected_crc;

	ret = _readRegisterRaw(addr, _rxBuffer);

	if (ret)
		return ret;

	expected_crc = crc8(_crc_table, _rxBuffer, 3, 0);
	if (expected_crc != _rxBuffer[3])
		return AD7441XR_ERR_CRC;
	

	*val = no_os_get_unaligned_be16(&_rxBuffer[1]);

	return 0;
}

/**
 * @brief Read register (raw value)
 * @param addr - SPI register address
 * @param val - Pointer to read buffer
 */
int AD7441XR::_readRegisterRaw(uint32_t addr, uint8_t *val)
{

	/**
	 * Reading a register on AD74413r requires writing the address to the READ_SELECT
	 * register first and then doing another spi read, which will contain the requested
	 * register value.
	 */

    _formatRegWrite(AD7441XR_READ_SELECT, addr, _txBuffer);

    spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
	digitalWrite(_cs, LOW);
	for (int i = 0; i < AD7441XR_FRAME_SIZE; i++)
   {
      spi.transfer(_txBuffer[i]);
   }
	// spi.transfer(_txBuffer, NULL, AD7441XR_FRAME_SIZE, NULL);
	digitalWrite(_cs, HIGH);
	spi.endTransaction();
    
	_formatRegWrite(AD7441XR_NOP, 0x00, _txBuffer);

    spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
	digitalWrite(_cs, LOW);
	for (int i = 0; i < AD7441XR_FRAME_SIZE; i++)
   {
      val[i] = spi.transfer(_txBuffer[i]);
   }
	// spi.transfer(_txBuffer, val, AD7441XR_FRAME_SIZE, NULL);
	digitalWrite(_cs, HIGH);
	spi.endTransaction();
    
    return AD7441XR_SUCCESS;
}

/**
 * @brief Chip communication init test
 */
int AD7441XR::_scratchTest()
{
	int ret;
	uint16_t val;
	uint16_t test_val = 0x1234;

	ret = _writeRegister(AD7441XR_SCRATCH, test_val);
	if (ret)
		return ret;

	ret = _readRegister(AD7441XR_SCRATCH, &val);
	if (ret)
		return ret;

	if (val != test_val)
		return AD7441XR_ERR_COMM_FAIL;

	return 0;
}

/**
 * @brief Perform a Soft Reset
 */
int AD7441XR::softReset()
{
	int ret;

	ret = _writeRegister(AD7441XR_CMD_KEY, AD7441XR_CMD_KEY_RESET_1);
	if (ret)
		return AD7441XR_ERR_COMM_FAIL;

	ret = _writeRegister(AD7441XR_CMD_KEY, AD7441XR_CMD_KEY_RESET_2);
	if (ret)
		return AD7441XR_ERR_COMM_FAIL;

	delay(AD7441XR_RESET_DELAY_MS);

	// Reset config
	cfg.adc_conv = AD7441XR_STOP_PWR_UP;
	cfg.live_status.value = 0;
	cfg.adc_busy = 0;
	cfg.adc_rdy = 0;
	for (int i = 0 ; i < AD7441XR_N_CHANNELS ; i++) {
		cfg.channel[i].enabled = false;
		cfg.channel[i].func = AD7441XR_HIGH_Z;
		cfg.channel[i].adc_raw = AD7441XR_ERR_CHANNEL_DISABLED;
		cfg.channel[i].adc_real = AD7441XR_ERR_CHANNEL_DISABLED;
		cfg.channel[i].adc_unit = U_NULL;
		cfg.channel[i].adc_timestamp = 0;
		cfg.channel[i].adc_sample_rate = AD7441XR_ADC_SAMPLE_20HZ;
		cfg.channel[i].dac_raw = 0;
		cfg.channel[i].dac_clr = 0;
		cfg.channel[i].din_threshold = 0;
	}
	return AD7441XR_SUCCESS;
}

/**
 * @brief Update chip SPI register from mask
 * @param addr - SPI register address
 * @param mask - Bitmask
 * @param val - Write value
 */
int AD7441XR::_updateRegister(uint32_t addr, uint16_t mask, uint16_t val)
{
    int ret;
	uint16_t c_val;

	ret = _readRegister(addr, &c_val);
	if (ret)
		return ret;

	c_val &= ~mask;
	c_val |= no_os_field_prep(mask, val);

	return _writeRegister(addr, c_val);
}

/**
 * @brief Write chip SPI register
 * @param addr - SPI register address
 * @param val - Write value
 */
int AD7441XR::_writeRegister(uint32_t addr, uint16_t val) {
    _formatRegWrite(addr, val, _txBuffer);

    spi.beginTransaction(SPISettings(AD7441XR_SPI_MAX_FREQ, MSBFIRST, AD7441XR_SPI_MODE));
    digitalWrite(_cs, LOW);
    
    // Utilisation d'un transfert unique pour plus d'efficacité
    spi.transfer(_txBuffer, AD7441XR_FRAME_SIZE);
    
    digitalWrite(_cs, HIGH);
    spi.endTransaction();

    return AD7441XR_SUCCESS;
}


// CUSTOM METHODS

/**
 * @brief AD7441XR initializer
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::begin()
{
   int ret;

	if (usingResetPin == true) {
		digitalWrite(_rstPin, HIGH);
	}

	crc8_populate_msb(_crc_table, AD7441XR_CRC_POLYNOMIAL);
	
	delay(AD7441XR_STARTUP_DELAY_MS);

	ret = softReset();
	if (ret) return AD7441XR_ERR_COMM_FAIL;

	ret = _clearErrors();
	if (ret) return AD7441XR_ERR_COMM_FAIL;

	ret = _scratchTest();
	if (ret) return AD7441XR_ERR_COMM_FAIL;
	
	return AD7441XR_SUCCESS;
}

/**
 * @brief Poll ADC & live status
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::poll()
{
	int ret;
	int poll_mode; // 0 = ADC off, 1 = single mode, 2 = continuous mode

	switch (cfg.adc_conv) // Check ADC conversion mode (off, single, continuous)
	{
	case AD7441XR_START_CONT:
		poll_mode = 2; // Poll ADC
		break;
	case AD7441XR_START_SINGLE:
		poll_mode = 1; // Poll ADC
		break;
	default:
		poll_mode = 0; // Do not poll ADC
		break;
	}

	// Update Live status
	ret = _updateLiveStatus();
	if (ret) return AD7441XR_ERR_COMM_FAIL;

	ret = _updateAlertStatus();
	if (ret) return AD7441XR_ERR_COMM_FAIL;

	if (poll_mode > 0) // If ADC enabled, check if conversion ready and fetch ADC values accordingly
	{
		if (cfg.adc_rdy == 1)
		{
			for (int i = 0; i < AD7441XR_N_CHANNELS; i++)
			{
				if (cfg.channel[i].enabled)
				{
					ret = _pollAdc(i);
					if (ret) return AD7441XR_ERR_COMM_FAIL;
				}
			}
			// Clear the adc ready flag after fetching values
			ret = _updateRegister(AD7441XR_LIVE_STATUS, AD7441XR_ADC_RDY_MASK, 1);
			if (ret) return AD7441XR_ERR_COMM_FAIL;
			cfg.adc_rdy = 0;
		}
	}

	return AD7441XR_SUCCESS;
}

/**
 * @brief Set ADC conversion mode (idle, single, continuous, off)
 * @param mode - Conversion mode
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::setAdcMode(enum ad7441xr_conv_seq mode)
{
	int ret;

	ret = _setAdcConversions(mode);
	if (ret) return AD7441XR_ERR_COMM_FAIL;
	
	return AD7441XR_SUCCESS;
}

/**
 * @brief Request an ADC reading
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::requestAdc()
{
	int ret;

	if (cfg.adc_conv != AD7441XR_START_CONT) {
		ret = _setAdcConversions(AD7441XR_START_SINGLE);
		if (ret) return AD7441XR_ERR_COMM_FAIL;
		ret = _updateLiveStatus();
		if (ret) return AD7441XR_ERR_COMM_FAIL;
	}

	return AD7441XR_SUCCESS;
}

/**
 * @brief Get alerts status
 * @return Alerts status bits value.
 */
long AD7441XR::getAlerts()
{
	return cfg.alert_status.value;
}

/**
 * @brief Get alerts list
 * @return Modifies an array of 16 "ad7441xr_alert_info" objects passed as argument
 */
bool AD7441XR::getAlertList(ad7441xr_alert_info alertList[16]) 
{
	ad7441xr_alert_status *status = &cfg.alert_status;
	
    alertList[0] = (ad7441xr_alert_info){status->error_bits.VI_ERR_A, "VI_ERR_A"};
    alertList[1] = (ad7441xr_alert_info){status->error_bits.VI_ERR_B, "VI_ERR_B"};
    alertList[2] = (ad7441xr_alert_info){status->error_bits.VI_ERR_C, "VI_ERR_C"};
    alertList[3] = (ad7441xr_alert_info){status->error_bits.VI_ERR_D, "VI_ERR_D"};
    alertList[4] = (ad7441xr_alert_info){status->error_bits.HI_TEMP_ERR, "HI_TEMP_ERR"};
    alertList[5] = (ad7441xr_alert_info){status->error_bits.CHARGE_PUMP_ERR, "CHARGE_PUMP_ERR"};
    alertList[6] = (ad7441xr_alert_info){status->error_bits.ALDO5V_ERR, "ALDO5V_ERR"};
    alertList[7] = (ad7441xr_alert_info){status->error_bits.AVDD_ERR, "AVDD_ERR"};
    alertList[8] = (ad7441xr_alert_info){status->error_bits.DVCC_ERR, "DVCC_ERR"};
    alertList[9] = (ad7441xr_alert_info){status->error_bits.ALDO1V8_ERR, "ALDO1V8_ERR"};
    alertList[10] = (ad7441xr_alert_info){status->error_bits.ADC_CONV_ERR, "ADC_CONV_ERR"};
    alertList[11] = (ad7441xr_alert_info){status->error_bits.ADC_SAT_ERR, "ADC_SAT_ERR"};
    alertList[12] = (ad7441xr_alert_info){status->error_bits.SPI_SCLK_CNT_ERR, "SPI_SCLK_CNT_ERR"};
    alertList[13] = (ad7441xr_alert_info){status->error_bits.SPI_CRC_ERR, "SPI_CRC_ERR"};
    alertList[14] = (ad7441xr_alert_info){status->error_bits.CAL_MEM_ERR, "CAL_MEM_ERR"};
    alertList[15] = (ad7441xr_alert_info){status->error_bits.RESET_OCCURRED, "RESET_OCCURRED"};

    return status->value != 0;
}

/**
 * @brief Check if ADC is busy
 * @return Busy state (0 or 1).
 */
int AD7441XR::isAdcBusy()
{
	return cfg.adc_busy;
}

/**
 * @brief Get real ADC reading (converted into V, mA or ohm depending on mode), only if ADC channel enabled
 * @param ch - ADC channel
 * @return Real value (float) in case of success, negative error code otherwise.
 */
float AD7441XR::getAdc(int ch)
{
   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }
   
   if (cfg.channel[ch].enabled) {
       return cfg.channel[ch].adc_real;
   }
   else {
       return AD7441XR_ERR_CHANNEL_DISABLED;
   }
}

/**
 * @brief Get raw ADC reading (0-65535), only if ADC channel enabled
 * @param ch - ADC channel
 * @return Raw ADC code in case of success, negative error code otherwise.
 */
long AD7441XR::getAdcRaw(int ch)
{
   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }
   
   if (cfg.channel[ch].enabled) {
       return cfg.channel[ch].adc_raw;
   }
   else {
       return AD7441XR_ERR_CHANNEL_DISABLED;
   }
}

/**
 * @brief Get one shot ADC reading (converted into V, mA or ohm depending on mode)
 * @param ch - ADC channel
 * @return Real value (float) in case of success, negative error code otherwise.
 */
float AD7441XR::getSingleAdc(int ch)
{
   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }

   int ret;
   uint16_t val;

   ret = _getAdcSingle(ch, &val);
   if (ret) {
       return AD7441XR_ERR_COMM_FAIL;
   }

   return cfg.channel[ch].adc_real;
}

/**
 * @brief Get ADC unit (0, V, mA, ohm), only if ADC channel enabled
 * @param ch - ADC channel
 * @return 0=null, 1=V, 2=mA, 3=ohm, negative error code otherwise.
 */
int AD7441XR::getAdcUnit(int ch)
{
   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }

   if (cfg.channel[ch].enabled) {
       return cfg.channel[ch].adc_unit;
   }
   else {
       return AD7441XR_ERR_CHANNEL_DISABLED;
   }
}

/**
 * @brief Set DAC value
 * @param ch - DAC channel
 * @param mval - DAC mv setpoint
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::setDac(int ch, float val) {
    if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
        return AD7441XR_ERR_INVALID_PARAM;
    }

    if (!cfg.channel[ch].enabled) {
        return AD7441XR_ERR_CHANNEL_DISABLED;
    }

    int ret;
    int set;
    
    switch (cfg.channel[ch].func) {
    case AD7441XR_VOLTAGE_OUT: {
        if (val >= 0 && val <= 11) {
            set = val * 1000;
        }
        else {
            return AD7441XR_ERR_INVALID_PARAM;
        }
        break;
    }
    case AD7441XR_CURRENT_OUT: {
        if (val >= 0 && val <= 25) {
            set = val * 440; // 0-25 mA => 0-11 V
        }
        else {
            return AD7441XR_ERR_INVALID_PARAM;
        }
        break;
    }
    default:
        return AD7441XR_ERR_INVALID_MODE;
    }

    uint32_t code;
    ret = _dacVoltageToCode(set, &code);
    if (ret) {
        return AD7441XR_ERR_INVALID_PARAM;
    }

    ret = _setDacCode(ch, code);
    if (ret) {
        return AD7441XR_ERR_COMM_FAIL;
    }

    cfg.channel[ch].dac_real = val;

    return AD7441XR_SUCCESS;
}

/**
 * @brief Get real DAC setpoint
 * @param ch - ADC channel
 * @return Real DAC setpoint (V or mA) in case of success, negative error code otherwise.
 */
float AD7441XR::getDac(int ch)
{
   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }

   if (!cfg.channel[ch].enabled) {
       return AD7441XR_ERR_CHANNEL_DISABLED;
   }

   if (cfg.channel[ch].func == AD7441XR_VOLTAGE_OUT || 
       cfg.channel[ch].func == AD7441XR_CURRENT_OUT) {
       return cfg.channel[ch].dac_real;
   }
   
   return AD7441XR_ERR_INVALID_MODE;
}

/**
 * @brief Get raw DAC code (0-65535)
 * @param ch - ADC channel
 * @return Raw DAC code in case of success, negative error code otherwise.
 */
long AD7441XR::getDacRaw(int ch)
{
   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }

   if (!cfg.channel[ch].enabled) {
       return AD7441XR_ERR_CHANNEL_DISABLED;
   }

   if (cfg.channel[ch].func == AD7441XR_VOLTAGE_OUT || 
       cfg.channel[ch].func == AD7441XR_CURRENT_OUT) {
       return cfg.channel[ch].dac_raw;
   }
   
   return AD7441XR_ERR_INVALID_MODE;
}

/**
 * @brief Get DIN state (0-1)
 * @param ch - ADC channel
 * @return DIN state, negative error code otherwise.
 */
int AD7441XR::getDi(int ch)
{
   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }

   if (!cfg.channel[ch].enabled) {
       return AD7441XR_ERR_CHANNEL_DISABLED;
   }

   if ((cfg.channel[ch].func == AD7441XR_DIGITAL_INPUT) || 
       (cfg.channel[ch].func == AD7441XR_DIGITAL_INPUT_LOOP)) {
       return cfg.channel[ch].din_state;
   }

   return AD7441XR_ERR_INVALID_MODE;
}

/**
 * @brief Set DIN Threshold value
 * @param ch - DIN channel
 * @param thr - DIN voltage threshold (0-16000 mvolts)
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::setDiThreshold(int ch, int mv)
{
   int ret;

   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }

   if (!cfg.channel[ch].enabled) {
       return AD7441XR_ERR_CHANNEL_DISABLED;
   }

   if (mv >= 0 && mv <= 16000) {
        ret = _setThreshold(ch, mv);
        if (ret) return AD7441XR_ERR_COMM_FAIL;
   }
   else {
       return AD7441XR_ERR_INVALID_PARAM;
   }

   return AD7441XR_SUCCESS;
}

/**
 * @brief Gets Live status value
 * @return Live status bits in case of success, negative error code otherwise.
 */
int AD7441XR::getTemp(int ch)
{
   if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
       return AD7441XR_ERR_INVALID_PARAM;
   }

   uint16_t temp;
   int ret;

   ret = _getTemp(ch, &temp);
   if (ret) return AD7441XR_ERR_COMM_FAIL;
   
   return temp;
}

/**
 * @brief Gets configuration structure
 * @return Configuration structure.
 */
ad7441xr_cfg AD7441XR::getCfg()
{
   return cfg;
}

/**
 * @brief Update busy & ready state from current live status
 * @param status - Current live status.
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_updateBusyRdy(union ad7441xr_live_status status)
{
	uint16_t val = status.value;
	int rdy;
	int busy;

	rdy = 1 - no_os_field_get(AD7441XR_ADC_RDY_MASK, val);
	busy = no_os_field_get(AD7441XR_ADC_BUSY_MASK, val);

	cfg.adc_busy = busy;
	cfg.adc_rdy = rdy;

	return AD7441XR_SUCCESS;
}

/**
 * @brief Convert the raw ADC value to real, according to the operation mode.
 * @param raw - The ADC raw value.
 * @param mode - The current ADC mode.
 * @param val - The ADC real measurement value.
 * @param unit - The ADC unit (depends on the operation mode).
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_adcRawToReal(uint16_t code, ad7441xr_op_mode mode, float *val, ad7441xr_adc_unit *unit)
{	
	int ret;
	float result;
	float adc_code = code;

	switch (mode) {
	case AD7441XR_HIGH_Z:
		{result = (adc_code * AD7441XR_RANGE_10V_SCALE) / AD7441XR_RANGE_10V_SCALE_DIV;
		*val = result/1000.0;
		*unit = U_V;
		break;}
	// case AD7441XR_VOLTAGE_OUT: // Former : measure output current in Voltage Output mode
	// 	/**
	// 	 * I_Rsense = (Vmin + (ADC_CODE/65535) * range) / Rsense
	// 	 */
	// 	{result = ((adc_code + AD7441XR_RANGE_5V_OFFSET) * AD7441XR_RANGE_5V_SCALE) / (AD7441XR_RSENSE * AD7441XR_RANGE_5V_SCALE_DIV);
	// 	*val = result/1000.0;
	// 	*unit = U_MA;
	// 	break;}
	// case AD7441XR_CURRENT_OUT: // Former : measure terminal voltage in Current Output mode
	case AD7441XR_VOLTAGE_OUT: // Modified : measure terminal voltage in Voltage Output mode
	case AD7441XR_VOLTAGE_IN:
		{result = (adc_code * AD7441XR_RANGE_10V_SCALE) / AD7441XR_RANGE_10V_SCALE_DIV;
		*val = result/1000.0;
		*unit = U_V;
		break;}
	case AD7441XR_CURRENT_IN_EXT_HART:
		if (cfg.chip_id == ad74412r)
			return AD7441XR_ERR_NOT_SUPPORTED;
	/* fallthrough */
	case AD7441XR_CURRENT_IN_LOOP_HART:
		if (cfg.chip_id == ad74412r)
			return AD7441XR_ERR_NOT_SUPPORTED;
	/* fallthrough */
	case AD7441XR_CURRENT_OUT: // Modified : measure output current in Current Output mode
	case AD7441XR_CURRENT_IN_EXT:
	case AD7441XR_CURRENT_IN_LOOP:
		{result = (adc_code * AD7441XR_RANGE_2V5_SCALE) / (AD7441XR_RANGE_2V5_SCALE_DIV * AD7441XR_RSENSE);
		*val = result;
		*unit = U_MA;
		break;}
	case AD7441XR_RESISTANCE:
		{result = (adc_code * AD7441XR_RTD_PULL_UP) / (AD7441XR_ADC_MAX_VALUE - adc_code);
		*val = result;
		*unit = U_OHM;
		break;}
	case AD7441XR_DIGITAL_INPUT:
	case AD7441XR_DIGITAL_INPUT_LOOP:
		{result = (adc_code * AD7441XR_RANGE_10V_SCALE) / AD7441XR_RANGE_10V_SCALE_DIV;
		*val = result/1000.0;
		*unit = U_V;
		break;}
	default:
		return AD7441XR_ERR_INVALID_MODE;
	}

	return AD7441XR_SUCCESS;
}

/**
 * @brief Set the threshold, for which a signal would be considered high,
 * when the ADC is running in digital input mode.
 * @param ch - The channel index.
 * @param threshold - The threshold value (in millivolts). The actual threshold
 * set might not match this value (~500mV max. error), since it's fairly
 * low resolution (29 possible values).
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_setThreshold(uint32_t ch, uint32_t mv)
{
	int ret;
	uint32_t dac_threshold;

	if (mv > AD7441XR_THRESHOLD_RANGE)
		return AD7441XR_ERR_INVALID_PARAM;

	/** Set a fixed range (0 - 16 V) for the threshold, so it would not depend on Vadd. */
	ret = _updateRegister(AD7441XR_DIN_THRESH, AD7441XR_DIN_THRESH_MODE_MASK, 1);
	if (ret)
		return ret;

	dac_threshold = AD7441XR_THRESHOLD_DAC_RANGE * mv / AD7441XR_THRESHOLD_RANGE;

	ret = _updateRegister(AD7441XR_DIN_THRESH, AD7441XR_COMP_THRESH_MASK, dac_threshold);

	if (ret)
		return ret;

	long real_threshold = (dac_threshold * AD7441XR_THRESHOLD_RANGE) / AD7441XR_THRESHOLD_DAC_RANGE;
	cfg.channel[ch].din_threshold = real_threshold;

	return 0;
}

/**
 * @brief Set the debounce time,
 * when the ADC is running in digital input mode.
 * @param ch - The channel index.
 * @param deb - The debounce setting (0-31). Please see datasheet for debounce time correspondance.
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_setDebounce(uint32_t ch, uint16_t deb)
{
	int ret;
	uint32_t debounce;

	if (deb > 31)
		return AD7441XR_ERR_INVALID_PARAM;

	ret = _updateRegister(AD7441XR_DIN_CONFIG(ch), AD7441XR_DEBOUNCE_TIME_MASK, deb);
	if (ret)
		return ret;

	cfg.channel[ch].din_debounce = deb;

	return 0;
}

/**
 * @brief Converts a millivolt value in the corresponding DAC 13 bit code.
 * @param mvolts - The millivolts value.
 * @param code - The resulting DAC code.
 * @return 0 in case of success, AD7441XR_ERR_INVALID_PARAM otherwise
 */
int AD7441XR::_dacVoltageToCode(uint32_t mvolts, uint32_t *code)
{
	if (mvolts > AD7441XR_DAC_RANGE)
		return AD7441XR_ERR_INVALID_PARAM;

	*code = mvolts * NO_OS_BIT(AD7441XR_DAC_RESOLUTION) / AD7441XR_DAC_RANGE;

	return 0;
}

/**
 * @brief Set and load a code for the DAC on a specific channel.
 * @param ch - The channel index.
 * @param dac_code - The code for the DAC.
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_setDacCode(uint32_t ch, uint16_t dac_code)
{
	int ret;

	ret = _writeRegister(AD7441XR_DAC_CODE(ch), dac_code);
	if (ret)
		return ret;

	ret = _writeRegister(AD7441XR_CMD_KEY, AD7441XR_CMD_KEY_LDAC);
	if (ret)
		return ret;
	
	cfg.channel[ch].dac_raw = dac_code;
	return 0;
}

/**
 * @brief Poll the ADC and update configuration.
 * @param ch - The channel index.
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_pollAdc(uint32_t ch)
{
	uint16_t raw;
	int ret = _getRawAdcResult(ch, &raw);

	if (ret) return ret;

	float real;
	ad7441xr_adc_unit unit;
	ret = _adcRawToReal(raw, cfg.channel[ch].func, &real, &unit);

	if (ret) return ret;

	cfg.channel[ch].adc_raw = raw;
	cfg.channel[ch].adc_real = real;
	cfg.channel[ch].adc_unit = unit;
	cfg.channel[ch].adc_timestamp = millis();

	if ((cfg.channel[ch].func == AD7441XR_DIGITAL_INPUT) || (cfg.channel[ch].func == AD7441XR_DIGITAL_INPUT_LOOP)) {
		uint16_t state;
		ret = _getDiState(ch, &state);
		if (ret) return ret;
	}

	return 0;
}

/**
 * @brief Read the die's temperature from the diagnostic register.
 * @param ch - The diagnostic channel on which the temperature reading
 * is assigned and enabled.
 * @param temp - The measured temperature (in degrees Celsius).
 * @return 0 in case of success, error value otherwise.
 */
int AD7441XR::_getTemp(uint32_t ch, uint16_t *temp)
{
	int ret;

	ret = _getDiag(ch, temp);
	if (ret)
		return ret;

	*temp = (*temp + AD7441XR_TEMP_OFFSET) * AD7441XR_TEMP_SCALE_DIV /
		AD7441XR_TEMP_SCALE;

	return 0;
}

/**
 * @brief Get the diagnostic value for a specific channel.
 * @param ch - The channel index.
 * @param diag_code - The diagnostic setting.
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_getDiag(uint32_t ch, uint16_t *diag_code)
{
	int ret;

	ret = _readRegister(AD7441XR_DIAG_RESULT(ch), diag_code);
	if (ret) return ret;

	*diag_code = no_os_field_get(AD7441XR_DIAG_RESULT_MASK, *diag_code);

	return 0;
}

/**
 * @brief Set the ADC mux mode
 * @param ch - The channel index.
 * @param mux - The mux operating mode (IO terminals or RSENSE).
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_setMux(uint32_t ch, enum ad7441xr_adc_mux mux)
{
	int ret;

	ret = _updateRegister(AD7441XR_ADC_CONFIG(ch), AD7441XR_ADC_MUX_MASK, mux);
	if (ret) return ret;

	return 0;
}

/**
 * @brief Set the ADC range
 * @param ch - The channel index.
 * @param range - The range setting.
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_setRange(uint32_t ch, enum ad7441xr_adc_range range)
{
	int ret;

	ret = _updateRegister(AD7441XR_ADC_CONFIG(ch), AD7441XR_ADC_RANGE_MASK, range);
	if (ret) return ret;

	return 0;
}

/**
 * @brief Get DIN state and update configuration
 * @param ch - The channel index.
 * @param state - The DIN state.
 * @return 0 in case of success, negative error code otherwise.
 */
int AD7441XR::_getDiState(uint32_t ch, uint16_t *state)
{
	int ret;

	ret = _readRegister(AD7441XR_DIN_COMP_OUT, state);
	if (ret) return ret;

	*state = no_os_field_get(AD7441XR_DIN_COMP_OUT_MASK(ch), *state);

	cfg.channel[ch].din_state = (int)*state;

	return 0;
}

/**
 * @brief Set the comparison mode for the digital input
 * @param ch - The channel index
 * @param useThreshold - True if threshold mode should be used, false if hysteresis mode should be used
 * @return 0 in case of success, negative error code otherwise
 */
int AD7441XR::setDiCompMode(int ch, bool useThreshold) {
    if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
        return AD7441XR_ERR_INVALID_PARAM;
    }

    if (!cfg.channel[ch].enabled) {
        return AD7441XR_ERR_CHANNEL_DISABLED;
    }

    // Bit 6 du registre DIN_CONFIG contrôle le mode de comparaison
    // 0 = mode hystérésis, 1 = mode seuil
    int ret = _updateRegister(AD7441XR_DIN_CONFIG(ch), 
                            NO_OS_BIT(6), 
                            useThreshold ? 1 : 0);
    if (ret) return AD7441XR_ERR_COMM_FAIL;

    return AD7441XR_SUCCESS;
}

/**
 * @brief Set GPO mode for a channel
 * @param ch - The channel index (0-3)
 * @param mode - GPO operating mode
 * @return 0 in case of success, negative error code otherwise
 */
int AD7441XR::setGpoMode(int ch, enum ad7441xr_gpo_mode mode) {
    if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
        return AD7441XR_ERR_INVALID_PARAM;
    }

    if (!cfg.channel[ch].enabled) {
        return AD7441XR_ERR_CHANNEL_DISABLED;
    }

    int ret = _updateRegister(AD7441XR_GPO_CONFIG(ch), 
                            AD7441XR_GPO_SELECT_MASK,
                            mode);
    if (ret) return AD7441XR_ERR_COMM_FAIL;

    return AD7441XR_SUCCESS;
}

/**
 * @brief Set GPO output state for a channel (only works in GPO_DATA mode)
 * @param ch - The channel index (0-3)
 * @param state - Output state (true = high, false = low)
 * @return 0 in case of success, negative error code otherwise
 */
int AD7441XR::setGpoOutput(int ch, bool state) {
    if (ch < 0 || ch >= AD7441XR_N_CHANNELS) {
        return AD7441XR_ERR_INVALID_PARAM;
    }

    if (!cfg.channel[ch].enabled) {
        return AD7441XR_ERR_CHANNEL_DISABLED;
    }

    int ret = _updateRegister(AD7441XR_GPO_CONFIG(ch), 
                            AD7441XR_GPO_DATA_MASK,
                            state ? 1 : 0);
    if (ret) return AD7441XR_ERR_COMM_FAIL;

    return AD7441XR_SUCCESS;
}

/**
 * @brief Set all GPO states simultaneously (only works when GPOs are in PARALLEL mode)
 * @param states - Bit mask for all channels (bit 0 = channel A, bit 1 = channel B, etc)
 * @return 0 in case of success, negative error code otherwise
 */
int AD7441XR::setGpoParallel(uint8_t states) {
    int ret = _writeRegister(AD7441XR_GPO_PARALLEL, states & 0x0F);
    if (ret) return AD7441XR_ERR_COMM_FAIL;

    return AD7441XR_SUCCESS;
}
