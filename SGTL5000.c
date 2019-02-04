/*
 * SGTL5000.c
 *
 *  Created on: Sep 19, 2018
 *      Author: jurajh
 *
 *  TODO: D_PROGRAMMING bit in CHIP_LINREG_CTRL register
 */
#include "SGTL5000.h"
#include "main.h"
#include "stm32h743xx.h"
#include <string.h>

/* global variables */
I2C_HandleTypeDef hi2c2;

/* private function declarations */
static void I2C_Init(void);
static void writeRegister(uint16_t regAddr, uint16_t value);
static uint16_t readRegister(uint16_t regAddr);
static void setBit(uint16_t regAddr, uint8_t bitPos);
static void clearBit(uint16_t regAddr, uint8_t bitPos);

/* SGTL 5000 read functions ------------------------------------------------------------------- */
uint8_t sgtlI2CInit()
{
	I2C_Init();

	if (sgtlReadPartId() == PART_ID)
	{
		return 0x0;
	}

	return 0x1;
}

uint8_t sgtlReadPartId()
{
	return (uint8_t)(((readRegister(CHIP_ID) & PARTID_Msk) >> PARTID_Pos));
}

uint8_t sgtlReadRevId()
{
	return (uint8_t)(((readRegister(CHIP_ID) & REVID_Msk) >> REVID_Pos));
}
uint8_t sgtlReadHeadPhoneShort()
{
	return (uint8_t)((readRegister(CHIP_ANA_STATUS) & LRSHORT_STS_Msk) >> LRSHORT_STS_Pos);
}

uint8_t sgtlReadHeadPhoneCaplessShort()
{
	return (uint8_t)((readRegister(CHIP_ANA_STATUS) & CSHORT_STS_Msk) >> CSHORT_STS_Pos);
}

uint8_t sgtlReadPLLState()
{
	return (uint8_t)((readRegister(CHIP_ANA_STATUS) & PLL_IS_LOCKED_Msk) >> PLL_IS_LOCKED_Pos);
}

/* SGTL 5000 write functions ------------------------------------------------------------------- */
void sgtlADCBlockEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_DIG_POWER, ADC_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_DIG_POWER, ADC_POWERUP_Pos);
	}
}

void sgtlDACBlockEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_DIG_POWER, DAC_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_DIG_POWER, DAC_POWERUP_Pos);
	}
}

void sgtlDAPBlockEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_DIG_POWER, DAP_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_DIG_POWER, DAP_POWERUP_Pos);
	}
}

void sgtlI2SOutEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_DIG_POWER, I2S_OUT_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_DIG_POWER, I2S_OUT_POWERUP_Pos);
	}
}

void sgtlI2SInEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_DIG_POWER, I2S_IN_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_DIG_POWER, I2S_IN_POWERUP_Pos);
	}
}

void sgtlI2SMode(uint8_t ms, uint8_t sclkFreq, uint8_t sclkInv, uint8_t dLen,
				 uint8_t i2sMode, uint8_t lrAlign, uint8_t lrPol)
{

	// // TODO: check valid bit combinations
	// uint16_t tmp = 0x0000;

	// // set master/slave bit
	// tmp |= ((ms << MS_Pos) & MS_Msk);

	// // set output frequency
	// tmp |= ((sclkFreq << SCLKFREQ_Pos) & SCLKFREQ_Msk);

	// // set clock edge
	// tmp |= ((sclkInv << SCLK_INV_Pos) & SCLK_INV_Msk);

	// // set data bit length
	// tmp |= ((dLen << DLEN_Pos) & DLEN_Msk);

	// // set I2S mode
	// tmp |= ((i2sMode << I2S_MODE_Pos) & I2S_MODE_Msk);

	// // set data alignment
	// tmp |= ((lrAlign << LRALIGN_Pos) & LRALIGN_Msk);

	// // set I2S_LRCLK polarity
	// tmp |= ((lrPol << LRPOL_Pos) & LRPOL_Msk);

	// PCM format A
	writeRegister(CHIP_I2S_CTRL, 0x0000);
}

void sgtlSampleRateMode(uint8_t rateMode)
{
	uint16_t tmp = readRegister(CHIP_CLK_CTRL);
	tmp &= ~RATE_MODE_Msk;
	writeRegister(CHIP_CLK_CTRL, tmp | (((rateMode << RATE_MODE_Pos)) & RATE_MODE_Msk));
}

void sgtlSystemSampleRate(uint8_t sysFs)
{
	uint16_t tmp = readRegister(CHIP_CLK_CTRL);
	tmp &= ~SYS_FS_Msk;
	writeRegister(CHIP_CLK_CTRL, tmp | (((sysFs << SYS_FS_Pos)) & SYS_FS_Msk));
}

void sgtlMasterClock(uint8_t mclkFreq)
{
	uint16_t tmp = readRegister(CHIP_CLK_CTRL);
	tmp &= ~MCLK_FREQ_Msk;
	writeRegister(CHIP_CLK_CTRL, tmp | (((mclkFreq << MCLK_FREQ_Pos)) & MCLK_FREQ_Msk));
}

void sgtlDAPMixerInputSwap(uint8_t lrSwap)
{
	if (lrSwap == LRSWAP_SWAP)
	{
		setBit(CHIP_SSS_CTRL, DAP_MIX_LRSWAP_Pos);
	}
	else
	{
		clearBit(CHIP_SSS_CTRL, DAP_MIX_LRSWAP_Pos);
	}
}

void sgtlDAPInputSwap(uint8_t lrSwap)
{
	if (lrSwap == LRSWAP_SWAP)
	{
		setBit(CHIP_SSS_CTRL, DAP_LRSWAP_Pos);
	}
	else
	{
		clearBit(CHIP_SSS_CTRL, DAP_LRSWAP_Pos);
	}
}

void sgtlDACInputSwap(uint8_t lrSwap)
{
	if (lrSwap == LRSWAP_SWAP)
	{
		setBit(CHIP_SSS_CTRL, DAC_LRSWAP_Pos);
	}
	else
	{
		clearBit(CHIP_SSS_CTRL, DAC_LRSWAP_Pos);
	}
}

void sgtlI2SInputSwap(uint8_t lrSwap)
{
	if (lrSwap == LRSWAP_SWAP)
	{
		setBit(CHIP_SSS_CTRL, I2S_LRSWAP_Pos);
	}
	else
	{
		clearBit(CHIP_SSS_CTRL, I2S_LRSWAP_Pos);
	}
}

void sgtlDAPMixInput(uint8_t input)
{
	if (input == INPUT_I2S_IN)
	{
		setBit(CHIP_SSS_CTRL, DAP_MIX_SELECT_Pos);
	}
	else
	{
		clearBit(CHIP_SSS_CTRL, DAP_MIX_SELECT_Pos);
	}
}

void sgtlDAPInput(uint8_t input)
{
	if (input == INPUT_I2S_IN)
	{
		setBit(CHIP_SSS_CTRL, DAP_SELECT_Pos);
	}
	else
	{
		clearBit(CHIP_SSS_CTRL, DAP_SELECT_Pos);
	}
}

void sgtlDACInput(uint8_t input)
{
	uint16_t tmp = readRegister(CHIP_SSS_CTRL);
	tmp &= ~DAC_SELECT_Msk;
	writeRegister(CHIP_SSS_CTRL, tmp | ((input << DAC_SELECT_Pos) & DAC_SELECT_Msk));
}

void sgtlI2SOutput(uint8_t input)
{
	uint16_t tmp = readRegister(CHIP_SSS_CTRL);
	tmp &= ~I2S_SELECT_Msk;
	writeRegister(CHIP_SSS_CTRL, tmp | ((input << I2S_SELECT_Pos) & I2S_SELECT_Msk));
}

// BULLSHIT TODO TODO mANY TODO
void sgtlVolumeBusyDAC(uint8_t volBusyRight, uint8_t volBusyLeft)
{
	uint16_t tmp = readRegister(CHIP_ADCDAC_CTRL);
	tmp &= ~(VOL_BUSY_DAC_RIGHT_Msk | VOL_BUSY_DAC_RIGHT_Msk);

	tmp |= ((volBusyRight << VOL_BUSY_DAC_RIGHT_Pos) & VOL_BUSY_DAC_RIGHT_Msk);
	tmp |= ((volBusyLeft << VOL_BUSY_DAC_LEFT_Pos) & VOL_BUSY_DAC_LEFT_Msk);

	writeRegister(CHIP_ADCDAC_CTRL, tmp);
}

void sgtlVolumeRampEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_ADCDAC_CTRL, VOL_RAMP_EN_Pos);
	}
	else
	{
		clearBit(CHIP_ADCDAC_CTRL, VOL_RAMP_EN_Pos);
	}
}

void sgtlVolumeRampType(uint8_t rampType)
{
	if (rampType == RAMP_EXPONENTIAL)
	{
		setBit(CHIP_ADCDAC_CTRL, VOL_EXPO_RAMP_Pos);
	}
	else
	{
		clearBit(CHIP_ADCDAC_CTRL, VOL_EXPO_RAMP_Pos);
	}
}

void sgtlDACMute(uint8_t muteLeft, uint8_t muteRight)
{
	uint16_t tmp = readRegister(CHIP_ADCDAC_CTRL);
	tmp &= ~(DAC_MUTE_RIGHT_Msk | DAC_MUTE_LEFT_Msk);

	tmp |= ((muteRight << DAC_MUTE_RIGHT_Pos) & DAC_MUTE_RIGHT_Msk);
	tmp |= ((muteLeft << DAC_MUTE_LEFT_Pos) & DAC_MUTE_LEFT_Msk);

	writeRegister(CHIP_ADCDAC_CTRL, tmp);
}

void sgtlADCHPFMode(uint8_t adcHpfMode)
{
	uint16_t tmp = readRegister(CHIP_ADCDAC_CTRL);
	tmp &= ~(ADC_HPF_BYPASS_Msk | ADC_HPF_FREEZE_Msk);

	if (adcHpfMode == ADC_HPF_BYPASS)
	{
		tmp |= ((0x1 << ADC_HPF_BYPASS_Pos) & ADC_HPF_BYPASS_Msk);
	}
	else if (adcHpfMode == ADC_HPF_FREEZE)
	{
		tmp |= ((0x1 << ADC_HPF_FREEZE_Pos) & ADC_HPF_FREEZE_Msk);
	};

	writeRegister(CHIP_ADCDAC_CTRL, tmp);
}

void sgtlDACVolRight(uint8_t volume)
{
	uint16_t tmp = readRegister(CHIP_DAC_VOL);
	tmp &= ~DAC_VOL_RIGHT_Msk;
	writeRegister(CHIP_DAC_VOL, tmp | ((volume << DAC_VOL_RIGHT_Pos) & DAC_VOL_RIGHT_Msk));
}

void sgtlDACVolLeft(uint8_t volume)
{
	uint16_t tmp = readRegister(CHIP_DAC_VOL);
	tmp &= ~DAC_VOL_LEFT_Msk;
	writeRegister(CHIP_DAC_VOL, tmp | ((volume << DAC_VOL_LEFT_Pos) & DAC_VOL_LEFT_Msk));
}

void sgtlADCVol6dbReductEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_ANA_ADC_CTRL, ADC_VOL_M6DB_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_ADC_CTRL, ADC_VOL_M6DB_Pos);
	}
}

void sgtlADCVolRight(uint8_t volume)
{
	uint16_t tmp = readRegister(CHIP_ANA_ADC_CTRL);
	tmp &= ~ADC_VOL_RIGHT_Msk;
	writeRegister(CHIP_ANA_ADC_CTRL, tmp | ((volume << ADC_VOL_RIGHT_Pos) & ADC_VOL_RIGHT_Msk));
}

void sgtlADCVolLeft(uint8_t volume)
{
	uint16_t tmp = readRegister(CHIP_ANA_ADC_CTRL);
	tmp &= ~ADC_VOL_LEFT_Msk;
	writeRegister(CHIP_ANA_ADC_CTRL, tmp | ((volume << ADC_VOL_LEFT_Pos) & ADC_VOL_LEFT_Msk));
}

void sgtlHeadPhoneVolRight(uint8_t volume)
{
	uint16_t tmp = readRegister(CHIP_ANA_HP_CTRL);
	tmp &= ~HP_VOL_RIGHT_Msk;
	writeRegister(CHIP_ANA_HP_CTRL, tmp | ((volume << HP_VOL_RIGHT_Pos) & HP_VOL_RIGHT_Msk));
}

void sgtlHeadPhoneVolLeft(uint8_t volume)
{
	uint16_t tmp = readRegister(CHIP_ANA_HP_CTRL);
	tmp &= ~HP_VOL_LEFT_Msk;
	writeRegister(CHIP_ANA_HP_CTRL, tmp | ((volume << HP_VOL_LEFT_Pos) & HP_VOL_LEFT_Msk) | ((volume << HP_VOL_RIGHT_Pos) & HP_VOL_RIGHT_Msk));
}

void sgtlLineOutMute(uint8_t mute)
{
	if (mute == MUTE)
	{
		setBit(CHIP_ANA_CTRL, MUTE_LO_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_CTRL, MUTE_LO_Pos);
	}
}

void sgtlHeadPhoneInput(uint8_t hpInput)
{
	if (hpInput == HP_INPUT_LINE_IN)
	{
		setBit(CHIP_ANA_CTRL, SELECT_HP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_CTRL, SELECT_HP_Pos);
	}
}

void sgtlHeadPhoneZCEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_ANA_CTRL, EN_ZCD_HP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_CTRL, EN_ZCD_HP_Pos);
	}
}

void sgtlHeadPhoneMute(uint8_t mute)
{
	if (mute == MUTE)
	{
		setBit(CHIP_ANA_CTRL, MUTE_HP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_CTRL, MUTE_HP_Pos);
	}
}

void sgtlADCInput(uint8_t adcInput)
{
	if (adcInput == ADC_INPUT_LINE_IN)
	{
		setBit(CHIP_ANA_CTRL, SELECT_ADC_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_CTRL, SELECT_ADC_Pos);
	}
}

void sgtlAnalogADCZCEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_ANA_CTRL, EN_ZCD_ADC_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_CTRL, EN_ZCD_ADC_Pos);
	}
}

void sgtlADCAnalogMute(uint8_t mute)
{
	if (mute == MUTE)
	{
		setBit(CHIP_ANA_CTRL, MUTE_ADC_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_CTRL, MUTE_ADC_Pos);
	}
}

void sgtlChargePumpOverrideEnable(uint8_t enable, uint8_t chargePumpSrc)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_LINREG_CTRL, VDDC_ASSN_OVRD_Pos);
		if (chargePumpSrc == CHARGE_PUMP_SRC_VDDIO)
		{
			setBit(CHIP_LINREG_CTRL, VDDC_MAN_ASSN_Pos);
		}
		else
		{
			clearBit(CHIP_LINREG_CTRL, VDDC_MAN_ASSN_Pos);
		}
	}
	else
	{
		clearBit(CHIP_LINREG_CTRL, VDDC_ASSN_OVRD_Pos);
	}
}

void sgtlAnalogGroundRefVoltage(uint8_t value)
{
	uint16_t tmp = readRegister(CHIP_REF_CTRL);
	tmp &= ~VAG_VAL_Msk;
	writeRegister(CHIP_REF_CTRL, tmp | ((value << VAG_VAL_Pos) & VAG_VAL_Msk));
}

void sgtlAnalogBiasCurrent(uint8_t value)
{
	uint16_t tmp = readRegister(CHIP_REF_CTRL);
	tmp &= ~BIAS_CTRL_Msk;
	writeRegister(CHIP_REF_CTRL, tmp | ((value << BIAS_CTRL_Pos) & BIAS_CTRL_Msk));
}

void sgtlAnalogSmallPopEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_REF_CTRL, SMALL_POP_Pos);
	}
	else
	{
		clearBit(CHIP_REF_CTRL, SMALL_POP_Pos);
	}
}

void sgtlMICBiasImpedance(uint8_t biasImpedance)
{
	uint16_t tmp = readRegister(CHIP_MIC_CTRL);
	tmp &= ~BIAS_RESISTOR_Msk;
	writeRegister(CHIP_MIC_CTRL, tmp | ((biasImpedance << BIAS_RESISTOR_Pos) & BIAS_RESISTOR_Msk));
}

void sgtlMICBiasVoltage(uint8_t biasVoltage)
{
	uint16_t tmp = readRegister(CHIP_MIC_CTRL);
	tmp &= ~BIAS_VOLT_Msk;
	writeRegister(CHIP_MIC_CTRL, tmp | ((biasVoltage << BIAS_VOLT_Pos) & BIAS_VOLT_Msk));
}

void sgtlMICGain(uint8_t micGain)
{
	uint16_t tmp = readRegister(CHIP_MIC_CTRL);
	tmp &= ~GAIN_Msk;
	writeRegister(CHIP_MIC_CTRL, tmp | ((micGain << GAIN_Pos) & GAIN_Msk));
}

void sgtlLineOutCurrent(uint8_t outCurrent)
{
	uint16_t tmp = readRegister(CHIP_LINE_OUT_CTRL);
	tmp &= ~OUT_CURRENT_Msk;
	writeRegister(CHIP_LINE_OUT_CTRL, tmp | ((outCurrent << OUT_CURRENT_Pos) & OUT_CURRENT_Msk));
}

void sgtlLineOutGroundVoltage(uint8_t value)
{
	uint16_t tmp = readRegister(CHIP_LINE_OUT_CTRL);
	tmp &= ~LO_VAGCNTRL_Msk;
	writeRegister(CHIP_LINE_OUT_CTRL, tmp | ((value << LO_VAGCNTRL_Pos) & LO_VAGCNTRL_Msk));
}

void sgtlLineOutVolRight(uint8_t volume)
{
	uint16_t tmp = readRegister(CHIP_LINE_OUT_VOL);
	tmp &= ~LO_VOL_RIGHT_Msk;
	writeRegister(CHIP_LINE_OUT_VOL, tmp | ((volume << LO_VOL_RIGHT_Pos) & LO_VOL_RIGHT_Msk));
}

void sgtlLineOutVolLeft(uint8_t volume)
{
	uint16_t tmp = readRegister(CHIP_LINE_OUT_VOL);
	tmp &= ~LO_VOL_LEFT_Msk;
	writeRegister(CHIP_LINE_OUT_VOL, tmp | ((volume << LO_VOL_LEFT_Pos) & LO_VOL_LEFT_Msk));
}

void sgtlDACMonoEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		clearBit(CHIP_ANA_POWER, DAC_MONO_Pos);
	}
	else
	{
		setBit(CHIP_ANA_POWER, DAC_MONO_Pos);
	}
}

void sgtlSimpleRegulatorPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, LINREG_SIMPLE_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, LINREG_SIMPLE_POWERUP_Pos);
	}
}

void sgtlStartupCircuitPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, STARTUP_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, STARTUP_POWERUP_Pos);
	}
}

void sgtlChargePumpPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, VDDC_CHRGPMP_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, VDDC_CHRGPMP_POWERUP_Pos);
	}
}

void sgtlPLLPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, PLL_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, PLL_POWERUP_Pos);
	}
}

void sgtlVDDDRegulatorPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, LINREG_D_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, LINREG_D_POWERUP_Pos);
	}
}

void sgtlPLLVCOPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, VCOAMP_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, VCOAMP_POWERUP_Pos);
	}
}

void sgtlVAGPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, VAG_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, VAG_POWERUP_Pos);
	}
}

void sgtlADCMonoEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		clearBit(CHIP_ANA_POWER, ADC_MONO_Pos);
	}
	else
	{
		setBit(CHIP_ANA_POWER, ADC_MONO_Pos);
	}
}

void sgtlReferenceBiasPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, REFTOP_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, REFTOP_POWERUP_Pos);
	}
}

void sgtlHeadPhoneAmpPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, HEADPHONE_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, HEADPHONE_POWERUP_Pos);
	}
}

void sgtlDACPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, DAC_POWERUP_ANA_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, DAC_POWERUP_ANA_Pos);
	}
}

void sgtlHeadPhoneCaplessEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_ANA_POWER, CAPLESS_HEADPHONE_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, CAPLESS_HEADPHONE_POWERUP_Pos);
	}
}

void sgtlADCPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, ADC_POWERUP_ANA_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, ADC_POWERUP_ANA_Pos);
	}
}

void sgtlLineOutPower(uint8_t power)
{
	if (power == POWER_UP)
	{
		setBit(CHIP_ANA_POWER, LINEOUT_POWERUP_Pos);
	}
	else
	{
		clearBit(CHIP_ANA_POWER, LINEOUT_POWERUP_Pos);
	}
}

void sgtlPLLIntDivisor(uint8_t intDivisor)
{
	uint16_t tmp = readRegister(CHIP_PLL_CTRL);
	tmp &= ~INT_DIVISOR_Msk;
	writeRegister(CHIP_PLL_CTRL, tmp | ((intDivisor << INT_DIVISOR_Pos) & INT_DIVISOR_Msk));
}

void sgtlPLLFracDivisor(uint16_t fracDivisor)
{
	uint16_t tmp = readRegister(CHIP_PLL_CTRL);
	tmp &= ~FRAC_DIVISOR_Msk;
	writeRegister(CHIP_PLL_CTRL, tmp | ((fracDivisor << FRAC_DIVISOR_Pos) & FRAC_DIVISOR_Msk));
}

void sgtlInternalZCOscEnable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_CLK_TOP_CTRL, ENABLE_INT_OSC_Pos);
	}
	else
	{
		clearBit(CHIP_CLK_TOP_CTRL, ENABLE_INT_OSC_Pos);
	}
}

void sgtlPLLInputDiv2Enable(uint8_t enable)
{
	if (enable == ENABLE_TRUE)
	{
		setBit(CHIP_CLK_TOP_CTRL, INPUT_FREQ_DIV2_Pos);
	}
	else
	{
		clearBit(CHIP_CLK_TOP_CTRL, INPUT_FREQ_DIV2_Pos);
	}
}

void sgtlTest()
{
	//setBit(CHIP_ADCDAC_CTRL, 1 << ADC_HPF_FREEZE_Pos);
	//setBit(CHIP_ADCDAC_CTRL, 1 << ADC_HPF_BYPASS_Pos);
	// uint16_t tmp = readRegister(CHIP_PAD_STRENGTH);
	// tmp &= ~I2S_DOUT_Msk;
	// writeRegister(CHIP_PAD_STRENGTH, tmp | ((0x3 << I2S_DOUT_Pos) & I2S_DOUT_Msk));
}

/* bit manipulation */
static void setBit(uint16_t regAddr, uint8_t bitPos)
{
	uint16_t tmp = readRegister(regAddr);
	writeRegister(regAddr, tmp | (0x1 << bitPos));
}

static void clearBit(uint16_t regAddr, uint8_t bitPos)
{
	uint16_t tmp = readRegister(regAddr);
	writeRegister(regAddr, tmp & ~(0x1 << bitPos));
}

/* -------------------------------- I2C glue functions --------------------------------- */

/* I2C write register glue function */
void writeRegister(uint16_t regAddr, uint16_t value)
{
	// most significant byte first
	uint8_t frame[] = {(uint8_t)(regAddr >> 8), (uint8_t)(regAddr & 0xFF),
					   (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, CODEC_ADDRW,
													   frame, 4, 10000);

	if (status != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* I2C read register glue function */
uint16_t readRegister(uint16_t regAddr)
{

	HAL_StatusTypeDef status;

	uint8_t buffer[2];

	uint8_t addrFrame[] =
		{
			(uint8_t)(regAddr >> 8),
			(uint8_t)(regAddr & 0xFF),
		};

	status = HAL_I2C_Master_Transmit(&hi2c2, CODEC_ADDRW, addrFrame, 2, 10000);

	if (status != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	status = HAL_I2C_Master_Receive(&hi2c2, CODEC_ADDRR, buffer, 2, 10000);

	if (status != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	uint16_t value = (uint16_t)((buffer[0] << 8) | buffer[1]);

	return value;
}

/* I2C init glue function */
static void I2C_Init(void)
{

	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10707DBC;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{

	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hi2c->Instance == I2C2)
	{

		// PF0     ------> I2C2_SDA
		// PF1     ------> I2C2_SCL

		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
		HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
	}
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{

	if (hi2c->Instance == I2C2)
	{
		__HAL_RCC_I2C2_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0 | GPIO_PIN_1);
	}
}