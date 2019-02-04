#include <inttypes.h>
#include "stm32h7xx_hal.h"

/* Codec I2C address */
#define CODEC_ADDRW							0b00010100
#define CODEC_ADDRR							0b00010101

/* Codec part ID */
#define PART_ID								0xA0

/* Chip registers */
#define CHIP_ID								0x0000
#define CHIP_DIG_POWER						0x0002
#define CHIP_CLK_CTRL						0x0004
#define CHIP_I2S_CTRL						0x0006
#define	CHIP_SSS_CTRL						0x000A
#define CHIP_ADCDAC_CTRL					0x000E
#define CHIP_DAC_VOL						0x0010
#define CHIP_PAD_STRENGTH					0x0014
#define CHIP_ANA_ADC_CTRL					0x0020
#define CHIP_ANA_HP_CTRL					0x0022
#define CHIP_ANA_CTRL						0x0024
#define CHIP_LINREG_CTRL					0x0026
#define CHIP_REF_CTRL						0x0028
#define CHIP_MIC_CTRL						0x002A
#define CHIP_LINE_OUT_CTRL					0x002C
#define	CHIP_LINE_OUT_VOL					0x002E
#define	CHIP_ANA_POWER						0x0030
#define	CHIP_PLL_CTRL						0x0032
#define CHIP_CLK_TOP_CTRL					0x0034
#define CHIP_ANA_STATUS						0x0036
#define CHIP_ANA_TEST2						0x003A
#define CHIP_SHORT_CTRL						0x003C

/* Digital audio processing subsystem registers */
#define DAP_CONTROL							0x0100
#define DAP_PEQ								0x0102
#define DAP_BASS_ENHANCE					0x0104
#define DAP_BASS_ENHANCE_CTRL				0x0106
#define DAP_AUDIO_EQ						0x0108
#define DAP_SGTL_SURROUND					0x010A
#define DAP_FILTER_COEF_ACCESS				0x010C
#define DAP_COEF_WR_B0_MSB					0x010E
#define DAP_COEF_WR_B0_LSB					0x0110
#define DAP_AUDIO_EQ_BASS_BAND0 			0x0116
#define DAP_AUDIO_EQ_BAND1					0x0118
#define DAP_AUDIO_EQ_BAND2					0x011A
#define DAP_AUDIO_EQ_BAND3					0x011C
#define DAP_AUDIO_EQ_TREBLE_BAND4			0x011E
#define DAP_MAIN_CHAN						0x0120
#define DAP_MIX_CHAN						0x0122
#define DAP_AVC_CTRL						0x0124
#define DAP_AVC_THRESHOLD					0x0126
#define DAP_AVC_ATTACK						0x0128
#define DAP_AVC_DECAY						0x012A
#define DAP_COEF_WR_B1_MSB					0x012C
#define DAP_COEF_WR_B1_LSB					0x012E
#define DAP_COEF_WR_B2_MSB					0x0130
#define DAP_COEF_WR_B2_LSB					0x0132
#define DAP_COEF_WR_A1_MSB					0x0134
#define DAP_COEF_WR_A1_LSB					0x0136
#define DAP_COEF_WR_A2_MSB					0x0138
#define DAP_COEF_WR_A2_LSB					0x013A

/* Bit definition for CHIP_ID register */
#define PARTID_Pos							8
#define PARTID_Msk							(0xFF << PARTID_Pos)
#define REVID_Pos							0
#define REVID_Msk							(0xFF << REVID_Pos)

/* Bit definition for CHIP_DIG_POWER register */
#define ADC_POWERUP_Pos						6
#define ADC_POWERUP_Msk						(0x1 << ADC_POWERUP_Pos)
#define DAC_POWERUP_Pos						5
#define DAC_POWERUP_Msk						(0x1 << DAC_POWERUP_Pos)
#define DAP_POWERUP_Pos						4
#define DAP_POWERUP_Msk						(0x1 << DAP_POWERUP_Pos)
#define	I2S_OUT_POWERUP_Pos					1
#define I2S_OUT_POWERUP_Msk					(0x1 << I2S_OUT_POWERUP_Pos)
#define I2S_IN_POWERUP_Pos					0
#define I2S_IN_POWERUP_Msk					(0x1 << I2S_IN_POWERUP_Pos)

/* Bit definition for CHIP_CLK_CTRL register */
#define RATE_MODE_Pos						4
#define RATE_MODE_Msk						(0x3 << RATE_MODE_Pos)
#define SYS_FS_Pos							2
#define SYS_FS_Msk							(0x3 << SYS_FS_Pos)
#define MCLK_FREQ_Pos						0
#define MCLK_FREQ_Msk						(0x3 << MCLK_FREQ_Pos)

/* Bit definition for CHIP_I2S_CTRL register */
#define SCLKFREQ_Pos						8
#define SCLKFREQ_Msk						(0x1 << SCLKFREQ_Pos)
#define MS_Pos								7
#define MS_Msk								(0x1 << MS_Pos)
#define SCLK_INV_Pos						6
#define SCLK_INV_Msk						(0x1 << SCLK_INV_Pos)
#define DLEN_Pos							4
#define DLEN_Msk							(0x3 << DLEN_Pos)
#define I2S_MODE_Pos						2
#define I2S_MODE_Msk						(0x3 << I2S_MODE_Pos)
#define LRALIGN_Pos							1
#define LRALIGN_Msk							(0x1 << LRALIGN_Pos)
#define LRPOL_Pos							0
#define LRPOL_Msk							(0x1 << LRPOL_Pos)

/* Bit definition for CHIP_SSS_CTRL register */
#define DAP_MIX_LRSWAP_Pos					14
#define DAP_MIX_LRSWAP_Msk					(0x1 << DAP_MIX_LRSWAP_Pos)
#define DAP_LRSWAP_Pos						13
#define DAP_LRSWAP_Msk						(0x1 << DAP_LRSWAP_Pos)
#define DAC_LRSWAP_Pos						12
#define DAC_LRSWAP_Msk						(0x1 << DAC_LRSWAP_Pos)
#define I2S_LRSWAP_Pos						10
#define I2S_LRSWAP_Msk						(0x1 << I2S_LRSWAP_Pos)
#define DAP_MIX_SELECT_Pos					8
#define DAP_MIX_SELECT_Msk					(0x3 << DAP_MIX_SELECT_Pos)
#define DAP_SELECT_Pos						6
#define DAP_SELECT_Msk						(0x3 << DAP_SELECT_Pos)
#define DAC_SELECT_Pos						4
#define DAC_SELECT_Msk						(0x3 << DAC_SELECT_Pos)
#define I2S_SELECT_Pos						0
#define I2S_SELECT_Msk						(0x3 << I2S_SELECT_Pos)

/* Bit definition for CHIP_ADCDAC_CTRL register */
#define VOL_BUSY_DAC_RIGHT_Pos				13
#define VOL_BUSY_DAC_RIGHT_Msk				(0x1 << VOL_BUSY_DAC_RIGHT_Pos)
#define VOL_BUSY_DAC_LEFT_Pos				12
#define VOL_BUSY_DAC_LEFT_Msk				(0x1 << VOL_BUSY_DAC_LEFT_Pos)
#define VOL_RAMP_EN_Pos						9
#define VOL_RAMP_EN_Msk						(0x1 << VOL_RAMP_EN_Pos)
#define VOL_EXPO_RAMP_Pos					8
#define VOL_EXPO_RAMP_Msk					(0x1 << VOL_EXPO_RAMP_Pos)
#define DAC_MUTE_RIGHT_Pos					3
#define DAC_MUTE_RIGHT_Msk					(0x1 << DAC_MUTE_RIGHT_Pos)
#define DAC_MUTE_LEFT_Pos					2
#define DAC_MUTE_LEFT_Msk					(0x1 << DAC_MUTE_LEFT_Pos)
#define ADC_HPF_FREEZE_Pos					1
#define ADC_HPF_FREEZE_Msk					(0x1 << ADC_HPF_FREEZE_Pos)
#define ADC_HPF_BYPASS_Pos					0
#define ADC_HPF_BYPASS_Msk					(0x1 << ADC_HPF_BYPASS_Pos)

/* Bit definition for CHIP_DAC_VOL register */
#define DAC_VOL_RIGHT_Pos					8
#define DAC_VOL_RIGHT_Msk					(0xFF << DAC_VOL_RIGHT_Pos)
#define DAC_VOL_LEFT_Pos					0
#define DAC_VOL_LEFT_Msk					(0xFF << DAC_VOL_LEFT_Pos)

/* Bit definition for CHIP_PAD_STRENGTH register */
#define I2S_LRCLK_Pos						8
#define I2S_LRCLK_Msk						(0x3 << I2S_LRCLK_Pos)
#define I2S_SCLK_Pos						6
#define I2S_SCLK_Msk						(0x3 << I2S_SCLK_Pos)
#define I2S_DOUT_Pos						4
#define I2S_DOUT_Msk						(0x3 << I2S_DOUT_Pos)
#define CTRL_DATA_Pos						2
#define CTRL_DATA_Msk						(0x3 << CTRL_DATA_Pos)
#define CTRL_CLK_Pos						0
#define CTRL_CLK_Msk						(0x3 << CTRL_CLK_Pos)

/* Bit definition for CHIP_ANA_ADC_CTRL register */
#define ADC_VOL_M6DB_Pos					8
#define ADC_VOL_M6DB_Msk					(0x1 << ADC_VOL_M6DB)
#define ADC_VOL_RIGHT_Pos					4
#define ADC_VOL_RIGHT_Msk					(0xF << ADC_VOL_RIGHT_Pos)
#define ADC_VOL_LEFT_Pos					0
#define ADC_VOL_LEFT_Msk					(0xF << ADC_VOL_LEFT_Pos)

/* Bit definition for CHIP_ANA_HP_CTRL register */
#define HP_VOL_RIGHT_Pos					8
#define HP_VOL_RIGHT_Msk					(0x7F << HP_VOL_RIGHT_Pos)
#define HP_VOL_LEFT_Pos						0
#define HP_VOL_LEFT_Msk						(0x7F << HP_VOL_LEFT_Pos)

/* Bit definition for CHIP_ANA_CTRL register */
#define MUTE_LO_Pos							8
#define MUTE_LO_Msk							(0x1 << MUTE_LO_Pos)
#define SELECT_HP_Pos						6
#define SELECT_HP_Msk						(0x1 << SELECT_HP_Pos)
#define EN_ZCD_HP_Pos						5
#define EN_ZCD_HP_Msk						(0x1 << EN_ZCD_HP_Pos)
#define MUTE_HP_Pos							4
#define MUTE_HP_Msk							(0x1 << MUTE_HP_Pos)
#define SELECT_ADC_Pos						2
#define SELECT_ADC_Msk						(0x1 << SELECT_ADC_Pos)
#define EN_ZCD_ADC_Pos						1
#define EN_ZCD_ADC_Msk						(0x1 << EN_ZCD_ADC_Pos)
#define MUTE_ADC_Pos						0
#define MUTE_ADC_Msk						(0x1 << MUTE_ADC_Pos)

/* Bit definition for CHIP_LINREG_CTRL register */
#define VDDC_MAN_ASSN_Pos					6
#define VDDC_MAN_ASSN_Msk					(0x1 << VDDC_MAN_ASSN_Pos)
#define VDDC_ASSN_OVRD_Pos					5
#define VDDC_ASSN_OVRD_Msk					(0x1 << VDDC_ASSN_OVRD_Pos)
#define D_PROGRAMMING_Pos					0
#define D_PROGRAMMING_Msk					(0xF << D_PROGRAMMING_Pos)

/* Bit definition for CHIP_REF_CTRL register */
#define VAG_VAL_Pos							4
#define VAG_VAL_Msk							(0x1F << VAG_VAL_Pos)
#define BIAS_CTRL_Pos						1
#define BIAS_CTRL_Msk						(0x7 << BIAS_CTRL_Pos)
#define SMALL_POP_Pos						0
#define SMALL_POP_Msk						(0x1 << SMALL_POP_Pos)

/* Bit definition for CHIP_MIC_CTRL register */
#define BIAS_RESISTOR_Pos					8
#define BIAS_RESISTOR_Msk					(0x3 << BIAS_RESISTOR_Pos)
#define BIAS_VOLT_Pos						4
#define BIAS_VOLT_Msk						(0x7 << BIAS_VOLT_Pos)
#define GAIN_Pos							0
#define GAIN_Msk							(0x3 << GAIN_Pos)

/* Bit definition for CHIP_LINE_OUT_CTRL register */
#define OUT_CURRENT_Pos						8
#define OUT_CURRENT_Msk						(0xF << OUT_CURRENT_Pos)
#define LO_VAGCNTRL_Pos						0
#define LO_VAGCNTRL_Msk						(0x3F << LO_VAGCNTRL_Pos)

/* Bit definition for CHIP_LINE_OUT_VOL register */
#define LO_VOL_RIGHT_Pos					8
#define LO_VOL_RIGHT_Msk					(0x1F << LO_VOL_RIGHT_Pos)
#define LO_VOL_LEFT_Pos						0
#define LO_VOL_LEFT_Msk						(0x1F << LO_VOL_LEFT_Pos)

/* Bit definition for CHIP_ANA_POWER register */
#define DAC_MONO_Pos						14
#define DAC_MONO_Msk						(0x1 << DAC_MONO_Pos)
#define LINREG_SIMPLE_POWERUP_Pos			13
#define LINREG_SIMPLE_POWERUP_Msk			(0x1 << LINREG_SIMPLE_POWERUP_Pos)
#define STARTUP_POWERUP_Pos					12
#define STARTUP_POWERUP_Msk					(0x1 << STARTUP_POWERUP_Pos)
#define VDDC_CHRGPMP_POWERUP_Pos			11
#define VDDC_CHRGPMP_POWERUP_Msk			(0x1 << VDDC_CHRGPMP_POWERUP)
#define PLL_POWERUP_Pos						10
#define PLL_POWERUP_Msk						(0x1 << PLL_POWERUP_Pos)
#define LINREG_D_POWERUP_Pos				9
#define LINREG_D_POWERUP_Msk				(0x1 << LINREG_D_POWERUP_Pos)
#define VCOAMP_POWERUP_Pos					8
#define VCOAMP_POWERUP_Msk					(0x1 << VCOAMP_POWERUP_Pos)
#define VAG_POWERUP_Pos						7
#define VAG_POWERUP_Msk						(0x1 << VAG_POWERUP_Pos)
#define ADC_MONO_Pos						6
#define ADC_MONO_Msk						(0x1 << ADC_MONO_Pos)
#define REFTOP_POWERUP_Pos					5
#define REFTOP_POWERUP_Msk					(0x1 << REFTOP_POWERUP_Pos)
#define HEADPHONE_POWERUP_Pos				4
#define HEADPHONE_POWERUP_Msk				(0x1 << HEADPHONE_POWERUP_Pos)
#define DAC_POWERUP_ANA_Pos					3
#define DAC_POWERUP_ANA_Msk					(0x1 << DAC_POWERUP_ANA_Pos)
#define CAPLESS_HEADPHONE_POWERUP_Pos		2
#define CAPLESS_HEADPHONE_POWERUP_Msk		(0x1 << CAPLESS_HEADPHONE_POWERUP_Pos)
#define ADC_POWERUP_ANA_Pos					1
#define ADC_POWERUP_ANA_Msk					(0x1 << ADC_POWERUP_ANA_Pos)
#define LINEOUT_POWERUP_Pos					0
#define LINEOUT_POWERUP_Msk					(0x1 << LINEOUT_POWERUP_Pos)

/* Bit definition for CHIP_PLL_CTRL register */
#define INT_DIVISOR_Pos						11
#define INT_DIVISOR_Msk						(0x3F << INT_DIVISOR_Pos)
#define FRAC_DIVISOR_Pos					0
#define FRAC_DIVISOR_Msk					(0x7FF << FRAC_DIVISOR_Pos)

/* Bit definition for CHIP_CLK_TOP_CTRL register */
#define ENABLE_INT_OSC_Pos					11
#define ENABLE_INT_OSC_Msk					(0x1 << ENABLE_INT_OSC_Pos)
#define INPUT_FREQ_DIV2_Pos					3
#define INPUT_FREQ_DIV2_Msk					(0x1 << INPUT_FREQ_DIV2_Pos)

/* Bit definition for CHIP_ANA_STATUS register */
#define LRSHORT_STS_Pos						9
#define LRSHORT_STS_Msk						(0x1 << LRSHORT_STS_Pos)
#define CSHORT_STS_Pos						8
#define CSHORT_STS_Msk						(0x1 << CSHORT_STS_Pos)
#define PLL_IS_LOCKED_Pos					4
#define PLL_IS_LOCKED_Msk					(0x1 << PLL_IS_LOCKED_Pos)

/* Bit definition for CHIP_ANA_TEST2 register */
#define LINEOUT_TO_VDDA_Pos					14
#define LINEOUT_TO_VDDA_Msk					(0x1 << LINEOUT_TO_VDDA_Pos)
#define SPARE_Pos							13
#define SPARE_Msk							(0x1 << SPARE_Pos)
#define MONOMODE_DAC_Pos					12
#define MONOMODE_DAC_Msk					(0x1 << MONOMODE_DAC_Pos)
#define VCO_TUNE_AGAIN_Pos					11
#define VCO_TUNE_AGAIN_Msk					(0x1 << VCO_TUNE_AGAIN)
#define LO_PASS_MASTERVAG_Pos				10
#define LO_PASS_MASTERVAG_Msk				(0x1 << LO_PASS_MASTERVAG_Pos)
#define INVERT_DAC_SAMPLE_CLOCK_Pos			9
#define INVERT_DAC_SAMPLE_CLOCK_Msk			(0x1 << INVERT_DAC_SAMPLE_CLOCK_Pos)
#define INVERT_DAC_DATA_TIMING_Pos			8
#define INVERT_DAC_DATA_TIMING_Msk			(0x1 << INVERT_DAC_DATA_TIMING_Pos)
#define DAC_EXTEND_RTZ_Pos					7
#define DAC_EXTEND_RTZ_Msk					(0x1 << DAC_EXTEND_RTZ_Pos)
#define DAC_DOUBLE_I_Pos					6
#define DAC_DOUBLE_I_Msk					(0x1 << DAC_DOUBLE_I_Pos)
#define DAC_DIS_RTZ_Pos						5
#define DAC_DIS_RTZ_Msk						(0x1 << DAC_DIS_RTZ_Pos)
#define DAC_CLASSA_Pos						4
#define DAC_CLASSA_Msk						(0x1 << DAC_CLASSA_Pos)
#define INVERT_ADC_SAMPLE_CLOCK_Pos			3
#define INVERT_ADC_SAMPLE_CLOCK_Msk			(0x1 << INVERT_ADC_SAMPLE_CLOCK_Pos)
#define INVERT_ADC_DATA_TIMING_Pos			2
#define INVERT_ADC_DATA_TIMING_Msk			(0x1 << INVERT_ADC_DATA_TIMING_Pos)
#define ADC_LESSI_Pos						1
#define ADC_LESSI_Msk						(0x1 << ADC_LESSI_Pos)
#define ADC_DITHEROFF_Pos					0
#define ADC_DITHEROFF_Msk					(0x1 << ADC_DITHEROFF_Pos)

/* Bit definition for CHIP_SHORT_CTRL register */
#define LVLADJR_Pos							12
#define LVLADJR_Msk							(0x7 << LVLADJR_Pos)
#define LVLADJL_Pos							8
#define LVLADJL_Msk							(0x7 << LVLADJL_Pos)
#define LVLADJC_Pos							4
#define LVLADJC_Msk							(0x7 << LVLADJC_Pos)
#define MODE_LR_Pos							2
#define MODE_LR_Msk							(0x3 << MODE_LR_Pos)
#define MODE_CM_Pos							0
#define MODE_CM_Msk							(0x3 << MODE_CM_Pos)

/* Enable / disable values */
#define ENABLE_FALSE						0x0
#define ENABLE_TRUE							0x1

/* Sample rate mode. MCLK_FREQ is still specified relative to the rate in SYS_FS */
#define RATE_MODE_SYS_FS					0x0
#define RATE_MODE_1_2						0x1
#define RATE_MODE_1_4						0x2
#define RATE_MODE_1_6						0x3

/* Internal system sample rate */
#define SYS_FS_32KHZ						0x0
#define SYS_FS_44_1KHZ						0x1
#define SYS_FS_48KHZ						0x2
#define SYS_FS_96KHZ						0x3

/* Incoming SYS_MCLK frequency */
#define MCLK_FREQ_256FS						0x0
#define MCLK_FREQ_384FS						0x1
#define MCLK_FREQ_512FS						0x2
#define MCLK_FREQ_PLL						0x3

/* I2S MS modes */
#define MS_SLAVE							0x0
#define MS_MASTER							0x1

/* I2S master mode I2S_SCLK output frequency */
#define SCLKFREQ_64FS						0x0
#define SCLKFREQ_32FS						0x1		// not supported by right justified mode

/* I2S clock edge */
#define SCLK_INV_RISING						0x0
#define SCLK_INV_FALLING					0x1

/* I2S data length in bits */
#define DLEN_32								0x0		// only valid for SCLKFREQ_64FS, not valid for right justified mode
#define DLEN_24								0x1		// only valid for SCLKFREQ_64FS
#define DLEN_20								0x2
#define DLEN_16								0x3

/* I2S data modes */
#define I2S_MODE_I2S_OR_LEFT				0x0		// use LRALIGN to specify
#define I2S_MODE_RIGHT						0x1
#define I2S_MODE_PCM						0x2

/* I2S alignment to data word - see datasheet for SGTL5000 */
#define LRALIGN_A							0x0
#define LRALIGN_B							0x1

/* I2S_LRCLK polarity */
#define LRPOL_0LEFT_1RIGHT					0x0
#define LRPOL_0RIGHT_1LEFT					0x1

/* Left / right channel swap */
#define LRSWAP_NORMAL						0x0
#define LRSWAP_SWAP							0x1

/* Data sources */
#define INPUT_ADC							0x0
#define INPUT_I2S_IN						0x1
#define INPUT_DAP							0x3

/* Volume states */
#define VOL_READY							0x0
#define VOL_BUSY							0x1

/* Volume ramp types */
#define RAMP_LINEAR							0x0
#define RAMP_EXPONENTIAL					0x1

/* Mute states */
#define UNMUTE								0x0
#define MUTE								0x1

/* ADC high pass filter modes */
#define ADC_HPF_NORMAL						0x0
#define ADC_HPF_FREEZE						0x1
#define ADC_HPF_BYPASS						0x2

/* DAC 0db volume offset */
#define DAC_VOLUME_0DB						0x3C

/* Headphone inputs */
#define HP_INPUT_DAC						0x0
#define HP_INPUT_LINE_IN					0x1

/* ADC inputs */
#define ADC_INPUT_MIC						0x0
#define ADC_INPUT_LINE_IN					0x1

/* Charge pump source when override enabled */
#define CHARGE_PUMP_SRC_VDDA				0x0
#define CHARGE_PUMP_SRC_VDDIO				0x1

/* MIC bias output impedance */
#define BIAS_IMPEDANCE_OFF					0x0
#define BIAS_IMPEDANCE_2K					0x1
#define BIAS_IMPEDANCE_4K					0x2
#define BIAS_IMPEDANCE_8K					0x3

/* MIC bias output voltage */
#define BIAS_VOLTAGE_1_25					0x0
#define BIAS_VOLTAGE_1_50					0x1
#define BIAS_VOLTAGE_1_75					0x2
#define BIAS_VOLTAGE_2_00					0x3
#define BIAS_VOLTAGE_2_25					0x4
#define BIAS_VOLTAGE_2_50					0x5
#define BIAS_VOLTAGE_2_75					0x6
#define BIAS_VOLTAGE_3_00					0x7

/* MIC gain */
#define MIC_GAIN_0DB						0x0
#define MIC_GAIN_20DB						0x1
#define MIC_GAIN_30DB						0x2
#define MIC_GAIN_40DB						0x3

/* Line out output current */
#define OUT_CURRENT_0_18A					0x0
#define OUT_CURRENT_0_27A					0x1
#define OUT_CURRENT_0_36A					0x3
#define OUT_CURRENT_0_45A					0x7
#define OUT_CURRENT_0_54A					0xF

/* Power down/up */
#define POWER_DOWN							0x0
#define POWER_UP							0x1

/* Headphone short detection */
#define HP_SHORT_NORMAL						0x0
#define HP_SHORT_DETECTED					0x1

/* PLL locked status */
#define PLL_NOT_LOCKED						0x0
#define PLL_LOCKED							0x1

/* Functions */
uint8_t sgtlI2CInit();
uint8_t sgtlReadPartId();
uint8_t sgtlReadRevId();
uint8_t sgtlReadHeadPhoneShort();
uint8_t sgtlReadHeadPhoneCaplessShort();

void sgtlADCBlockEnable(uint8_t enable);
void sgtlDACBlockEnable(uint8_t enable);
void sgtlDAPBlockEnable(uint8_t enable);
void sgtlI2SOutEnable(uint8_t enable);
void sgtlI2SInEnable(uint8_t enable);
void sgtlI2SMode(uint8_t ms, uint8_t sclkFreq, uint8_t sclkInv, uint8_t dLen,uint8_t i2sMode, uint8_t lrAlign, uint8_t lrPol);
void sgtlSampleRateMode(uint8_t rateMode);
void sgtlSystemSampleRate(uint8_t sysFs);
void sgtlMasterClock(uint8_t mclkFreq);
void sgtlDAPMixerInputSwap(uint8_t lrSwap);
void sgtlDAPInputSwap(uint8_t lrSwap);
void sgtlDACInputSwap(uint8_t lrSwap);
void sgtlI2SInputSwap(uint8_t lrSwap);
void sgtlDAPMixInput(uint8_t input);
void sgtlDAPInput(uint8_t input);
void sgtlDACInput(uint8_t input);
void sgtlI2SOutput(uint8_t input);
void sgtlVolumeBusyDAC(uint8_t volBusyRight, uint8_t volBusyLeft);
void sgtlVolumeRampEnable(uint8_t enable);
void sgtlVolumeRampType(uint8_t rampType);
void sgtlDACMute(uint8_t muteLeft, uint8_t muteRight);
void sgtlADCHPFMode(uint8_t adcHpfMode);
void sgtlDACVolRight(uint8_t volume);
void sgtlDACVolLeft(uint8_t volume);
void sgtlADCVol6dbReductEnable(uint8_t enable);
void sgtlADCVolRight(uint8_t volume);
void sgtlADCVolLeft(uint8_t volume);
void sgtlHeadPhoneVolRight(uint8_t volume);
void sgtlHeadPhoneVolLeft(uint8_t volume);
void sgtlLineOutMute(uint8_t mute);
void sgtlHeadPhoneInput(uint8_t hpInput);
void sgtlHeadPhoneZCEnable(uint8_t enable);
void sgtlHeadPhoneMute(uint8_t mute);
void sgtlADCInput(uint8_t adcInput);
void sgtlAnalogADCZCEnable(uint8_t enable);
void sgtlADCAnalogMute(uint8_t mute);
void sgtlChargePumpOverrideEnable(uint8_t enable, uint8_t chargePumpSrc);
void sgtlAnalogGroundRefVoltage(uint8_t value);
void sgtlAnalogBiasCurrent(uint8_t value);
void sgtlAnalogSmallPopEnable(uint8_t enable);
void sgtlMICBiasImpedance(uint8_t biasImpedance);
void sgtlMICBiasVoltage(uint8_t biasVoltage);
void sgtlMICGain(uint8_t micGain);
void sgtlLineOutCurrent(uint8_t outCurrent);
void sgtlLineOutGroundVoltage(uint8_t value);
void sgtlLineOutVolRight(uint8_t volume);
void sgtlLineOutVolLeft(uint8_t volume);
void sgtlDACMonoEnable(uint8_t enable);
void sgtlSimpleRegulatorPower(uint8_t power);
void sgtlStartupCircuitPower(uint8_t power);
void sgtlChargePumpPower(uint8_t power);
void sgtlPLLPower(uint8_t power);
void sgtlVDDDRegulatorPower(uint8_t power);
void sgtlPLLVCOPower(uint8_t power);
void sgtlVAGPower(uint8_t power);
void sgtlADCMonoEnable(uint8_t enable);
void sgtlReferenceBiasPower(uint8_t power);
void sgtlHeadPhoneAmpPower(uint8_t power);
void sgtlDACPower(uint8_t power);
void sgtlHeadPhoneCaplessEnable(uint8_t enable);
void sgtlADCPower(uint8_t power);
void sgtlLineOutPower(uint8_t power);
void sgtlPLLIntDivisor(uint8_t intDivisor);
void sgtlPLLFracDivisor(uint16_t fracDivisor);
void sgtlInternalZCOscEnable(uint8_t enable);
void sgtlPLLInputDiv2Enable(uint8_t enable);
void sgtlTest();