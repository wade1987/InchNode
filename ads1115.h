/* 
 * File:   ads1115.h
 * Author: Thinkpad
 *
 * Created on 2017?3?19?, ??5:47
 */

#ifndef ADS1115_H
#define	ADS1115_H

#include "mcc_generated_files/mcc.h"


#ifdef	__cplusplus
extern "C" {
#endif


#define DELAY_US(x) __delay_us(x)
#define BIT_SHIFT	0


/*=========================================================================  
    I2C IO define
    -----------------------------------------------------------------------*/  

#define ADS_RE_COUNT 5

#define SET_SDA_IN() IO_RA4_SetDigitalInput()
#define SET_SDA_OUT() IO_RA4_SetDigitalOutput()
#define SET_SCL_OUT() IO_RA5_SetDigitalOutput()


#define SetSCL() IO_RA5_SetHigh()
#define ClrSCL() IO_RA5_SetLow()

#define SetSDA() IO_RA4_SetHigh()
#define ClrSDA() IO_RA4_SetLow()

#define SDA_IN() IO_RA4_GetValue()


  
/*=========================================================================  
    I2C ADDRESS/BITS  
    -----------------------------------------------------------------------*/  
#define ADS1015_ADDRESS                 (0x90)    // 1001 000 (ADDR = GND)  
/*=========================================================================*/  
  
/*=========================================================================  
    CONVERSION DELAY (in uS)  
    -----------------------------------------------------------------------*/  
#define ADS1015_CONVERSIONDELAY         (10)  
#define ADS1115_CONVERSIONDELAY         (8)  
/*=========================================================================*/  
  
/*=========================================================================  
    POINTER REGISTER  
    -----------------------------------------------------------------------*/  
#define ADS1015_REG_POINTER_MASK        (0x03)  
#define ADS1015_REG_POINTER_CONVERT     (0x00)  
#define ADS1015_REG_POINTER_CONFIG      (0x01)  
#define ADS1015_REG_POINTER_LOWTHRESH   (0x02)  
#define ADS1015_REG_POINTER_HITHRESH    (0x03)  
/*=========================================================================*/  
  
/*=========================================================================  
    CONFIG REGISTER  
    -----------------------------------------------------------------------*/  
#define ADS1015_REG_CONFIG_OS_MASK      (0x8000)  
#define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion  
#define ADS1015_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress  
#define ADS1015_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion  
  
#define ADS1015_REG_CONFIG_MUX_MASK     (0x7000)  
#define ADS1015_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)  
#define ADS1015_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3  
#define ADS1015_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3  
#define ADS1015_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3  
#define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0  
#define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1  
#define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2  
#define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3  
  
#define ADS1015_REG_CONFIG_PGA_MASK     (0x0E00)  
#define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3  
#define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1  
#define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)  
#define ADS1015_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4  
#define ADS1015_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8  
#define ADS1015_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16  
  
#define ADS1015_REG_CONFIG_MODE_MASK    (0x0100)  
#define ADS1015_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode  
#define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)  
  
#define ADS1015_REG_CONFIG_DR_MASK      (0x00E0)  
#define ADS1015_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second  
#define ADS1015_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second  
#define ADS1015_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second  
#define ADS1015_REG_CONFIG_DR_920SPS    (0x0060)  // 920 samples per second  
#define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)  
#define ADS1015_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second  
#define ADS1015_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second  
  
#define ADS1015_REG_CONFIG_CMODE_MASK   (0x0010)  
#define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)  
#define ADS1015_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator  
  
#define ADS1015_REG_CONFIG_CPOL_MASK    (0x0008)  
#define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)  
#define ADS1015_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active  
  
#define ADS1015_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted  
#define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)  
#define ADS1015_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator  
  
#define ADS1015_REG_CONFIG_CQUE_MASK    (0x0003)  
#define ADS1015_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions  
#define ADS1015_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions  
#define ADS1015_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions  
#define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)  
/*=========================================================================*/  
  
typedef enum  
{  
    GAIN_TWOTHIRDS    = ADS1015_REG_CONFIG_PGA_6_144V,  
    GAIN_ONE          = ADS1015_REG_CONFIG_PGA_4_096V,  
    GAIN_TWO          = ADS1015_REG_CONFIG_PGA_2_048V,  
    GAIN_FOUR         = ADS1015_REG_CONFIG_PGA_1_024V,  
    GAIN_EIGHT        = ADS1015_REG_CONFIG_PGA_0_512V,  
    GAIN_SIXTEEN      = ADS1015_REG_CONFIG_PGA_0_256V  
} adsGain_t;  
  
extern void ads1015_init(void);  
  
extern void readADC_Differential_0_1_part1(void);
extern int16_t readADC_Differential_0_1_part2(void);
  
//extern void get_adc(volatile uint16_t *res0, volatile uint16_t *res1);  




#ifdef	__cplusplus
}
#endif

#endif	/* ADS1115_H */

