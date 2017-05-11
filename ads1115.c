/**************************************************************************/  
/*!  
    @file     Adafruit_ADS1015.c  
    @author   K.Townsend (Adafruit Industries)  
    @license  BSD (see license.txt)  
  
    Driver for the ADS1015/ADS1115 ADC  
  
    This is a library for the Adafruit MPL115A2 breakout  
    ----> https://www.adafruit.com/products/???  
  
    Adafruit invests time and resources providing this open source code,  
    please support Adafruit and open-source hardware by purchasing  
    products from Adafruit!  
  
    @section  HISTORY  
  
    v1.0 - First release  
    Edit by FangZheng 2014-12-11  
*/  
  
#include "ads1115.h"  
  
static uint16_t m_i2cAddress;  
//static uint16_t m_conversionDelay;  
static uint16_t m_bitShift;  
static adsGain_t m_gain;  
//static uint16_t m_chn;  

static void IIC_Delay(void)  
{  
    DELAY_US(1);  
}  
static void IIC_Delay_Half(void)  
{  
    DELAY_US(1);  
}  
static uint16_t IsCharOne(void)  
{  
    uint16_t retcode = 0;  
    if (SDA_IN())  
    {  
        retcode = 1;  
    }  
    return retcode;  
}  
  
static void StartRom(void)  
{  
    SET_SDA_OUT();  
    SET_SCL_OUT();  
    SetSDA();  
    IIC_Delay();  
    SetSCL();  
    IIC_Delay();  
    ClrSDA();  
    IIC_Delay();  
    ClrSCL();  
    IIC_Delay();  
      
}  
  
static void StopRom(void)  
{  
    ClrSDA();  
    IIC_Delay();  
    SetSCL();  
    IIC_Delay();  
    SetSDA();  
    IIC_Delay();  
    ClrSCL();  
    IIC_Delay();  
}  
  
static uint16_t SendToRom(uint16_t data)  
{  
    uint16_t count, retcode = 0;  
    for (count = 0; count < 8; count++)  
    {  
        if ((data & 0x80) == 0x80)  
        {  
            SetSDA();  
        }  
        else  
        {  
            ClrSDA();  
        }  
        IIC_Delay();  
        SetSCL();  
        data = data << 1;  
        IIC_Delay();  
        ClrSCL();  
        //IIC_Delay();  
    }  
    SET_SDA_IN();  
    IIC_Delay();  
    SetSCL();  
    //IIC_Delay();  
    IIC_Delay_Half();  
    retcode = IsCharOne();  
    IIC_Delay_Half();  
    //IIC_Delay();  
    ClrSCL();  
    IIC_Delay();  
    SET_SDA_OUT();  
      
    return retcode;  
}  
  
static uint16_t ReceiveFromRom(void)  
{  
    uint16_t m = 0, count;  
    SET_SDA_IN();  
    IIC_Delay();  
    for (count = 0; count < 8; count++)  
    {  
        m = m << 1;  
        SetSCL();  
        //IIC_Delay();  
        //DELAY_US(3);  
        IIC_Delay_Half();  
        if (IsCharOne())  
        {  
            m |= 0x01;  
        }  
        //IIC_Delay();  
        //DELAY_US(2);  
        IIC_Delay_Half();  
        ClrSCL();  
        IIC_Delay();  
    }  
    SET_SDA_OUT();  
      
    return m;  
}  
  
static void mack(void)  
{  
    ClrSDA();  
    IIC_Delay();  
    SetSCL();  
    IIC_Delay();  
    ClrSCL();  
    IIC_Delay();  
}  
  
static void nmack(void)  
{  
    SetSDA();  
    IIC_Delay();  
    SetSCL();  
    IIC_Delay();  
    ClrSCL();  
    IIC_Delay();  
}  
void writeRegister(uint16_t i2cAddress, uint16_t reg, uint16_t value)   
{  
    uint16_t i;  
    
    for(i = 0; i < ADS_RE_COUNT; i++)  
    {                 
        StopRom();  
        //IIC_Delay();  
        IIC_Delay();  
        StartRom();  
        if(SendToRom(i2cAddress) == 1)  
            continue;  
        if(SendToRom(reg&0xFF) == 1)  
            continue;  
        if(SendToRom(value>>8) == 1)  
            continue;  
        if(SendToRom(value&0xFF) == 1)  
            continue;  
          
        break;          
    }      
    StopRom();   
    IIC_Delay();    
      
    //return i;  
}  
  
uint16_t readRegister(uint16_t i2cAddress, uint16_t reg)   
{  
    uint16_t i, ret;  
   
    for(i = 0; i < ADS_RE_COUNT; i++)  
    {                 
        StopRom();  
        IIC_Delay();  
        StartRom();  
        if(SendToRom(i2cAddress) == 1)  
            continue;  
        if(SendToRom(ADS1015_REG_POINTER_CONVERT) == 1)  
            continue;  
        StopRom();  
        IIC_Delay();  
        StartRom();  
        if(SendToRom(i2cAddress|0x01) == 1)  
            continue;  
        ret = ReceiveFromRom() << 8;  
        mack();  
  
        ret |= ReceiveFromRom();  
        mack();  //ÕâÀï²»ÊÇnmackå?
  
        break;          
    }      
    StopRom();   
    IIC_Delay();    
      
    return ret;    
}  

  
/**************************************************************************/  
/*!  
    @brief  Instantiates a new ADS1015 class w/appropriate properties  
*/  
/**************************************************************************/  
void ads1015_init(void)  
{  
    m_i2cAddress = ADS1015_ADDRESS;  
    //m_conversionDelay = ADS1015_CONVERSIONDELAY;  
    m_bitShift = BIT_SHIFT;  
    m_gain = GAIN_TWO; /* +/- 2.048V range (limited to VDD +0.3V max!) */  
   // m_chn = 0;  
}  
  
#if 0  
/**************************************************************************/  
/*!  
    @brief  Sets the gain and input voltage range  
*/  
/**************************************************************************/  
void setGain(adsGain_t gain)  
{  
    m_gain = gain;  
}  
  
/**************************************************************************/  
/*!  
    @brief  Gets a gain and input voltage range  
*/  
/**************************************************************************/  
adsGain_t getGain()  
{  
    return m_gain;  
}  
#endif  
  
/**************************************************************************/  
/*!  
    @brief  Gets a single-ended ADC reading from the specified channel  
*/  
/**************************************************************************/  
#if 0
uint16_t readADC_SingleEnded(uint16_t channel)  
{  
    uint16_t  config;  
    if (channel > 3)  
    {  
        return 0;  
    }  
  
    // Start with default values  
    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)  
            ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)  
            ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)  
            ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)  
            ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)  
            ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)  
  
    // Set PGA/voltage range  
    config |= m_gain;  
  
    // Set single-ended input channel  
    switch (channel)  
    {  
    case (0):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;  
        break;  
    case (1):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;  
        break;  
    case (2):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;  
        break;  
    case (3):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;  
        break;  
    }  
  
    // Set 'start single-conversion' bit  
    config |= ADS1015_REG_CONFIG_OS_SINGLE;  
  
    // Write config register to the ADC  
    writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);  
  
    // Wait for the conversion to complete  
    //DELAY_US(m_conversionDelay);  
    DELAY_US(350);  
  
    // Read the conversion results  
    // Shift 12-bit results right 4 bits for the ADS1015  
    return readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;  
} 
#endif
  
#if 0
void read_adc_config(void)  
{  
    uint16_t  config;  
  
    // Start with default values  
    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)  
            ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)  
            ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)  
            ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)  
            ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)  
            ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)  
  
    // Set PGA/voltage range  
    config |= m_gain;  
  
    // Set single-ended input channel  
    switch (m_chn)  
    {  
    case (0):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;  
        break;  
    case (1):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;  
        break;  
    case (2):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;  
        break;  
    case (3):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;  
        break;  
    }  
  
    // Set 'start single-conversion' bit  
    config |= ADS1015_REG_CONFIG_OS_SINGLE;  
  
    // Write config register to the ADC  
    writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);  
      
}  
  
void read_adc_reg(volatile uint16_t *res0, volatile uint16_t *res1)  
{  
    if(m_chn)  
    {  
        *res1 = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;  
        m_chn = 0;  
    }  
    else  
    {  
        *res0 = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;  
        m_chn = 1;  
    }  
}  
void get_adc(volatile uint16_t *res0, volatile uint16_t *res1)  
{  
    #define PER_CNT 6  
    static volatile uint16_t i = 0;  
  
    if(i == 0)  
    {  
        read_adc_config();  
    }  
  
    if(i == PER_CNT)  
    {  
        read_adc_reg(res0, res1);  
    }  
    if(i++ > PER_CNT) i = 0;  
}  
#endif


#if 0
/**************************************************************************/  
/*!  
    @brief  Reads the conversion results, measuring the voltage  
            difference between the P (AIN0) and N (AIN1) input.  Generates  
            a signed value since the difference can be either  
            positive or negative.  
*/  
/**************************************************************************/  
int16_t readADC_Differential_0_1(void)  
{  
    uint16_t res, config;  
    // Start with default values  
    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)  
             ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)  
             ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)  
             ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)  
             ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)  
             ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)  
  
    // Set PGA/voltage range  
    config |= m_gain;  
  
    // Set channels  
    config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1;          // AIN0 = P, AIN1 = N  
  
    // Set 'start single-conversion' bit  
    config |= ADS1015_REG_CONFIG_OS_SINGLE;  
  
    // Write config register to the ADC  
    writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);  
  
    // Wait for the conversion to complete  
    DELAY_US(m_conversionDelay);  
  
    // Read the conversion results  
    res = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;  
    if (m_bitShift == 0)  
    {  
        return (int16)res;  
    }  
    else  
    {  
        // Shift 12-bit results right 4 bits for the ADS1015,  
        // making sure we keep the sign bit intact  
        if (res > 0x07FF)  
        {  
            // negative number - extend the sign to 16th bit  
            res |= 0xF000;  
        }  
        return (int16_t)res;  
    }  
}  
#endif

void readADC_Differential_0_1_part1(void)  
{  
    uint16_t  config;  
    // Start with default values  
    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)  
             ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)  
             ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)  
             ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)  
             ADS1015_REG_CONFIG_DR_920SPS   | // 1600 samples per second (default)  
             ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)  
  
    // Set PGA/voltage range  
    config |= m_gain;  
  
    // Set channels  
    config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1;          // AIN0 = P, AIN1 = N  
  
    // Set 'start single-conversion' bit  
    config |= ADS1015_REG_CONFIG_OS_SINGLE;  
  
    // Write config register to the ADC  
    writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);  
  
   
}

int16_t readADC_Differential_0_1_part2(void)  
{  
   uint16_t res;  
  
    // Read the conversion results  
    res = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> BIT_SHIFT;  
    if (m_bitShift == 0)  
    {  
        return (int16_t)res;  
    }  
    else  
    {  
        // Shift 12-bit results right 4 bits for the ADS1015,  
        // making sure we keep the sign bit intact  
        if (res > 0x07FF)  
        {  
            // negative number - extend the sign to 16th bit  
            res |= 0xF000;  
        }  
        return (int16_t)res;  
    }  
}



#if 0
  
/**************************************************************************/  
/*!  
    @brief  Reads the conversion results, measuring the voltage  
            difference between the P (AIN2) and N (AIN3) input.  Generates  
            a signed value since the difference can be either  
            positive or negative.  
*/  
/**************************************************************************/  
int16_t readADC_Differential_2_3(void)  
{  
    uint16_t res, config;  
    // Start with default values  
    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)  
             ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)  
             ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)  
             ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)  
             ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)  
             ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)  
  
    // Set PGA/voltage range  
    config |= m_gain;  
  
    // Set channels  
    config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3;          // AIN2 = P, AIN3 = N  
  
    // Set 'start single-conversion' bit  
    config |= ADS1015_REG_CONFIG_OS_SINGLE;  
  
    // Write config register to the ADC  
    writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);  
  
    // Wait for the conversion to complete  
    DELAY_US(m_conversionDelay);  
  
    // Read the conversion results  
    res = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;  
    if (m_bitShift == 0)  
    {  
        return (int16)res;  
    }  
    else  
    {  
        // Shift 12-bit results right 4 bits for the ADS1015,  
        // making sure we keep the sign bit intact  
        if (res > 0x07FF)  
        {  
            // negative number - extend the sign to 16th bit  
            res |= 0xF000;  
        }  
        return (int16_t)res;  
    }  
}  
  
/**************************************************************************/  
/*!  
    @brief  Sets up the comparator to operate in basic mode, causing the  
            ALERT/RDY pin to assert (go from high to low) when the ADC  
            value exceeds the specified threshold.  
  
            This will also set the ADC in continuous conversion mode.  
*/  
/**************************************************************************/  
void startComparator_SingleEnded(uint16_t channel, uint16_t threshold)  
{  
    // Start with default values  
    uint16_t config = ADS1015_REG_CONFIG_CQUE_1CONV   | // Comparator enabled and asserts on 1 match  
                    ADS1015_REG_CONFIG_CLAT_LATCH   | // Latching mode  
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)  
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)  
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)  
                    ADS1015_REG_CONFIG_MODE_CONTIN  | // Continuous conversion mode  
                    ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode  
  
    // Set PGA/voltage range  
    config |= m_gain;  
  
    // Set single-ended input channel  
    switch (channel)  
    {  
    case (0):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;  
        break;  
    case (1):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;  
        break;  
    case (2):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;  
        break;  
    case (3):  
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;  
        break;  
    }  
  
    // Set the high threshold register  
    // Shift 12-bit results left 4 bits for the ADS1015  
    writeRegister(m_i2cAddress, ADS1015_REG_POINTER_HITHRESH, threshold << m_bitShift);  
  
    // Write config register to the ADC  
    writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);  
}  
  
/**************************************************************************/  
/*!  
    @brief  In order to clear the comparator, we need to read the  
            conversion results.  This function reads the last conversion  
            results without changing the config value.  
*/  
/**************************************************************************/  
int16_t getLastConversionResults(void)  
{  
    uint16_t res;  
    // Wait for the conversion to complete  
    DELAY_US(m_conversionDelay);  
  
    // Read the conversion results  
    res = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;  
    if (m_bitShift == 0)  
    {  
        return (int16)res;  
    }  
    else  
    {  
        // Shift 12-bit results right 4 bits for the ADS1015,  
        // making sure we keep the sign bit intact  
        if (res > 0x07FF)  
        {  
            // negative number - extend the sign to 16th bit  
            res |= 0xF000;  
        }  
        return (int16_t)res;  
    }  
}  
#endif  
