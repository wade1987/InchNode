/**
  Generated Main Source File I AM MAC

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.15
        Device            :  PIC12F1840
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "mcc_generated_files/mcc.h"
#include <stdlib.h>
#include "ads1115.h"

/*
                         Macros
 */
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

//cali eeprom
#define	CALI_BUF_ADDR  0

//led
#define LedOn()         IO_RA0_SetHigh()  
#define LedOff()        IO_RA0_SetLow()
#define LedToggle()     IO_RA0_Toggle()

//uart
#define UartRecBufLen    8
#define UartSendBufLen   8

#define ADC_ValueBufLen	 3

#define CALI_BUf_LEN 16

/*
                         Types
 */

typedef struct
{
	uint16_t OriginalValue;
	uint16_t ActualValue;
}stTab;

/*
                          Varibles
 */
char StringBuf[10];

uint16_t LedTimerCnt;
uint8_t UartRecCnt=0;
uint8_t UartRecBuf[UartRecBufLen];
uint8_t UartByteToBeSendCnt=0;
uint8_t UartSendCnt;
uint8_t UartSendBuf[UartSendBufLen];
uint8_t ADCStep;
uint16_t ADCDelayCnt;
int16_t ADC_Value;
int16_t ADC_ValueBuf[ADC_ValueBufLen];
stTab	CaliBuf[CALI_BUf_LEN];
stTab	CaliTab[CALI_BUf_LEN+1];
uint8_t CaliTabSize;
uint8_t CalibratingFlag;
uint16_t Length; //单位mm
uint16_t ActualLength; //单位mm
uint8_t CaliTabSendFlag;
uint16_t ADLinearVal;
uint8_t SendDataToMainBoardStep;
uint16_t SendTmr;

/*
                         Main application
 */

void EEpromBufferRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	uint8_t i;

	for(i = 0 ; i < NumByteToRead ; i++)
	{
		pBuffer[i] = eeprom_read(ReadAddr+i);
	}

}

void EEpromBuffeWirte(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	uint8_t i;

	for(i = 0 ; i < NumByteToWrite ; i++)
	{
		eeprom_write(WriteAddr+i, pBuffer[i]);
	}

}


/******************************************************************************
*                           wade@2016-09-05
* Function Name  :  LinearLookUpTab()
* Description       :  查表并计算线性值，电池电量计算及PM2.5 甲醛标定用
* Input               :  Tab ---表格数组
*                  	  	 TabSiz---表格长度
*                  	  	 OriginalVal---原始值
* Output             :  16位结果
* Return             :  返回的值可能是表格值也可能是两个表格值之间
* 			       的线性值
*******************************************************************************/

uint16_t LinearLookUpTab(stTab *Tab, uint8_t TabSiz, uint16_t val)
{
	uint8_t i;
	uint32_t ret;

	//查表并计算
	
	if(val < Tab[0].OriginalValue)
	{
		ret = ((((uint32_t)val - 0) * ((uint32_t)Tab[0].ActualValue - 0)) / ((uint32_t)Tab[0].OriginalValue - 0)) + 0;
	}
	for( i = 0 ; i < TabSiz-1 ; i++) 
	{
		if((val >= Tab[i].OriginalValue) && (val < Tab[i+1].OriginalValue))
		{
			ret = (((uint32_t)(val - Tab[i].OriginalValue) * (uint32_t)(Tab[i+1].ActualValue - Tab[i].ActualValue)) \
				/ (uint32_t)(Tab[i+1].OriginalValue - Tab[i].OriginalValue)) + (uint32_t)Tab[i].ActualValue;
		}
	}
	if(val >=  Tab[TabSiz-1].OriginalValue)
	{
		ret = Tab[TabSiz-1].ActualValue;
	}

	return (uint16_t)ret;

}

/*******************************************************************************
*                wade@2015-09-07
* Function Name  : UpdateCaliBuf
* Description    : 更新CaliBuf 内的值
* Input          :  None
* Output         : None
* Return         : None
*******************************************************************************/

void UpdateCaliBuf(void)
{
	uint8_t tmp1[4];
	uint8_t i;

	for(i = 0 ; i < CALI_BUf_LEN ; i++)
	{
		EEpromBufferRead(tmp1, CALI_BUF_ADDR+i*4, 4);
		CaliBuf[i].OriginalValue = ((u16)tmp1[0] << 8) | ((u16)tmp1[1]);
		CaliBuf[i].ActualValue = ((u16)tmp1[2] << 8) | ((u16)tmp1[3]);
	}
}

/*******************************************************************************
*                wade@2015-09-07
* Function Name  : EraseCaliDatabase
* Description    : 擦除标定库内所有内容
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void EraseCaliDatabase(void)
{
	uint8_t i;
	
	for(i = 0 ; i < CALI_BUf_LEN*4 ; i++)
	{
		eeprom_write(CALI_BUF_ADDR+i, 0xff);
	}
}

/*******************************************************************************
*                wade@2015-09-07
* Function Name  : Update2CaliDatabase
* Description    : 将新的标定值放入数据库
* Input          : 校准设备的标准值
* Output         : None
* Return         : None
*******************************************************************************/

void Update2CaliDatabase(uint16_t ori_val, uint16_t ActualValue)
{
	uint8_t i, j, m, k;
	uint16_t OriginalValue;
	uint8_t tmp[4];

	OriginalValue = ori_val;		

	//每段前1/3 段不标定，防止段与段之间交错
	if((ActualValue % 30) < 10) //10~30mm,每30mm标定一段
	{
		return;
	} 

	//超出范围
	if((OriginalValue > 52800) || (ActualValue > 1200)) // 1.2m
	{
		return;
	}
	//if((OriginalValue == 0) || (ActualValue == 0))
	//{
		//return;
	//}

	//定位标定值在数据库的位置
	i = ActualValue / 30;
	if(i >= CALI_BUf_LEN)
		i = CALI_BUf_LEN - 1;

	//更新CaliBuf
	UpdateCaliBuf();

	CaliBuf[i].OriginalValue = OriginalValue;
	CaliBuf[i].ActualValue = ActualValue;

	//判断校准数据库有没有负增长的现象
	//如果有，要么是校准仪要么是sport 的传感器有问题
	for(m = 0 ; m < CALI_BUf_LEN-1 ; m++)
	{
		for(j = m+1 ; j < CALI_BUf_LEN ; j++)
		{
			//如果CaliBuf 中的数据有效，说明该段已被标定
			if((CaliBuf[j].OriginalValue <= 52800) && (CaliBuf[j].ActualValue <= 1200) &&\
				(CaliBuf[m].OriginalValue <= 52800) && (CaliBuf[m].ActualValue <= 1200))
				//(CaliBuf[j+1].OriginalValue >= 0) && (CaliBuf[j+1].OriginalValue <= 2000)&& (CaliBuf[j+1].ActualValue >= 0) && (CaliBuf[j+1].ActualValue <= 2000))
			{
				if((CaliBuf[m].OriginalValue > CaliBuf[j].OriginalValue) || (CaliBuf[m].ActualValue > CaliBuf[j].ActualValue))
				{
					//出现负增长
					//清除标定数据库所有的内容
					for(k = 0 ; k < CALI_BUf_LEN ; k++)
					{
						CaliBuf[k].OriginalValue = 0xffff;
						CaliBuf[k].ActualValue = 0xffff;
					}
					//然后只留当前最新的一个标定值
					CaliBuf[i].OriginalValue = OriginalValue;
					CaliBuf[i].ActualValue = ActualValue;
				}
			}
		}
	}
	
	//开始烧录标定值
	EraseCaliDatabase();
	for(j = 0 ; j < CALI_BUf_LEN ; j++)
	{
		tmp[0] = (CaliBuf[j].OriginalValue >> 8);
		tmp[1] = CaliBuf[j].OriginalValue;
		tmp[2] = (CaliBuf[j].ActualValue >> 8);
		tmp[3] = CaliBuf[j].ActualValue;

		//标定值保存在eeprom中
		EEpromBuffeWirte(tmp, CALI_BUF_ADDR+j*4, 4);
	}
	
}

/*******************************************************************************
*                wade@2015-09-07
* Function Name  : ConstructCaliTab
* Description    : 根据标定库建立标定表格
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void ConstructCaliTab(void)
{
	uint8_t i, j=0;

	//更新CaliBuf
	UpdateCaliBuf();

	for(i = 0 ; i < CALI_BUf_LEN ; i++)
	{
		if((CaliBuf[i].OriginalValue <= 52800) && (CaliBuf[i].ActualValue <= 1200))
		{
			CaliTab[j].OriginalValue = CaliBuf[i].OriginalValue;
			CaliTab[j].ActualValue = CaliBuf[i].ActualValue;
			j++;
		}
	}

	//说明该设备没有标定过
	if(j == 0)
	{
		CaliTabSize = j;
	}
	//只标定过一个点
	else if(j == 1)
	{
		CaliTabSize = j;
	}
	//标定过两个以上的点
	else
	{
		CaliTabSize = j+1;
		//在顶端再增加一个点
		CaliTab[j].OriginalValue = 52800; 
		CaliTab[j].ActualValue = ((CaliTab[j-1].ActualValue-CaliTab[j-2].ActualValue)*(52800 - CaliTab[j-1].OriginalValue))\
								/(CaliTab[j-1].OriginalValue - CaliTab[j-2].OriginalValue) + CaliTab[j-1].ActualValue;
	}
}

/*******************************************************************************
*                           wade@2016-09-09
* Function Name  :  Dfu_CaliTabSend
* Description    : 发送标定表格
* Input          :  None
* Output         :  None
* Return         :  None
*******************************************************************************/

void CaliTabSendProcess(void)
{
	uint8_t i;

	if(CaliTabSendFlag)
	{
		CaliTabSendFlag = 0;
		UpdateCaliBuf();
		//发送校准表格
		printf("The original and actual value are:\r\n");

		for(i = 0 ; i < 16 ; i++)
		{
			printf("  No.%2d %4d-%4d mm { %5d , %5d },\r\n", i, i*30, (i+1)*30-1, CaliBuf[i].OriginalValue, CaliBuf[i].ActualValue);
		}
	}

}


/*******************************************************************************
*                wade@2015-09-07
* Function Name  : Cali()
* Description    : 对输入的值通过标定表格进行校准
* Input          : 需要校准的值
* Output         : 校准后的值
* Return         : None
*******************************************************************************/

uint16_t Cali(uint16_t InData)
{
	//说明该设备没有标定过
	if(CaliTabSize == 0)
	{
		return InData;
	}
	//只标定过一个点
	else if(CaliTabSize == 1)
	{
		//printf("Indata:%d,CaliTab.ActVal:%d,CaliTab.Ori:%d",InData,CaliTab[0].ActualValue,CaliTab[0].OriginalValue);
		return ((uint32_t)InData*(uint32_t)CaliTab[0].ActualValue) / (uint32_t)CaliTab[0].OriginalValue;
	}
	//标定过两个以上的点 
	else
	{
		return LinearLookUpTab(CaliTab, CaliTabSize, InData);
	}
}


//将AD 采样到的分压值转换成和电阻线性相关的值
uint16_t ChangeADS1115ValueToResitor(int16_t ADC_value)
{
	uint32_t tmp;
	
	if(ADC_value <= 0)
		return 0;
	else
	{
		tmp = (uint32_t)ADC_value*52800; //52800是3.3V 按正负2.048v参考电压推算出的16 位AD值
		tmp = tmp/(52800-ADC_value);
		//虽然tmp 值最大有可能到8万多，但只要保正标准电阻
		//大于待测电阻的最大值，tmp值就不会超过65535，所以
		//返回16位整型即可
		return (uint16_t)tmp;
	}
}

void ADCProcess(void)
{
	
	switch(ADCStep)
	{
		//初始化ads1115
		case 0:
			ads1015_init();
			ADCStep = 1;
			break;
		//启动ADC
		case 1:
			readADC_Differential_0_1_part1();
			ADCStep = 2;
			ADCDelayCnt = 0;
			break;
		//等待ADC 转换完成
		case 2:
			if(ADCDelayCnt >= 8)	//1.4~1.6ms
			//if(ADCDelayCnt >= 2000)	//400ms
			{
				ADCStep = 3;
			}
			break;
		case 3:
			//读取AD 值
			ADC_Value = readADC_Differential_0_1_part2();
			//将AD 值转换成和电阻线性相关的值ADLinearVal
			ADLinearVal = ChangeADS1115ValueToResitor(ADC_Value);
			//标定
			if(CalibratingFlag)
			{
				Update2CaliDatabase(ADLinearVal, ActualLength);
				CalibratingFlag = 0;
			}
			//将tmp 转换成实际长度
			Length = Cali(ADLinearVal);
			//发送标定后的长度
			
			
			ADCStep = 1;
			break;
		default:
			ADCStep = 0;
			break;
	}
}

//??Uart??SendNum???
void UartSend(uint8_t SendNum)
{
    UartByteToBeSendCnt = SendNum;
    UartSendCnt=0;
}

//Uart????
void UartSendProcess(void)
{
    while(UartByteToBeSendCnt > 0)
    {
        if(eusartTxBufferRemaining > 0)
        {
            UartByteToBeSendCnt--;
            EUSART_Write(UartSendBuf[UartSendCnt++]);  // send a byte to TX 
        }
    }
}

//?????????????
void UartDriverRecBufferClr(void)
{
    //uint8_t temp;
    
    while(eusartRxCount)
    {
        EUSART_Read();
    }
}

//Uart????
void UartRecProcess(void)
{
   // uint8_t SendCnt=0;
    
    //???????????
    if(eusartRxCount > 0)
    {
        UartRecBuf[UartRecCnt++] = EUSART_Read();  // read a byte for RX
        
        if(UartRecCnt == 1)
        {
            if(UartRecBuf[0] != 0xf0)
            {
                UartRecCnt = 0;
                UartDriverRecBufferClr();
            }
        }
        else if(UartRecCnt == 2)
        {
            if(UartRecBuf[1] != 0xf1)
            {
                UartRecCnt = 0;
                UartDriverRecBufferClr();
            }
        }
        //?????????
        else if(UartRecCnt == 4)
        {
            UartRecCnt = 0;
            UartDriverRecBufferClr();

			//清除标定数据
            if((UartRecBuf[2] == 0xff)&&(UartRecBuf[3] == 0xff))
            {
				EraseCaliDatabase();
				ConstructCaliTab();
        	}
			//发送标定表格
			else if((UartRecBuf[2] == 0xfe)&&(UartRecBuf[3] == 0xfe))
			{
				CaliTabSendFlag = 1;
			}
			//记录相应的标定数据
			else
			{
				//发送标定消息给main 执行
				if(CalibratingFlag == 0)
				{
					ActualLength = ((u16)UartRecBuf[2] << 8) | ((u16)UartRecBuf[3]);
					CalibratingFlag = 1;

					//原封不动返回刚才标定发来的值
					/*SendCnt = 0;
		            UartSendBuf[SendCnt++] = UartRecBuf[2];
		            UartSendBuf[SendCnt++] = UartRecBuf[3];
	            	UartSend(SendCnt);*/
	            	printf("The Cali value is: %d mm\r\n",ActualLength);
				} 
			}
        }
        else if(UartRecCnt >= UartRecBufLen)
        {
            UartRecCnt = 0;
            UartDriverRecBufferClr();
        }
       
    } 
}

void SendDataToMainBoardProcess(void)
{
	static uint8_t SendCount=0;
	
	switch(SendDataToMainBoardStep)
	{
		//上电前1 s 发送正规数据给主板(每8ms 发一次)，即0xaa 0x55 0xXX 0xXX
		case 0:
			if(SendCount >= 125) // 125 * 8ms = 1s
			{
				SendDataToMainBoardStep = 2;
			}
			else
			{
				SendTmr = 0;
				SendDataToMainBoardStep++;
			}
			break;
		case 1:
			if(SendTmr >= 40) // 40*200us=8ms
			{
				//发送0xaa 0x55 0xXX 0xXX 给主板
	            UartSendBuf[0] = 0xaa;
	            UartSendBuf[1] = 0x55;
				UartSendBuf[2] = (uint8_t)(Length >> 8);
				UartSendBuf[3] = (uint8_t)Length;
            	UartSend(4);
			
				SendDataToMainBoardStep = 0;
				SendCount++;
			}
			break;
		//上电1s 以后就一直发送调试数据(每200ms 发一次)，包含AD,ADLiner,Length
		case 2:
			SendTmr = 0;
			SendDataToMainBoardStep++;
			break;
		case 3:
			if(SendTmr >= 1000) // 1000*200us=200ms
			{
				printf("AD: %d ADLinear: %d Length: %d mm\r\n",ADC_Value,ADLinearVal,Length);
				SendDataToMainBoardStep--;
			}
			break;
		default:
			SendDataToMainBoardStep = 0;
			break;
	}
}

void BaseTimeTickISR(void)  //200us per interrupt
{
    LedTimerCnt++;
    ADCDelayCnt++;
    SendTmr++;
}

void main(void)
{
	uint8_t tmp;
	
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    TMR0_SetInterruptHandler(BaseTimeTickISR);

	ConstructCaliTab();
	
    while (1)
    {
        if(LedTimerCnt >= 1000) // 200ms
        {
            LedTimerCnt = 0;
            //LedToggle();
            //printf("The Length is 38.5cm\r\n");
            //ADC_Value = ADC_GetConversion(channel_AN2);
            //itoa(ADC_Value,StringBuf,10);
            //printf("The AD Value is:%d\r\n",ADC_Value);
			//printf("AD: %d ADLinear: %d Length: %d mm\r\n",ADC_Value,ADLinearVal,Length);
        }
        UartRecProcess();
        UartSendProcess();
		ADCProcess();
		CaliTabSendProcess();
		SendDataToMainBoardProcess();
        
    }
}
/**
 End of File
*/
