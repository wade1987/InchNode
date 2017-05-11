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
uint16_t Length; //��λmm
uint16_t ActualLength; //��λmm
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
* Description       :  �����������ֵ����ص������㼰PM2.5 ��ȩ�궨��
* Input               :  Tab ---�������
*                  	  	 TabSiz---��񳤶�
*                  	  	 OriginalVal---ԭʼֵ
* Output             :  16λ���
* Return             :  ���ص�ֵ�����Ǳ��ֵҲ�������������ֵ֮��
* 			       ������ֵ
*******************************************************************************/

uint16_t LinearLookUpTab(stTab *Tab, uint8_t TabSiz, uint16_t val)
{
	uint8_t i;
	uint32_t ret;

	//�������
	
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
* Description    : ����CaliBuf �ڵ�ֵ
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
* Description    : �����궨������������
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
* Description    : ���µı궨ֵ�������ݿ�
* Input          : У׼�豸�ı�׼ֵ
* Output         : None
* Return         : None
*******************************************************************************/

void Update2CaliDatabase(uint16_t ori_val, uint16_t ActualValue)
{
	uint8_t i, j, m, k;
	uint16_t OriginalValue;
	uint8_t tmp[4];

	OriginalValue = ori_val;		

	//ÿ��ǰ1/3 �β��궨����ֹ�����֮�佻��
	if((ActualValue % 30) < 10) //10~30mm,ÿ30mm�궨һ��
	{
		return;
	} 

	//������Χ
	if((OriginalValue > 52800) || (ActualValue > 1200)) // 1.2m
	{
		return;
	}
	//if((OriginalValue == 0) || (ActualValue == 0))
	//{
		//return;
	//}

	//��λ�궨ֵ�����ݿ��λ��
	i = ActualValue / 30;
	if(i >= CALI_BUf_LEN)
		i = CALI_BUf_LEN - 1;

	//����CaliBuf
	UpdateCaliBuf();

	CaliBuf[i].OriginalValue = OriginalValue;
	CaliBuf[i].ActualValue = ActualValue;

	//�ж�У׼���ݿ���û�и�����������
	//����У�Ҫô��У׼��Ҫô��sport �Ĵ�����������
	for(m = 0 ; m < CALI_BUf_LEN-1 ; m++)
	{
		for(j = m+1 ; j < CALI_BUf_LEN ; j++)
		{
			//���CaliBuf �е�������Ч��˵���ö��ѱ��궨
			if((CaliBuf[j].OriginalValue <= 52800) && (CaliBuf[j].ActualValue <= 1200) &&\
				(CaliBuf[m].OriginalValue <= 52800) && (CaliBuf[m].ActualValue <= 1200))
				//(CaliBuf[j+1].OriginalValue >= 0) && (CaliBuf[j+1].OriginalValue <= 2000)&& (CaliBuf[j+1].ActualValue >= 0) && (CaliBuf[j+1].ActualValue <= 2000))
			{
				if((CaliBuf[m].OriginalValue > CaliBuf[j].OriginalValue) || (CaliBuf[m].ActualValue > CaliBuf[j].ActualValue))
				{
					//���ָ�����
					//����궨���ݿ����е�����
					for(k = 0 ; k < CALI_BUf_LEN ; k++)
					{
						CaliBuf[k].OriginalValue = 0xffff;
						CaliBuf[k].ActualValue = 0xffff;
					}
					//Ȼ��ֻ����ǰ���µ�һ���궨ֵ
					CaliBuf[i].OriginalValue = OriginalValue;
					CaliBuf[i].ActualValue = ActualValue;
				}
			}
		}
	}
	
	//��ʼ��¼�궨ֵ
	EraseCaliDatabase();
	for(j = 0 ; j < CALI_BUf_LEN ; j++)
	{
		tmp[0] = (CaliBuf[j].OriginalValue >> 8);
		tmp[1] = CaliBuf[j].OriginalValue;
		tmp[2] = (CaliBuf[j].ActualValue >> 8);
		tmp[3] = CaliBuf[j].ActualValue;

		//�궨ֵ������eeprom��
		EEpromBuffeWirte(tmp, CALI_BUF_ADDR+j*4, 4);
	}
	
}

/*******************************************************************************
*                wade@2015-09-07
* Function Name  : ConstructCaliTab
* Description    : ���ݱ궨�⽨���궨���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void ConstructCaliTab(void)
{
	uint8_t i, j=0;

	//����CaliBuf
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

	//˵�����豸û�б궨��
	if(j == 0)
	{
		CaliTabSize = j;
	}
	//ֻ�궨��һ����
	else if(j == 1)
	{
		CaliTabSize = j;
	}
	//�궨���������ϵĵ�
	else
	{
		CaliTabSize = j+1;
		//�ڶ���������һ����
		CaliTab[j].OriginalValue = 52800; 
		CaliTab[j].ActualValue = ((CaliTab[j-1].ActualValue-CaliTab[j-2].ActualValue)*(52800 - CaliTab[j-1].OriginalValue))\
								/(CaliTab[j-1].OriginalValue - CaliTab[j-2].OriginalValue) + CaliTab[j-1].ActualValue;
	}
}

/*******************************************************************************
*                           wade@2016-09-09
* Function Name  :  Dfu_CaliTabSend
* Description    : ���ͱ궨���
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
		//����У׼���
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
* Description    : �������ֵͨ���궨������У׼
* Input          : ��ҪУ׼��ֵ
* Output         : У׼���ֵ
* Return         : None
*******************************************************************************/

uint16_t Cali(uint16_t InData)
{
	//˵�����豸û�б궨��
	if(CaliTabSize == 0)
	{
		return InData;
	}
	//ֻ�궨��һ����
	else if(CaliTabSize == 1)
	{
		//printf("Indata:%d,CaliTab.ActVal:%d,CaliTab.Ori:%d",InData,CaliTab[0].ActualValue,CaliTab[0].OriginalValue);
		return ((uint32_t)InData*(uint32_t)CaliTab[0].ActualValue) / (uint32_t)CaliTab[0].OriginalValue;
	}
	//�궨���������ϵĵ� 
	else
	{
		return LinearLookUpTab(CaliTab, CaliTabSize, InData);
	}
}


//��AD �������ķ�ѹֵת���ɺ͵���������ص�ֵ
uint16_t ChangeADS1115ValueToResitor(int16_t ADC_value)
{
	uint32_t tmp;
	
	if(ADC_value <= 0)
		return 0;
	else
	{
		tmp = (uint32_t)ADC_value*52800; //52800��3.3V ������2.048v�ο���ѹ�������16 λADֵ
		tmp = tmp/(52800-ADC_value);
		//��Ȼtmp ֵ����п��ܵ�8��࣬��ֻҪ������׼����
		//���ڴ����������ֵ��tmpֵ�Ͳ��ᳬ��65535������
		//����16λ���ͼ���
		return (uint16_t)tmp;
	}
}

void ADCProcess(void)
{
	
	switch(ADCStep)
	{
		//��ʼ��ads1115
		case 0:
			ads1015_init();
			ADCStep = 1;
			break;
		//����ADC
		case 1:
			readADC_Differential_0_1_part1();
			ADCStep = 2;
			ADCDelayCnt = 0;
			break;
		//�ȴ�ADC ת�����
		case 2:
			if(ADCDelayCnt >= 8)	//1.4~1.6ms
			//if(ADCDelayCnt >= 2000)	//400ms
			{
				ADCStep = 3;
			}
			break;
		case 3:
			//��ȡAD ֵ
			ADC_Value = readADC_Differential_0_1_part2();
			//��AD ֵת���ɺ͵���������ص�ֵADLinearVal
			ADLinearVal = ChangeADS1115ValueToResitor(ADC_Value);
			//�궨
			if(CalibratingFlag)
			{
				Update2CaliDatabase(ADLinearVal, ActualLength);
				CalibratingFlag = 0;
			}
			//��tmp ת����ʵ�ʳ���
			Length = Cali(ADLinearVal);
			//���ͱ궨��ĳ���
			
			
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

			//����궨����
            if((UartRecBuf[2] == 0xff)&&(UartRecBuf[3] == 0xff))
            {
				EraseCaliDatabase();
				ConstructCaliTab();
        	}
			//���ͱ궨���
			else if((UartRecBuf[2] == 0xfe)&&(UartRecBuf[3] == 0xfe))
			{
				CaliTabSendFlag = 1;
			}
			//��¼��Ӧ�ı궨����
			else
			{
				//���ͱ궨��Ϣ��main ִ��
				if(CalibratingFlag == 0)
				{
					ActualLength = ((u16)UartRecBuf[2] << 8) | ((u16)UartRecBuf[3]);
					CalibratingFlag = 1;

					//ԭ�ⲻ�����ظղű궨������ֵ
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
		//�ϵ�ǰ1 s �����������ݸ�����(ÿ8ms ��һ��)����0xaa 0x55 0xXX 0xXX
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
				//����0xaa 0x55 0xXX 0xXX ������
	            UartSendBuf[0] = 0xaa;
	            UartSendBuf[1] = 0x55;
				UartSendBuf[2] = (uint8_t)(Length >> 8);
				UartSendBuf[3] = (uint8_t)Length;
            	UartSend(4);
			
				SendDataToMainBoardStep = 0;
				SendCount++;
			}
			break;
		//�ϵ�1s �Ժ��һֱ���͵�������(ÿ200ms ��һ��)������AD,ADLiner,Length
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
