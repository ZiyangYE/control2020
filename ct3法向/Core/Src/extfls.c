#include "extfls.h"

extern SPI_HandleTypeDef hspi1;

void delay_cycles(uint16_t i);

void extdac_out(uint32_t dat);

void delay_cycles(uint16_t i)
{
	while(i)
	{
		i--;
	}
}

void vout_level(int32_t  s)
{
	int32_t tmp;
	int32_t val;
	
	if(s==0)
	{
		SERVO_FWD_DISENA;
		SERVO_REV_DISENA;
	}
	else if(s<0)
	{
		SERVO_REV_ENABLE;
		SERVO_FWD_DISENA;	
	}
	else
	{
		SERVO_FWD_ENABLE;
		SERVO_REV_DISENA;			
	}
	
	
	val=abs(s);
	
	if(val > 10000000)
		val = 10000000;
	else if(val<0)
		val=0;
	
	tmp = val * (4095.0/ 10000000);  
	tmp &= 0x0000ffff;
	extdac_out(tmp);
}


void extdac_out(uint32_t dat)
{
	uint8_t i;
	
	SYNC_LOW;
	
	dat = dat<<2 ;
	
	for(i=0;i<16;i++) 
	{ 
		SCLK_HIGH;
		delay_cycles(5);
		
		if(0x8000&dat) 
		{ 
			DIN_HIGH; 
		} 
		else 
		{ 
		 DIN_LOW; 
		}
		delay_cycles(5);
		
		SCLK_LOW;
		delay_cycles(5);
		
		dat<<=1;
	}
	
	SYNC_HIGH;
	delay_cycles(5);
}

void init_adc(void){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	delay_cycles(10);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
	delay_cycles(10);
}
#if 0
uint16_t SPI_FPGA_SendByte(uint16_t byte)
{
	uint16_t recvbyte;
	while(HAL_SPI_GetState(&hspi1)!=HAL_SPI_STATE_READY);
	if(HAL_SPI_Transmit(&hspi1,(uint8_t*)&byte,1,100)!=HAL_OK)return 0xFFFF;
	while(HAL_SPI_GetState(&hspi1)!=HAL_SPI_STATE_READY);
  if(HAL_SPI_Receive(&hspi1,(uint8_t*)&recvbyte,1,100)!=HAL_OK)return 0xFFFF;
  return recvbyte;
}

uint16_t SPI_FPGA_Read(uint8_t address)
{
	uint16_t tmp = (uint16_t)address<<10 | 0x2ab;
	
	return(SPI_FPGA_SendByte(tmp));
}

void SPI_FPGA_Write(uint8_t RegAddr, uint8_t data)
{
	
	uint16_t tmp  = (uint16_t)RegAddr<<10 | data; 
    SPI_FPGA_SendByte(tmp);
}
#endif
HAL_StatusTypeDef SPI_FPGA_Read(uint8_t address,uint16_t *pu16RxData)
{
	HAL_StatusTypeDef ret;
	uint16_t tmp = (uint16_t)address<<10 | 0x2ab; 

	ret= HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tmp, (uint8_t *)pu16RxData, 1,\
                                          0x1000);
	return ret;
}

HAL_StatusTypeDef SPI_FPGA_Write(uint8_t RegAddr, uint8_t data)
{
		HAL_StatusTypeDef ret;
    //SPI_FPGA_CS_HIGH();
		//delay(10);
	
		uint16_t tmp  = (uint16_t)RegAddr<<10 | data; 
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&tmp, 2, 0x1000);
	
    //SPI_FPGA_CS_LOW();
	return ret;
}


void get_data(int32_t* getr){
	uint8_t i;
	
	uint16_t cpld_dat[26];
	uint16_t tmp_dat;
	
	for(i=0; i<26; i++)
	{
		SPI_FPGA_Read(i,&tmp_dat);
		cpld_dat[i] = (uint8_t)tmp_dat;
	}
	for(i=0;i<6;i++)getr[i]=cpld_dat[i];
	getr[6]=(cpld_dat[0]<<24)+(cpld_dat[1]<<16)+(cpld_dat[2]<<8);
	getr[7]=(cpld_dat[3]<<24)+(cpld_dat[4]<<16)+(cpld_dat[5]<<8);
}
