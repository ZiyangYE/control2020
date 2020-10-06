/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "extfls.h"
#include "ctr.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CPLD_RST_CLR 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#define CPLD_RST_SET 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
char sendbuf[96];
uint8_t recvbuf[64];
volatile uint8_t recvCnt;
int32_t getr[8];
//状态量，当前是否稳定 0 不稳定 1 临界稳定 2 稳定
int stableStaus;
//误差量
int error;
//上次的误差量
//0 2
int lastError;
//积分输出
//0 2
int zInt;
//变速积分输出
//2 3
int zSlo;
//暂态输出
//0 2
int zTrn;
//输出
int z;
//补偿输出
//0 1 2 3
int zBas;
//消抖稳定状态
//0 2 3
int stableCnt;
//消抖稳定方向
//0 1 2 3
int stableVec;
//控制模式 0 位置控制 1 速度控制 2 力位控制 3 力速控制
uint8_t controlMode;

//积分目标值域
//1
int targetP[128];
//当前位置
//1
int pt;

//目标值
float target;
int z;

int hForc;
int hPosi;

uint8_t g_bAdcOver =0;


//位置一阶误差阈值
double sMaxThre;
//位置一阶输出
double sMaxOp;
//位置二阶误差阈值
double sSndThre;
//位置二阶P
double sSndP;
//位置二阶I
double sSndI;
//位置三阶I
double sTrdI;
//位置三阶D
double sTrdD;
//位置三阶I饱和阈值
double sTrdDF;
//位置接近误差阈值
double sClsThre;
//位置稳定周期数
double sStbT;

//启动减速比
double SLB;
//速率输出偏移量
double dsDif;
//速率输出调整周期
double dsChgT;
//速率输出输出调整量
double dsChgV;
//速率输出限幅
double dsMaxOp;

//力一阶误差阈值
double fMaxThre;
//力一阶输出
double fMaxOp;
//力二阶误差阈值
double fSndThre;
//力二阶P
double fSndP;
//力二阶I
double fSndI;
//力三阶I
double fTrdI;
//力三阶D
double fTrdD;
//力置三阶I饱和阈值
double fTrdDF;
//力接近误差阈值
double fClsThre;
//力变速积分I
double fSpcI;
//力去纹波阈值
double fReMThre;
//力去纹波P
double fReMP;
//力稳定周期数
double fStbT;

//力速输出偏移量
double dfDif;
//力速输出调整周期
double dfChgT;
//力速输出输出调整量
double dfChgV;
//力速输出限幅
double dfMaxOp;
//力速变速积分周期
double dfSpcT;
//力速变速积分比
double dfSpcI;
//力速变速积分权重
double dfSpcM;
//力速差归零阈值倍数
double dfDifZThre;

//设备类型0 法向 1切向
uint8_t devType;

//配置状态
uint8_t cfgAv;

FLASH_EraseInitTypeDef fls0;
uint32_t tmp0;

//从 0x1F000 开始 到0x1F300
uint8_t cfgbuf[289+4+8];

uint32_t reading_adr=0;
uint32_t writing_adr=0;

void m2(double* d1){
	for(int i=0;i<8;i++)
		*(uint8_t*)(d1+i)=*(volatile uint8_t*)(reading_adr+i);
	reading_adr+=8;
}

void readCfg(){
	for(int i=0;i<289+4;i++)
		*(uint8_t*)&cfgbuf[i]=*(volatile uint8_t*)(i+0x0801F000+i);
	if(cfgbuf[0]==0x01 && cfgbuf[1]==0xFE){
		uint8_t test0=0;
		uint8_t test1=0;
		for(int i=2;i<289+1;i++){
			test0^=cfgbuf[i];
			test1^=cfgbuf[i+1];
		}
		if(test0==cfgbuf[289+2]&&test1==cfgbuf[289+3]){
			reading_adr=(uint32_t)&cfgbuf[2];
			m2(&sMaxThre);
			m2(&sMaxOp);
			m2(&sSndThre);
			m2(&sSndP);
			m2(&sSndI);
			m2(&sTrdI);
			m2(&sTrdD);
			m2(&sTrdDF);
			m2(&sClsThre);
			m2(&sStbT);

			m2(&SLB);
			m2(&dsDif);
			m2(&dsChgT);
			m2(&dsChgV);
			m2(&dsMaxOp);

			m2(&fMaxThre);
			m2(&fMaxOp);
			m2(&fSndThre);
			m2(&fSndP);
			m2(&fSndI);
			m2(&fTrdI);
			m2(&fTrdD);
			m2(&fTrdDF);
			m2(&fClsThre);
			m2(&fSpcI);
			m2(&fReMThre);
			m2(&fReMP);
			m2(&fStbT);

			m2(&dfDif);
			m2(&dfChgT);
			m2(&dfChgV);
			m2(&dfMaxOp);
			m2(&dfSpcT);
			m2(&dfSpcI);
			m2(&dfSpcM);
			m2(&dfDifZThre);

			*(uint8_t*)(&devType)=*(volatile uint8_t*)(reading_adr);
			cfgAv=1;
		}
	}
	cfgAv=0;
}

//一次36*8+1个数据
//289个数据
//2头+289+2校验
void editCfg(int stage){
	static int wrcnt;
	switch (stage)
	{
	case 0://z输出
		if(cfgAv!=1)vout_level(0);
		break;
	case 1://读取数据并写入
		if(cfgAv==2){
			int tmpCnt=recvCnt;
			if(tmpCnt>5&&recvbuf[0]==0x03&&recvbuf[1]==0xF1){
				memcpy(cfgbuf+recvbuf[2]*256+recvbuf[3],&recvbuf[5+recvbuf[2]*256+recvbuf[3]],recvbuf[4]);
				wrcnt+=recvbuf[4];
				if(wrcnt>290){
					uint8_t test0=0;
					uint8_t test1=0;
					for(int i=2;i<289+1;i++){
						test0^=cfgbuf[i];
						test1^=cfgbuf[i+1];
					}
					if(test0==cfgbuf[289+2]&&test1==cfgbuf[289+3]){
						sendbuf[0]=0x04;
						sendbuf[1]=0xF2;
						sendbuf[2]=0x05;
						sendbuf[3]=0xF3;
						CDC_Transmit_FS((uint8_t *)sendbuf,4);

						int adr=0x0801F000;
						HAL_FLASH_Unlock();
						fls0.TypeErase=FLASH_TYPEERASE_PAGES;
						fls0.PageAddress=adr;
						fls0.NbPages=1;
						HAL_FLASHEx_Erase(&fls0,&tmp0);

						for(int i=0;i<(293+4);i+=4)				
						HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,0x0801F000+i,*(uint32_t*)&cfgbuf[i]);
										
						HAL_FLASH_Lock();

						HAL_Delay(1);
						NVIC_SystemReset();
					}
				}
			}
		}
		break;
	case 2://上传数据
		wrcnt=0;
		HAL_Delay(10);
		CDC_Transmit_FS((uint8_t *)cfgbuf,293);
		HAL_Delay(10);
		CDC_Transmit_FS((uint8_t *)cfgbuf,293);
		HAL_Delay(10);
		CDC_Transmit_FS((uint8_t *)cfgbuf,293);
		break;
	default:
		break;
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//额外保护
	int extraPUpLimit=150000000;//145mm
	int extraPDnLimit=-10000;//-0.01mm
	int extraFUpLimit=100000000;//1000kN
	//设定保护
	int pUpLimit=2000000000;
	int pDnLimit=-100000000;
	int fUpLimit=2000000000;
	//0TODO 非调试状态，moded=0
	uint8_t moded=0;//已校准=1，未校准=0
	//int modeD[2]={-1111400,27600};
	int modeD[2]={0,0};
	//float modeK[2]={0.0483973,-0.035609};
	float modeK[2]={1,1};
	
	int modedZ=0;
	int modingact=0;
	int moding[8];
	//DEBUG
	int modeAV=0;
	
	uint32_t mem[9];
	
	int sysStableCnt;
	int lhPosi;
	int lhForc;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	vout_level(0);
	
	//读取校准状态 读取设限状态
	//0x0801F800校准数据 +0 D[0] +4 D[1] +8 K[0] +12 K[1] +16 0x01 +20 0xFE
	//0x0801FC00设限数据 +0 pU +4 0x02 +8 0xFD +12 pD +16 0x03 +20 0xFC +24 fU +28 0x04 +32 0xFB
	uint32_t adr=0x0801F800;
	if(*(volatile uint32_t*)(adr+16)==0x01 && *(volatile uint32_t*)(adr+20)==0xFE){
		*(uint32_t*)&modeD[0] = *(volatile uint32_t*)adr;
		*(uint32_t*)&modeD[1] = *(volatile uint32_t*)(adr+4);
		*(uint32_t*)&modeK[0] = *(volatile uint32_t*)(adr+8);
		*(uint32_t*)&modeK[1] = *(volatile uint32_t*)(adr+12);
		
		moded=1;
		modeAV=1;
	}
	adr=0x0801FC00;
	if(*(volatile uint32_t*)(adr+4)==0x02 && *(volatile uint32_t*)(adr+8)==0xFD){
		*(uint32_t*)&pUpLimit = *(volatile uint32_t*)(adr);
	}
	adr+=12;
	if(*(volatile uint32_t*)(adr+4)==0x03 && *(volatile uint32_t*)(adr+8)==0xFC){
		*(uint32_t*)&pDnLimit = *(volatile uint32_t*)(adr);
	}
	adr+=12;
	if(*(volatile uint32_t*)(adr+4)==0x04 && *(volatile uint32_t*)(adr+8)==0xFB){
		*(uint32_t*)&fUpLimit = *(volatile uint32_t*)(adr);
	}
	
	if(cfgAv!=1)moded=0;

	HAL_Delay(3000);
	
	
	CPLD_RST_CLR;
	HAL_Delay(1);
	CPLD_RST_SET;
	for(int i=0;i<5;i++){
		while(g_bAdcOver == 0);
		get_data(getr);
		hPosi=(int)(getr[7]/256+modeD[0])/modeK[0];
		g_bAdcOver=0;
	}
	stableStaus=0;
	sysStableCnt=0;
	if(moded){
		controlMode=0;
		target=hPosi;
	}else{
		controlMode=4;
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	
	
  while (1)
  {
		if(g_bAdcOver)
		{
			get_data(getr);
			g_bAdcOver = 0;
			
			hPosi=(int)(getr[7]/256+modeD[0])/modeK[0];
			hForc=(int)(getr[6]/256+modeD[1])/modeK[1];

			
			sendbuf[0]=	0xFE;
			sendbuf[1]=	0xFE;
			sendbuf[2]=	moded;
			sendbuf[3]=	devType;
			sendbuf[4]=	controlMode;
			sendbuf[5]=	(hPosi>>24);
			sendbuf[6]=	(hPosi>>16)&0xFF;
			sendbuf[7]=	(hPosi>>8)&0xFF;
			sendbuf[8]=	hPosi&0xFF;
			sendbuf[9]=	(hForc>>24);
			sendbuf[10]=(hForc>>16)&0xFF;
			sendbuf[11]=(hForc>>8)&0xFF;
			sendbuf[12]=hForc&0xFF;
			sendbuf[13]=0;
			sendbuf[14]=0xFF;
			sendbuf[15]=0xFF;
			for(int i=2;i<13;i++)sendbuf[13]^=sendbuf[i];
			
			CDC_Transmit_FS((uint8_t *)sendbuf,16);
			//sprintf(sendbuf,"force:|%d| *10mN,\tposition:|%d| nm\n",hForc,hPosi);
			//CDC_Transmit_FS((uint8_t *)sendbuf,strlen(sendbuf));
			
			if(moded==2){
				controlMode=0;
			}
			controlCore();
			
			
			if(hPosi>pUpLimit){z=5000000;}
			if(hPosi<pDnLimit){z=-5000000;}
			if(moded==1)if(hForc>fUpLimit){z=1000000;}
			
			if(hPosi>extraPUpLimit){z=5000000;}
			if(hPosi<extraPDnLimit){z=-5000000;}
			if(moded==1)if(hForc>extraFUpLimit){z=1000000;}
			
			if(moded==0)z=modedZ;
			
			vout_level(-z);
			editCfg(0);

			if(stableStaus==1)stableStaus=2;
			if(stableStaus==0){
				if(abs(lhPosi-hPosi)<100000 && abs(lhForc-hForc)<200000){
					sysStableCnt++;
					if(sysStableCnt>2){
						stableStaus=1;
					}
				}else{
					sysStableCnt=0;
				}
			}
			lhPosi=hPosi;
			lhForc=hForc;
		}
		if(recvCnt!=0){
			editCfg(1);
			uint8_t tmpbuf[64];
			uint8_t test;
			int tmpCnt=recvCnt;
			memcpy(tmpbuf,recvbuf,tmpCnt);
			recvCnt=0;
			for(int i=0;i<tmpCnt-7;i++){
				if(tmpbuf[i]==0xFE && tmpbuf[i+7]==0xFF){
					test=0;
					for(int j=1;j<6;j++)test^=tmpbuf[i+j];
					if(test==tmpbuf[i+6]){
						int data;
						*(uint32_t*)(&data)=(tmpbuf[i+2]<<24)+(tmpbuf[i+3]<<16)+(tmpbuf[i+4]<<8)+(tmpbuf[i+5]);
						//开始处理下发数据包
						switch(tmpbuf[i+1]>>4){
							case 0:
								//校准
								switch(tmpbuf[i+1]&0x0F){
									case 0://开始校准
										modedZ=0;
										moded=0;
										modingact&=0x00;
										break;
									case 1://设定输出
										modedZ=-data;
										break;
									case 2://读取0位posi
										moding[0]=getr[7]/256;
										modingact |= 0x01;
										break;
									case 3://读取满位posi
										moding[1]=getr[7]/256;
										modingact |= 0x02;
										moding[5]=data;
										break;
									case 4://完成位移校准
										modeK[0]=(float)(((double)(moding[1]-moding[0]))/(double)(moding[5]));
										modeD[0]=-moding[0]-modeK[0]*100000;
										moded=1;
										modedZ=0;
										target=(int)(getr[7]/256+modeD[0])/modeK[0];
										modeAV=1;
										//Flash 写入
										adr=0x0801F800;
										HAL_FLASH_Unlock();
										fls0.TypeErase=FLASH_TYPEERASE_PAGES;
										fls0.PageAddress=adr;
										fls0.NbPages=1;
										HAL_FLASHEx_Erase(&fls0,&tmp0);
										
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr,*(uint32_t*)&modeD[0]);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+4,*(uint32_t*)&modeD[1]);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+8,*(uint32_t*)&modeK[0]);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+12,*(uint32_t*)&modeK[1]);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+16,0x01);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+20,0xFE);
										
										HAL_FLASH_Lock();
										break;
									case 5://设定位移
										target=data;
										break;
									case 6://读取0位力
										moding[2]=getr[6]/256;
										modingact |= 0x04;
										break;
									case 7://读取参考位力
										moding[3]=getr[6]/256;
										modingact |= 0x08;
										moding[4]=data;
										break;
									case 8://完成校准
										
										modeK[1]=(float)(((double)(moding[3]-moding[2]))/(double)moding[4]);
										
										
										modeD[1]=-moding[2]-modeK[1]*1000;
									
										moded=1;
										modedZ=0;
										modeAV=1;
										//Flash 写入
										adr=0x0801F800;
										HAL_FLASH_Unlock();
										fls0.TypeErase=FLASH_TYPEERASE_PAGES;
										fls0.PageAddress=adr;
										fls0.NbPages=1;
										HAL_FLASHEx_Erase(&fls0,&tmp0);
										
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr,*(uint32_t*)&modeD[0]);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+4,*(uint32_t*)&modeD[1]);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+8,*(uint32_t*)&modeK[0]);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+12,*(uint32_t*)&modeK[1]);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+16,0x01);
										HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,adr+20,0xFE);
										
										HAL_FLASH_Lock();
									
										break;
									case 0x0A://开始力校准
										moded=2;
										modedZ=0;
										target=0;
										break;
									
									default://取消校准
										if(modeAV==1){
											moded=1;
											modedZ=0;
										}else{
											modedZ=0;
										}
										break;
								}
								break;
							case 1:
								//设限
								adr=0x0801FC00;
								for(int wrti=0;wrti<9;wrti+=4){
									*(uint32_t*)&mem[wrti] = *(volatile uint32_t*)(adr+4*wrti);
								}
								switch(tmpbuf[i+1]&0x0F){
									case 0://位移上限
										pUpLimit=data;
										mem[1]=0x02;
										mem[2]=0xFD;
										break;
									case 1://位移下限
										pDnLimit=data;
										mem[4]=0x03;
										mem[5]=0xFC;
										break;
									case 2://力上限
										fUpLimit=data;
										mem[7]=0x04;
										mem[8]=0xFB;
										break;
									default:
										break;
								}
								//TODO 写入flash
								break;
							case 2:
								//运动
								if(moded!=1)break;
								sysStableCnt=0;
								stableStaus=0;
								controlMode=tmpbuf[i+1]&0x0F;
								if((controlMode&0x01)==0x1)
									target=data/1000.0;
								else
									target=data;
								break;
							case 3:
								//配置数据
								cfgAv=2;
								editCfg(2);
								break;
						}
						//完成处理下发数据包
						break;
					}
				}
			}
			
			
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
