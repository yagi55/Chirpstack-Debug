/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATANUM 20											// usart2 receiving ring buffer size
#define WTDATANUM 127										// usart3 receiving ring buffer size
//#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  6)

#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  6)
static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */


static uint8_t DEBG= 0;										// DEBG=1: use usart2 (debug purpose)
															// DEBG=0: use usart1 (standard use)
static uint8_t FBM = 1 ;									// FBM =1 : standard use
															// FBM =0 : No Feed Back mode(dac_value fixed)
static uint8_t test_cv=0;									// test_cv = 0 : normal
															// test_cv = 1 : Do not update dac_value( stay dac_value)
static uint8_t adxl_flag = 1;								// adxl_flag = 0: default enable adxl345
															// adxl_flag = 1: default disable adxl345
static uint8_t auto_power_operation = 1;					// auto_power_operation = 0: Disable
														    // auto_power_operation = 1: Enable

static int16_t dac_value = 0;								// current dac_value(initial value is 0)

/**ADC1 GPIO Configuration
 PC1     ------> ADC1_IN2
 PA0     ------> ADC1_IN5
 PA5     ------> ADC1_IN10
 PA6     ------> ADC1_IN11
 PA7     ------> ADC1_IN12
 PB0     ------> ADC1_IN15
 */
static int ADC02[512];										// Current value of battery charging sensed by mlabo 25A
static int ADC05[512]; 										// DC/DC Current value sensed by Xm ohm
static int ADC10[512]; 									  	// AC current value sensed by mlabo 200A
static int ADC11[512];										// DC/DC regulator output voltage
static int ADC12[512];										// Differential Voltage prove Sensor(under development...)
static int ADC15[512];										// Charger or battery voltage
static int ADC10_frame_value[50]={0};						// 50 data of ADC10(1sec value)

static int n = 0; 											// ADC call back function counter
static int ac_flag = 0;										// ac current snapshot command flag--!!!!test!!!!

static uint32_t tim_counter =0; 			  				// for TIM_2 call back function counter
static int target_value = 0;								// charge current target value
static int new_target_value = 0;							// for new target value
//static int i_update_counter = -1;							// 5sec target_value update counter
//static int p_update_counter = -1;							// 5sec power source update counter
//static int pending_flag = 0;								// power source update pending flag

static float rms_ac = 0 ;									// av10 ac rms value(AC current rms value)

uint8_t serialData[DATANUM] = {0}; 	  						// usart2(or1) receive ring buffer array
char current_rx_data[DATANUM] = {0};  						// array of cut out data from usart2 ring buffer
static int	indexRead = 0;									// usart2(or1) ring buffer read index
static int former_indexRead=0;								// usart2(or1) ring buffer former read index(1 sample before indexRead)

uint8_t water_serialData[WTDATANUM] = {0}; 	  				// usart3 receive ring buffer array
char water_current_rx_data[WTDATANUM] = {0};  				// array of cut out data from usart3 ring buffer
static int	water_indexRead = 0;							// usart3 ring buffer read index
static int water_former_indexRead=0;						// usart3 ring buffer former read index(1 sample before water_indexRead)

struct water_sensor{										// Structure of water sensor data
	char *data_type;										// data type
	char cnt[10];											// data count
	char unix_time[20];										// unix time(13 digits)
	char water_temp[5];										// water temperature x 100
	char chsum[2];											// check sum (hex)
} ;

static struct water_sensor ws_static;						// water_sensor static data area
static struct water_sensor *pws_static;						// water_sensor static data pointer definition(for test purpose)
static int chsum_flag =0;									// check sum flag (0:OK, 2;NG)
static int water_tmp_flag = 0;	 							// water temperature sensor active flag
static float water_temp_2 = 0;								// current water temperature

//for i2c variables------------------------------------------------------------*/
uint8_t i2cRxBuffer[32]={0};								// i2C Receive Buffer
int16_t LM75B_temp[3]={0};									// LM75B Temperature data buffer (for 12 bit data)
uint8_t i2cTxLM75Buffer[2]={0,1};							// i2c Transmit Buffer(pointer byte,config data)
static	float LM75B_d;										// i2c temperature #1
static  float LM75B_e;										// i2c Temperature #2

float acc_value[3]={0};
int16_t ADXL345_acc[3]={0};

//for Cosel extended USART function variables---------------------------------

static uint8_t u5TxBuffer[5] ={254,0xEE,232,224,225};		// Input Voltage Monitor command data
static uint8_t u5RxBuffer[10];								// UART5 receive buffer
static uint8_t u5Buffer[32]={0};							// Command and Output Buffer(transmit 5byte, receive 5byte and reserve 22byte)
static uint8_t t_flag = 0;									// Input Voltage monitor request flag(1:request, 0: no request)
static uint8_t f_flag = 0;									// FAN Speed monitor request flag(1:request, 0:no request)
static float input_voltage = 0;								// Input Voltage value


//power operation and visa operation related flag
static uint8_t visa_flag = 0;								// Current 0 protect operation visa test purpose
static int C[10] = {0,624,895,1031,1200,1500,0,0,0,0};		// target_value ZONE 閾値
static int pending_flag = 0;								// pending_flag for power source operation.
static int i_counter = 0;									// wait timer for target_value change
static int p_counter = 0;									// pending_flag counter (wait xxx sec)
static uint8_t volt_set =0;									// sign for target_value change




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

void clear_buffer(char *buf, uint8_t i);					// String(char) array clear function. Put array top address and array size
void readSerial();											// USART2 (command,log etc.. )read serial data by DMA function
void water_readSerial();									// USART3 (water sensor)read serial data by DMA function
void data_cut();											// USART2 data cut function
void water_data_cut();										// USART2 data cut function
void command_interpret(void);								// command interpretation for usart 2
void water_command_interpret();								// water sensor data output interpretation
float LM75B_temp_get( uint8_t i2c_ad);						// get temperature data

static uint8_t adxl_register_write( uint8_t reg, uint8_t reg_data);				//AXDL345 Register Write
static uint8_t adxl_register_read( uint8_t reg);								// AXDL345 Register Read(single read)
static uint8_t adxl_register_multi_read( uint8_t reg, uint8_t read_data);       // AXDL345 FiFo Read(32byte read)


void uartclear_buffer(uint8_t *buf, uint8_t i);												// uint8_t buffer clear ( for uart5)
float respons_interpret(uint8_t t1, uint8_t t2, uint8_t t3, uint8_t t4);					// Cosel received command data interpretation.
void send_20bit_command(uint8_t dev_ad, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4);	// Make and send Cosel 20bit Transmit command

void target_value_update();									// target_value update feature for auto power operation
void pending_flag_operation();								// pending flag operation. if pending flag=0, do nothing!


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2){

		int diff_value;										// Differential value between target value and current value
		int interval_flag;									// Only operates when intervla_flag == 0
		interval_flag = tim_counter%3;				 		// It may equivalent to Delay factor ( equal to 60ms )

		if(auto_power_operation == 1){
			if(target_value != new_target_value){
				target_value_update();
			}else if(target_value == new_target_value){
				pending_flag_operation();
			}else{
				// may not have any condition
			}
		}else if(auto_power_operation ==0){
			target_value = new_target_value;
		}else{
			// may not have any condition
		}

		if(dac_value >=3010){								// secure operation	purpose
			dac_value = 0;
			new_target_value = 0;
			printf("9814, dac_value>=3010 \r\n");
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		}else if(dac_value <0){
			dac_value = 0;
		}
		if(target_value ==0){
			dac_value =0; 									// press return key then dac_value go to zero!
		}

		if(FBM==0){
			dac_value = target_value; //enable only for non feed back mode!!!//
		}else if(FBM==1){
		   //	nothing to do
		}


		if(tim_counter%50 == 25){
			//printf("water_indexRead = %d\r\n",water_indexRead);
			//printf("water_former_indexRead = %d\r\n",water_former_indexRead);
			//printf("serialData = %s\r\n", serialData);
			//printf("current_rx_data = %s\r\n", current_rx_data);
			data_cut();
			water_data_cut();
		}


		//dac_value = target_value;
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
		//HAL_Delay(2); 				// tentative value
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)aADCxConvertedData, 6);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(2);
		int sum02 = 0;int sum05 = 0;int sum10 = 0;int sum11 = 0;int sum12 = 0;int sum15 = 0;
		int av02 = 0 ;int av05  = 0;int av10  = 0;int av11  = 0;int av12  = 0;int av15  = 0;
		for(int j= 0;j<=511;j++){
				sum02 = sum02 + ADC02[j];
				sum05 = sum05 + ADC05[j];
				sum10 = sum10 + ADC10[j];
				sum11 = sum11 + ADC11[j];
				sum12 = sum12 + ADC12[j];
				sum15 = sum15 + ADC15[j];
				//printf("ADC0[%d] = , %d\r\n", j, ADC0[j]);
		}
		av02 = sum02/512;
		av05 = sum05/512;
		av10 = sum10/512;
		av11 = sum11/512;
		av12 = sum12/512;
		av15 = sum15/512;

		if(av15 > 3050){
			dac_value = 0; new_target_value = 0;
			printf("9814, av15 >3050 \r\n");
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			//printf("av15 = %d\r\n", av15);
		}

		//Operation for protecting over voltage for DC/DC converter-----
		if(visa_flag==1 && av05 ==0 && target_value > 200){
			dac_value = 0; new_target_value = 0;
			printf("9814, av05=0 \r\n");
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			visa_flag = 0;
		}

		// Disable protecting over voltage operation for DC/DC converter -----
		if(visa_flag==1 && target_value ==0){
			printf("9814, target_value set 0 \r\n");
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			visa_flag = 0;
		}


		ADC10_frame_value[tim_counter%50] = av10;
		int sum10_frame=0;
		for(int fn=0;fn<50;fn++){
			sum10_frame = sum10_frame + ADC10_frame_value[fn];
		}
		int av10_frame_average = sum10_frame/50;

		float av02_BC = (float)av02*(-0.0063)+ 24.114;		   				// Battery charging current(mlabo 25A)
		float av05_SC = 0.0074*(float)av05 + 0.4012;						// Shunt resister current value(4m ohm)
		float av10_SC = (float)av10*(-0.0073)+ 27.345;						// Input current of DC/DC converter ()
		float av10_SC_FAV = (float)av10_frame_average*(-0.0073)+ 27.345;	// Input current of DC/DC converter (frame average)



//---  Constant current operation main routine.-----------------------------------------
		diff_value = target_value - av05;
		//int accelerate_p = 100;  // for test purpose value

		if( interval_flag == 0 && FBM == 1  && test_cv == 0   ){ 		// always check interval_flag == 0 now.
			if(diff_value > 300 ){
				dac_value = dac_value + 10;
			}else if(diff_value > 300){
				dac_value = dac_value + 4;
			}else if(diff_value < -300 ){
				dac_value = dac_value - 4;
				if(dac_value<0) dac_value = 0;
			}else if(diff_value <=300 && diff_value >75 && interval_flag == 0 ){
				dac_value = dac_value + 1;
			}else if(diff_value >=-300 && diff_value < -75 && interval_flag == 0 ){
				dac_value = dac_value -1 ;
				if(dac_value<0) dac_value = 0;
			}
		}else if(FBM==0 || test_cv !=0){
			// nothing to do
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

		if( tim_counter%100 == 0 ){  			// 2 sec interval execution
			//printf("test2!! \r\n");
			LM75B_d = LM75B_temp_get( 0x48 );
			//LM75B_e = LM75B_temp_get( 0x49 );
		}


		if(tim_counter%50 == 0){
			//printf("t_flag = %d\r\n",t_flag);
			printf(" %d,", t_flag);							// t_flag (tentative status)
			printf(" %d," , (int)tim_counter/5);			// tim counter (== 100ms)
			printf(" %d,", av05);							// shunt current adc value
			printf(" %.2f,", av05_SC);						// shunt current value[A]
			printf(" %.3f,", rms_ac);						// AC current rms value
			printf(" %d,", av11);							// DC/DC output voltage value
			printf(" %d,", av15);							// Battery voltage
			printf(" %d,", av02);							// Charging current of ADC value from mlabo25
			printf(" %.3f,",av02_BC);						// Charging current [A]
			printf(" %d,", dac_value);						// dac_value for Master DC/DC regulator
			printf(" %d,", target_value);					// target current value by ADC value
			printf(" %.2f,", LM75B_d);						// Additional temperature value 1
			printf(" %.2f,", LM75B_e);						// Additional temperature value 2
			printf(" %.2f,", water_temp_2/100);				// Water temperature
			printf(" %.2f,", input_voltage);				// Input voltage value
			printf(" %d,", av10);							// av10 value(DC input current)
			printf(" %.2f,",av10_SC);						// av10 current value(DC input)
			printf(" %.2f,",av10_SC_FAV);					// av10 current value(DC input) frame average
			float input_power = av10_SC_FAV * input_voltage;
			printf(" %.1f,\r\n",input_power);				// AUV receipt power
			//input_voltage = 0;
			//printf(" %s\r\n",ws_static.water_temp);
		}


		if(tim_counter%50 == 25){
			int AC_average;      						   // Calculate AC_avereage in following process.
			int sum_ac = 0;
			int sump_ac = 0;
			int na10=0;

			while(na10 <512){
				sum_ac = sum_ac + ADC10[na10];
				na10++;
			}
			AC_average = sum_ac/512;
			na10=0;
			while(na10 <512){
				sump_ac = sump_ac + pow(( ADC10[na10]- AC_average),2);
				na10++;
			}
			rms_ac = (float)sqrt(sump_ac/512);
			 //   rms_ac = 0.0395*rms_ac - 0.2028;
			rms_ac = 0.0184*rms_ac - 0.0871;  // for new charging board value

			if(ac_flag ==1){
				printf("9999, %.3f, ", rms_ac);
				for(int na10 =0;na10<512;na10++){
					printf(" %d,",ADC10[na10]);
				}
				ac_flag =0;
			}
		}

		//----for ADXL345 operation----------------------------------

		// fifo operation test----------------------------
		if( tim_counter%200 == 150 && adxl_flag ==0){
			//uint8_t XX = adxl_register_fifo_read(0x32);
			adxl_register_multi_read( 0x34, 6);
			//adxl_register_multi_read( 0x35, 6);
		}
        //-----------------------------------------------

		if( tim_counter%200 == 150 && adxl_flag == 2){

			ADXL345_acc[0] = 0; ADXL345_acc[1] = 0; ADXL345_acc[2] = 0;

			//adxl_register_read(0x00);
			//printf(" register 0x00 value is %x\r\n",adxl_register_read(0x00));
			//printf(" register 0x2C value is %x\r\n",adxl_register_read(0x2C));

			uint8_t X0 = adxl_register_read(0x32);
			uint8_t X1 = adxl_register_read(0x33);
			uint8_t Y0 = adxl_register_read(0x34);
			uint8_t Y1 = adxl_register_read(0x35);
			uint8_t Z0 = adxl_register_read(0x36);
			uint8_t Z1 = adxl_register_read(0x37);

			ADXL345_acc[0] = (int16_t)X1<<8 | (int16_t)X0;
			ADXL345_acc[1] = (int16_t)Y1<<8 | (int16_t)Y0;
			ADXL345_acc[2] = (int16_t)Z1<<8 | (int16_t)Z0;

			printf(" register X1-X0 value is %x\r\n",ADXL345_acc[0]);
			printf(" register Y1-Y0 value is %x\r\n",ADXL345_acc[1]);
			printf(" register Z1-Z0 value is %x\r\n",ADXL345_acc[2]);

			acc_value[0] = (float)ADXL345_acc[0] * 0.078;
			acc_value[1] = (float)ADXL345_acc[1] * 0.078;
			acc_value[2] = (float)ADXL345_acc[2] * 0.078;
			printf(" X acc is %f\r\n",acc_value[0]);
			printf(" Y acc is %f\r\n",acc_value[1]);
			printf(" Z acc is %f\r\n",acc_value[2]);

		  }
	//------------------------end of LM75B operation--------------------

	tim_counter++;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);

    if(DEBG){
  	  HAL_UART_Receive_DMA(&huart2,serialData,DATANUM);
    }else{
  	  HAL_UART_Receive_DMA(&huart1,serialData,DATANUM);
    }
    HAL_UART_Receive_DMA(&huart3,water_serialData,WTDATANUM);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);



  // LM75B Config data
    int8_t i2cTxLM75Buffer[2];
    i2cTxLM75Buffer[0] = 0x1;
    i2cTxLM75Buffer[1] = 0x0;

  // LM75B configration register initial set
    HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)(0x48)<<1, (uint8_t*)i2cTxLM75Buffer,2,100) ;
    i2cTxLM75Buffer[0]=0;
    HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)(0x48)<<1, (uint8_t*)i2cTxLM75Buffer,1,100) ;

   // ADXL345 initial set
   //if(adxl_flag == 0){
		adxl_register_write(0x2d,0x00);			// Initial value set
		adxl_register_write(0x2d,0x08);			// Start measurement
		adxl_register_write(0x31,0x01);			// g range set (2g)
		adxl_register_write(0x1E,0x0);			// X-Offset register set (value = 0)
		adxl_register_write(0x1F,0x0);			// Y-Offset register set (value = 0)
		adxl_register_write(0x20,0x2);			// Z-Offset register set (value = 0)
		adxl_register_write(0x38,0x9F);			// Set stream FiFo mode and FiFo depth(32)
		adxl_register_write(0x2c,0x0A);			// Output data rate
   //}else if(adxl_flag ==1){
  	 // do nothing
   //}

   HAL_UART_Receive_IT(&huart5, (uint8_t *)u5RxBuffer,10);
   //HAL_UART_Transmit(&huart5,u5TxBuffer, 5, 0xFFFF);
   //send_20bit_command(3, 0x1E, 0x08, 0x00, 0x01);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  readSerial();
	  water_readSerial();

	  if(t_flag == 1){
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	  	 //HAL_UART_Transmit(&huart5,u5TxBuffer, 5, 0xFFFF);
		 send_20bit_command(3, 0x1E, 0x08, 0x00, 0x01);			// MON_VIN Command
	  	 t_flag = 0;
	  }else if(f_flag == 1 ){
		 send_20bit_command(3, 0x1E, 0x08, 0x0C, 0x00);			// MON_FAN_SPEED
		 //send_20bit_command(3, 0x1E, 0x08, 0x00, 0x1f);		// MON_VIN_FREQUENCY
		 //send_20bit_command(3, 0x1E, 0x08, 0x00, 0x01);		// MON_VIN Command
		 f_flag = 0;
	  }else if(t_flag ==0 || f_flag ==0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	  }else{

		  // do nothing?
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 959;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 2400;
  huart5.Init.WordLength = UART_WORDLENGTH_9B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_EVEN;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE{
	if(DEBG){
		HAL_UART_Transmit(&huart2,(uint8_t *)&ch, 1, 0xFFFF);
	}else{
		HAL_UART_Transmit(&huart1,(uint8_t *)&ch, 1, 0xFFFF);
	}
		return ch;
}

void clear_buffer(char *buf, uint8_t i){
	int8_t j = 0;
	char *cptr;
	cptr = buf;
	while(j < i){
		*cptr = 0;
		cptr++;
		j++;
	}
}

void readSerial(){
	int index;
	if(DEBG){
		index = DATANUM - huart2.hdmarx->Instance->CNDTR;
	}else{
		index = DATANUM - huart1.hdmarx->Instance->CNDTR;
	}
	indexRead = index;
}

void water_readSerial(){
	int water_index = WTDATANUM - huart3.hdmarx->Instance->CNDTR;
	water_indexRead = water_index;
}



void data_cut(){
	uint8_t *s_ptr;
	char *c_ptr;
	c_ptr = current_rx_data;

	if( (serialData[indexRead-2]=='\r'  && serialData[indexRead-1]=='\n')  ||
		(serialData[DATANUM-1]  =='\r'  && serialData[indexRead-1]=='\n')  ||
		(indexRead == 0  && serialData[DATANUM-2]  =='\r'  &&  serialData[DATANUM-1]=='\n' )
		){

		s_ptr = serialData + former_indexRead;
		if(indexRead  > former_indexRead){
			int count = indexRead - former_indexRead;

			while(count > 0){
				*c_ptr = *s_ptr;
				c_ptr++; s_ptr++; count--;
			}
			//printf("current_rx_data = %s\r\n", current_rx_data);
			command_interpret();

			clear_buffer(current_rx_data ,DATANUM);
			former_indexRead = indexRead ;

		}else if(indexRead < former_indexRead){
			int count1 = DATANUM - former_indexRead;

			while(count1 > 0 ){
				*c_ptr = *s_ptr;
				c_ptr++; s_ptr++; count1--;
			}
			s_ptr = serialData;

			int count2 = indexRead;
			while(count2 > 0){
				*c_ptr = *s_ptr;
				c_ptr++; s_ptr++; count2--;
			}
			//printf("current_rx_data = %s\r\n", current_rx_data);
			command_interpret();

			clear_buffer(current_rx_data ,DATANUM);
			former_indexRead = indexRead ;
		}else {
			// nothing to do when indexRead =former indexRead
		}

	}else{

		// nothing to do?
	}
}

void water_data_cut(){
	uint8_t *s_ptr;
	char *c_ptr;
	c_ptr = water_current_rx_data;
//	printf("test \r\n");

	if( (water_serialData[water_indexRead-2]=='\r'  && water_serialData[water_indexRead-1]=='\n')  ||
		(water_serialData[WTDATANUM-1]  =='\r'  && water_serialData[water_indexRead-1]=='\n')  ||
		(water_indexRead == 0  && water_serialData[WTDATANUM-2]  =='\r'  &&  water_serialData[WTDATANUM-1]=='\n' )
		){

		s_ptr = water_serialData + water_former_indexRead;
		if(water_indexRead  > water_former_indexRead){
			int wtcount = water_indexRead - water_former_indexRead;

			while(wtcount > 0){
				*c_ptr = *s_ptr;
				c_ptr++; s_ptr++; wtcount--;
			}
			printf("water_current_rx_data = %s", water_current_rx_data);
			water_command_interpret();

			clear_buffer(water_current_rx_data ,WTDATANUM);
			water_former_indexRead = water_indexRead ;

			}
		else if(water_indexRead < water_former_indexRead){
			int wtcount1 = WTDATANUM - water_former_indexRead;

			while(wtcount1 > 0 ){
				*c_ptr = *s_ptr;
				c_ptr++; s_ptr++; wtcount1--;
			}
			s_ptr = water_serialData;

			int wtcount2 = water_indexRead;
			while(wtcount2 > 0){
				*c_ptr = *s_ptr;
				c_ptr++; s_ptr++; wtcount2--;
			}
			printf("pwater_current_rx_data = %s", water_current_rx_data);
			water_command_interpret();

			clear_buffer(water_current_rx_data ,WTDATANUM);
			water_former_indexRead = water_indexRead ;
		}else {
			// nothing to do when indexRead =former indexRead
		}

	}else{
		// nothing to do?
	}
}

void command_interpret(){
	int kazu = atoi(current_rx_data);
	char s1[] = "ac\r\n";		// Enables AC command
	char s2[] = "s\r\n";		// Start water temperature sensor command
	char s3[] = "t\r\n";		// Stop water temperature sensor command
	char s4[] = "z\r\n";		// test_cv to stay dac_value
	char s5[] = "q\r\n";		// test_cv to normal mode(equal to initial value)
	char s6[] = "x\r\n";		// adxl_flag: For adxl345 enable/disable toggle command
	char s7[] = "v\r\n";		// t_flag: Input voltage monitor request command(for DC/DC extension command)
	char s8[] = "f\r\n";		// f_flag: Fan speed monitor flag(only test purpose)
	char s9[] = "w\r\n";		// visa_flag: visa command related command
	char s10[] = "r\r\n";		// GPIO_PIN_8 "set" command
	char s11[] = "rst\r\n";		// CPU reset
	char s12[] = "y\r\n";		// "y" volt_set command = 1
	char s13[] = "n\r\n";		// N/A

	//target_value = kazu;
	//printf("kazu = %d\r\n",kazu);
	if(kazu != 0){
		new_target_value = kazu;
		if(new_target_value > 3300){
			new_target_value = 0;
		}
		printf("new_target_value =  %d\r\n", new_target_value);

	}else if(kazu ==0){
		//target_vbalue = kazu;
		int cp_result1 = strcmp(current_rx_data, s1);
		int cp_result2 = strcmp(current_rx_data, s2);
		int cp_result3 = strcmp(current_rx_data, s3);
		int cp_result4 = strcmp(current_rx_data, s4);
		int cp_result5 = strcmp(current_rx_data, s5);
		int cp_result6 = strcmp(current_rx_data, s6);
		int cp_result7 = strcmp(current_rx_data, s7);
		int cp_result8 = strcmp(current_rx_data, s8);
		int cp_result9 = strcmp(current_rx_data, s9);
		int cp_result10 = strcmp(current_rx_data,s10);
		int cp_result11 = strcmp(current_rx_data,s11);
		int cp_result12 = strcmp(current_rx_data,s12);
		int cp_result13 = strcmp(current_rx_data,s13);

		//printf ("compare result1 = %d\r\n ", cp_result1);
		if(cp_result1 == 0){
			ac_flag =1;
		}else if(cp_result2 ==0 ){
			//printf("s receive r\n\");
			HAL_UART_Transmit(&huart3,(uint8_t *)s2, 1 , 0xFFFF);
		}else if(cp_result3 ==0 ){
			HAL_UART_Transmit(&huart3,(uint8_t *)s3, 1 , 0xFFFF);
		}else if(cp_result4 ==0){
			test_cv = 1;
		}else if(cp_result5 ==0){
			test_cv = 0;
		}else if(cp_result6 ==0){
			adxl_flag = (adxl_flag + 1)%2;
		}else if(cp_result7 ==0){
			t_flag = 1;
		}else if(cp_result8 ==0){
			f_flag = 1;
		}else if(cp_result9 ==0){
			visa_flag = 1;
			printf("set visa_flag =1 \r\n");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		}else if(cp_result10 == 0){
			//HAL_NVIC_SystemReset() ;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		}else if(cp_result11 == 0){
			printf("cpu RST \r\n");
			HAL_NVIC_SystemReset();

		}else if(cp_result12 == 0){
			printf("match command and response \r\n");
			volt_set = 1;
		}else{
			new_target_value = kazu; 	// for zero input
		}
	}
}

void water_command_interpret(){

	// Struct water_sensor ws;
	char *p;
	char ch_sum = 0;
	char *w_ptr;

	ws_static.data_type = 0;
	clear_buffer(ws_static.cnt,10);
	clear_buffer(ws_static.unix_time,20);
	clear_buffer(ws_static.water_temp,5);
	clear_buffer(ws_static.unix_time,2);

	w_ptr = water_current_rx_data;
	uint8_t c_count = 0;
	while(c_count < 4 ){
		if(*w_ptr !=','){
			ch_sum = ch_sum ^ *w_ptr;
			w_ptr++;
		}else{
			w_ptr++;
			c_count++;
		}
	}
	//printf("chsum calc = %x\r\n",ch_sum);

	p = strtok(water_current_rx_data,",");
	ws_static.data_type = p;
	//printf("data_type =  %s\r\n",ws_static.data_type);
	pws_static = &ws_static;
	//printf("data_type * = %s\r\n",pws_static->data_type);				//only for test puropse

	p= strtok(NULL,",");
	if(p!=NULL){
		strcpy(ws_static.cnt, p);
		//printf("cnt =  %s\r\n",ws_static.cnt);
		strcpy(pws_static->cnt, p);
		//printf("cnt * =  %s\r\n",pws_static->cnt);

	}
	p= strtok(NULL,",");
	if(p!=NULL){
		strcpy(ws_static.unix_time, p);
		//printf("unix_time =  %s\r\n",ws_static.unix_time);
	}
	p= strtok(NULL,",");
	if(p!=NULL){
		strcpy(ws_static.water_temp,p);
		//printf("water_temp =  %s\r\n",ws_static.water_temp);
	}
	p= strtok(NULL,",");
	if(p!=NULL){
		strcpy(ws_static.chsum , p);
		//printf("chsum =  %s\r\n",p);
	}

	long chsum_int = strtol(ws_static.chsum, NULL, 16);
	//printf("chsum_int = %lx\r\n", chsum_int);

	if(chsum_int== (int)ch_sum){
		chsum_flag = 0;
		water_temp_2 = atof(ws_static.water_temp);
		//printf("water_temp2 = %.2f\r\n",water_temp_2/100);

		printf("chsum OK \r\n");
	}else{
		chsum_flag = 2;
		printf("chsum NG \r\n");
	}
}



void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1)
{
	if(n >= 0 && n <= 511 ){
		ADC02[n]= aADCxConvertedData[0];
		ADC05[n]= aADCxConvertedData[1];
		ADC10[n]= aADCxConvertedData[2];
		ADC11[n]= aADCxConvertedData[3];
		ADC12[n]= aADCxConvertedData[4];
		ADC15[n]= aADCxConvertedData[5];
		n++;
	}else if( n >= 512){
		HAL_ADC_Stop_DMA(hadc1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  		n =0;
	}
}


float LM75B_temp_get(uint8_t i2c_ad){
	if(HAL_I2C_Master_Receive(&hi2c2,(uint16_t)(i2c_ad)<<1, (uint8_t*)i2cRxBuffer,2,100) != HAL_TIMEOUT){
				//i2cRxBuffer[0]= 0xC9;  Test for minus operation
				//i2cRxBuffer[1]= 0x20;  t = -54.875deg
		float LM75B_value;
		if((i2cRxBuffer[0]&0x80) == 0){
			LM75B_temp[1] = (int16_t)i2cRxBuffer[0]<<8 & 0xFF00;
			LM75B_temp[0] = (int16_t)i2cRxBuffer[1] & 0x00FF ;
			LM75B_temp[2] = ( LM75B_temp[1] | LM75B_temp[0] )>> 5 ;
			LM75B_temp[2] =  LM75B_temp[2] & 0x07FF;
			LM75B_value = (float)LM75B_temp[2]/8;
		}else{
			LM75B_temp[1] = (int16_t)i2cRxBuffer[0]<<8 & 0xFF00;
			LM75B_temp[0] = (int16_t)i2cRxBuffer[1] & 0x00FF ;
			LM75B_temp[2] = ( LM75B_temp[1] | LM75B_temp[0] );
			LM75B_temp[2] = LM75B_temp[2]/32;
			LM75B_value = (float)LM75B_temp[2]/8;
		}

		// printf("LM75_Temperature = %6.2f\r\n", LM75B_d);
			i2cRxBuffer[0]=0;
			i2cRxBuffer[1]=0;
			return LM75B_value;

		}else{
					MX_I2C2_Init();
		return 999;
	}
}


static uint8_t adxl_register_write( uint8_t reg, uint8_t reg_data){
	uint8_t reg_data_set[2];
	reg_data_set[0]= reg;
	reg_data_set[1]= reg_data;
	HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)(0x1D)<<1, (uint8_t*)reg_data_set,2,100) ;
	reg_data_set[0]= 0;
	reg_data_set[1]= 0;
}

static uint8_t adxl_register_read( uint8_t reg){
	HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)(0X1D)<<1, (uint8_t*)&reg,1,100);
	HAL_I2C_Master_Receive(&hi2c2,(uint16_t)(0x1D)<<1, (uint8_t*)i2cRxBuffer,1,100);
	printf( "return  %x\r\n",i2cRxBuffer[0]);
	return i2cRxBuffer[0];
}

static uint8_t adxl_register_multi_read( uint8_t reg, uint8_t read_data){
	HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)(0X1D)<<1, (uint8_t*)&reg,1,100);
	uint8_t read_counter = 0;
	uint8_t *buffer_p;
	buffer_p = i2cRxBuffer;
	while(read_counter < read_data){
		HAL_I2C_Master_Receive(&hi2c2,(uint16_t)(0x1D)<<1, (uint8_t*)buffer_p,1,10);
		printf( "return buffer[%d]  %x\r\n",read_counter,i2cRxBuffer[read_counter]);
		buffer_p++;
		read_counter++;
	}
	//printf( "return  %x\r\n",i2cRxBuffer[0]);
	//return i2cRxBuffer[0];
}


void uartclear_buffer(uint8_t *buf, uint8_t i){
	int8_t j = 0;
	uint8_t *cptr;
	cptr = buf;
	while(j < i){
		*cptr = 0;
		cptr++;
		j++;
	}
}

float respons_interpret(uint8_t t1 ,uint8_t t2,uint8_t t3,uint8_t t4){
	uint16_t first_byte  = (uint16_t)t1 & 0x0001;
	uint16_t second_byte = (uint16_t)t2 & 0x001F;
	uint16_t third_byte  = (uint16_t)t3 & 0x001F;
	uint16_t fourth_byte = (uint16_t)t4 & 0x001F;
	first_byte  = first_byte<<15;
	second_byte = second_byte<<10;
	third_byte  = third_byte<<5;
	//printf("first_byte = %#x\r\n", first_byte);
	//printf("second_byte = %#x\r\n", second_byte);
	//printf("third_byte = %#x\r\n", third_byte);
	uint16_t res = (first_byte | second_byte | third_byte | fourth_byte);
	float f_res;
	f_res = (float)res;
	//f_res = f_res/100;
	//printf("res = %#x\r\n", res);
	//printf("res = %d\r\n", res);
	//printf("res = %5.1f\r\n", f_res);
    return f_res;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart == &huart5){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		//printf("hello \r\n");
		uint8_t *bptr;
		uint8_t *ptr;
		uint8_t j = 0;
		bptr = u5Buffer;
		ptr = u5RxBuffer;

		while(j<10){
			*bptr = *ptr;
			//printf("u5Buffer[ %d ] =  %d \r\n",j,u5Buffer[j]);
			ptr++;
			bptr++;
			j++;
		}

		float res = respons_interpret(u5Buffer[6], u5Buffer[7], u5Buffer[8], u5Buffer[9]);
		//printf("input voltage(rms) =  %5.1f\r\n",res);



		uint8_t hantei;
		hantei = u5Buffer[4] & 0x1F;
		if(hantei == 0x01){
			input_voltage = res/100;
		}else if(hantei == 0x00){
			printf("fan speed %f\r\n",res);
		}


		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		j=0;
		uartclear_buffer(u5Buffer,10);
		HAL_UART_Receive_IT(&huart5, (uint8_t *)u5RxBuffer,10);

	}

}

void send_20bit_command(uint8_t dev_ad, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4){

	uint8_t command_20bit[5];
	command_20bit[0]  = c1|(dev_ad<<5);
	command_20bit[2]  = c2|(dev_ad<<5);
	command_20bit[3]  = c3|(dev_ad<<5);
	command_20bit[4]  = c4|(dev_ad<<5);

	//uint8_t ch_sum = c1 ^ c2 ^ c3 ^ c4;
	uint8_t ch_sum = c1 + c2 + c3 + c4;
	ch_sum = ch_sum & 0x0f;
	command_20bit[1] = (dev_ad<<5)|(ch_sum<<1);
	command_20bit[1] = command_20bit[1] & 0xFE;

	HAL_UART_Transmit(&huart5,command_20bit, 5, 0xFFFF);

	/*
	for(int i = 0;i<5;i++){
		printf("command_20bit[%d] = %d \r\n",i,command_20bit[i]);
	}
	 */
}

void target_value_update(){

// target_value Zone 0 ------------
	if(target_value == 0 && new_target_value  <= C[3]){
		// もしかしたらやや危険かもしれない
		target_value = new_target_value;
	}else if (target_value == 0 && new_target_value > C[3]){
		printf("can't set target_value for out of range \r\n");
		new_target_value = target_value;

	}else{
		// do nothing
	}

// target_value Zone 1 ------------
	if(C[1] >= target_value  && target_value >0  && new_target_value == 0){
		if(i_counter == 0 ){
			printf("9816,volt X0,\r\n");
			i_counter ++;
		}else if(volt_set  == 1 ){
			printf("target_value = 0 \r\n");
			target_value = new_target_value;
			dac_value = 0;
			i_counter = 0;
			pending_flag =0;
		}else if(volt_set !=1 && i_counter == 100){
			printf("can't set VOLT X0\r\n");
			printf("force set target_value = 0 \r\n");
			target_value =0;
			dac_value = 0;
			i_counter =0;	// may not have any condition
		}else{
			i_counter ++;
		}

	}else if(C[1] >= target_value  && target_value >0 && C[1]>=new_target_value && new_target_value > 0   ){
		target_value = new_target_value;

	}else if(C[1] >= target_value  && target_value >0 && C[2]>=new_target_value && new_target_value >C[1] ){
		if(i_counter == 0){
			printf("9816,VOLT X2,\r\n");
			i_counter++;
		}else if(i_counter == 200 && volt_set ==1){
			target_value = new_target_value;
			i_counter = 0;
		}else if(i_counter == 200 && volt_set !=1){
			printf("can't set VOLT X2 \r\n");
			//target_value = new_target_value;
			i_counter = 0;
		}else{
			i_counter++;
		}
	}else if(C[1] >= target_value  && target_value >0 && C[3]>=new_target_value && new_target_value >C[2] ){
		new_target_value = target_value;
		printf("can't set target_value \r\n" );
		i_counter = 0;

	}else if (C[1] >= target_value  && target_value >0 && new_target_value >C[3] ){
		new_target_value = target_value;
		printf("can't set target_value \r\n" );
		i_counter = 0;

	}else{
		// may not have any condition
	}


// target_value Zone 2 ------------
	if(C[2] >= target_value  && target_value > C[1]  && new_target_value == 0){
		if(i_counter == 0 ){
			printf("9816,volt X0,\r\n");
			i_counter ++;
		}else if(volt_set  == 1 ){
			printf("target_value = 0 \r\n");
			target_value = new_target_value;
			dac_value = 0;
			i_counter = 0;
			volt_set = 0;
		}else if(volt_set !=1 && i_counter == 100){
			printf("can't set VOLT 60 \r\n");
			printf("force set target_value = 0 \r\n");
			target_value =0;
			dac_value = 0;
			i_counter =0;
			volt_set = 0; // may not have any condition
		}else{
			i_counter ++;
		}
	}else if(C[2] >= target_value  && target_value > C[1] && C[1]>=new_target_value && new_target_value > 0 ){
		target_value = new_target_value;
		pending_flag = 1;

	}else if(C[2] >= target_value  && target_value > C[1] && C[2]>=new_target_value && new_target_value > C[1]   ){
		target_value = new_target_value;

	}else if(C[2] >= target_value  && target_value > C[1] && C[3]>=new_target_value && new_target_value >C[2] ){
		if(i_counter == 0){
			printf("9816,volt X3,\r\n");
			i_counter++;
		}else if(i_counter == 200 && volt_set ==1){
			target_value = new_target_value;
			i_counter = 0;
		}else if(i_counter == 200 && volt_set !=1){
			printf("can't set VOLT X3 \r\n");
			i_counter = 0;
		}else{
			i_counter++;
		}

	}else if (C[2] >= target_value  && target_value > C[1] && new_target_value >C[3] ){
		new_target_value = target_value;
		i_counter = 0;

	}else{
		// may not have any condition
	}

// target_value Zone 3 ------------

	if(C[3] >= target_value  && target_value > C[2]  && new_target_value == 0){
		if(i_counter == 0 ){
			printf("9816,volt X0,\r\n");
			i_counter ++;
		}else if(volt_set  == 1 ){
			printf("target_value = 0 \r\n");
			target_value = new_target_value;
			dac_value = 0;
			i_counter = 0;
			volt_set = 0;
		}else if(volt_set !=1 && i_counter == 100){
			printf("can't set VOLT X0 \r\n");
			printf("force set target_value = 0 \r\n");
			target_value =0;
			dac_value = 0;
			i_counter =0;
			volt_set = 0; // may not have any condition
		}else{
			i_counter ++;
		}
	}else if(C[3] >= target_value  && target_value > C[2] && C[1]>=new_target_value && new_target_value > 0 ){
		new_target_value = target_value;
		i_counter = 0;

	}else if(C[3] >= target_value  && target_value > C[2] && C[2]>=new_target_value && new_target_value > C[1]   ){
		target_value = new_target_value;
		pending_flag = 2;

	}else if(C[3] >= target_value  && target_value > C[2] && C[3]>=new_target_value && new_target_value >C[2] ){
		target_value = new_target_value;

	}else if (C[3] >= target_value  && target_value > C[2] && new_target_value >C[3] ){
		new_target_value = target_value;
		printf("can't set target_value \r\n" );
		i_counter = 0;

	}else{
		// may not have any condition
	}
}

void pending_flag_operation(){

	if(pending_flag ==1){
		p_counter++;
		if(p_counter == 200){
			printf("9816,VOLT X1,\r\n");
			pending_flag = 0;
			p_counter = 0;
		}
	}else if(pending_flag ==2){
		p_counter++;
		if(p_counter == 200){
			printf("9816,VOLT X2,\r\n");
			pending_flag = 0;
			p_counter = 0;
		}
	}else{
		p_counter =0;
		pending_flag = 0;
	}
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
