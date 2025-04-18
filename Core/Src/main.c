/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int SPEED;//Ҫ�ﵽ���ٶ�
float PitchTemp = 0;
float YawTemp = 0;
float YawAngleCalibration = -42;
float PitchAngleCalibration = 3 ;
float PitchAverage = 0;
float YawAverage = 0;
int YawCorrectionSpeed = 0;
float PitchError = 0;
float SUM = 0;
int	flag=0;
typedef union {
    uint16_t   U16Data;
    int16_t    I16Data;
    int16_t    T16Data;
    uint8_t    C8Data[2];
}DataJointUnion;
typedef union {
    int16_t	   P16Data;
    uint8_t    P8Data[2];
}DataPitchUnion;

typedef union {
    int16_t	   Y16Data;
    uint8_t    Y8Data[2];
}DataYawUnion;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	UART2_RECV_BUFF_LEN			11
#define UART1_RECV_BUFF_LEN         11
#define UART1_TRANS_BUFF_LEN        64
#define UART2_SEND_BUFF_LEN         64
#define	Pitch_Buff_LEN				64
#define	Yaw_Buff_LEN				64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osSemaphoreId myBinarySem_UART1GY953RecvHandle;
osSemaphoreId myBinarySem_UART2SendHandle;
osSemaphoreId myBinarySem_UART1ToUART2TransReadyHandle;
osSemaphoreId myBinarySem_UART2RecvHandle;
osSemaphoreId myBinarySem_MotorSpdChangeHandle;
osSemaphoreId myBinarySem_YawCorrectionHandle;
osSemaphoreId myBinarySem_PitchCorrectionHandle;
osSemaphoreId myBinarySem_YawCorrectHandle;
/* USER CODE BEGIN PV */
uint8_t     UART1RecvBuff[UART1_RECV_BUFF_LEN];
uint8_t     UART1TransToUART2Buff[UART1_TRANS_BUFF_LEN];
int16_t     PitchBuff[Pitch_Buff_LEN];
int16_t     YawBuff[Pitch_Buff_LEN];
uint16_t    UART1TransToUART2Size=0;
uint16_t    PitchSize=0;
uint16_t    YawSize=0;
uint16_t    UART2SendSize=0;
uint8_t     UART2SendBuff[UART2_SEND_BUFF_LEN];
uint8_t 	UART2RecvBuff[UART2_RECV_BUFF_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskUART1GY953(void const * argument);
void StartTaskUART2(void const * argument);
void StartTaskMotorCtrl(void const * argument);
void StartTaskPID(void const * argument);
void StartTaskYawCorrect(void const * argument);

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem_UART1GY953Recv */
  osSemaphoreDef(myBinarySem_UART1GY953Recv);
  myBinarySem_UART1GY953RecvHandle = osSemaphoreCreate(osSemaphore(myBinarySem_UART1GY953Recv), 1);

  /* definition and creation of myBinarySem_UART2Send */
  osSemaphoreDef(myBinarySem_UART2Send);
  myBinarySem_UART2SendHandle = osSemaphoreCreate(osSemaphore(myBinarySem_UART2Send), 1);

  /* definition and creation of myBinarySem_UART1ToUART2TransReady */
  osSemaphoreDef(myBinarySem_UART1ToUART2TransReady);
  myBinarySem_UART1ToUART2TransReadyHandle = osSemaphoreCreate(osSemaphore(myBinarySem_UART1ToUART2TransReady), 1);

  /* definition and creation of myBinarySem_UART2Recv */
  osSemaphoreDef(myBinarySem_UART2Recv);
  myBinarySem_UART2RecvHandle = osSemaphoreCreate(osSemaphore(myBinarySem_UART2Recv), 1);

  /* definition and creation of myBinarySem_MotorSpdChange */
  osSemaphoreDef(myBinarySem_MotorSpdChange);
  myBinarySem_MotorSpdChangeHandle = osSemaphoreCreate(osSemaphore(myBinarySem_MotorSpdChange), 1);

  /* definition and creation of myBinarySem_YawCorrection */
  osSemaphoreDef(myBinarySem_YawCorrection);
  myBinarySem_YawCorrectionHandle = osSemaphoreCreate(osSemaphore(myBinarySem_YawCorrection), 1);

  /* definition and creation of myBinarySem_PitchCorrection */
  osSemaphoreDef(myBinarySem_PitchCorrection);
  myBinarySem_PitchCorrectionHandle = osSemaphoreCreate(osSemaphore(myBinarySem_PitchCorrection), 1);

  /* definition and creation of myBinarySem_YawCorrect */
  osSemaphoreDef(myBinarySem_YawCorrect);
  myBinarySem_YawCorrectHandle = osSemaphoreCreate(osSemaphore(myBinarySem_YawCorrect), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTaskUART1GY953, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTaskUART2, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTaskMotorCtrl, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, StartTaskPID, osPriorityIdle, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, StartTaskYawCorrect, osPriorityIdle, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//�����������ж�����ʼ
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        osSemaphoreRelease(myBinarySem_UART1GY953RecvHandle);
        HAL_UART_Receive_IT(&huart1, UART1RecvBuff, UART1_RECV_BUFF_LEN);//�жϽ��ܴ��������ݲ��ͷŽ����ű�
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        osSemaphoreRelease(myBinarySem_UART2SendHandle);//�ͷ�UART2�ű�
    }
}

// Motor Control ������ƺ���

void MotorCtrl(uint8_t chn, int16_t spd)
{
	if(spd>200){spd = 200;}

	if(spd<-200){spd = -200;}
	
	switch(chn)
	{
		case 0 :
			spd = spd + 1500;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, spd);	//scope 500~2500
			break;
		case 1 :
			spd = -1 * spd + 1500;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, spd);	// scope 500~2500
			break;
		default:break;
	}
}

// Motor Calibration ���������������
void MotorCalib(uint8_t chn, int16_t spd)
{
  int16_t   dead_down_left, dead_down_right,dead_up_left,dead_up_right;
	dead_down_left=-150+3*27;
	dead_up_left=-150+3*62;
	dead_down_right = -150+3*32;
	dead_up_right = -150+3*67;
	switch(chn)
	{
	case 0:
		if(spd < 0)
		{
			spd = spd + dead_down_left;
		}
		else if(spd > 0)
		{
			spd = spd + dead_up_left;
		}
		break;
	case 1:
		if(spd < 0)
		{
			spd = spd + dead_down_right;
		}
		else if (spd > 0)
		{
			spd = spd + dead_up_right;
		}
		break;
	default:
		break;
	}

	MotorCtrl(chn, spd);
}


/* spd > 0  Motor0 Turn Clockwise ���0 ˳ʱ��ת�� С������*/
/* spd > 0  Motor1 Turn Clockwise ���1 ��ʱ��ת�� С������*/
// ���㸩�����뺽���ƽ��ֵ�Ƕ�
void CalculateAverage(void)//�Ժ���Ҫ�Ľ����˲��㷨
{
	
	int number = 5;
	float SumOfPitch = 0;
	float SumOfYaw = 0;
	for(int i = 0; i < number; i++)
	{
		SumOfPitch +=  PitchTemp;
		SumOfYaw +=  YawTemp;
		osDelay(20);//���Delay�����ã�50Hz����Ƶ��
	}
	PitchAverage = SumOfPitch / number;  // ȫ�ֱ���
	YawAverage = SumOfYaw / number;      // ȫ�ֱ���
}
struct _YawPIDParam
{
	float ActualYaw;        // ����ʵ��ֵ
	float err;                // ����ƫ��ֵ
	float err_last;           // ������һ��ƫ��ֵ
	float Kp;                 //�������ϵ��
	float Ki;                 //�������ϵ��
	float Kd;                 //����΢��ϵ��
	float integral;           //�������ֵ
}YawPIDParam;

struct _PitchPIDParam
{
	float ActualPitch;		  // ����ʵ��ֵ
	float err;                // ����ƫ��ֵ
	float err_last;           // ������һ��ƫ��ֵ
	float Kp;                 //�������ϵ��
	float Ki;                 //�������ϵ��
	float Kd;                 //����΢��ϵ��
	float integral;           //�������ֵ
}PitchPIDParam;
void YawPIDInit()
{
	YawPIDParam.ActualYaw = 0;
	YawPIDParam.err = 0;
	YawPIDParam.err_last = 0;
	YawPIDParam.Kp = 1;//
	YawPIDParam.Ki = 0.2;
	YawPIDParam.Kd = 1.4;
	YawPIDParam.integral = 0;

}
int YawPIDControl(float YawAverage)
{
	YawPIDInit();
	YawPIDParam.ActualYaw = YawAverage;
	YawPIDParam.err = YawPIDParam.ActualYaw - YawAngleCalibration;
	YawPIDParam.integral += YawPIDParam.err;
	YawPIDParam.ActualYaw = YawPIDParam.Kp* YawPIDParam.err + YawPIDParam.Ki * YawPIDParam.integral + YawPIDParam.Kd * (YawPIDParam.err - YawPIDParam.err_last);
	//UART2SendIntegerVariable(YawPIDParam.Kp);
	YawPIDParam.err_last = YawPIDParam.err;
	YawCorrectionSpeed = (int)(10* YawPIDParam.ActualYaw);//100*PID����ֵ���õ�YawCorrectionSpeed
	//UART2SendIntegerVariable(YawCorrectionSpeed);
	osSemaphoreRelease(myBinarySem_YawCorrectionHandle);//�ͷ�Yaw�ű�
	return YawCorrectionSpeed;
}
int PitchPIDControl(float PitchAverage)
{
		PitchPIDParam.ActualPitch = PitchAverage; // �����Ǿ�ֵ���ݸ�ʵ�ʸ�����
		PitchPIDParam.err = PitchPIDParam.ActualPitch - PitchAngleCalibration;  // ƫ��
		PitchPIDParam.integral += PitchPIDParam.err;
		if (PitchPIDParam.integral > 100 || PitchPIDParam.integral < -100)
		{
			PitchPIDParam.integral = 100;
		}

		if (PitchPIDParam.err >= 2.0 || -2.0 >= PitchPIDParam.err)
		{
			PitchPIDParam.Kp = 1;                                // �������ϵ��
			PitchPIDParam.Ki = 0.0015;                                // �������ϵ��
			PitchPIDParam.Kd = 4.0;
		}
		else if (PitchPIDParam.err < 2.0 && -2.0 < PitchPIDParam.err)
		{
			PitchPIDParam.Kp = 0.8;                                // �������ϵ��
			PitchPIDParam.Ki = 0.0015;                                // �������ϵ��
			PitchPIDParam.Kd = 3.2;
		}
		PitchPIDParam.ActualPitch = PitchPIDParam.Kp * PitchPIDParam.err + PitchPIDParam.Ki * PitchPIDParam.integral + PitchPIDParam.Kd * (PitchPIDParam.err - PitchPIDParam.err_last);
		PitchPIDParam.err_last = PitchPIDParam.err;
		SPEED = (int)(10 * PitchPIDParam.ActualPitch);//����ȫ�ֱ���speed
		PitchError = PitchPIDParam.err; // ����ȫ�ֱ���PitchError��SPEED
		
		return SPEED;
}
void update_all_data(void)
{
	CalculateAverage();//��������ȫ�ֱ���PitchAverage��YawAverage��ƽ���˲���
	YawPIDControl(YawAverage);//�ͷ�Yaw�ű겢�õ�ȫ�ֱ���YawCorrectionSpeed
	PitchPIDControl(PitchAverage);// ����ȫ�ֱ���PitchError��SPEED
}
void back_to_ground(void)
{
	update_all_data();
	SPEED = 200;
	osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
	osDelay(500);//�����������Ȩ0.5s,�����ΰ岻ƽ��
	while(PitchError>1.3 || PitchError<-1.3)
	{
			while (YawCorrectionSpeed < -2)//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++�ڡ�-��-2������2��+�������Χ��Yaw�����������·�
			{ 	
				YawCorrectionSpeed = -YawCorrectionSpeed;
				osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
				osDelay(10);//�����������Ȩ
//						MotorCalib(1, -YawCorrectionSpeed);
//						MotorCalib(0, YawCorrectionSpeed);
				if (YawCorrectionSpeed < 5 && YawCorrectionSpeed > -5)//Yaw�������-5��5������Pitch������Yaw�����ܶ�SPEED=0
				{
					osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
					osDelay(10);//����MotorCtrl����
					break;
				}
				update_all_data();//�������е�ȫ�ֱ���
			}

		while (YawCorrectionSpeed > 2)//�ڡ�-��-2������2��+�������Χ��Yaw�����������·�
				{ 
					osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
					osDelay(10);//����MotorCtrl����
	//						MotorCalib(1, -YawCorrectionSpeed);
	//						MotorCalib(0, YawCorrectionSpeed);
					if (YawCorrectionSpeed < 5 && YawCorrectionSpeed > -5)//Yaw�������-5��5������Pitch������Yaw�����ܶ�SPEED=0
					{
						osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
						osDelay(10);//����MotorCtrl����
						break;
					}
					update_all_data();//�������е�ȫ�ֱ���
				}
	}
	while(1)
	{
		SPEED = 0;
		osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
		osDelay(1);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskUART1GY953 */
/* USER CODE END Header_StartTaskUART1GY953 */
void StartTaskUART1GY953(void const * argument)
{
  /* USER CODE BEGIN StartTaskUART1GY953 */
	  uint8_t cmd0[]={0xA5,0x15,0xBA};// acceleration data
    uint8_t cmd1[]={0xA5,0x35,0xDA};// magnet data
    uint8_t cmd2[]={0xA5,0x45,0xEA};// Euler data(50Hz)

    DataJointUnion  tempUnion;
    DataPitchUnion	PitchUnion;
    DataYawUnion	YawUnion;
    uint8_t isDirectTransToUART2=0;
    
    //osDelay(1000);//���Ǹ���ģ�
    HAL_UART_Transmit(&huart1, cmd1, sizeof(cmd1), 100);//��Ƭ����Ҫmagnet data
    HAL_UART_Transmit(&huart1, cmd2, sizeof(cmd2), 100);//��Ƭ����ҪEuler data
    
    HAL_UART_Receive_IT(&huart1, UART1RecvBuff, UART1_RECV_BUFF_LEN);//��Ƭ���õ�data
  /* Infinite loop */
    for(;;)
	    { 
	        osSemaphoreWait(myBinarySem_UART1GY953RecvHandle, osWaitForever);//һֱ�ȴ��ж�����UART1�ű꣬20ms��һ��
	        UART1TransToUART2Size   = 0;
	        PitchSize=0;
	        if(isDirectTransToUART2) {
	            memcpy(UART1TransToUART2Buff, UART1RecvBuff, UART1_RECV_BUFF_LEN);
	            UART1TransToUART2Size   = UART1_RECV_BUFF_LEN;
	        } else {
	            switch(UART1RecvBuff[2]) {
	                case    0x15    :
	                    tempUnion.C8Data[1]     = UART1RecvBuff[4];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[5];
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "Ax:%05d ", tempUnion.U16Data);
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 9;
	                    tempUnion.C8Data[1]     = UART1RecvBuff[6];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[7];
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "Ay:%05d ", tempUnion.U16Data);
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 9;
	                    tempUnion.C8Data[1]     = UART1RecvBuff[8];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[9];
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "Az:%05d \r\n", tempUnion.U16Data);
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 9 + 2;

	                    break;
	                case    0x35    :
	
	                    tempUnion.C8Data[1]     = UART1RecvBuff[4];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[5];
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "Mx:%06d ", tempUnion.I16Data);
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 10;
	                    tempUnion.C8Data[1]     = UART1RecvBuff[6];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[7];
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "My:%06d ", tempUnion.I16Data);
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 10;
	                    tempUnion.C8Data[1]     = UART1RecvBuff[8];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[9];
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "Mz:%06d \r\n", tempUnion.I16Data);
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 10 + 2;
	
	                    break;
	                case    0x45    :
	                    tempUnion.C8Data[1]     = UART1RecvBuff[4];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[5];
	                    tempUnion.T16Data = tempUnion.T16Data/0x64;
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "R:%06d ", tempUnion.T16Data);
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 9;
	                    tempUnion.C8Data[1]     = UART1RecvBuff[6];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[7];
	                    tempUnion.T16Data = tempUnion.T16Data/0x64;
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "P:%06d ", tempUnion.T16Data);
	
	                    /*�����ǲɼ�*/
	                    PitchUnion.P8Data[1]     = UART1RecvBuff[6];
	                    PitchUnion.P8Data[0]     = UART1RecvBuff[7];
	                    PitchUnion.P16Data = PitchUnion.P16Data/0x64;
	                    PitchTemp=(float)PitchUnion.P16Data;
	                    //PitchSize  = PitchSize + 9;
	
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 9;
	                    tempUnion.C8Data[1]     = UART1RecvBuff[8];
	                    tempUnion.C8Data[0]     = UART1RecvBuff[9];
	                    tempUnion.T16Data = tempUnion.T16Data/0x64;
	                    sprintf((UART1TransToUART2Buff + UART1TransToUART2Size), "Y:%06d \r\n", tempUnion.T16Data);
	                    UART1TransToUART2Size   = UART1TransToUART2Size + 9 + 2;
	
	                    /*�����ȡֵ*/
	                    YawUnion.Y8Data[1]     = UART1RecvBuff[8];
	                    YawUnion.Y8Data[0]     = UART1RecvBuff[9];
	                    YawUnion.Y16Data = YawUnion.Y16Data/0x64;
	                    YawTemp = (float)YawUnion.Y16Data;//�õ�һ�ε�ֵ
	                    //YawSize  = YawSize + 9;
	
	                    break;
	            }
	        }
	        osSemaphoreRelease(myBinarySem_UART1ToUART2TransReadyHandle);//�ͷ�UART1ToUART2�ű�
	        osDelay(1);//���첻�ϵõ���ֵ
	    }
  /* USER CODE END StartTaskUART1GY953 */
}

/* USER CODE BEGIN Header_StartTaskUART2 */
/* USER CODE END Header_StartTaskUART2 */
void StartTaskUART2(void const * argument)
{
  /* USER CODE BEGIN StartTaskUART2 */
	  HAL_UART_Transmit_IT(&huart2, "helloReady!", 11);

  /* Infinite loop */
    for(;;)
    {
        osSemaphoreWait(myBinarySem_UART1ToUART2TransReadyHandle, osWaitForever);//һֱ��UART1ToUART2�ű�
        osSemaphoreWait(myBinarySem_UART2SendHandle, osWaitForever);//ΪʲôҪ��UART2�ű�100ms������ű����жϲ���
        memcpy(UART2SendBuff, UART1TransToUART2Buff, UART1TransToUART2Size);
        UART2SendSize   = UART1TransToUART2Size;
        HAL_UART_Transmit_IT(&huart2, UART2SendBuff, UART2SendSize);
        osDelay(1000);//ÿ1000ms�����Դ�һ�����ݣ�����ٶ�����ν��ֻ�ǿ������
    }
  /* USER CODE END StartTaskUART2 */
}

/* USER CODE BEGIN Header_StartTaskMotorCtrl */
/* USER CODE END Header_StartTaskMotorCtrl */
void StartTaskMotorCtrl(void const * argument)
{
  /* USER CODE BEGIN StartTaskMotorCtrl */
	//����ִ��
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//��ʱ��1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);//��ʱ��2
  /* Infinite loop */
	
  for(;;)
  {
//    osSemaphoreWait (myBinarySem_YawCorrectionHandle, osWaitForever);//�ݲ���Ҫ�ȴ��ű�
		/*------------------------------ע��������Ĳ��֣�������osdelay��λ��---------------------*/
//		while (YawCorrectionSpeed < -3 && YawCorrectionSpeed > 3)
//		{
//			CalculateAverage();
//			YawPIDControl(YawAverage);
			osSemaphoreWait(myBinarySem_MotorSpdChangeHandle,osWaitForever);
			MotorCalib(0, -YawCorrectionSpeed);//SPEED��Դ��PitchPIDControl��ֻ����Pitch��
			MotorCalib(1, YawCorrectionSpeed);
			osDelay(1);//��ֻ�ǲ��϶�ѭ�� = 20ms*5���н������е��������
			MotorCalib(0, SPEED);
			MotorCalib(1, SPEED);
			osDelay(1);
//		}	
		/*------------------------------ע��������Ĳ���---------------------*/
//		osSemaphoreWait (myBinarySem_PitchCorrectionHandle, osWaitForever); 
  }
  /* USER CODE END StartTaskMotorCtrl */
}

/* USER CODE BEGIN Header_StartTaskPID */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPID */
void StartTaskPID(void const * argument)
{
  /* USER CODE BEGIN StartTaskPID */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskPID */
}

/* USER CODE BEGIN Header_StartTaskYawCorrect */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskYawCorrect */
void StartTaskYawCorrect(void const * argument)
{
  /* USER CODE BEGIN StartTaskYawCorrect */
  /* Infinite loop */
	int err;
  int array=0;
	for(;;)
    {	
		update_all_data();//�������е�ȫ�ֱ���
		err = YawCorrectionSpeed;
		while (YawCorrectionSpeed < -2)//�ڡ�-��-2������2��+�������Χ��Yaw�����������·�
					{ 	
						YawCorrectionSpeed = -YawCorrectionSpeed;
						osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
						osDelay(10);//�����������Ȩ
//						MotorCalib(1, -YawCorrectionSpeed);
//						MotorCalib(0, YawCorrectionSpeed);
						if (YawCorrectionSpeed < 5 && YawCorrectionSpeed > -5)//Yaw�������-5��5������Pitch������Yaw�����ܶ�SPEED=0
						{
							osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
							osDelay(10);//����MotorCtrl����
							break;
						}
						update_all_data();//�������е�ȫ�ֱ���
					}

		while (YawCorrectionSpeed > 2)//�ڡ�-��-2������2��+�������Χ��Yaw�����������·�
					{ 
						osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
						osDelay(10);//����MotorCtrl����
//						MotorCalib(1, -YawCorrectionSpeed);
//						MotorCalib(0, YawCorrectionSpeed);
						if (YawCorrectionSpeed < 5 && YawCorrectionSpeed > -5)//Yaw�������-5��5������Pitch������Yaw�����ܶ�SPEED=0
						{
							osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
							osDelay(10);//����MotorCtrl����
							break;
						}
						update_all_data();//�������е�ȫ�ֱ���
					}
		while (YawCorrectionSpeed >= -2 && YawCorrectionSpeed <= 2)//Yaw�ڡ�-2��2������ǰ��
			{
			while(PitchError<=3 && PitchError>=-3 && flag==0)//Pitch�ڡ�-��3���ڣ����ڵ����ϣ�flag����һ��ȫ��ִֻ��һ��
			{
				SPEED = 100;
				osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
				osDelay(10);//����MotorCtrl����
//				osDelay(8000);//8���ߵ���������=8000*100mm
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);//����
				update_all_data();//�������е�ȫ�ֱ���
				
			}
			if(PitchError>3 || PitchError<-3)
			{
				flag=1;
			}
				
			update_all_data();//�������е�ȫ�ֱ���
			if (YawCorrectionSpeed < 5 && YawCorrectionSpeed > -5)//Yaw�ڡ�-5��5���ڵ���Pitch������Yaw
			{
				if(PitchError<=1.5 && PitchError >=-1.5)
				{
					array++;
					SPEED = 0;
					osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
					osDelay(10);//����MotorCtrl����
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
					if(array>=150)
					{ 
						back_to_ground();
					}
					//UART2SendIntegerVariable(array);
	
				}
				else
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
					MotorCalib(0, SPEED);
					MotorCalib(1, SPEED);
					CalculateAverage();//��������ȫ�ֱ���PitchAverage��YawnAverage��ƽ���˲��������ͷ�MotorSpdChange�ű�
					break;
				}
			}
		update_all_data();//�������е�ȫ�ֱ���
  	osDelay(1);
    }

	}
  /* USER CODE END StartTaskYawCorrect */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
