/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint32_t Rising_adge = 0U; // нарастающий фронт 
uint32_t Pulse_length = 0U;
uint32_t Falling_edge = 0U; // падающий фронт
uint32_t pFLatency;
uint16_t Reg_FLAG = 0;// регистр флагов*/
uint8_t Data[64] = {0}; //массив для передачи по UART
uint8_t Ch_Ex_DS;
uint8_t Ch_Fal_DS;
uint8_t Buf_Read = 0;
uint8_t Bit_Rd = 0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
#define In_DS         0  //флаг инициализации DS18B20
#define Presence_DS   1  //флаг присутствия DS18B20
#define Slot_Read     2  //флаг создания слота чтения
#define Slot_Write    3  //флаг создания слота записи
#define Sl_Zero       4  //флаг создания нуля
#define Presence_Zero 5  //флаг наличия нуля
#define SL_One        6  //флаг создания единицы
#define Presence_One  7  //флаг наличия еденицы
#define Tim_ok        8  //флаг отработки захвата 
*/

//#define LCD_8_BITS    (uint8_t)9U

#ifndef LCD_8_BITS

 #define LCD_4_BITS      (uint8_t)5U

#endif

#ifdef LCD_8_BITS

 #define QTY_REP      LCD_8_BITS

#else

 #define QTY_REP      LCD_4_BITS

#endif
#define RS       GPIO_PIN_0
#define ENA      GPIO_PIN_2

#define DB4      GPIO_PIN_3
#define DB5      GPIO_PIN_4
#define DB6      GPIO_PIN_5
#define DB7      GPIO_PIN_6
#define DB_DS    GPIO_PIN_7
    
#define LCD_PORT  GPIOE

#define DB4_N      1
#define DB5_N      2
#define DB6_N      4
#define DB7_N      8

#define RS_N       4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t String[] = "HELLO!!!";
uint8_t String_1[] = "NO DS18B20";
uint8_t String_2[] = "TEMPERATURA";
uint8_t String_3[] = "EXIST DS18B20";
UART_HandleTypeDef  huart2;
char TmCh = 1;
uint8_t str[] = "Проверка передачи UART\r\n\0";


 
typedef struct 
{
  uint8_t Temperature_LSB ;
  uint8_t Temperature_MSB ;
  uint8_t ThRegisterorUserByte_1 ;
  uint8_t ThRegisterorUserByte_2;
  uint8_t ConfigurationRegister ;
  uint8_t Res_1 ;
  uint8_t Res_2 ;
  uint8_t Res_3 ;
  uint8_t C_R_C;
} Memory_DS;


Memory_DS  MEMORY_DS;     //создадим обьект структуры памяти DS
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void Data_load_LCD(const uint8_t Data);
void Comand_load_LCD(const uint8_t Data);
void Init_LCD(void);
void SendLCD(const uint8_t Data);
void DecomString(const uint8_t String[]);
uint8_t Init_DS (void);
void IRQHand_htim6( void);// функция обработчика прерывания от TIM6
void IRQHand_htim7( void);
void IRQHand_htim1_In_DS( TIM_HandleTypeDef *htim); // обработчик для инициализации DS
void Sl_Write(uint8_t bit);//слот записи в DS
uint8_t Sl_Read(void);        //слот чтения из DS
void Comand_DS (uint8_t Com); // команда в DS
void Data_Read_DS (uint8_t Number_of_bytes); // чтение данных от DS
void Print_number ( uint8_t Number );

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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 HAL_Delay(15);
  Init_LCD();
  
  Comand_load_LCD(0x01);// чистим табло
  HAL_Delay(2);
  Comand_load_LCD(0x02);
  HAL_Delay(2);
  DecomString(String);
   
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
  //Init_DS ();
  if(Init_DS () )
   {
      Comand_DS (0xCC);
      Comand_DS (0x44);
   }
 // Sl_Read ();
  while(Sl_Read ())
  {
  }
  /*int r = 0;
  while ( r < 15)
  {
    Sl_Read ();
    //snprintf(Data, 63, "Pulse_length %lu \n",Pulse_length);
   // HAL_UART_Transmit_IT(&huart2, (uint8_t*)Data, strlen(Data));
    r++;
  }*/
  
  if(Init_DS () )
  {
      Comand_DS (0xCC);
      Comand_DS (0xBE);
      Data_Read_DS (2);
      uint8_t Numb =  MEMORY_DS.Temperature_LSB;
      //Comand_load_LCD(0x01);// чистим табло
      //HAL_Delay(2);
      //Comand_load_LCD(0x02);
      //HAL_Delay(2);
      //Print_number ( Numb );
      
   }
  
    snprintf(Data, 63, "Содержание регистра %lu \n",MEMORY_DS.Temperature_LSB);
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)Data, strlen(Data));
 
  HAL_Delay(100);
    }
  }


uint8_t Init_DS (void)
{
  Reg_FLAG = 0;
  
   TIM6->PSC = 10;
   TIM6->ARR = 420;
  /*
  htim6.Init.Prescaler = 10;
  htim6.Init.Period = 430;
  HAL_TIM_Base_Init(&htim6);
  __HAL_TIM_CLEAR_IT(&htim6, TIM_FLAG_UPDATE);
  */
    TIM7->PSC = 10;
    TIM7->ARR = 550;
    /*
  htim7.Init.Prescaler = 10;
  htim7.Init.Period = 550;
  HAL_TIM_Base_Init(&htim7);
  __HAL_TIM_CLEAR_IT(&htim7, TIM_FLAG_UPDATE);*/
  
  Reg_FLAG |= (1<<In_DS);  //выставляем флаг инициализации DS
  HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_RESET);
  HAL_TIM_Base_Start_IT(&htim6); // вкл. контроль времени импулса сброса
 
  while ((!(Reg_FLAG & (1<<Absence_DS))&&(!(Reg_FLAG & (1<<Presence_DS))))||
            ((Reg_FLAG & (1<<Absence_DS))&&(Reg_FLAG & (1<<Presence_DS))))  // если нет флагов наличия или отсутствия датчика 
  {                                                                                                
    
  }
  if(Reg_FLAG & (1<<Absence_DS)) //если нет  присутствия DS  
  { 
    HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);// стоп с прерыванием по захвату
    HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_2);
    Reg_FLAG = 0;
    return 0;
  }
  
  if(Reg_FLAG & (1<<Presence_DS))  //если  есть присутствие DS
  { 
    HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);// стоп с прерыванием по захвату
    HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_2);
    Reg_FLAG = 0;
    return 1;
  }
}


void Comand_DS (uint8_t Com)
{
  for(int i=0;i < 8;i++)
  {
  uint8_t Bit = 0x01;
  Bit &=  Com;  //младший бит команды в буфер 
  Com >>= 1;  // сдвигаем оставшиеся биты в право
  Sl_Write(Bit);  // Передаем бит
 /* if((i == 7)&&(Bit == 1))  // если последним битом была 1
    {
      Reg_FLAG |= (1<<Delay_Fl);  // делаем задержку перед выходом
      htim6.Init.Prescaler = 10;// таймер на ~2 us для создания
      htim6.Init.Period = 6;    // импульса после последнего слота
      HAL_TIM_Base_Init(&htim6);
      __HAL_TIM_CLEAR_IT(&htim6, TIM_FLAG_UPDATE);
      HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_RESET);
      HAL_TIM_Base_Start_IT(&htim6);
      while (Reg_FLAG & (1<<Delay_Fl))
        {
        } 
    }*/
  }
 /* Reg_FLAG |= (1<<Delay_Fl);  // делаем задержку перед выходом
  htim6.Init.Prescaler = 10; //создадим задержку перед выходом
  htim6.Init.Period = 100;
  HAL_TIM_Base_Init(&htim6);
  __HAL_TIM_CLEAR_IT(&htim6, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim6); // запуск  таймера
    while (Reg_FLAG & (1<<Delay_Fl))
    {
    } */
    Reg_FLAG = 0;
}

void Data_Read_DS (uint8_t Number_of_bytes) // Передаем количество байт подлежащих приему 
{
  Buf_Read = 0;
  uint8_t Quantity_of_bit,Tmp_byt = 0;
  Quantity_of_bit = Number_of_bytes * 8; //определяем количество бит для приема
  
  for(int i=0; i < Quantity_of_bit; i++)
  {
     Tmp_byt = Sl_Read();  // Принятый бит во временный байт
     Tmp_byt <<= 7;
     Buf_Read |= Tmp_byt;  // затем в буфер приема
     Buf_Read >>= 1;       // буфер сдвигаем
     switch (i)
     {
       case 7:
         MEMORY_DS.Temperature_LSB = Buf_Read;
         break;
       case 15:
         MEMORY_DS.Temperature_MSB = Buf_Read;
         break;
       case 23: 
         MEMORY_DS.ThRegisterorUserByte_1 = Buf_Read;
         break;
       case 31:
         MEMORY_DS.ThRegisterorUserByte_2 = Buf_Read;
         break;
       case 63:
         MEMORY_DS.C_R_C = Buf_Read;
         break;
      }
  }
  
}

void Sl_Write(uint8_t bit)//слот записи
{
   Reg_FLAG = 0;
  if(!bit)
  {
    TIM6->PSC = 10;
    TIM6->ARR = 78;
    TIM6->CNT = 0;
    /*htim6.Init.Prescaler = 10;// таймер на ~60 us
    htim6.Init.Period = 55;
    HAL_TIM_Base_Init(&htim6);
    __HAL_TIM_CLEAR_IT(&htim6, TIM_FLAG_UPDATE);*/
    Reg_FLAG |= (1<<Slot_Write)|(1<<Sl_Zero);  //ставим флаги слота записи и создания 0
  }
  if(bit)
  {
    TIM6->PSC = 10;
    TIM6->ARR = 3;
    TIM6->CNT = 0;
    /*htim6.Init.Prescaler = 10;// таймер на ~20 us
    htim6.Init.Period = 7;
    HAL_TIM_Base_Init(&htim6);
    __HAL_TIM_CLEAR_IT(&htim6, TIM_FLAG_UPDATE);*/
    Reg_FLAG |= (1<<Slot_Write)|(1<<SL_One);  //ставим флаги слота записи и создания 1
  }
   HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_RESET);
   HAL_TIM_Base_Start_IT(&htim6);
    
   while (!(Reg_FLAG & (1<<END_Sl)))
   {
   
   }
   Reg_FLAG = 0;// снимаем флаги
}

uint8_t Sl_Read(void)//слот чтения
{
     Bit_Rd = 0;                  // чистим буфер приема
     Reg_FLAG = 0;                // снимаем флаги
     Reg_FLAG |= (1<<Slot_Read);  //ставим флаг слота чтения
     
     TIM6->PSC = 10;
     TIM6->ARR = 3;     // таймер на ~2 us
     TIM6->CNT = 0;
     
     TIM7->PSC = 10;
     TIM7->ARR = 30;// 52;
     TIM7->CNT = 0;

     /*htim6.Init.Prescaler = 10;   // таймер на ~2 us
     htim6.Init.Period = 3;
     HAL_TIM_Base_Init(&htim6);
     __HAL_TIM_CLEAR_IT(&htim6, TIM_FLAG_UPDATE);*/
     __HAL_TIM_SET_COUNTER(&htim1, 0x0001); // обнуление счётчика 
     HAL_TIM_IC_Start(&htim1,TIM_CHANNEL_1);// запуск  захвата сигнала без прерывания
     HAL_TIM_IC_Start(&htim1,TIM_CHANNEL_2);
     
     HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_RESET);
     HAL_TIM_Base_Start_IT(&htim6);
     
     while ((!(Reg_FLAG & (1<<END_Sl)))|(!(Reg_FLAG & (1<<Tim_ok)))) // ждем окончания слота 
        {                                                               // и отработки захвата  
   
        }
      Reg_FLAG = 0;// снимаем флаги
      return Bit_Rd;
}


void IRQHand_htim6( void)
{
   HAL_TIM_Base_Stop_IT(&htim6);
  // __HAL_TIM_CLEAR_IT(&htim6, TIM_FLAG_UPDATE);
    
  if(Reg_FLAG & (1<<In_DS)) // если инициируем DS
  {
    __HAL_TIM_SET_COUNTER(&htim1, 0x0000); // обнуление счётчика
    HAL_TIM_IC_Start/*_IT*/(&htim1,TIM_CHANNEL_1);// запуск с прерыванием по захвату
    HAL_TIM_IC_Start/*_IT*/(&htim1,TIM_CHANNEL_2);
    HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_SET); // отпускаем линию
    /*HAL_TIM_Base_Start_IT(&htim7);*/
  }
  //==================================================================
  if(Reg_FLAG & (1<<Slot_Write)) // если создаем слот записи
   {
    if(Reg_FLAG & (1<<Sl_Zero))  // если записываем ноль
    {
    HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_SET); // отпускаем линию
    Reg_FLAG |= (1<<END_Sl);  // ставим флаг завершения слота
    }
   //-------------------------------------------------------
    if(Reg_FLAG & (1<<SL_One))  // если передаем еденицу
    {
        HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_SET); // отпускаем линию
       TIM7->PSC = 10;
       TIM7->ARR = 63;
       TIM7->CNT = 0;
        /*htim7.Init.Prescaler = 10;// таймер 7 на ~xx us для
        htim7.Init.Period = 38;   // выравнивания длинны слота
        HAL_TIM_Base_Init(&htim7);
        __HAL_TIM_CLEAR_IT(&htim7, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&htim7);*/
      }
  }
  //============================================================
  if(Reg_FLAG & (1<<Slot_Read)) // если создаем слот чтения
  {
    //if(!(Reg_FLAG & (1<<Pause)))
   // {
    HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_SET);
 //   }
    /*TIM7->PSC = 10;
    TIM7->ARR = 300;
    TIM7->CNT = 0;*/
       /*htim7.Init.Prescaler = 10;// таймер 7 на ~55 us для
       htim7.Init.Period = 30;   // выравнивания длинны слота
       HAL_TIM_Base_Init(&htim7);
       __HAL_TIM_CLEAR_IT(&htim7, TIM_FLAG_UPDATE);
       HAL_TIM_Base_Start_IT(&htim7);*/
   }
  //=============================================================
  if(Reg_FLAG & (1<<Delay_Fl)) // если нужна задержка
  {
     HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_SET);
     Reg_FLAG ^= (1<<Delay_Fl);  //снимаем флаг задержки
  }
}
 

void IRQHand_htim7(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
    __HAL_TIM_CLEAR_IT(&htim7, TIM_FLAG_UPDATE);
    
  if(Reg_FLAG & (1<<In_DS)) // если инициируем DS
  {
      Falling_edge =  Rising_adge = 0;
      HAL_TIM_IC_Stop(&htim1,TIM_CHANNEL_1);
      HAL_TIM_IC_Stop(&htim1,TIM_CHANNEL_2);
      Falling_edge = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); // чтение значения в регистре захвата/сравнения
      Rising_adge = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2); // чтение значения в регистре захвата/сравнения
    
      if(Rising_adge > Falling_edge)
       {
         Pulse_length = Rising_adge - Falling_edge;
       }
      if((Pulse_length > 15)&&(Pulse_length < 600))
       {
         Reg_FLAG ^= (1<<In_DS); // снимаем флаг инициализации DS
         Reg_FLAG |= (1<<Presence_DS); //ставим флаг присутствия DS
       }
      if(Pulse_length > 680)
       {
         Reg_FLAG ^= (1<<In_DS); //снимаем флаг инициализации DS
         Reg_FLAG |= (1<<Absence_DS);// ставим флаг отсутствия DS
       }
      
    /*if(!((Reg_FLAG & (1<<Absence_DS))||(Reg_FLAG & (1<<Presence_DS)))) // если нет флагов наличия датчика
  {
     Reg_FLAG ^= (1<<In_DS); //  то снимаем флаг инициализации DS
     Reg_FLAG |= (1<<Absence_DS);// и выстовляем флаг отсутствия датчика
  }
// ждем появления флагов наличия или отсутствия датчика

   // HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);
   // HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_2);
    //HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_RESET);*/
  }
  //----------------------------------------------------------
  if((Reg_FLAG & (1<<Slot_Write))&&(Reg_FLAG & (1<<SL_One))) // если создаем слот записи и записываем еденицу
  {
     Reg_FLAG |= (1<<END_Sl);  // ставим флаг завершения слота
  }
    //---------------------------------------------------------
  if(Reg_FLAG & (1<<Slot_Read)) // если создаем слот чтения
  {
      Falling_edge =  Rising_adge = 0;
      HAL_TIM_IC_Stop(&htim1,TIM_CHANNEL_1);
      HAL_TIM_IC_Stop(&htim1,TIM_CHANNEL_2);
      Falling_edge = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); // чтение значения в регистре захвата/сравнения
      Rising_adge = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2); // чтение значения в регистре захвата/сравнения
        
       Bit_Rd = 0;
      if(Rising_adge > Falling_edge)
        {
         Pulse_length = Rising_adge - Falling_edge;
        }
        /*if((Reg_FLAG & (1<<Slot_Read))&&(Reg_FLAG & (1<<Rising_Flag)))// если создаем слот чтения и захватили нарастающий фронт 
         {*/
        if(Pulse_length <= 22) //(Pulse_length > 15)&&(Pulse_length <= 21))
         {
           Bit_Rd = 1;
         }
        if((Pulse_length > 22)&&(Pulse_length < 30))
         {
          Bit_Rd = 0;
         }
         /*}*/
      
      
      Reg_FLAG |= (1<<Tim_ok); // флаг отработки захвата
      Reg_FLAG |= (1<<END_Sl);  // ставим флаг завершения слота
  }
 }
 


void IRQHand_htim1_In_DS (TIM_HandleTypeDef *htim)
{
  Pulse_length = 0;
  if(Reg_FLAG & (1<<In_DS))// если инициируем DS
  {
    
  if((Reg_FLAG & (1<<In_DS))&&(Reg_FLAG & (1<<Rising_Flag)))//  если инициализируем DS и захватили нарастающтй фронт 
  { 
    if(Rising_adge > Falling_edge)
    {
    Pulse_length = Rising_adge - Falling_edge;
    }
    if((Pulse_length > 15)&&(Pulse_length < 600))
    {
      Reg_FLAG ^= (1<<In_DS); // снимаем флаг инициализации DS
      Reg_FLAG |= (1<<Presence_DS); //ставим флаг присутствия DS
       HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);
       HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_2);
       HAL_TIM_Base_Stop_IT(&htim7);
       __HAL_TIM_CLEAR_IT(&htim7, TIM_FLAG_UPDATE);
    }
    if(Pulse_length > 680)
    {
      Reg_FLAG ^= (1<<In_DS); //снимаем флаг инициализации DS
      Reg_FLAG |= (1<<Absence_DS);// ставим флаг отсутствия DS
      HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);
      HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_2);
      HAL_TIM_Base_Stop_IT(&htim7);
       __HAL_TIM_CLEAR_IT(&htim7, TIM_FLAG_UPDATE);
    }
  }
 }
  //=========================================================
  if(Reg_FLAG & (1<<Slot_Read))// если создаем слот чтения
  {
    Bit_Rd = 0;
    if(Rising_adge > Falling_edge)
    {
    Pulse_length = Rising_adge - Falling_edge;
    }
    if((Reg_FLAG & (1<<Slot_Read))&&(Reg_FLAG & (1<<Rising_Flag)))// если создаем слот чтения и захватили нарастающий фронт 
    {
      if(Pulse_length <= 22) //(Pulse_length > 15)&&(Pulse_length <= 21))
      {
        Bit_Rd = 1;
      }
      if((Pulse_length > 22)&&(Pulse_length < 30))
      {
        Bit_Rd = 0;
      }
    }
  }
}



void Init_LCD (void)
{
 SendLCD(DB5_N|DB4_N) ;
 HAL_Delay(5);
 SendLCD(DB5_N|DB4_N) ;
 HAL_Delay(1);
 SendLCD(DB5_N|DB4_N) ;
 
 SendLCD(DB5_N) ;
 
 SendLCD(DB5_N);
 SendLCD(DB7_N|DB6_N);
 
 SendLCD(0);
 SendLCD(DB7_N); 
 
 SendLCD(0);
 SendLCD(DB4_N); 
 
 SendLCD(0);
 SendLCD(DB5_N|DB4_N);

 SendLCD(NULL);
 SendLCD(DB7_N|DB6_N); 
  }
 
 void Data_load_LCD(const uint8_t Data) 
{
   uint8_t DataSend=0 ;
   DataSend=(Data&0xf0)>>4 ;
   DataSend|= (1<<RS_N) ;
   SendLCD(DataSend);
   
   DataSend=Data&0x0f ;
   DataSend|= (1<<RS_N) ;
   SendLCD(DataSend); 
  }

void Comand_load_LCD(const uint8_t Data)
{
   uint8_t DataSend=0 ;
   DataSend=(Data&0xf0)>>4 ;
   DataSend|= (0<<RS_N) ;
   SendLCD(DataSend);
   
   DataSend=Data&0x0f ;
   DataSend|= (0<<RS_N) ;
   SendLCD(DataSend);
}

void SendLCD(const uint8_t Data)   
{
 GPIO_PinState Action ;   
 for ( uint8_t i=0; i<QTY_REP; i++ )
 {
   if( ( Data & ( 1<<i ) ) )             
   {
     Action=GPIO_PIN_SET;
   }
   else
   {
     Action=GPIO_PIN_RESET;
   }
   switch ( i )
   {
      case 0:  HAL_GPIO_WritePin(LCD_PORT, DB4, Action); break;// DB4    
      case 1:  HAL_GPIO_WritePin(LCD_PORT, DB5, Action); break; //DB5       
      case 2:  HAL_GPIO_WritePin(LCD_PORT, DB6, Action); break;//DB6       
      case 3:  HAL_GPIO_WritePin(LCD_PORT, DB7, Action); break;//DB7 
      case 4:  HAL_GPIO_WritePin(LCD_PORT, RS, Action) ; break;//RS
     default: break;
   }      
  }
  HAL_GPIO_WritePin(LCD_PORT, ENA, GPIO_PIN_SET);// E=1
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_PORT, ENA, GPIO_PIN_RESET);// E=0
  HAL_Delay(1);   
 }

void DecomString(const uint8_t String[])
{
  for (uint8_t i=0;i<(strlen(String));i++)
  {
    Data_load_LCD(String[i]);
  }
  
}

void Print_number ( uint8_t Number )
{
   Data_load_LCD(Number);
}

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
