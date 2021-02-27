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
uint8_t Data[26] = {0}; //массив для передачи по UART
uint8_t Ch_Ex_DS;
uint8_t Ch_Fal_DS;
uint8_t Buf_Read = 0;
uint8_t Bit_Rd = 0;
uint8_t Conf_Reg = 0; // регистр конфигурации
uint32_t  Tmp_reg_msb = 0; // временные регистры для обработки данных от датчика
uint32_t  Tmp_reg_lsb = 0;
float  Tmp_reg_mat = 0;
float  Tmp_reg_tmp = 0;

iButtons *START_List; // указатель на первый датчик
iButtons *END_List; //указатель на последний датчик
iButtons *TMP_List; // временный указатель на элемент списка

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
#define Postv_Temper  5  //флаг положительной температуры
#define SL_One        6  //флаг создания единицы
#define Negatv_Temper 7  //флаг отрицательной температуры
#define Tim_ok        8  //флаг отработки захвата 
#define Falling_Flag  9  //флаг падающего  фронта
#define Rising_Flag   10 //флаг нарастающего фронта
#define Pause         11 //флаг паузы
#define END_Sl        12 //флаг завершения слота
#define Absence_DS    13 //флаг отсутствия DS
#define TMP           14 //Временный флаг
#define Delay_Fl      15 //Флаг задержки
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t String[] = "HELLO!!!";
uint8_t String_1[] = "NO DS18B20";
uint8_t String_2[] = "TEMPERATURA ";
uint8_t String_3[] = "ERR MEASUREMENTS";
UART_HandleTypeDef  huart2;
char TmCh = 1;
uint8_t str[] = "Проверка передачи UART\r\n\0";



Memory_DS  MEMORY_DS;     //создадим обьект структуры памяти DS обьявлена в main.h
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void Data_load_LCD(const uint8_t Data);
void Comand_load_LCD(const uint8_t Data);
void Init_LCD(void);
void SendLCD(const uint8_t Data);
void DecomString(const uint8_t String[]);
//uint8_t Init_DS (void);
//void IRQHand_htim6( void);    // функция обработчика прерывания от TIM6
//void IRQHand_htim7( void);
//void Sl_Write(uint8_t bit);   //слот записи в DS
//uint8_t Sl_Read(void);        //слот чтения из DS
//void Comand_DS (uint8_t Com); // команда в DS
//void Data_Read_DS (uint8_t Number_of_bytes); // чтение данных от DS
//void Print_number ( float Number );
//void Def_and_out_Temp (void); // определение и вывод температуры
//void Search_ROM(void);    // определение ROM кода подключенных датчиков


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
   DecomString(String_2);
   
   
   /* USER CODE END 2 */
   
   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   
   Comand_load_LCD(0xC0);
   HAL_Delay(2);
   START_List =  Search_ROM ();   // поиск ROM
   TMP_List = START_List;
   while (TMP_List)
    {
      uint64_t NUMBER = TMP_List->Sens_OR.ROM_CODE;
      snprintf(Data, 27, "ROMcode: %llX\n",NUMBER);
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)Data, strlen(Data));
      HAL_Delay(100);
      TMP_List = (iButtons*)TMP_List->next;
    }
   TMP_List = START_List;
   while(1)
    {
      uint64_t NUMBER = TMP_List->Sens_OR.ROM_CODE;
      Def_and_out_Temp (NUMBER);
      HAL_Delay(1500);
      TMP_List = (iButtons*)TMP_List->next;
      if(!TMP_List)
       {
         TMP_List = START_List;
       }
    }
//   uint64_t NUMBER =0;
//   if(Init_DS () )
//    {
//      Comand_DS (0x33);     // Команда чтения ром
//      NUMBER = Read_ROM ();          // читаем ром
//     }
//   while(1)
//    {
//    }
 }
/* USER CODE END WHILE */


uint8_t Init_DS (void)  // Инициализация датчика DS
 {
   Reg_FLAG = 0;
   
   TIM6->PSC = 10;
   TIM6->ARR = 420;
   
   TIM7->PSC = 10;
   TIM7->ARR = 550;
   
   Reg_FLAG |= (1<<In_DS);  //выставляем флаг инициализации DS
   HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_RESET);
   HAL_TIM_Base_Start_IT(&htim6); // вкл. контроль времени импулса сброса
   
   while ((!(Reg_FLAG & (1<<Absence_DS))&&(!(Reg_FLAG & (1<<Presence_DS))))||
          ((Reg_FLAG & (1<<Absence_DS))&&(Reg_FLAG & (1<<Presence_DS))))  // если нет флагов наличия или отсутствия датчика 
    {                                                                                                
      
    }
   if(Reg_FLAG & (1<<Absence_DS)) //если нет  присутствия DS  
    { 
      Reg_FLAG = 0;
      return 0;
    }
   
   if(Reg_FLAG & (1<<Presence_DS))  //если  есть присутствие DS
    { 
      Reg_FLAG = 0;
      return 1;
    }
 }

void Comand_DS (uint8_t Com) // Передача команды датчику DS
 {
   for(int i=0;i < 8;i++)
    {
      uint8_t Bit = 0x01;
      Bit &=  Com;  //младший бит команды в буфер 
      Com >>= 1;  // сдвигаем оставшиеся биты в право
      Sl_Write(Bit);  // Передаем бит
    }
   Reg_FLAG = 0;
 }


void  Select_ID_DS (uint64_t Com) // выбор датчика DS по ID
 {
   for(int i=0;i < 64;i++)
    {
      uint8_t Bit = 0x01;
      Bit &=  Com;  //младший бит команды в буфер 
      Com >>= 1;  // сдвигаем оставшиеся биты в право
      Sl_Write(Bit);  // Передаем бит
    }
   Reg_FLAG = 0;
 }


void Data_Read_DS (uint8_t Number_of_bytes) // Прием данных от датчика DS 
 {
   Buf_Read = 0;
   uint8_t Quantity_of_bit,Tmp_byt = 0;
   Quantity_of_bit = Number_of_bytes * 8; //определяем количество бит для приема
   
   for(int i=0; i < Quantity_of_bit; i++)
    {
      Tmp_byt = 0;
      Tmp_byt = Sl_Read();  // Принятый бит во временный байт
      Tmp_byt <<= 7;
      Buf_Read |= Tmp_byt;  // затем в буфер приема
      if((i != 7)&&(i != 15)&&(i != 23)&&(i != 31)&&(i != 39)&&(i != 71))
       {
         Buf_Read >>= 1;       // буфер сдвигаем
       }
      switch (i)
       {
         case 7:
         MEMORY_DS.Temperature_LSB = Buf_Read;
         Buf_Read = 0;
         break;
         case 15:
         MEMORY_DS.Temperature_MSB = Buf_Read;
         Buf_Read = 0;
         break;
         case 23: 
         MEMORY_DS.ThRegisterorUserByte_1 = Buf_Read;
         Buf_Read = 0;
         break;
         case 31:
         MEMORY_DS.ThRegisterorUserByte_2 = Buf_Read;
         Buf_Read = 0;
         break;
         case 39:
         MEMORY_DS.ConfigurationRegister = Buf_Read;
         Buf_Read = 0;
         break;
         case 71:
         MEMORY_DS.C_R_C = Buf_Read;
         Buf_Read = 0;
         break;
       }
    }
   
 }

void Sl_Write(uint8_t bit)// создание слота записи
 {
   Reg_FLAG = 0;
   if(!bit)
    {
      TIM6->PSC = 10;
      TIM6->ARR = 55;
      TIM6->CNT = 0;
      
      Reg_FLAG |= (1<<Slot_Write)|(1<<Sl_Zero);  //ставим флаги слота записи и создания 0
    }
   if(bit)
    {
      TIM6->PSC = 10;
      TIM6->ARR = 3;
      TIM6->CNT = 0;
      
      Reg_FLAG |= (1<<Slot_Write)|(1<<SL_One);  //ставим флаги слота записи и создания 1
    }
   HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_RESET);
   HAL_TIM_Base_Start_IT(&htim6);
   
   while (!(Reg_FLAG & (1<<END_Sl)))
    {
      
    }
   Reg_FLAG = 0;// снимаем флаги
 }

uint8_t Sl_Read(void)// создание слота чтения
 {
   Bit_Rd = 0;                  // чистим буфер приема
   Reg_FLAG = 0;                // снимаем флаги
   Reg_FLAG |= (1<<Slot_Read);  //ставим флаг слота чтения
   
   TIM6->PSC = 10;
   TIM6->ARR = 3;     // таймер на ~2 us
   TIM6->CNT = 0;
   
   TIM7->PSC = 10;
   TIM7->ARR = 28;// 52;
   TIM7->CNT = 0;
   
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

void IRQHand_htim6( void) // Обработчик прерывания Tim6
 {
   HAL_TIM_Base_Stop_IT(&htim6);
   
   if(Reg_FLAG & (1<<In_DS)) // если инициируем DS
    {
      __HAL_TIM_SET_COUNTER(&htim1, 0x0000); // обнуление счётчика
      HAL_TIM_IC_Start/*_IT*/(&htim1,TIM_CHANNEL_1);// запуск с прерыванием по захвату
      HAL_TIM_IC_Start/*_IT*/(&htim1,TIM_CHANNEL_2);
      HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_SET); // отпускаем линию
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
         TIM7->ARR = 40;
         TIM7->CNT = 0;
         HAL_TIM_Base_Start_IT(&htim7);
         
       }
    }
   //============================================================
   if(Reg_FLAG & (1<<Slot_Read)) // если создаем слот чтения
    {
      HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_SET);
    }
   //=============================================================
   if(Reg_FLAG & (1<<Delay_Fl)) // если нужна задержка
    {
      HAL_GPIO_WritePin(GPIOE, Ex_DS_Pin, GPIO_PIN_SET);
      Reg_FLAG ^= (1<<Delay_Fl);  //снимаем флаг задержки
    }
 }

void IRQHand_htim7(void) // Обработчик прерывания  Tim7
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
      if(Pulse_length <= 22) //(Pulse_length > 15)&&(Pulse_length <= 21))
       {
         Bit_Rd = 1;
       }
      if((Pulse_length > 22)&&(Pulse_length < 30))
       {
         Bit_Rd = 0;
       }
      
      Reg_FLAG |= (1<<Tim_ok); // флаг отработки захвата
      Reg_FLAG |= (1<<END_Sl);  // ставим флаг завершения слота
    }
 }


void Def_and_out_Temp (uint64_t ID) // определение и вывод температуры если датчик один ID = 0
 {
   float Measurement_Res = 0;
   if(Init_DS () )
    {
      if(ID)                 // если задан адрес
       {
         Comand_DS (0X55);    // то шлем команду РОМ соответствия для нужного датчика
         Select_ID_DS(ID);    // затем передаем ID нужного датчика
       }
      else
       {
         Comand_DS (0xCC);     // иначе пропуск ром,команда для всех датчиков
       }
      Comand_DS (0X55);    // то шлем команду РОМ соответствия для нужного датчика
      Select_ID_DS(ID);    // затем передаем ID нужного датчика  
      Comand_DS (0x44);     // делаем замер температуры
    }
   while(!Sl_Read ())         // ждем окончания преобразований
    {
    }
   if(Init_DS () )
    {
      if(ID)                 // если задан адрес
       {
         Comand_DS (0X55);    // то шлем команду РОМ соответствия для нужного датчика
         Select_ID_DS(ID);    // затем передаем ID нужного датчика
         Comand_DS (0xBE);     // читаем память
         Data_Read_DS (9);
       }
      else
       {
         Comand_DS (0xCC);     // иначе пропуск ром,команда для всех датчиков
         Comand_DS (0xBE);     // читаем память
         Data_Read_DS (9);
       }
      
      Conf_Reg = MEMORY_DS.ConfigurationRegister;
      switch ( Conf_Reg)
       {
         case (0x7F):
         {
           Measurement_Res = 0.0625;
           break;
         }
         case (0x5F):
         {
           Measurement_Res = 0.125;
           break;
         }
         case (0x3F):
         {
           Measurement_Res = 0.25;
           break;
         }
         case (0x1F):
         {
           Measurement_Res = 0.5;
           break;
         }
       }
      Tmp_reg_msb = MEMORY_DS.Temperature_MSB;
      Tmp_reg_lsb = MEMORY_DS.Temperature_LSB;
      Tmp_reg_msb <<= 8;                   // сдвигаем влево на 8 разрядов
      Tmp_reg_msb |= Tmp_reg_lsb;          // соединяем
      
      if(Tmp_reg_msb & 0xF800)// ((0<<7)&&(0<<6)&&(0<<5)&&(0<<4)&&(0<<3)))// если в 7,6,5,4,3 битах 1 то
       {
         Reg_FLAG |= (1<<Negatv_Temper);  //выставляем флаг отрицательной температуры
         Tmp_reg_msb ^= 0xFFFF;           // инвертируем биты
         Tmp_reg_msb += 0x01;             //прибавляем 1
       }
      else                                // иначе то
       {
         Reg_FLAG |= (1<<Postv_Temper);   //выставляем флаг положительной температуры
       } 
      
      Tmp_reg_mat = Tmp_reg_msb;
      Tmp_reg_mat *= Measurement_Res;  //КОЭФФИЦИЕНТ УСТАНАВЛИВАТЬ ПРОГРАМНО!!!!!
      if(Tmp_reg_tmp != Tmp_reg_mat)
       {      
         Tmp_reg_tmp = Tmp_reg_mat;        // сохраним значение 
         if( Reg_FLAG & (1<<Negatv_Temper))
          {
            Comand_load_LCD(0xC0);
            Data_load_LCD('-');
            HAL_Delay(2);
            Print_number ( Tmp_reg_mat );
          }
         if( Reg_FLAG & (1<<Postv_Temper))
          {
            Comand_load_LCD(0xC0);
            HAL_Delay(2);
            Print_number ( Tmp_reg_mat );
          }
       }
      else
       {
         Comand_load_LCD(0x01);
         HAL_Delay(2);
         Comand_load_LCD(0x02);
         HAL_Delay(2);
         DecomString(String_3);
       }
    }
 }

void Init_LCD (void)  // Инициализация Дисплея
 {
   SendLCD(DB5_N|DB4_N) ;// установка функций
   HAL_Delay(5);
   SendLCD(DB5_N|DB4_N) ;// установка функций
   HAL_Delay(1);
   SendLCD(DB5_N|DB4_N) ;// установка функций
   
   SendLCD(DB5_N) ;      // установка функций 4 разрядный игтерфейс
   
   SendLCD(DB5_N);       // функция 4х разр.интерфейс
   SendLCD(DB7_N|DB6_N); // DB7_N - число строк-2,DB6_N-матрица 5х10
   
   SendLCD(0);
   SendLCD(DB7_N);      //отключение табло
   
   SendLCD(0);
   SendLCD(DB4_N);      // очистка табло
   
   SendLCD(0);          // установка режима ввода данных 
   SendLCD(DB5_N|DB4_N);      // DB5_N - инкремент счетчика,
   
   SendLCD(0);       // включение дисплея и отображение курсора
   SendLCD(DB7_N|DB6_N); // курсор не отображается
   
   SendLCD(DB4_N);
   SendLCD(DB6_N); 
   
   
 }

void Data_load_LCD(const uint8_t Data) // Запись данных в Дисплей
 {
   uint8_t DataSend=0 ;
   DataSend=(Data&0xf0)>>4 ;
   DataSend|= (1<<RS_N) ;
   SendLCD(DataSend);
   
   DataSend=Data&0x0f ;
   DataSend|= (1<<RS_N) ;
   SendLCD(DataSend); 
 }

void Comand_load_LCD(const uint8_t Data) // Запись команды в Дисплей
 {
   uint8_t DataSend=0 ;
   DataSend=(Data&0xf0)>>4 ;
   DataSend|= (0<<RS_N) ;
   SendLCD(DataSend);
   
   DataSend=Data&0x0f ;
   DataSend|= (0<<RS_N) ;
   SendLCD(DataSend);
 }

void SendLCD(const uint8_t Data) // управление портом при работе с Дисплеем  
 {
   GPIO_PinState Action ;   
   for (uint8_t i=0;i<QTY_REP;i++ )
    {
      if((Data&(1<<i )))             
       {
         Action=GPIO_PIN_SET;
       }
      else
       {
         Action=GPIO_PIN_RESET;
       }
      switch (i)
       {
         case 0:  HAL_GPIO_WritePin(LCD_PORT,DB4,Action); break;// DB4    
         case 1:  HAL_GPIO_WritePin(LCD_PORT,DB5,Action); break; //DB5       
         case 2:  HAL_GPIO_WritePin(LCD_PORT,DB6,Action); break;//DB6       
         case 3:  HAL_GPIO_WritePin(LCD_PORT,DB7,Action); break;//DB7 
         case 4:  HAL_GPIO_WritePin(LCD_PORT,RS,Action) ; break;//RS
         default: break;
       }      
    }
   HAL_GPIO_WritePin(LCD_PORT,ENA,GPIO_PIN_SET);// E=1
   HAL_Delay(1);
   HAL_GPIO_WritePin(LCD_PORT,ENA,GPIO_PIN_RESET);// E=0
   HAL_Delay(1);   
 }

void DecomString(const uint8_t String[]) // Разложение строки для передачи и вывода на дисплее
 {
   for (uint8_t i=0;i<(strlen(String));i++)
    {
      Data_load_LCD(String[i]);
    }
   
 }

void Print_number (float Number) // Вывод числа на Дисплее
 {
   snprintf((char*)Data,17,"%.4f",Number);
   
   DecomString(Data);
 }

void Print_number_16 (uint64_t Number) // Вывод числа на Дисплее
 {
   snprintf((char*)Data,17,"%X",Number);
   
   DecomString(Data);
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
