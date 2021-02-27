/*
*******************************************************************************
* Файл содержит функции для работы с датчиком DS18B20
* Функции организации 1-Write интерфейса
* Поиска ROM кода датчика
*******************************************************************************
*/


#include "main.h"

 extern uint16_t Reg_FLAG;
 iButt Sens_OR;   // Переменная для структуры содержащей данные датчика
 iButtons *Sensor; // указатель на адрес структуры данных датчика
 iButtons *START_Sens_OR = NULL; // указатель на первый датчик
 iButtons *END_Sens_OR = NULL; //указатель на последний датчик
 iButtons *TMP_Sens_OR = NULL; // временный указатель на элемент списка
 iButtons *TMPX_Sens_OR = NULL; // временный указатель на элемент списка
 
iButtons* Search_ROM (void)  // Определение ROM-кода датчака(ов)
{
  uint8_t Bit_A = 0;              //переменные для хранения ответов датчика
  uint8_t Bit_B = 0;
  uint8_t bit_write ;             // бит передаваемый на запись
  uint8_t pass_counter = 0;       // счетчик количества проходов
  uint8_t numb_of_elem = 0;       // количество элеменов списка (ДАТЧИКОВ)
  bit_write = 0;                  // Бит для записи отключающий элементы имеющие 0 в данной позиции
  uint8_t Known_conflict = 0;      //Известный конфликт
  uint8_t Next_Known_conflict = 0; // следующий известный конфликт
  
  do
   {
     if(Init_DS () )
      {
        Comand_DS (0xF0);           // команда поиск ROM
        Sens_OR.ROM_CODE = 0;       // обнулим поле
        Sens_OR.Number = 0;
        Sens_OR.Number_Bit_end = 0;
        Reg_FLAG = 0;                //сброс флагов
        
        
        if((pass_counter)&&(END_Sens_OR)&&(START_Sens_OR)) // если проход не первый
         {
           TMP_Sens_OR = Target_Sens_OR(pass_counter,numb_of_elem,END_Sens_OR); // определяем адрес элемента с которого продолжим поиск
         }
        
        for (uint8_t i=0;i<64;i++)      // цикл для чтения 64 бит
         {
           Bit_A = Sl_Read();      // Принятый бит содержащий основное значение во временный байт
           Bit_B = Sl_Read();      // Бит в дополнительном коде во временный байт   
           
           if((Bit_A)&&(Bit_B))
            {
              Reg_FLAG |= (1<<Fl_err_conf); // выствляем флаг ошибки при приеме бит
              break;                        // выходим из цикла
            }
           
           if((Bit_A==0)&&(Bit_B==0))      //если есть конфликт разрядов
            {
              if(!pass_counter)// если проход первый
               {
                 Sens_OR.Number_Bit_end = i;            // сохраним номер бита на которм возник конфликт
                 Sens_OR.Bit_Err_Write = bit_write;     // сохраним бит который выдали на запись
                 Sl_Write(Sens_OR.Bit_Err_Write);       //Отключаем элементы имеющие 0 в данной позиции ROM code
               }
              
              if(pass_counter)                            // если проход не первый 
               {
                 
                 TMPX_Sens_OR = (iButtons*)START_Sens_OR;  // Проверка на известность конфликта ранее
                 do
                  {
                    if((TMPX_Sens_OR->Sens_OR.Number_Bit_end == i)&&(TMPX_Sens_OR->Sens_OR.Number_Bit_end != TMP_Sens_OR->Sens_OR.Number_Bit_end))
                     {
                       Next_Known_conflict = 1; // Конфликт известен но он нас не интересует
                     }
                    
                    if((TMPX_Sens_OR->Sens_OR.Number_Bit_end == i)&&(TMPX_Sens_OR->Sens_OR.Number_Bit_end == TMP_Sens_OR->Sens_OR.Number_Bit_end)) // флаг известности конфликта нужно выставлять когда i будет равно Number_Bit_end установленном в том списке на который указывает TMPX_Sens_OR
                     {
                       Known_conflict = 1;   // Выставляем флаг известного конфликта
                     }
                    TMPX_Sens_OR = (iButtons*)TMPX_Sens_OR->next;
                  }while (TMPX_Sens_OR);  // Крутим пока не переберем все элемнты списка
                 
                 if ((Known_conflict)&&(!Next_Known_conflict))// если конфликт известен и актуален 
                  {
                    Sens_OR.Bit_Err_Write = bit_write+1;  // сохраним бит который выдали на запись
                  }
                 
                 else
                  {
                    Sens_OR.Bit_Err_Write = bit_write;  // сохраним бит который выдали на запись
                  }
                 Sens_OR.Number_Bit_end = i;
                 Sl_Write(Sens_OR.Bit_Err_Write);      //Отключаем элементы имеющие 1 в данной позиции ROM code
                 bit_write = 0;                        // на всякий случай
               }
              
              Insert_and_sav_ROM_cod (Sens_OR.Bit_Err_Write,i); // Выставим бит которым отключаем одни из конфликтующих элементов
              
              if(!START_Sens_OR)                     // Создадим первый элемент спика
               {
                 START_Sens_OR =  Next_iButton(&Sens_OR,START_Sens_OR,END_Sens_OR); 
                 END_Sens_OR = START_Sens_OR;
               }
              else                                   // Создадим последующие элементы спика
               {
                 if(!pass_counter)                   // если проход первый
                  {
                    END_Sens_OR = Next_iButton(&Sens_OR,START_Sens_OR,END_Sens_OR);
                  }
                 if((pass_counter)&&(!Known_conflict)&&(!Next_Known_conflict)) // если проход не первый и конфликт не известен
                  {
                    ADD_iButton (&Sens_OR,TMP_Sens_OR); //вставка элемента в середину списка
                  }
               }
              Known_conflict = 0;                    // снимаем флаг
              Next_Known_conflict = 0;              // снимаем флаг
            }
           
           if(Bit_A != Bit_B)  // если нет конфликта разрядов
            {
              Insert_and_sav_ROM_cod (Bit_A,i);           // Вставим полученный бит 
                                                        // Устанавливаем бит которым ответили датчики       
              Sl_Write(Bit_A);                          // и на котором не было конфликта
              
              if((!START_Sens_OR)&&(i==63))             // если прочитали последний бит ROM и
               {                                       // небыло ни одного конфликта  т.е элемент один
                 START_Sens_OR =  Next_iButton(&Sens_OR,START_Sens_OR,END_Sens_OR); // Создадим элемент спика
                 END_Sens_OR = START_Sens_OR;
               }
              if((START_Sens_OR)&&(i==63))             // если прочитали последний бит ROM и
               { 
                 if(!pass_counter)                    //если проход первый
                  {
                    END_Sens_OR = Next_iButton(&Sens_OR,START_Sens_OR,END_Sens_OR);              
                  }
                 if(pass_counter)                     // если проход не первый
                  {
                    TMP_Sens_OR->Sens_OR = Sens_OR;
                  }
               }
            }
         }
        
        if(!(Reg_FLAG & (1<<Fl_err_conf))) // если нет ошибки при преме бит
         {
           pass_counter ++;                                // увеличиваем количество проходов
           numb_of_elem = Counting_of_elem(START_Sens_OR); // посчитаем количество элементов
           Reg_FLAG = 0;                                   //сброс флагов
         }
      }
   }while (pass_counter != numb_of_elem);               // Крутим пока количество проходов не будет равно количеству элементов списка
  return START_Sens_OR;
}


iButtons*Next_iButton(iButt*Sens_OR,iButtons*START_Sens_OR,iButtons*END_Sens_OR) // создание последующегоэлемента в списке датчиков 
{
  iButtons*Tmp;                               // временный адрес
  if (!START_Sens_OR)
  {
    Tmp=(iButtons*)malloc(sizeof(iButtons));  //выделяем память
    Tmp->Sens_OR = *Sens_OR;                  // сохраняем данные датчика
    Tmp->pref=NULL;                           //адрес предидущего датчика 0
    Tmp->next=NULL;                           //адрес следующего датчика 0
    return Tmp;
  }
  else
  {  
    Tmp=(iButtons*)malloc(sizeof(iButtons));  //выделяем память
    Tmp->Sens_OR = *Sens_OR;                  //сохраняем данные нового датчика
    END_Sens_OR->next =(struct iButtons*) Tmp;//в последнем элементе списка сохраняем адрес вновь созданного как следующего
    Tmp->pref = (struct iButtons*)END_Sens_OR;//В созданном элементе списка сохраняем адрес предидущего элемента       
    Tmp->next = NULL;                         //В созданном элементе списка адрес следующего в 0
    return Tmp;
  }
}

void ADD_iButton (iButt*Sens_OR,iButtons*START_Sens_OR) //вставка элемента в середину списка
{
    iButtons*Tmp;                              // временный адрес
    Tmp = (iButtons*)malloc(sizeof(iButtons)); //выделяем память
    Tmp->Sens_OR = *Sens_OR;                   //сохраняем данные нового датчика
    Tmp->next=START_Sens_OR->next;             // В предидущем элементе списка сохраняем в адрес вставляемого элемента адрес созданного элемента
    START_Sens_OR->next = (struct iButtons*)Tmp;
    Tmp->pref = (struct iButtons*) START_Sens_OR;//В созданном элементе списка сохраняем адрес предидущего элемента       
 }

void Insert_and_sav_ROM_cod (uint8_t Bit_A,int i)  // Функция вставки и сохранения бита в ROM code
{
     uint64_t TMP_ROM;
     TMP_ROM = 0;
     TMP_ROM |= Bit_A ;                     // полученный бит записываем во временный буфер
     TMP_ROM <<= 63;                        // сдвигаем всё в лево
     Sens_OR.ROM_CODE |= TMP_ROM;           // сохраняем 
     if(i!=63)
      {
     Sens_OR.ROM_CODE >>= 1;                // двигаем все биты в право кроме последнего раза
      }
}

uint8_t Counting_of_elem(iButtons*START_Sens_OR) // функция посчета количества элементов в списке и упорядочивания их номеров
{
  uint8_t elem = 0 ;
  iButtons *TMP_Sens = NULL;
  TMP_Sens = (iButtons*)START_Sens_OR;
  while (TMP_Sens )            //пока TMP_Sens указывает не конец списка
  {
  TMP_Sens->Sens_OR.Number = elem;           //упорядочиваем номер элемента в списке
  TMP_Sens = (iButtons*)TMP_Sens->next;      // определяем адрес следующего
  elem++;                                    // увеличиваем номер элемента
  }
  return elem; 
}

iButtons*Target_Sens_OR(uint8_t pass_counter,uint8_t numb_of_elem,iButtons*END_Sens_OR)// Функция определения адреса элемента от которого 
{                                                                                      //идёт дальнейший поиск
  uint8_t num = 0;
  iButtons*TMP_Sensor = (iButtons*)END_Sens_OR;
  num =  TMP_Sensor->Sens_OR.Number;
  while (num != ((numb_of_elem - pass_counter)-1)) // номер нужного элемента списка равен разнице количества элементов списка и 
  {                                            //количества проходов
    TMP_Sensor = (iButtons*)TMP_Sensor->pref;
    num =  TMP_Sensor->Sens_OR.Number;
  }
  return TMP_Sensor;
}

uint64_t Read_ROM (void)                    // функция чтения ROM кода от ОДНОГО УСТРОЙСТВА НА ШИНЕ!!!
 {
   uint64_t Tmp_Buf_ROM = 0;
   uint64_t Data_ROM = 0;
   char Bit_ROM = 0;
   for(int i=0;i<64;i++)
    {
      Bit_ROM = Sl_Read();                     // читаем бит     
      Tmp_Buf_ROM |= Bit_ROM ;                 // полученный бит записываем во временный буфер
      Tmp_Buf_ROM <<= 63;                      // буфер сдвигаем влево до конца
      Data_ROM |= Tmp_Buf_ROM;                 // обеденяем временный буфер итоговый
      if(i!=63)
       {
         Data_ROM >>= 1;                          //итог сдвигаем в право кроме последнего раза 
       }
      Bit_ROM = 0;          
      Tmp_Buf_ROM = 0;
    }
   return Data_ROM;
 }





/************************ (C) COPYRIGHT ДимосПотапосЭлектроникс *****END OF FILE****/