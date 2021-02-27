/*
*******************************************************************************
* ���� �������� ������� ��� ������ � �������� DS18B20
* ������� ����������� 1-Write ����������
* ������ ROM ���� �������
*******************************************************************************
*/


#include "main.h"

 extern uint16_t Reg_FLAG;
 iButt Sens_OR;   // ���������� ��� ��������� ���������� ������ �������
 iButtons *Sensor; // ��������� �� ����� ��������� ������ �������
 iButtons *START_Sens_OR = NULL; // ��������� �� ������ ������
 iButtons *END_Sens_OR = NULL; //��������� �� ��������� ������
 iButtons *TMP_Sens_OR = NULL; // ��������� ��������� �� ������� ������
 iButtons *TMPX_Sens_OR = NULL; // ��������� ��������� �� ������� ������
 
iButtons* Search_ROM (void)  // ����������� ROM-���� �������(��)
{
  uint8_t Bit_A = 0;              //���������� ��� �������� ������� �������
  uint8_t Bit_B = 0;
  uint8_t bit_write ;             // ��� ������������ �� ������
  uint8_t pass_counter = 0;       // ������� ���������� ��������
  uint8_t numb_of_elem = 0;       // ���������� �������� ������ (��������)
  bit_write = 0;                  // ��� ��� ������ ����������� �������� ������� 0 � ������ �������
  uint8_t Known_conflict = 0;      //��������� ��������
  uint8_t Next_Known_conflict = 0; // ��������� ��������� ��������
  
  do
   {
     if(Init_DS () )
      {
        Comand_DS (0xF0);           // ������� ����� ROM
        Sens_OR.ROM_CODE = 0;       // ������� ����
        Sens_OR.Number = 0;
        Sens_OR.Number_Bit_end = 0;
        Reg_FLAG = 0;                //����� ������
        
        
        if((pass_counter)&&(END_Sens_OR)&&(START_Sens_OR)) // ���� ������ �� ������
         {
           TMP_Sens_OR = Target_Sens_OR(pass_counter,numb_of_elem,END_Sens_OR); // ���������� ����� �������� � �������� ��������� �����
         }
        
        for (uint8_t i=0;i<64;i++)      // ���� ��� ������ 64 ���
         {
           Bit_A = Sl_Read();      // �������� ��� ���������� �������� �������� �� ��������� ����
           Bit_B = Sl_Read();      // ��� � �������������� ���� �� ��������� ����   
           
           if((Bit_A)&&(Bit_B))
            {
              Reg_FLAG |= (1<<Fl_err_conf); // ��������� ���� ������ ��� ������ ���
              break;                        // ������� �� �����
            }
           
           if((Bit_A==0)&&(Bit_B==0))      //���� ���� �������� ��������
            {
              if(!pass_counter)// ���� ������ ������
               {
                 Sens_OR.Number_Bit_end = i;            // �������� ����� ���� �� ������ ������ ��������
                 Sens_OR.Bit_Err_Write = bit_write;     // �������� ��� ������� ������ �� ������
                 Sl_Write(Sens_OR.Bit_Err_Write);       //��������� �������� ������� 0 � ������ ������� ROM code
               }
              
              if(pass_counter)                            // ���� ������ �� ������ 
               {
                 
                 TMPX_Sens_OR = (iButtons*)START_Sens_OR;  // �������� �� ����������� ��������� �����
                 do
                  {
                    if((TMPX_Sens_OR->Sens_OR.Number_Bit_end == i)&&(TMPX_Sens_OR->Sens_OR.Number_Bit_end != TMP_Sens_OR->Sens_OR.Number_Bit_end))
                     {
                       Next_Known_conflict = 1; // �������� �������� �� �� ��� �� ����������
                     }
                    
                    if((TMPX_Sens_OR->Sens_OR.Number_Bit_end == i)&&(TMPX_Sens_OR->Sens_OR.Number_Bit_end == TMP_Sens_OR->Sens_OR.Number_Bit_end)) // ���� ����������� ��������� ����� ���������� ����� i ����� ����� Number_Bit_end ������������� � ��� ������ �� ������� ��������� TMPX_Sens_OR
                     {
                       Known_conflict = 1;   // ���������� ���� ���������� ���������
                     }
                    TMPX_Sens_OR = (iButtons*)TMPX_Sens_OR->next;
                  }while (TMPX_Sens_OR);  // ������ ���� �� ��������� ��� ������� ������
                 
                 if ((Known_conflict)&&(!Next_Known_conflict))// ���� �������� �������� � �������� 
                  {
                    Sens_OR.Bit_Err_Write = bit_write+1;  // �������� ��� ������� ������ �� ������
                  }
                 
                 else
                  {
                    Sens_OR.Bit_Err_Write = bit_write;  // �������� ��� ������� ������ �� ������
                  }
                 Sens_OR.Number_Bit_end = i;
                 Sl_Write(Sens_OR.Bit_Err_Write);      //��������� �������� ������� 1 � ������ ������� ROM code
                 bit_write = 0;                        // �� ������ ������
               }
              
              Insert_and_sav_ROM_cod (Sens_OR.Bit_Err_Write,i); // �������� ��� ������� ��������� ���� �� ������������� ���������
              
              if(!START_Sens_OR)                     // �������� ������ ������� �����
               {
                 START_Sens_OR =  Next_iButton(&Sens_OR,START_Sens_OR,END_Sens_OR); 
                 END_Sens_OR = START_Sens_OR;
               }
              else                                   // �������� ����������� �������� �����
               {
                 if(!pass_counter)                   // ���� ������ ������
                  {
                    END_Sens_OR = Next_iButton(&Sens_OR,START_Sens_OR,END_Sens_OR);
                  }
                 if((pass_counter)&&(!Known_conflict)&&(!Next_Known_conflict)) // ���� ������ �� ������ � �������� �� ��������
                  {
                    ADD_iButton (&Sens_OR,TMP_Sens_OR); //������� �������� � �������� ������
                  }
               }
              Known_conflict = 0;                    // ������� ����
              Next_Known_conflict = 0;              // ������� ����
            }
           
           if(Bit_A != Bit_B)  // ���� ��� ��������� ��������
            {
              Insert_and_sav_ROM_cod (Bit_A,i);           // ������� ���������� ��� 
                                                        // ������������� ��� ������� �������� �������       
              Sl_Write(Bit_A);                          // � �� ������� �� ���� ���������
              
              if((!START_Sens_OR)&&(i==63))             // ���� ��������� ��������� ��� ROM �
               {                                       // ������ �� ������ ���������  �.� ������� ����
                 START_Sens_OR =  Next_iButton(&Sens_OR,START_Sens_OR,END_Sens_OR); // �������� ������� �����
                 END_Sens_OR = START_Sens_OR;
               }
              if((START_Sens_OR)&&(i==63))             // ���� ��������� ��������� ��� ROM �
               { 
                 if(!pass_counter)                    //���� ������ ������
                  {
                    END_Sens_OR = Next_iButton(&Sens_OR,START_Sens_OR,END_Sens_OR);              
                  }
                 if(pass_counter)                     // ���� ������ �� ������
                  {
                    TMP_Sens_OR->Sens_OR = Sens_OR;
                  }
               }
            }
         }
        
        if(!(Reg_FLAG & (1<<Fl_err_conf))) // ���� ��� ������ ��� ����� ���
         {
           pass_counter ++;                                // ����������� ���������� ��������
           numb_of_elem = Counting_of_elem(START_Sens_OR); // ��������� ���������� ���������
           Reg_FLAG = 0;                                   //����� ������
         }
      }
   }while (pass_counter != numb_of_elem);               // ������ ���� ���������� �������� �� ����� ����� ���������� ��������� ������
  return START_Sens_OR;
}


iButtons*Next_iButton(iButt*Sens_OR,iButtons*START_Sens_OR,iButtons*END_Sens_OR) // �������� �������������������� � ������ �������� 
{
  iButtons*Tmp;                               // ��������� �����
  if (!START_Sens_OR)
  {
    Tmp=(iButtons*)malloc(sizeof(iButtons));  //�������� ������
    Tmp->Sens_OR = *Sens_OR;                  // ��������� ������ �������
    Tmp->pref=NULL;                           //����� ����������� ������� 0
    Tmp->next=NULL;                           //����� ���������� ������� 0
    return Tmp;
  }
  else
  {  
    Tmp=(iButtons*)malloc(sizeof(iButtons));  //�������� ������
    Tmp->Sens_OR = *Sens_OR;                  //��������� ������ ������ �������
    END_Sens_OR->next =(struct iButtons*) Tmp;//� ��������� �������� ������ ��������� ����� ����� ���������� ��� ����������
    Tmp->pref = (struct iButtons*)END_Sens_OR;//� ��������� �������� ������ ��������� ����� ����������� ��������       
    Tmp->next = NULL;                         //� ��������� �������� ������ ����� ���������� � 0
    return Tmp;
  }
}

void ADD_iButton (iButt*Sens_OR,iButtons*START_Sens_OR) //������� �������� � �������� ������
{
    iButtons*Tmp;                              // ��������� �����
    Tmp = (iButtons*)malloc(sizeof(iButtons)); //�������� ������
    Tmp->Sens_OR = *Sens_OR;                   //��������� ������ ������ �������
    Tmp->next=START_Sens_OR->next;             // � ���������� �������� ������ ��������� � ����� ������������ �������� ����� ���������� ��������
    START_Sens_OR->next = (struct iButtons*)Tmp;
    Tmp->pref = (struct iButtons*) START_Sens_OR;//� ��������� �������� ������ ��������� ����� ����������� ��������       
 }

void Insert_and_sav_ROM_cod (uint8_t Bit_A,int i)  // ������� ������� � ���������� ���� � ROM code
{
     uint64_t TMP_ROM;
     TMP_ROM = 0;
     TMP_ROM |= Bit_A ;                     // ���������� ��� ���������� �� ��������� �����
     TMP_ROM <<= 63;                        // �������� �� � ����
     Sens_OR.ROM_CODE |= TMP_ROM;           // ��������� 
     if(i!=63)
      {
     Sens_OR.ROM_CODE >>= 1;                // ������� ��� ���� � ����� ����� ���������� ����
      }
}

uint8_t Counting_of_elem(iButtons*START_Sens_OR) // ������� ������� ���������� ��������� � ������ � �������������� �� �������
{
  uint8_t elem = 0 ;
  iButtons *TMP_Sens = NULL;
  TMP_Sens = (iButtons*)START_Sens_OR;
  while (TMP_Sens )            //���� TMP_Sens ��������� �� ����� ������
  {
  TMP_Sens->Sens_OR.Number = elem;           //������������� ����� �������� � ������
  TMP_Sens = (iButtons*)TMP_Sens->next;      // ���������� ����� ����������
  elem++;                                    // ����������� ����� ��������
  }
  return elem; 
}

iButtons*Target_Sens_OR(uint8_t pass_counter,uint8_t numb_of_elem,iButtons*END_Sens_OR)// ������� ����������� ������ �������� �� �������� 
{                                                                                      //��� ���������� �����
  uint8_t num = 0;
  iButtons*TMP_Sensor = (iButtons*)END_Sens_OR;
  num =  TMP_Sensor->Sens_OR.Number;
  while (num != ((numb_of_elem - pass_counter)-1)) // ����� ������� �������� ������ ����� ������� ���������� ��������� ������ � 
  {                                            //���������� ��������
    TMP_Sensor = (iButtons*)TMP_Sensor->pref;
    num =  TMP_Sensor->Sens_OR.Number;
  }
  return TMP_Sensor;
}

uint64_t Read_ROM (void)                    // ������� ������ ROM ���� �� ������ ���������� �� ����!!!
 {
   uint64_t Tmp_Buf_ROM = 0;
   uint64_t Data_ROM = 0;
   char Bit_ROM = 0;
   for(int i=0;i<64;i++)
    {
      Bit_ROM = Sl_Read();                     // ������ ���     
      Tmp_Buf_ROM |= Bit_ROM ;                 // ���������� ��� ���������� �� ��������� �����
      Tmp_Buf_ROM <<= 63;                      // ����� �������� ����� �� �����
      Data_ROM |= Tmp_Buf_ROM;                 // ��������� ��������� ����� ��������
      if(i!=63)
       {
         Data_ROM >>= 1;                          //���� �������� � ����� ����� ���������� ���� 
       }
      Bit_ROM = 0;          
      Tmp_Buf_ROM = 0;
    }
   return Data_ROM;
 }





/************************ (C) COPYRIGHT ����������������������� *****END OF FILE****/