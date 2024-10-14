#include <SPI.h>
#include <mcp2515.h>
#include <TimerOne.h>

// Definimos el pin de interrupcion
#define INT_PIN 2

#define buffsize 252// 14 cada mensaje


byte buffer_CAN[buffsize];
byte Irx=0;
byte Itx=0;


struct can_frame canMsg;
struct can_frame canMsg_TX;

MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);
  SPI.begin();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  canMsg_TX.can_id  = 0x7DF;
  canMsg_TX.can_dlc = 8;
  canMsg_TX.data[0] = 0x02;
  canMsg_TX.data[1] = 0x01;
  canMsg_TX.data[2] = 0x0D;
  canMsg_TX.data[3] = 0x00;
  canMsg_TX.data[4] = 0x00;
  canMsg_TX.data[5] = 0x00;
  canMsg_TX.data[6] = 0x00;
  canMsg_TX.data[7] = 0x00;

  // Paramos todas las interrupciones antes de contigurar un timer
  noInterrupts();
  
  Timer1.initialize(100000);         // Dispara cada 250 ms
  Timer1.attachInterrupt(ISR_PIT); // Activa la interrupcion y la asocia a ISR_PIT

  // Inicializamos el pin de interrupcion como entrada con pullup 
  pinMode(INT_PIN, INPUT_PULLUP);
  // Adjuntamos la funcion "inter" al pint de interrupcion para que dispare la interrupcion en LOW
  attachInterrupt(digitalPinToInterrupt(INT_PIN), interCANRX, LOW);
  
  Serial.println("---CAN Read---");
  Serial.flush();
  
  // Activamos interrupciones nuevamente
  interrupts();
}

void loop() {
    /*while(Itx!=Irx){
        Serial.flush();
        Serial.write(buffer_CAN[Itx]);
        Itx++;
        Itx=Itx%buffsize;
    }*/
}



// Funcion a ejecutar cuando la interrupcion se active
void interCANRX() {
  byte temp=0;
  unsigned int To=0;
  
  
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      

      

      //if(canMsg.can_id==513){
      To=micros();
  Serial.println(To);
/*
      temp=0x00FF &  canMsg.can_id  >> 8;
      buffer_CAN[Irx]=temp;
      Irx++;
      temp=0x00FF &  canMsg.can_id ;
      buffer_CAN[Irx]=temp;
      Irx++;          
      
      buffer_CAN[Irx]=canMsg.data[0];
      Irx++;
      buffer_CAN[Irx]=canMsg.data[1];
      Irx++;
      buffer_CAN[Irx]=canMsg.data[2];
      Irx++;
      buffer_CAN[Irx]=canMsg.data[3];
      Irx++;
      buffer_CAN[Irx]=canMsg.data[4];
      Irx++;
      buffer_CAN[Irx]=canMsg.data[5];
      Irx++;
      buffer_CAN[Irx]=canMsg.data[6];
      Irx++;
      buffer_CAN[Irx]=canMsg.data[7];
      Irx++;

      temp=0x00FF &  To  >> 8;
      buffer_CAN[Irx]=temp;
      Irx++;
      temp=0xFF &  To ;
      buffer_CAN[Irx]=temp;
      Irx++;
      
      buffer_CAN[Irx]=13;
      Irx++;
      buffer_CAN[Irx]=10;

      Irx=0;

      
      Serial.write(buffer_CAN,14);
      Serial.flush();*/
      
      /*Irx++; 
      
      Irx=Irx%buffsize;*/
      
    // }
      
    /*Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }
    Serial.println();*/
  }
}

void ISR_PIT(){   
  unsigned long To=0;
  To=micros();
      Serial.println(To);
    //mcp2515.sendMessage(&canMsg_TX);// SOLO ENVIA VARIOS SI ESTA CONECTADO AL RECEPTOR
}
