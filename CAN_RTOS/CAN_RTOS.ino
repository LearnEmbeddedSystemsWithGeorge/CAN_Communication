// Include ChibiOS library for AVR microcontrollers
#include <ChibiOS_AVR.h> // Note: This is a port of ChibiOS to Arduino
// Include CAN libraries
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

// Function prototypes
void UART_Init(void);
void CAN_Init(void);


// Stack size to be assigned to the thread
// More info http://chibios.sourceforge.net/docs3/rt/group__threads.html
static THD_WORKING_AREA(waTh1, 100); // Working area 1 of 100 bytes
static THD_WORKING_AREA(waTh2, 200); // Working area 2 of 200 bytes
static THD_WORKING_AREA(waTh3, 200); // Working area 3 of 200 bytes

// Structure to pass to threads
struct threadData
{
  int _blinkTime;
  int _lightPin;
};

// Global variables to be used with CAN messages
char myChar;
char toggleSTM32LED;

// Blink LED thread function
static THD_FUNCTION (blinkerThread, arg)
{
  //setup thread vars
  threadData *thisData  = (threadData*)arg;
  int lightPin          = thisData->_lightPin;
  int blinkTime         = thisData->_blinkTime;

  //set the LED pinMode
  pinMode(lightPin, OUTPUT);

  while(1)
  {
    //blink
    digitalWrite(lightPin, HIGH);
    chThdSleep(blinkTime);
    digitalWrite(lightPin, LOW);
    chThdSleep(blinkTime);
  }
}

// Switch STM32 LED thread
static THD_FUNCTION (ledSwitchThread, arg)
{
  threadData *thisData  = (threadData*)arg;
  int lightPin          = thisData->_lightPin;
  tCAN message;
  
  pinMode(lightPin, OUTPUT);
  digitalWrite(lightPin, HIGH);
  
  while(1)
  {
    if (Serial.available())
    {
      myChar = Serial.read();

      if(myChar == 'b')
      {
        toggleSTM32LED = 0x10;
      }
      if(myChar == 'n')
      {
        toggleSTM32LED = 0x00;
      }
    }

    // Check if we have STM32 messages on CAN
    if (mcp2515_check_message()) // Check for any messages
    {
      if (mcp2515_get_message(&message)) //Get the message
      {
        if(message.id == 0x101U || message.id == 0x103U) // Check the ID of message
        {
          Serial.print("ID: ");
          Serial.print(message.id,HEX);
          Serial.print(", ");
          Serial.print("DLC: ");
          Serial.print(message.header.length,DEC);
          Serial.print(", ");
          Serial.print("Data: ");
          for(int i=0;i<message.header.length;i++)
          { 
            Serial.print(message.data[i],HEX);
            Serial.print(" ");
          }
          Serial.println("Message received!");
        }
      }
    }

    // Execute every 500ms
    chThdSleep(500);
  }
}

// Send CAN message thread
static THD_FUNCTION (messageSendThread, arg)
{
  tCAN message;
  
  while(1)
  {
    message.id              = 0x631; //formatted in HEX
    message.header.rtr      = 0;
    message.header.length   = 8; //formatted in DEC

    if(toggleSTM32LED == 0x10)
      message.data[0] = 0x50;
    else
      message.data[0] = 0x40;
      
    message.data[1] = 0x05;
    message.data[2] = 0x30;
    message.data[3] = 0xFA; //formatted in HEX
    message.data[4] = 0xBE;
    message.data[5] = 0x40;
    message.data[6] = 0x30;
    message.data[7] = 0x22;
    
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    mcp2515_send_message(&message);
    
    Serial.println("CAN message was sent");
    
    chThdSleep(500);
  }
}

//-------------- UART INIT -----------
void UART_Init(void)
{
  // Init serial communication at 9600 baud
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  
  Serial.println("Serial is running!");

  // Just a delay
  delay(300);
}


//-------------- CAN INIT -----------
void CAN_Init(void)
{
  //Init MCP2515 CAN controller to 500KBaud
  if(Canbus.init(CANSPEED_500))
  {
    Serial.println("CAN Init ok");
  }
  else
  {
    Serial.println("Can't init CAN");
  }

  // Just a delay
  delay(300);
}

//------------- ChibiOS SETUP -------------------------------
void chSetup()
{
  // Create thread data set 1
  threadData set1;
  set1._lightPin = 7; // D7 PIN
  set1._blinkTime = 300;
  
  // Create thread data set 2
  threadData set2;
  set2._lightPin = 8; // D8 PIN


  // Schedule blinkerThread 
  chThdCreateStatic(waTh1, sizeof(waTh1), NORMALPRIO, blinkerThread, (void*)&set1);

  // Schedule ledSwitchThread
  chThdCreateStatic(waTh2, sizeof(waTh2), NORMALPRIO, ledSwitchThread, (void*)&set2);

  // Schedule messageSendThread
  chThdCreateStatic(waTh3, sizeof(waTh3), NORMALPRIO, messageSendThread, NULL);
  
  // IDLE
  while(1)
  {
    chThdSleep(10000);
  }
}

//--------------------- Arduino SETUP ----------------------------
void setup()
{
  UART_Init();
  CAN_Init();
  
  // initialize and start ChibiOS
  chBegin(chSetup); // Spawn point of ChibiOS
  
  // should not return
  while(1);
}

//-------------------- Arduino LOOP ----------------------------
void loop()
{
  /* not used */
}
