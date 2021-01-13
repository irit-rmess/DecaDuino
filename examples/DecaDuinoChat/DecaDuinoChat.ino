// DecaDuinoChat
// This sketch shows how to use the DecaDuino library to send and receive ascii messages via the Serial port over
// the UWB radio
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>
#include <string.h>
#include <stdlib.h>

#define MAX_FRAME_LEN 120
uint8_t rxData[MAX_FRAME_LEN];
uint16_t rxLen;
int rxFrames;
#ifdef ARDUINO_DWM1001_DEV
DecaDuino decaduino(SS1, DW_IRQ);
#define LED_ON LOW
#define LED_OFF HIGH
#else
DecaDuino decaduino;
#define LED_ON HIGH
#define LED_OFF LOW
#endif

uint32_t dev_id;
uint64_t euid;

void promptCmd()
{
  char string[29];
  // Depends on how the standard library is implemented for platform
  // Works with DecaDuino and DWM1001-DEV
  sprintf(string, "%08lX-%08lX%08lX > ",
    dev_id,
    (uint32_t)(euid >> 32), (uint32_t)(euid & 0xFFFFFFFF)
  );
  Serial.print(string);
}

void printConfig()
{
  Serial.println("Current configuration");
  Serial.println("---------------------");
  Serial.print(" * channel = ");
  Serial.println(decaduino.getChannel());
  Serial.print(" * datarate = ");
  Serial.println(DW1000_DATARATE[decaduino.getDataRate()]);
  Serial.print(" * preamble length = ");
  Serial.println(decaduino.getPreambleLength());
}

void printHelp()
{
  Serial.println("Available commands");
  Serial.println("------------------");
  Serial.println(" * config -> prints the current configuration");
  Serial.println(" * help -> prints this message");
  Serial.println(" * channel -> prompts and sets the channel");
  Serial.println(" * datarate -> prompts and sets the datarate");
  Serial.println(" * preamble -> prompts and sets the preamble length");
}

char readChar()
{
  while (Serial.available() == 0);
  return Serial.read();
}

// ASCII Keycode
#define BACKSPACE 8
#define DEL       127
void handleChar(char c, char * buffer, int * index)
{
  if ((c == BACKSPACE || c == DEL) && *index > 0)
  {
    buffer[--(*index)] = 0;
    Serial.write(BACKSPACE);
    Serial.print(' ');
    Serial.write(BACKSPACE);
  }
  else if (isalnum(c) || isspace(c) || ispunct(c))
  {
    buffer[(*index)++] = c;
    Serial.write(c);
  }
}
 
int promptInt()
{
  char buffer[10];
  int index = 0;
  Serial.print("Please enter a number: ");
  char c = readChar();
  while (c != '\r' && c != '\n')
  {
    handleChar(c, buffer, &index);
    c = readChar();
  }
  buffer[index] = 0;
  char *end;
  long l = strtol(buffer, &end, 10);
  if (*end != '\0')
  {
    Serial.println("Error: Incorrect input");
    return -1;
  }
  Serial.println();
  return (int) l;
}

void setChannel()
{
  Serial.println("Channel configuration");
  Serial.println("---------------------");
  Serial.println(" * 1, f = 3494.4MHz, bw = 499.2MHz");
  Serial.println(" * 2, f = 3993.6MHz, bw = 499.2MHz");
  Serial.println(" * 3, f = 4492.8MHz, bw = 499.2MHz");
  Serial.println(" * 4, f = 3993.6MHz, bw = 1331.2MHz");
  Serial.println(" * 5, f = 6489.6MHz, bw = 499.2MHz");
  Serial.println(" * 7, f = 6489.6MHz, bw = 1081.6MHz");
  int i = promptInt();
  if (i != -1)
  {
    decaduino.setChannel(i);
  }
  else
  {
    Serial.println("Error: Can't set Channel");
  }
}

void setDataRate()
{
  Serial.println("DataRate configuration");
  Serial.println("----------------------");
  Serial.println(" * 1, 110 Kbps");
  Serial.println(" * 2, 850 Kbps");
  Serial.println(" * 3, 6,8 Mbps");
  int i = promptInt();
  if (i != -1)
  {
    decaduino.setDataRate((dw1000_datarate_t) i);
  }
  else
  {
    Serial.println("Error: Can't set DataRate");
  }
}

void setPreamble()
{
  Serial.println("Preamble Length configuration");
  Serial.println("-----------------------------");
  Serial.println(" * 64");
  Serial.println(" * 128");
  Serial.println(" * 256");
  Serial.println(" * 512");
  Serial.println(" * 1024");
  Serial.println(" * 1536");
  Serial.println(" * 2048");
  Serial.println(" * 4096");
  int i = promptInt();
  if (i != -1)
  {
    decaduino.setPreambleLength(i);
  }
  else
  {
    Serial.println("Error: Can't set Preamble");
  }
}
 
void handleCmd(char * cmdline)
{
   if (!strcmp("config", cmdline)) printConfig();
   else if (!strcmp("help", cmdline)) printHelp();
   else if (!strcmp("channel", cmdline)) setChannel();
   else if (!strcmp("datarate", cmdline)) setDataRate();
   else if (!strcmp("preamble", cmdline)) setPreamble();
   else Serial.println("Error: command not recognized");
}

void sendData(char * data, int size)
{
  // LED ON, disable RX, send, wait sent complete, re-enable RX and LED OFF
  digitalWrite(LED_BUILTIN, LED_ON);
  decaduino.plmeRxDisableRequest(); // Always disable RX before request frame sending
  Serial.println("Sending... ");
  decaduino.pdDataRequest((uint8_t *)data, size);
  #ifdef ARDUINO_DWM1001_DEV
  digitalWrite(LED_BLUE, LED_ON);
  #endif
  while ( !decaduino.hasTxSucceeded() )decaduino.engine();
  #ifdef ARDUINO_DWM1001_DEV
  digitalWrite(LED_BLUE, LED_OFF);
  #endif
  Serial.println("done!");
  decaduino.plmeRxEnableRequest();
  digitalWrite(LED_BUILTIN, LED_OFF);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_OFF);
  #ifdef ARDUINO_DWM1001_DEV
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, LED_OFF);
  digitalWrite(LED_BLUE, LED_OFF);
  #endif

  Serial.begin(115200); // Init Serial port

  #ifndef ARDUINO_DWM1001_DEV
  SPI.setSCK(14); // Set SPI clock pin (pin 14 on DecaWiNo board)
  #endif

  // Init DecaDuino and blink if initialisation fails
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) { digitalWrite(LED_BUILTIN, LED_ON); delay(50); digitalWrite(LED_BUILTIN, LED_OFF); delay(50); }
  }

  dev_id = decaduino.getDevID();
  euid = decaduino.getEuid();
  printConfig();

  // Set RX buffer and enable RX
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
  rxFrames = 0;
  Serial.println("DecaDuino chat ready!");
  promptCmd();
}

void loop()
{
  decaduino.engine();

  static char cmdline[MAX_FRAME_LEN];
  static int index = 0;
  // This is the sender part ****************************************************************
  // Get chars on Serial and prepare buffer
  while ( ( Serial.available() > 0 ) && ( index < MAX_FRAME_LEN ) ) {
    char c = readChar();
    if ( (c=='\n')||(c=='\r')||(index==MAX_FRAME_LEN) ) {
      Serial.println();
      cmdline[index] = 0;
      if (index > 0)
      {
        if (cmdline[0] == '/') handleCmd((char *)(cmdline + 1));
        else sendData(cmdline, index);
      }
      index = 0;
      promptCmd();
    }
    else
    {
      handleChar(c, cmdline, &index);
    }
  }

  // This is the receiver part **************************************************************
  if ( decaduino.rxFrameAvailable() ) {
    // LED ON, print received chars, re-enable RX and LED OFF
    digitalWrite(LED_BUILTIN, LED_ON);
    #ifdef ARDUINO_DWM1001_DEV
    digitalWrite(LED_GREEN, LED_ON);
    #endif
    Serial.println();
    Serial.print("["); Serial.print(++rxFrames); Serial.print("] ");
    Serial.print(rxLen);
    Serial.print("bytes received: ");
    rxData[rxLen] = 0;
    Serial.println((char*)rxData);
    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    digitalWrite(LED_BUILTIN, LED_OFF);
    #ifdef ARDUINO_DWM1001_DEV
    digitalWrite(LED_GREEN, LED_ON);
    #endif
  }
}


