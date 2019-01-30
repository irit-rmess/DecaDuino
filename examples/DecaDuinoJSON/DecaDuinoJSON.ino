#include <ArduinoJson.h>
#include "base64.hpp"
#include <SPI.h>
#include <EEPROM.h>
#include <WiNoIO.h>
#include <DecaDuino.h>

#define DEBUG_MESSAGE(...) if (debug) Serial.printf(__VA_ARGS__)

#define MAX_MESSAGE_LEN 120
DecaDuino decaduino;
WiNoIO wino;
char txBuffer[MAX_MESSAGE_LEN];
int txBufferLen = 0;
char rxBuffer[MAX_MESSAGE_LEN];
uint16_t rxBufferLen = 0;

int debug = false; //true;
int current_power = 0;
unsigned long current_frequency = 0;
int current_bandwidth = 0;
int current_dataRate = 0;
int current_preambleLength = 0;
int tx_token = 0;
char stored_nodeID[64];

uint32_t statsTimeout;
int stats_rxPacketsReceived = 0;
int stats_rxPacketsReceivedOK = 0;
int stats_txPacketsReceived = 0;
int stats_txPacketsEmitted = 0;
#define STATS_PERIOD 60000 //ms

char json_buffer[2048];
int json_buffer_len = 0;
#define JSON_TIMEOUT 200 //ms

int parseCommand ( char json[] )
{
  char json_buffer_temp[2048];
  StaticJsonBuffer<2048> jsonBuffer;
  memcpy(json_buffer_temp, json, strlen(json));
  JsonObject& root = jsonBuffer.parseObject(json_buffer_temp);

  if (!root.success()) {
    DEBUG_MESSAGE("parseObject() failed\r\n");
    return false;
  }

  const char* message_type = root["message_type"];
  if (strcmp(message_type, "DecaDuino_tx")) {
    return false;
  }

  int token = root["message"]["token"];
  const char* nodeID = root["message"]["txInfo"]["nodeID"];
  int immediately = root["message"]["txInfo"]["immediately"];
  unsigned long frequency = root["message"]["txInfo"]["frequency"];
  int power = root["message"]["txInfo"]["power"];
  int preambleLength = root["message"]["txInfo"]["preambleLength"];
  const char* modulation = root["message"]["txInfo"]["modulation"];
  int bandwidth = root["message"]["txInfo"]["UWBModulationInfo"]["bandwidth"];
  int dataRate = root["message"]["txInfo"]["UWBModulationInfo"]["dataRate"];
  const char* phyPayload = root["message"]["phyPayload"];
  unsigned char phyPayloadBytes[MAX_MESSAGE_LEN];
  unsigned int payload_length = decode_base64((unsigned char*)phyPayload, phyPayloadBytes);

  if ( debug )
  {
    // Print values for debug
    Serial.println("Decoded JSON data:");
    Serial.print("  token: ");
    Serial.println(token);
    Serial.print("  nodeID: ");
    Serial.println(nodeID);
    Serial.print("  immediately: ");
    Serial.println(immediately);
    Serial.print("  frequency: ");
    Serial.println(frequency);
    Serial.print("  power: ");
    Serial.println(power);
    Serial.print("  preambleLength: ");
    Serial.println(preambleLength);
    Serial.print("  modulation: ");
    Serial.println(modulation);
    Serial.print("  bandwidth: ");
    Serial.println(bandwidth);
    Serial.print("  dataRate: ");
    Serial.println(dataRate);
    Serial.print("  phyPayload: ");
    Serial.println(phyPayload);
    Serial.print("  phyPayloadBytes: |");
    for (unsigned int i=0; i<payload_length; i++ ) Serial.printf("%02X|", phyPayloadBytes[i]);
    Serial.printf(" (len=%dbytes)\r\n\r\n", payload_length);
  }

  if ( strcmp(nodeID, stored_nodeID) != 0 )
  {
    DEBUG_MESSAGE("This message is not destinated to %s\r\n\r\n", stored_nodeID);
    return false;
  }

  if ( frequency != 0 || bandwidth != 0 )
  {
    if ( frequency != current_frequency || bandwidth != current_bandwidth )
    {
      if ( frequency == 0 )
      {
        frequency = current_frequency;
      }
      else
      {
        DEBUG_MESSAGE("Setting frequency to %luHz\r\n\r\n", frequency);
      }

      if ( bandwidth == 0 )
      {
        bandwidth = current_bandwidth;
      }
      else
      {
        DEBUG_MESSAGE("Setting bandwidth to %dkHz\r\n\r\n", bandwidth);
      }

      switch ( frequency )
      {
        case (unsigned long) 3494400000:
          if      ( bandwidth == 499200) decaduino.setChannel(1);
          else {
            DEBUG_MESSAGE("Incompatible bandwidth\r\n");
            return false;
          }
          break;
        case (unsigned long) 3993600000:
          if      ( bandwidth == 499200) decaduino.setChannel(2);
          else if ( bandwidth == 1331200) decaduino.setChannel(4);
          else {
            DEBUG_MESSAGE("Incompatible bandwidth\r\n");
            return false;
          }
          break;
        case (unsigned long) 4492800000:
          if      ( bandwidth == 499200) decaduino.setChannel(3);
          else {
            DEBUG_MESSAGE("Incompatible bandwidth\r\n");
            return false;
          }
          break;
        case (unsigned long) 6489600000:
          if      ( bandwidth == 499200) decaduino.setChannel(5);
          else if ( bandwidth == 1081600) decaduino.setChannel(7);
          else {
            DEBUG_MESSAGE("Incompatible bandwidth\r\n");
            return false;
          }
          break;
        default:
          DEBUG_MESSAGE("%luHz is not a valid frequency\r\n\r\n", frequency);
          break;
      }
      current_frequency = frequency;
    }
  }

  if ( power != 0 )
  {
    // Command includes txPower parameter
    if ( power != current_power )
    {
      // Set transceiver transmit power
      DEBUG_MESSAGE("Setting txPower to %ddBm\r\n\r\n", power);
      DEBUG_MESSAGE("Power setting not implemented\r\n");
      //current_power = power;
    }
  }

  if ( preambleLength != 0 && preambleLength != current_preambleLength )
  {
    DEBUG_MESSAGE("Setting preamble length to %d\r\n", preambleLength);
    decaduino.setPreambleLength(preambleLength);
    current_preambleLength = preambleLength;
  }

  if ( dataRate != 0 && dataRate != current_dataRate )
  {
    DEBUG_MESSAGE("Setting data rate to %d bits/s\r\n", dataRate);
    switch ( dataRate )
    {
      case 110000:
        decaduino.setDataRate(DW1000_DATARATE_110KBPS);
        break;
      case 850000:
        decaduino.setDataRate(DW1000_DATARATE_850KBPS);
        break;
      case 6800000:
        decaduino.setDataRate(DW1000_DATARATE_6_8MBPS);
        break;
      default:
        DEBUG_MESSAGE("Incompatible data rate\r\n");
        return false;
    }
  }

  // Send frame if length != 0
  if ( payload_length != 0 )
  {
    if ( debug )
    {
      Serial.printf("New frame to send frame: |");
      for (unsigned int i=0; i<payload_length; i++ ) Serial.printf("%02X|", phyPayloadBytes[i]);
      Serial.printf(" (len=%dbytes)\r\n\r\n", payload_length);
    }
    memcpy(txBuffer, phyPayloadBytes, payload_length);
    tx_token = token;
    txBufferLen = payload_length;
    stats_txPacketsReceived++;
  }
  return true;
}


void send_stats()
{
  Serial.printf("{\"message_type\":\"DecaDuino_stats\",\"message\":{\"nodeID\":\"%s\",\"time\":\"%ld\",\"rxPacketsReceived\":%d,\"rxPacketsReceivedOK\":%d,\"txPacketsReceived\":%d,\"txPacketsEmitted\":%d}}\r\n", stored_nodeID, millis(), stats_rxPacketsReceived, stats_rxPacketsReceivedOK, stats_txPacketsReceived, stats_txPacketsEmitted );
  stats_rxPacketsReceived = 0;
  stats_rxPacketsReceivedOK = 0;
  stats_txPacketsReceived = 0;
  stats_txPacketsEmitted = 0;
  statsTimeout = millis() + STATS_PERIOD + random(STATS_PERIOD/2);
}

void setup()
{
  randomSeed(analogRead(A13));
  Serial.begin(115200);
  if (!wino.loadConfig() ) while (1) { Serial.println("WiNoIO Error"); delay(5000); }
  SPI.setSCK(wino.getTrxSckPin());
  if (!decaduino.init()) while (1) { wino.rgbDraw(255,0,0); delay(50); wino.rgbDraw(255,0,0);  delay(50); Serial.println("DecaDuino Init Error"); }
  sprintf(stored_nodeID,"%x",wino.getLabel());
  //current_power = 0;
  switch ( decaduino.getChannel() )
  {
    case 1:
      current_frequency = (unsigned long) 3494400000;
      current_bandwidth = 499200;
      break;
    case 2:
      current_frequency = (unsigned long) 3993600000;
      current_bandwidth = 499200;
      break;
    case 3:
      current_frequency = (unsigned long) 4492800000;
      current_bandwidth = 499200;
      break;
    case 4:
      current_frequency = (unsigned long) 3993600000;
      current_bandwidth = 1331200;
      break;
    case 5:
      current_frequency = (unsigned long) 6489600000;
      current_bandwidth = 499200;
      break;
    case 7:
      current_frequency = (unsigned long) 6489600000;
      current_bandwidth = 1081600;
      break;
  }
  current_preambleLength = decaduino.getPreambleLength();
  switch ( decaduino.getDataRate() )
  {
    case DW1000_DATARATE_110KBPS:
      current_dataRate = 110000;
      break;
    case DW1000_DATARATE_850KBPS:
      current_dataRate = 850000;
      break;
    case DW1000_DATARATE_6_8MBPS:
      current_dataRate = 6800000;
      break;
  }

  decaduino.setRxBuffer((uint8_t *)rxBuffer, &rxBufferLen);
  decaduino.plmeRxEnableRequest();
}

int isJsonComplete ( char json[] ) 
{
  // Analyse JSON string and return true if the number of {} and [] is coherent
  int len = strlen(json);
  int bracket_count = 0;
  int brace_count = 0;

  for (int i=0; i<len; i++)
  {
    if ( json[i] == '{' ) brace_count++;
    else if ( json[i] == '}' ) brace_count--;
    else if ( json[i] == '[' ) bracket_count++;
    else if ( json[i] == ']' ) bracket_count--;
  }

  if ( brace_count == 0 && bracket_count == 0 ) return true;
  else return false;
}

void sendData(char * data, int size)
{
  decaduino.plmeRxDisableRequest(); // Always disable RX before request frame sending
  decaduino.pdDataRequest((uint8_t *)data, size);
  while ( !decaduino.hasTxSucceeded() );
  decaduino.plmeRxEnableRequest();
}

void reset_json_buffer()
{
    json_buffer[0] = '\0';
    json_buffer_len = 0;
}

void loop()
{
  static uint32_t json_timeout = 0;

  wino.rgbDraw(0,255,0);

  if ( millis() > statsTimeout ) send_stats();
  if ( millis() > json_timeout ) reset_json_buffer();

  // Get chars from Serial
  while ( Serial.available() > 0 )
  {
    char c = Serial.read();
    
    if ( json_buffer_len == 0 )
    {
      if (c == '\r' || c == '\t' || c == ' ' || c == '\n' ) 
      {
        continue;
      }
      else 
      {
        // First char: set timeout
        json_timeout = millis() + JSON_TIMEOUT;      
      }
    }

    if (c != '\r' && c != '\t')
    {
      json_buffer[json_buffer_len++] = c;
    }

    if (json_buffer_len != 0)
    {
        // Always put a NULL char at the end of the string for the parser
        json_buffer[json_buffer_len] = '\0';
        if ( isJsonComplete(json_buffer) )
        {
          if ( parseCommand(json_buffer) == true ) reset_json_buffer();
        }
    }
  }

  // Send frame if length != 0
  if ( txBufferLen != 0 )
  {
    if ( debug )
    {
      Serial.printf("Sending frame: |");
      for ( int i=0; i<txBufferLen; i++ ) Serial.printf("%02X|", txBuffer[i]);
      Serial.printf(" (len=%dbytes)\r\n\r\n", txBufferLen);
    }
    wino.rgbDraw(255,0,0);

    sendData(txBuffer, txBufferLen);

    stats_txPacketsEmitted++;
    txBufferLen = 0;
    Serial.printf("{\"message_type\":\"DecaDuino_ack\",\"message\":{\"nodeID\":\"%s\",\"token\":%d, \"timestamp\":%ld, \"tx_dw_timestamp_hex\":\"0x", stored_nodeID, tx_token, micros());
    decaduino.printUint64(decaduino.getLastTxTimestamp());
    Serial.printf("\"p}}\r\n");
  }

  // Check incoming messages from radio
  if ( decaduino.rxFrameAvailable() )
  {
    wino.rgbDraw(0,0,0);
    unsigned char phyPayloadBase64[MAX_MESSAGE_LEN*2];
    unsigned int base64_length = encode_base64((uint8_t *)rxBuffer, rxBufferLen, phyPayloadBase64);

    if ( base64_length != 0 )
    {
      Serial.printf("{\"message_type\":\"DecaDuino_rx\",\"message\":{");
      Serial.printf("\"txInfo\":{\"frequency\":%lu,\"modulation\":\"UWB\",\"UWBModulationInfo\":{", current_frequency);
      Serial.printf("\"bandwidth\":%d,\"dataRate\":%d}},", current_bandwidth, current_dataRate);
      Serial.printf("\"rxInfo\":{\"nodeID\":\"%s\",\"timestamp\":%ld,\"rx_dw_timestamp_hex\":\"0x", stored_nodeID, micros());
      decaduino.printUint64(decaduino.getLastRxTimestamp());
      Serial.printf("\",");
      Serial.printf("\"rssi\":%d,\"UWBSNR\":%d,\"size\":%d},",-1,-1,rxBufferLen);
      Serial.printf("\"phyPayload\":\"%s\"}}\r\n", phyPayloadBase64);
    }

    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    stats_rxPacketsReceived++;
    stats_rxPacketsReceivedOK++;
  }
}
