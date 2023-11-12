/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           Protocol.cpp
** Latest modified Date:2016-06-01
** Latest Version:      V1.0.0
** Descriptions:        Protocol interface
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Liu Zhufu
** Created date:        2016-03-14
** Version:             V1.0.0
** Descriptions:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#include "Protocol.h"
#include <stdio.h>
#include <string.h>
#include <HardwareSerial.h>

/*********************************************************************************************************
** Protocol buffer definition
*********************************************************************************************************/
#define RAW_BYTE_BUFFER_SIZE    256
#define PACKET_BUFFER_SIZE  4

// Serial
uint8_t gSerialTXRawByteBuffer[RAW_BYTE_BUFFER_SIZE];
uint8_t gSerialRXRawByteBuffer[RAW_BYTE_BUFFER_SIZE];
Packet gSerialTXPacketBuffer[PACKET_BUFFER_SIZE];
Packet gSerialRXPacketBuffer[PACKET_BUFFER_SIZE];

ProtocolHandler gSerialProtocolHandler;

/*********************************************************************************************************
** Function name:       ProtocolInit
** Descriptions:        Init the protocol buffer etc.
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void ProtocolInit(void)
{
  // Init Serial protocol
  RingBufferInit(&gSerialProtocolHandler.txRawByteQueue, gSerialTXRawByteBuffer, RAW_BYTE_BUFFER_SIZE, sizeof(uint8_t));
  RingBufferInit(&gSerialProtocolHandler.rxRawByteQueue, gSerialRXRawByteBuffer, RAW_BYTE_BUFFER_SIZE, sizeof(uint8_t));
  RingBufferInit(&gSerialProtocolHandler.txPacketQueue, gSerialTXPacketBuffer, PACKET_BUFFER_SIZE, sizeof(Packet));
  RingBufferInit(&gSerialProtocolHandler.rxPacketQueue, gSerialRXPacketBuffer, PACKET_BUFFER_SIZE, sizeof(Packet));
}

/*********************************************************************************************************
** Function name:       ProtocolProcess
** Descriptions:        Process the protocol
** Input parameters:    selectRobot, *message, expectAnswer
** Output parameters:   manipuliertes message-Objekt
** Returned value:      None
*********************************************************************************************************/
void ProtocolProcess(int selectRobot, Message *message, bool expectAnswer = false)
{
  // Methode ist zum Teil vorbereitet für zwei Robterarme, mit selectRobot kann ausgewählt werden
  Message returnMessage;
  MessageProcess(&gSerialProtocolHandler);
  if (RingBufferGetCount(&gSerialProtocolHandler.txRawByteQueue)) {
    uint8_t data;
    if (selectRobot == 0) {
      // Right robot
      while (RingBufferIsEmpty(&gSerialProtocolHandler.txRawByteQueue) == false) {
        RingBufferDequeue(&gSerialProtocolHandler.txRawByteQueue, &data);
        Serial1.write(data);
      }
    }
    else if (selectRobot == 1) {
      // Left robot
      while (RingBufferIsEmpty(&gSerialProtocolHandler.txRawByteQueue) == false) {
        RingBufferDequeue(&gSerialProtocolHandler.txRawByteQueue, &data);
        Serial2.write(data);
      }
    }
  }
  if (MessageRead(&gSerialProtocolHandler, &returnMessage) == ProtocolNoError && expectAnswer) {
    // Wenn Antwort erwartet wird. Das wird in diesem Fall nur bei Abfrage der Koordinaten
    message->id = returnMessage.id;
    message->rw = returnMessage.rw;
    message->isQueued = returnMessage.isQueued ;
    message->paramsLen = returnMessage.paramsLen;
    if (returnMessage.paramsLen) {
      memcpy(&message->params[0], &returnMessage.params[0], returnMessage.paramsLen);
    }
  }
}
