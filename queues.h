#ifndef QUEUES_H
#define QUEUES_H

#include <Arduino.h>
#include <NimBLEDevice.h>

typedef enum{
  BLE = 1,
  SER
} type;

typedef struct{
  uint8_t q[512];
  int index_top;
} queue_uint8;

typedef struct{
  int length;
  int start_byte;
  int payload1;
  int payload2;
  int comm;
  int ind_last;
  bool f_lendet;
  bool f_finish;
} packet;

extern bool deviceConnected;

inline void queue_init(queue_uint8 *queue){
  for(int i = 0; i < 512; i++){
    queue->q[i] = 0;
  }
  queue->index_top = 0;
}

inline void queue_add(queue_uint8* queue, int value){
  if(queue->index_top >= 511){
    queue_init(queue);
    return;
  } else {
    queue->q[queue->index_top] = (uint8_t)value;
    queue->index_top++;
  }
}

inline void queue_back(queue_uint8 *queue){
  if(queue->index_top > 0){
    queue->q[queue->index_top - 1] = 0;
    queue->index_top--;
  }
}

inline void queue_back_reset(queue_uint8 *queue){
  while(queue->index_top > 0){
    queue->q[queue->index_top - 1] = 0;
    queue->index_top--;
  }
}

inline void packet_init(packet *pckt){
  pckt->length = 0;
  pckt->start_byte = 0;
  pckt->comm = 0;
  pckt->payload1 = 0;
  pckt->payload2 = 0;
  pckt->ind_last = 0;
  pckt->f_lendet = false;
  pckt->f_finish = false;
}

inline void packet_cal_length(packet *pckt, queue_uint8 *queue){
  if(queue->index_top > 2){
    pckt->start_byte = queue->q[0];
    pckt->f_lendet = true;

    if(pckt->start_byte == 2){
      pckt->payload1 = queue->q[1];
      pckt->length = pckt->start_byte + pckt->payload1 + 3;
      pckt->ind_last = pckt->length - 1;
    }

    if(pckt->start_byte == 3){
      pckt->payload1 = queue->q[1];
      pckt->payload2 = queue->q[2];
      pckt->length = pckt->start_byte + (pckt->payload1 * 256) + pckt->payload2 + 3;
      pckt->ind_last = pckt->length - 1;
    }

    if(pckt->length <= 0 || pckt->length > 512){
      queue_back_reset(queue);
      packet_init(pckt);
    }
  }
}

inline void packet_printer(packet *pckt){
  Serial.printf("[%i-%i-%i-%i-%i-%i]",
                pckt->start_byte,
                pckt->length,
                pckt->payload1,
                pckt->payload2,
                pckt->ind_last,
                pckt->f_finish);
}

inline void queue_printer(queue_uint8 *queue){
  Serial.print(" [ q: ");
  for(int i = 0; i < queue->index_top; i++){
    Serial.printf("%i ", queue->q[i]);
  }
  Serial.print("]");
}

inline void packet_handler(queue_uint8 *queue,
                           packet *pckt,
                           uint8_t val,
                           int type,
                           NimBLECharacteristic* pTx){
  queue_add(queue, val);

  if(!(queue->q[0] == 2 || queue->q[0] == 3)){
    queue_back(queue);
    return;
  }

  if(queue->index_top == 3){
    packet_cal_length(pckt, queue);
  }

  if(queue->index_top == 4){
    if(pckt->start_byte >= 0 && pckt->start_byte < queue->index_top){
      pckt->comm = queue->q[pckt->start_byte];
    }
  }

  if(pckt->f_lendet && (queue->index_top == pckt->length)){
    pckt->f_finish = true;
  }

  if(pckt->f_finish){
    if(type == BLE){
      Serial1.write(queue->q, pckt->length);
      Serial.print("\n[From BLE]");
    }

    if(type == SER){
      if(deviceConnected && pTx != nullptr){
        pTx->setValue(queue->q, pckt->length);
        bool ok = pTx->notify();
        Serial.printf("\n[From SER notify=%d]", ok ? 1 : 0);
      } else {
        Serial.print("\n[From SER no-client]");
      }
    }

    packet_printer(pckt);
    queue_printer(queue);

    queue_init(queue);
    packet_init(pckt);
  }
}

#endif