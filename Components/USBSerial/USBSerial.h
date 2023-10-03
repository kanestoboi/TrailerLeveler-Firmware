#ifndef SERIAL_H
#define SERIAL_H

void USBSerial_Init(void);
void USBSerial_Process();

void USBSerial_Write();
void USBSerial_WriteByte(uint8_t data);
void USBSerial_WriteBlocking(uint8_t *data, uint16_t length);

#endif