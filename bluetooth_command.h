#ifndef BLUETOOTH_COMMAND_H
#define BLUETOOTH_COMMAND_H

#include <stdint.h>

void BluetoothCommand_Init(void);
uint8_t BluetoothCommand_ConsumeLapCommand(uint8_t *laps);
void BluetoothCommand_SendLapCommand(uint8_t laps);
void BluetoothCommand_SendAccepted(uint8_t laps);
void BluetoothCommand_SendFinished(uint8_t laps);

/* CCS调试时只需观察这三个变量。 */
extern volatile uint8_t g_bluetoothLastByte;
extern volatile uint8_t g_bluetoothLapCommand;
extern volatile uint32_t g_bluetoothRxCount;

#endif
