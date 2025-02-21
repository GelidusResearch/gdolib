/****************************************************************************
 * Wrapper for GDOLIB to use EspSoftwareSerial
 *
 * Copyright (c) 2023-25 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 */

#ifndef _GDOSOFTWARESERIAL_H
#define _GDOSOFTWARESERIAL_H
#include <inttypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

    extern esp_err_t serial_start(QueueHandle_t rxQueue, MessageBufferHandle_t *txBuffer, TaskHandle_t *serialtask, gpio_num_t rxPin, gpio_num_t txPin);
    extern esp_err_t serial_stop();
    extern esp_err_t serial_set_protocol(gdo_protocol_type_t protocol);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // _GDOSOFTWARESERIAL_H
