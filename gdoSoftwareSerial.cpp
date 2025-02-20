/****************************************************************************
 * Wrapper for GDOLIB to use EspSoftwareSerial
 *
 * Copyright (c) 2023-25 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 */
#ifdef USE_GDOLIB_SWSERIAL
// skip entire file if not compiling with s/w serial
#include "gdo_priv.h"
#include "SoftwareSerial.h"
#include "gdoSoftwareSerial.h"

static const uint8_t SECPLUS2_CODE_LEN = 19;
static const uint32_t SECPLUS2_PREAMBLE = 0x00550100;

struct TaskParams
{
    gpio_num_t rxPin;
    gpio_num_t txPin;
    MessageBufferHandle_t txBuffer;
    QueueHandle_t rxQueue;
};
typedef struct TaskParams TaskParams_t;

enum SecPlus2ReaderMode : uint8_t
{
    SCANNING,
    RECEIVING,
};

class SecPlus2Reader
{
private:
    bool m_is_reading = false;
    uint32_t m_msg_start = 0;
    size_t m_byte_count = 0;
    uint8_t m_rx_buf[SECPLUS2_CODE_LEN] = {0x55, 0x01, 0x00};
    SecPlus2ReaderMode m_mode = SCANNING;
    const char *TAG = "ratgdo-reader";

public:
    SecPlus2Reader() = default;

    bool push_byte(uint8_t inp)
    {
        bool msg_ready = false;

        switch (m_mode)
        {
        case SCANNING:
            m_msg_start <<= 8;
            m_msg_start |= inp;
            m_msg_start &= 0x00FFFFFF;

            if (m_msg_start == SECPLUS2_PREAMBLE)
            {
                m_byte_count = 3;
                m_mode = RECEIVING;
            }
            break;

        case RECEIVING:
            m_rx_buf[m_byte_count] = inp;
            m_byte_count += 1;

            if (m_byte_count == SECPLUS2_CODE_LEN)
            {
                m_mode = SCANNING;
                m_msg_start = 0;
                msg_ready = true;
            }
            break;
        }
        return msg_ready;
    };

    uint8_t *fetch_buf(void)
    {
        return m_rx_buf;
    }
};

TaskHandle_t serial_task_handle;
TaskParams_t serial_task_params;
SoftwareSerial swSerial;
SecPlus2Reader reader;
gdo_protocol_type_t serial_protocol = GDO_PROTOCOL_UNKNOWN;

/***************************** LOCAL FUNCTION DECLARATIONS ****************************/
void serial_task(TaskParams_t *arg);

esp_err_t serial_start(QueueHandle_t rxQueue, MessageBufferHandle_t *txBuffer, TaskHandle_t *serialTask, gpio_num_t rxPin, gpio_num_t txPin)
{
    // rxQueue is passed into us, that is where we will send received packets
    // txBuffer we will create and return, that is where packets to transmit will be sent to us

    if (!txBuffer || !serialTask)
        return ESP_ERR_INVALID_ARG;

    *txBuffer = xMessageBufferCreate(sizeof(serial_transmit_t) + 8);

    if (!*txBuffer)
        return ESP_ERR_NO_MEM;

    serial_task_params.txBuffer = *txBuffer;
    serial_task_params.rxQueue = rxQueue;
    serial_task_params.rxPin = rxPin;
    serial_task_params.txPin = txPin;

    // Assume Sec+1.0 protocol to start
    swSerial.begin(1200, SWSERIAL_8E1, rxPin, txPin, true);
    serial_set_protocol(serial_protocol);

    // Create task to loop on checking for serial port data.
    // High priority as it needs to handle in real time, pin to CPU 1 so does not share with HomeKit/WiFi/mdns/etc.
#ifdef CONFIG_FREERTOS_UNICORE
    if (xTaskCreate((TaskFunction_t)serial_task, "serial_task", 4096, &serial_task_params, 15, &serial_task_handle) != pdPASS)
#else
    if (xTaskCreatePinnedToCore((TaskFunction_t)serial_task, "serial_task", 4096, &serial_task_params, 15, &serial_task_handle, 1) != pdPASS)
#endif
    {
        serial_stop();
        return ESP_ERR_NO_MEM;
    }
    *serialTask = serial_task_handle;

    // Use modified EspSoftwareSerial that will notify us if a byte is available
    // to read. This avoids need to have task spin in a tight loop polling for data.
    swSerial.enableNotify(serial_task_handle);
    return ESP_OK;
}

esp_err_t serial_stop()
{
    if (!swSerial)
        return ESP_FAIL;
    swSerial.end();
    if (serial_task_handle)
    {
        vTaskDelete(serial_task_handle);
        serial_task_handle = NULL;
    }
    if (serial_task_params.txBuffer)
    {
        vMessageBufferDelete(serial_task_params.txBuffer);
        serial_task_params.txBuffer = NULL;
    }
    return ESP_OK;
}

esp_err_t serial_set_protocol(gdo_protocol_type_t protocol)
{
    serial_protocol = protocol;
    ESP_LOGI(TAG, "Set serial port protocol to %s", gdo_protocol_type_to_string(protocol));
    if (protocol == GDO_PROTOCOL_SEC_PLUS_V2)
    {
        swSerial.begin(9600, SWSERIAL_8N1);
        swSerial.enableIntTx(false);
        swSerial.enableAutoBaud(true); // found in ratgdo/espsoftwareserial branch autobaud
    }
    else
    {
        swSerial.begin(1200, SWSERIAL_8E1);
        swSerial.enableAutoBaud(false);
    }
    swSerial.flush();
    return ESP_OK;
}

void serial_task(TaskParams *arg)
{
    int64_t last_rx;
    static const uint8_t RX_LENGTH = 2; // for Sec+1.0
    uint8_t rxPacket[RX_LENGTH];        // for Sec+1.0
    uint16_t byteCount = 0;             // for Sec+1.0
    bool readingMsg = false;            // for Sec+1.0
    serial_event_t serial_event;        // for received packets
    serial_transmit_t txPacket;         // for outbound packets

    ESP_LOGI(TAG, "Starting GDO software serial task.");
    for (;;)
    {
        while (swSerial.available())
        {
            // data is available on the serial port, read it.
            uint8_t serialByte = swSerial.read();
            last_rx = esp_timer_get_time();
            if (serial_protocol == GDO_PROTOCOL_SEC_PLUS_V2)
            {
                //--------------- Sec+2.0 ----------------
                if (reader.push_byte(serialByte))
                {
                    // Full packet received, send it to rxQueue
                    serial_event.event = SERIAL_EVENT_DATA;
                    serial_event.size = SECPLUS2_CODE_LEN;
                    memcpy(serial_event.packet, reader.fetch_buf(), SECPLUS2_CODE_LEN);
                    if (xQueueSendToBack(arg->rxQueue, &serial_event, 0) == pdFALSE)
                    {
                        // Queue is full, cannot send, log error and ignore.
                        ESP_LOGE(TAG, "Unable to queue RX packet");
                    }
                }
            }
            else
            {
                //--------------- Sec+1.0 ----------------
                if (swSerial.readParity() != swSerial.parityEven(serialByte))
                {
                    readingMsg = false;
                    byteCount = 0;
                    ESP_LOGW(TAG, "Parity error 0x%02X", serialByte);
                    continue;
                }

                bool gotMessage = false;
                if (!readingMsg)
                {
                    // valid?
                    if (serialByte >= 0x30 && serialByte <= 0x3A)
                    {
                        rxPacket[0] = serialByte;
                        byteCount = 1;
                        readingMsg = true;
                    }
                    // is it single byte command?
                    if (serialByte >= 0x30 && serialByte <= 0x37)
                    {
                        rxPacket[1] = 0;
                        readingMsg = false;
                        gotMessage = true;
                    }
                }
                else
                {
                    // save next byte
                    rxPacket[byteCount++] = serialByte;
                    if (byteCount == RX_LENGTH)
                    {
                        readingMsg = false;
                        gotMessage = true;
                    }

                    if (gotMessage == false && (esp_timer_get_time() - last_rx) > (100 * 1000))
                    {
                        ESP_LOGW(TAG, "RX message timeout");
                        // if we have a partial packet and it's been over 100ms since last byte was read,
                        // the rest is not coming (a full packet should be received in ~20ms),
                        // discard it so we can read the following packet correctly
                        readingMsg = false;
                        byteCount = 0;
                    }
                }

                if (gotMessage)
                {
                    serial_event.event = SERIAL_EVENT_DATA;
                    serial_event.size = byteCount;
                    serial_event.packet[0] = rxPacket[0];
                    serial_event.packet[1] = rxPacket[1];
                    if (xQueueSendToBack(arg->rxQueue, &serial_event, 0) == pdFALSE)
                    {
                        // Queue is full, cannot send, log error and ignore.
                        ESP_LOGE(TAG, "Unable to queue RX packet");
                    }
                    byteCount = 0;
                }
            }
        }

        // We have read all data from serial port.  Now look and see if there is a packet to send.
        if (xMessageBufferReceive(arg->txBuffer, &txPacket, sizeof(txPacket), 0))
        {
            esp_err_t err = ESP_OK;
            // If size > 1 then it must be Sec+2.0.
            if (txPacket.size > 1)
            {
                //--------------- Sec+2.0 ----------------
                gpio_set_level(arg->txPin, 1);
                ets_delay_us(1300);
                gpio_set_level(arg->txPin, 0);
                ets_delay_us(130);

                // check to see if anyone else is continuing to assert the bus after we have released it
                if (gpio_get_level(arg->rxPin))
                {
                    err = ESP_FAIL;
                    ESP_LOGE(TAG, "Collision detected trying to send packet");
                    // There is a test for this in the sending gdolib code and it requeues, so this may never occur.
                }
                else
                {
                    swSerial.enableRx(false); // Disable RX so our own packet is not immediately read back.
                    swSerial.write(txPacket.packet, txPacket.size);
                    ets_delay_us(100); // Why is this needed?
                    swSerial.enableRx(true);
                }
            }
            else
            {
                //--------------- Sec+1.0 ----------------
                if (gpio_get_level(arg->rxPin))
                {
                    err = ESP_FAIL;
                    ESP_LOGE(TAG, "Collision detected trying to send packet");
                    // There is a test for this in the sending gdolib code and it requeues, so this may never occur.
                }
                else
                {
                    swSerial.write(*txPacket.packet);
                }
            }
            // Notify sending task that packet has been sent
            xTaskNotify(txPacket.sendingTask, err, eSetValueWithOverwrite);
        }

        if (!swSerial.available())
        {
            // Wait until we are notified of either a packet to send (by gdolib)
            // or that a byte is available to read (by SoftwareSerial)
            // vTaskDelay(pdMS_TO_TICKS(2));
            if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(1000)) == pdFALSE)
            {
                if (serial_protocol == GDO_PROTOCOL_SEC_PLUS_V2)
                    // Sec+2.0 does not have heartbeat on serial, so it always times out.
                    ESP_LOGV(TAG, "Timeout in xTaskNotifyWait");
                else
                    // Sec+1.0 timeout is likely to indicate a problem
                    ESP_LOGW(TAG, "Timeout in xTaskNotifyWait");
            }
        }
    }
}
#endif
