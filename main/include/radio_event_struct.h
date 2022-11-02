#include <Arduino.h>

#ifndef RADIO_EVENT_STRUCT_H_
#define RADIO_EVENT_STRUCT_H_

typedef enum
{
    RADIO_TYPE_SEND = 1,
    RADIO_TYPE_REPEAT = 2
} radio_type_t;

struct radio_event_data_t
{
    byte device_id;
    byte source_device_id;
    byte receive_from_device_id;
    byte type;
    byte target_device_id;

    radio_event_data_t(byte device_id, byte source_device_id, byte receive_from_device_id, byte type, byte target_device_id)
    {
        this->device_id = device_id;
        this->source_device_id = source_device_id;
        this->receive_from_device_id = receive_from_device_id;
        this->type = type;
        this->target_device_id = target_device_id;
    }

    byte *get_byte_array_data(byte *data)
    {
        data[0] = this->device_id;
        data[1] = this->source_device_id;
        data[2] = this->receive_from_device_id;
        data[3] = this->type;
        data[4] = this->target_device_id;
        return data;
    }

    void print()
    {
        ESP_LOGI("Radio Event", "device_id: %d, source_device_id: %d, receive_from_device_id: %d, type: %d, target_device_id: %d", device_id, source_device_id, receive_from_device_id, type, target_device_id);
    }

    void to_json_string(String &json) const
    {
        json = "{";
        json += "\"device_id\":" + String(device_id) + ",";
        json += "\"source_device_id\":" + String(source_device_id) + ",";
        json += "\"receive_from_device_id\":" + String(receive_from_device_id) + ",";
        json += "\"type\":" + String(type) + ",";
        json += "\"target_device_id\":" + String(target_device_id);
        json += "}";
    }

    void to_json_char_array(char *json) const
    {
        sprintf(json, "{\"device_id\":%d,\"source_device_id\":%d,\"receive_from_device_id\":%d,\"type\":%d,\"target_device_id\":%d}", device_id, source_device_id, receive_from_device_id, type, target_device_id);
    }
};

#endif // RADIO_EVENT_STRUCT_H_