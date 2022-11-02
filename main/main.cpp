#include "Arduino.h"
#include "device.h"

#include "db-logger.h"
#include "wifi_manager.h"
#include "power-manager.h"
#include "lora-radio.h"
#include "pixel-manager.h"

#include "button.h"

#include "radio_event_struct.h"

void button_callback(button_event_id_t type)
{
  switch (type)
  {
  case BUTTON_PRESSED:
  {
    ESP_LOGI("Button", "pressed");
    Pixel_Manager::getInstance()->set_color(255, 0, 0);
    break;
  }
  case BUTTON_RELEASED:
    ESP_LOGI("Button", "released");
    Pixel_Manager::getInstance()->set_color(0, 255, 0);
    break;
  case BUTTON_HOLD:
  {
    ESP_LOGI("Button", "hold");
    Pixel_Manager::getInstance()->set_color(255, 255, 0);

    Device *device = Device::getInstance();
    radio_event_data_t event_data = radio_event_data_t(device->get_device_id(), device->get_device_id(), device->get_device_id(), RADIO_TYPE_SEND, device->get_device_id());
    byte data[5];
    event_data.get_byte_array_data(data);
    Lora_radio::getInstance()->send_data(data, sizeof(data));
    break;
  }
  case BUTTON_DOUBLE_CLICK:
    ESP_LOGI("Button", "double click");
    Pixel_Manager::getInstance()->set_color(0, 0, 255);
    break;
  default:
    break;
  }
  return;
}
byte last_repeat_source_id = 0;

void lora_callback(byte *data, int len)
{
  ESP_LOGI("Lora", "received data");
  Pixel_Manager::getInstance()->set_color(255, 0, 255);
  radio_event_data_t event_data = radio_event_data_t(data[0], data[1], data[2], data[3], data[4]);
  event_data.print();

  if (event_data.type == RADIO_TYPE_REPEAT || event_data.type == RADIO_TYPE_SEND)
  {
    if (event_data.target_device_id == Device::getInstance()->get_device_id())
    {
      ESP_LOGI("Lora", "I'm target device id and do not send to myself");
    }
    else if (event_data.source_device_id == Device::getInstance()->get_device_id())
    {
      ESP_LOGI("Lora", "do not send to self");
    }
    else if (event_data.source_device_id == last_repeat_source_id)
    {
      ESP_LOGI("Lora", "last source id is same as current source id");
    }
    else if ((event_data.type == RADIO_TYPE_SEND || event_data.type == RADIO_TYPE_REPEAT) && event_data.source_device_id != Device::getInstance()->get_device_id())
    {
      ESP_LOGI("Lora", "receive from device %d event type %d", event_data.source_device_id, event_data.type);

      event_data.receive_from_device_id = event_data.device_id;
      event_data.device_id = Device::getInstance()->get_device_id();
      event_data.type = RADIO_TYPE_REPEAT;

      byte data[5];
      event_data.get_byte_array_data(data);
      Lora_radio::getInstance()->send_data(data, sizeof(data));
    }

    last_repeat_source_id = event_data.source_device_id;
  }
  Pixel_Manager::getInstance()->set_color(0, 255, 0);
  return;
}

void setup()
{
  Device::getInstance()->begin();

  DB_Logger::init();
  // DB_Logger::clear_log_file();

  WiFi_Manager::init();
  WiFi_Manager::connect_to_wifi("lisod", "lisod.ua");

  Power_Manager::getInstance()->begin(Device::getInstance()->get_device_number());
  DB_Logger::log(ESP_LOG_INFO, "Power Manager initialized");

  Lora_radio::getInstance()->begin();
  Lora_radio::getInstance()->set_callback(lora_callback);
  DB_Logger::log(ESP_LOG_INFO, "Lora radio initialized");

  Pixel_Manager::getInstance()->begin();
  DB_Logger::log(ESP_LOG_INFO, "Pixel manager initialized");

  if (CONFIG_PIN_BUTTON != -1)
  {
    Button::getInstance()->begin(button_callback);
    DB_Logger::log(ESP_LOG_INFO, "Button initialized");
  }
}

void loop()
{
  delay(3000);
  last_repeat_source_id = 0;
}
