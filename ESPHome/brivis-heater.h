#include "esphome.h"
#define get_briviscustomcomponent(constructor) static_cast<BrivisCustomComponent *>(const_cast<custom::CustomClimateConstructor *>(&constructor)->get_climate(0))

//#define LISTENONLY

class BrivisCustomComponent : public Component, public UARTDevice, public Climate {
  private:
    const char* TAG = {"brivis"};

    const uint16_t table_byte[256] = {
      0x0000, 0x8005, 0x800f, 0x000a, 0x801b, 0x001e, 0x0014, 0x8011, 0x8033, 0x0036,
      0x003c, 0x8039, 0x0028, 0x802d, 0x8027, 0x0022, 0x8063, 0x0066, 0x006c, 0x8069,
      0x0078, 0x807d, 0x8077, 0x0072, 0x0050, 0x8055, 0x805f, 0x005a, 0x804b, 0x004e,
      0x0044, 0x8041, 0x80c3, 0x00c6, 0x00cc, 0x80c9, 0x00d8, 0x80dd, 0x80d7, 0x00d2,
      0x00f0, 0x80f5, 0x80ff, 0x00fa, 0x80eb, 0x00ee, 0x00e4, 0x80e1, 0x00a0, 0x80a5,
      0x80af, 0x00aa, 0x80bb, 0x00be, 0x00b4, 0x80b1, 0x8093, 0x0096, 0x009c, 0x8099,
      0x0088, 0x808d, 0x8087, 0x0082, 0x8183, 0x0186, 0x018c, 0x8189, 0x0198, 0x819d,
      0x8197, 0x0192, 0x01b0, 0x81b5, 0x81bf, 0x01ba, 0x81ab, 0x01ae, 0x01a4, 0x81a1,
      0x01e0, 0x81e5, 0x81ef, 0x01ea, 0x81fb, 0x01fe, 0x01f4, 0x81f1, 0x81d3, 0x01d6,
      0x01dc, 0x81d9, 0x01c8, 0x81cd, 0x81c7, 0x01c2, 0x0140, 0x8145, 0x814f, 0x014a,
      0x815b, 0x015e, 0x0154, 0x8151, 0x8173, 0x0176, 0x017c, 0x8179, 0x0168, 0x816d,
      0x8167, 0x0162, 0x8123, 0x0126, 0x012c, 0x8129, 0x0138, 0x813d, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811f, 0x011a, 0x810b, 0x010e, 0x0104, 0x8101, 0x8303, 0x0306,
      0x030c, 0x8309, 0x0318, 0x831d, 0x8317, 0x0312, 0x0330, 0x8335, 0x833f, 0x033a,
      0x832b, 0x032e, 0x0324, 0x8321, 0x0360, 0x8365, 0x836f, 0x036a, 0x837b, 0x037e,
      0x0374, 0x8371, 0x8353, 0x0356, 0x035c, 0x8359, 0x0348, 0x834d, 0x8347, 0x0342,
      0x03c0, 0x83c5, 0x83cf, 0x03ca, 0x83db, 0x03de, 0x03d4, 0x83d1, 0x83f3, 0x03f6,
      0x03fc, 0x83f9, 0x03e8, 0x83ed, 0x83e7, 0x03e2, 0x83a3, 0x03a6, 0x03ac, 0x83a9,
      0x03b8, 0x83bd, 0x83b7, 0x03b2, 0x0390, 0x8395, 0x839f, 0x039a, 0x838b, 0x038e,
      0x0384, 0x8381, 0x0280, 0x8285, 0x828f, 0x028a, 0x829b, 0x029e, 0x0294, 0x8291,
      0x82b3, 0x02b6, 0x02bc, 0x82b9, 0x02a8, 0x82ad, 0x82a7, 0x02a2, 0x82e3, 0x02e6,
      0x02ec, 0x82e9, 0x02f8, 0x82fd, 0x82f7, 0x02f2, 0x02d0, 0x82d5, 0x82df, 0x02da,
      0x82cb, 0x02ce, 0x02c4, 0x82c1, 0x8243, 0x0246, 0x024c, 0x8249, 0x0258, 0x825d,
      0x8257, 0x0252, 0x0270, 0x8275, 0x827f, 0x027a, 0x826b, 0x026e, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822f, 0x022a, 0x823b, 0x023e, 0x0234, 0x8231, 0x8213, 0x0216,
      0x021c, 0x8219, 0x0208, 0x820d, 0x8207, 0x0202
    };

    sensor::Sensor *sensor_{nullptr};
    text_sensor::TextSensor *text_sensor_{nullptr};


    static const uint8_t Controller = 0x21;
    static const uint8_t Furnace = 0x31;

    bool rxReady = false;
    uint8_t rxPacket[13];
    unsigned long txStart = 0;
    bool txReady = false;
    bool txBusy = false;
    uint8_t txPacket[13];
    uint8_t txPos = 0;
    bool gotAck = false;
    int update = 0; // 0=no change, 1=fan speed change, 2=heat on, 3=heat off
    uint8_t tempCompare = 0;


    uint16_t Crc16umtsByte(uint16_t crc, uint8_t const *data, size_t len)
    {
      if (data == NULL)
        return 0;
      for (size_t i = 0; i < len; i++) {
        crc = (crc << 8) ^
          table_byte[((crc >> 8) ^ data[i]) & 0xff];
      }
      return crc;
    }

    void PrintBuffer(const uint8_t *buffer, int length)
    {
      char str[80];
      char hex[3];
      uint16_t crc;

      if (length < 3) {
        strcpy(str,"Sh");
      } else if (length == -1) {
        strcpy(str,"Ov");
        length = 20;
      } else if (buffer[0] != length - 1) {
        strcpy(str,"Ln");
      } else {
        crc = Crc16umtsByte(0x0000, buffer, length);
        if (crc) {
          strcpy(str,"Er");
        } else {
          strcpy(str, "Ok");
        }
      }

      for (int i = 0; i < length; i++) {
        strcat(str, ",");
        sprintf(hex, "%02X", buffer[i]);
        strcat(str, hex);
      }

      ESP_LOGD(TAG, str);
      text_sensor_->publish_state(str);
    }

    void CheckBuffer(const uint8_t *buffer, int length)
    {
      char str[80];
      char hex[3];
      uint16_t crc;

      if (length < 3) {
        return;
      } else if (length == -1) {
        return;
      } else if (buffer[0] != length - 1) {
        return;
      } else {
        crc = Crc16umtsByte(0x0000, buffer, length);
        if (crc) {
          return;
        }
      }

      memcpy(rxPacket, buffer, buffer[0]);
      rxReady = true;
    }

    void SendPacket(uint8_t length, uint8_t data[], bool broadcast) {
#ifndef LISTENONLY
      uint8_t buffer[13];
      uint16_t crc;

      buffer[0] = length + 4; // this is the length of included data (src,dst, payload, crc16)
      buffer[1] = Controller; // this is my address
      buffer[2] = broadcast ? 0 : Furnace; // this is the destination address
      for (uint8_t i = 0; i < length; i++) { // add the payload
        buffer[3 + i] = data[i];
      }
      crc = Crc16umtsByte(0x0000, buffer, 3 + length); // add the crc16
      buffer[3 + length] = crc >> 8;
      buffer[4 + length] = crc & 0xFF;

      memcpy(txPacket, buffer, 5 + length);
      txReady = true;
//      PrintBuffer(buffer, 5 + length);
#endif
    }

    void ProcessPacket() {
      uint8_t AckBuf[] = {0x0FF};

      if ((rxPacket[1] == Controller) && (rxPacket[2] == Furnace)) { // Controller to Furnace

      } else if  ((rxPacket[1] == Furnace) && (rxPacket[2] == Controller)) { // Furnace to Controller
        switch (rxPacket[3]) {
          case 0x01: // Furnace searching for Controller
            this->cancel_timeout("InitTimeout");
            ESP_LOGD(TAG, "init ack");
            SendPacket(1, AckBuf, false);
            break;

          case 0x05: // Furnace sending status
            ESP_LOGD(TAG, "furnace status report");
            SendPacket(1, AckBuf, false);
            break;

          case 0xFB: // Furnace status request - reponse
            gotAck = true;
            ESP_LOGD(TAG, "received furnace status");
            break;

          case 0xFF: // Acknowlegement
            gotAck = true;
            ESP_LOGD(TAG, "received Ack");
            break;
        }
      }
      rxReady = false;
    }

  public:
    BrivisCustomComponent(UARTComponent *parent) : UARTDevice(parent) {}

    void ReceiveLoop() {
      const int max_line_length = 20;
      static uint8_t buffer[max_line_length];
      static int len = 0;
      static unsigned long last = 0;
      unsigned long now = millis();

      if (len && (now - last > 15) && !available()) {
        len = 0;
        return;
      }

      while (available()) {
        uint8_t c;
        last = now;
        read_byte(&c);
        buffer[len++] = c;
        if (len == 20) {
          PrintBuffer(buffer, -1);
          len = 0;
          continue;
        }
        if (len == buffer[0] + 1) {
          PrintBuffer(buffer, len);
          CheckBuffer(buffer, len);
          txStart = now + 50;
          len = 0;
        }
      }
    }

    void SendLoop() {
      unsigned long now = millis();

      if ((txReady) && (now > txStart)) {
        if (txPos == 0) {
          ESP_LOGD(TAG, "Ack");
        }
        txBusy = true;
        txReady = false;
      }

      if (txBusy) {
        write_byte(txPacket[txPos++]);
        if (txPos > (txPacket[0])) {
          txPos = 0;
          txBusy = false;
        }
      }
    }

    const void SendInit() {
#ifndef LISTENONLY
      uint8_t InitBuf[] = {0x19};

      ESP_LOGD(TAG, "init");
      this->set_timeout("InitTimeout", 1000, [this] { this->InitTimeout(); });
      SendPacket(1, InitBuf, true);
#endif
    }

    const void SendIdle() {
#ifndef LISTENONLY
      uint8_t IdleBuf[] = {0x08};
      gotAck = false;

      ESP_LOGD(TAG, "idle");
      SendPacket(1, IdleBuf, false);
#endif
    }

    const void SendFanSpeed() {
#ifndef LISTENONLY
      const uint8_t speeds[17] = {0x00, 0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
      uint8_t speed = 0;
      if (this->mode == climate::CLIMATE_MODE_FAN_ONLY) {
        ESP_LOGD(TAG, "Fan: %s", this->custom_fan_mode->c_str());
        speed = atoi(this->custom_fan_mode->c_str());
      } else {
        ESP_LOGD(TAG, "Fan: OFF");
      }

      uint8_t FanBuf[] = {0x1E, speeds[speed]};
      gotAck = false;

      ESP_LOGD(TAG, "fan speed");
      SendPacket(2, FanBuf, false);
#endif
    }

    const void SendFurnaceStatusRequest() {
#ifndef LISTENONLY
      uint8_t IdleBuf[] = {0x06};
      gotAck = false;

      ESP_LOGD(TAG, "furnace status request");
      SendPacket(1, IdleBuf, false);
#endif
    }

    const void SendTime(uint8_t hour, uint8_t minute, uint8_t day) {
#ifndef LISTENONLY
      uint8_t TimeBuf[] = {0x07, hour, minute, day};
      gotAck = false;

      ESP_LOGD(TAG, "time");
      SendPacket(4, TimeBuf, false);
#endif
    }

    const void SendPingFurnace() {
#ifndef LISTENONLY
      uint8_t PingFurnaceBuf[] = {0x09};
      gotAck = false;

      ESP_LOGD(TAG, "ping furnace");
      SendPacket(1, PingFurnaceBuf, false);
#endif
    }

    const void SendTemperatureInfo() {
#ifndef LISTENONLY
      uint8_t currentTemp = round(this->current_temperature * 2);
      uint8_t targetTemp = round(this->target_temperature);
      tempCompare = (targetTemp * 2 < currentTemp) ? 0x00 : 0xFF;

      ESP_LOGD(TAG, "----- target: %d current: %d payload0: %d", targetTemp, currentTemp, tempCompare);

      uint8_t TemperatureInfoBuf[] = {0x0B, tempCompare, targetTemp, currentTemp, 0x00, 0x60, 0x00};
      gotAck = false;

      ESP_LOGD(TAG, "temperature info");
      SendPacket(7, TemperatureInfoBuf, false);
#endif
    }

    const void SendLongPing() {
#ifndef LISTENONLY
      uint8_t LongPingBuf[] = {0x29, 0x00};
      gotAck = false;

      ESP_LOGD(TAG, "long ping");
      SendPacket(2, LongPingBuf, false);
#endif
    }

    void InitTimeout() {
      ESP_LOGD(TAG, "init timeout");
      SendInit();
    }

    const bool GotAck() {
      return gotAck;
    }

    const uint8_t GetMode() {
      return this->mode;
    }

    const int GetUpdate() {
      int tmp = update;
      update = 0;
      return tmp;
    }

    const bool GetChanged() {
      uint8_t currentTemp = round(this->current_temperature * 2);
      uint8_t targetTemp = round(this->target_temperature);
      ESP_LOGD(TAG, "GetChanged Now=%d Current=%d Target=%d", tempCompare, currentTemp, targetTemp);
      if (tempCompare == 0) {
        return targetTemp * 2 >= currentTemp;
      } else {
        return targetTemp * 2 < currentTemp;
      }
    }

    void Resend() {
#ifndef LISTENONLY
      txReady = true;
#endif
    }

    void control(const ClimateCall &call) override {
      if (call.get_mode().has_value()) {
        // User requested mode change
        ESP_LOGD(TAG, "mode changed");
        ClimateMode mode = this->mode;
        this->mode = *call.get_mode();

        // send the fan speed if fan only is turned on or off
        if ((this->mode == climate::CLIMATE_MODE_FAN_ONLY) || ((this->mode != climate::CLIMATE_MODE_FAN_ONLY) && (mode == climate::CLIMATE_MODE_FAN_ONLY))) { 
          update = 1;
          // SendFanSpeed();
        }

        // send the temp info if heat is turned on
        if (this->mode == climate::CLIMATE_MODE_HEAT) { 
          update = 2;
//          SendTemperatureInfo();
        }

        // send the idle/fan if heat is turned off
        if ((this->mode != climate::CLIMATE_MODE_HEAT) && (mode == climate::CLIMATE_MODE_HEAT)) { 
          update = 3;
          // SendIdle();
        }

        // Update state
        this->publish_state(); // this should be removed???
      }
      if (call.get_target_temperature().has_value()) {
        // User requested target temperature change
        ESP_LOGD(TAG, "target temp changed");
        float temp = *call.get_target_temperature();
        // Send target temp to climate
        // ...

        // Publish updated state
        this->target_temperature = temp;
        this->publish_state();
      }
      if (call.get_custom_fan_mode().has_value()) {
        // User requested fan speed change
        ESP_LOGD(TAG, "fan speed changed");
        std::string fan = *call.get_custom_fan_mode();
        // Update state
        this->custom_fan_mode = fan;
        if (this->mode == climate::CLIMATE_MODE_FAN_ONLY) { // only send the new speed to the furnace if the fan is on
          SendFanSpeed();
        }

        this->publish_state(); // this should be removed
      }
    }

    ClimateTraits traits() override {
      // The capabilities of the climate device
      auto traits = climate::ClimateTraits();
      traits.set_supports_current_temperature(this->sensor_ != nullptr);
      traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_FAN_ONLY});
//      traits.set_supported_fan_modes({climate::CLIMATE_FAN_OFF, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH});
      traits.set_supported_custom_fan_modes({"01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15", "16"});
      traits.set_supports_two_point_target_temperature(false);
// deprecated      traits.set_supports_away(false);
      traits.set_visual_min_temperature(8);
      traits.set_visual_max_temperature(30);
      traits.set_visual_temperature_step(0.1);
      return traits;
    }

    void set_sensor(sensor::Sensor *sensor) { 
      this->sensor_ = sensor; 
    }

    void set_text_sensor(text_sensor::TextSensor *sensor) { 
      this->text_sensor_ = sensor; 
    }

    void setup() override {

      if (this->sensor_) {
        this->sensor_->add_on_state_callback([this](float state) {
          this->current_temperature = state;

          // current temperature changed, publish state
          this->publish_state();
        });
        this->current_temperature = this->sensor_->state;
      } else
        this->current_temperature = NAN;

      this->target_temperature = 17;
      this->custom_fan_mode = std::string("01");

      this->set_interval(5, [this] { this->ReceiveLoop(); });
      this->set_interval(10, [this] { this->SendLoop(); });
    }

    void loop() override {
      if (rxReady) {
#ifdef LISTENONLY
        rxReady = false; // if listening only, just discard the packet
#else
        ProcessPacket(); // process the packet if not just listening
#endif
      }
    }
};