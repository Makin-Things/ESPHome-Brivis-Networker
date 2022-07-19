#include "esphome.h"
#define get_briviscustomcomponent(constructor) static_cast<BrivisCustomComponent *>(const_cast<custom::CustomTextSensorConstructor *>(&constructor)->get_text_sensor(0))

#define LISTENONLY

class BrivisCustomComponent : public Component, public UARTDevice, public TextSensor {
  private:
    uint16_t const table_byte[256] = {
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

    const uint8_t Controller = 0x21;
    const uint8_t Furnace = 0x31;

    bool rxReady = false;
    uint8_t rxPacket[13];
    unsigned long txStart = 0;
    bool txReady = false;
    bool txBusy = false;
    uint8_t txPacket[13];
    uint8_t txPos = 0;

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

      publish_state(str);
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
      uint8_t AckBuf[] = {0xFF};

      if ((rxPacket[1] == Controller) && (rxPacket[2] == Furnace)) { // Controller to Furnace

      } else if  ((rxPacket[1] == Furnace) && (rxPacket[2] == Controller)) { // Furnace to Controller
        switch (rxPacket[3]) {
          case 0x01: // Furnace searching for Controller
            SendPacket(1, AckBuf, false);
            break;

          case 0x05: // Furnace sending status
            SendPacket(1, AckBuf, false);
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
          publish_state("Ack");
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

    const void Announce() {
#ifndef LISTENONLY
      uint8_t AnnounceBuf[] = {0x19};

      publish_state("announce");
      SendPacket(1, AnnounceBuf, true);
#endif
    }

    void InitTimeout() {
      publish_state("timeout");
    }

    void setup() override {
      publish_state("setup");
      this->set_interval(5, [this] { this->ReceiveLoop(); });
      this->set_interval(10, [this] { this->SendLoop(); });

#ifndef LISTENONLY
      // Broadcast "I'm Alive" at startup
      this->set_timeout(1000, [this] { this->InitTimeout(); });
//      SendPacket(1, AnnounceBuf, true);
#endif
    }

    void loop() override {
      if (rxReady) {
#ifdef LISTENONLY
        rxReady = false;
#else
        ProcessPacket();
#endif
      }
    }
};