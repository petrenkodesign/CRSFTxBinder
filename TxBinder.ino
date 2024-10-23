// All packets are in the CRSF format [sync] [len] [type] [payload] [crc8] with a maximum total size of 64 bytes.
// more on https://github.com/crsf-wg/crsf/wiki

#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#include <SoftwareSerial.h>
SoftwareSerial TX(2, 3);

#define DEBUG Serial

uint16_t channels[16] = {0};
uint8_t crsf_packet[26];
const char *status = "Press button...";
int lcdbtn = 0;

uint8_t _crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0;  // Initial CRC value
    uint8_t polynomial = 0xD5;  // Polynomial value used in CRC8 (polynomial for CRSF)

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];  // XOR with the current data byte

        for (uint8_t bit = 0; bit < 8; bit++) {  // through each bit
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;  // If the high bit is set, XOR with the polynomial
            } else {
                crc <<= 1;  // If not, just shift to the left
            }
        }
    }

    return crc; 
}

void ch2crsf(uint8_t packet[], int16_t channels[]) {
    packet[0] = 0xEE; // CRSF Address - 0xEE - Transmitter module, not handset
    packet[1] = 24; //  length of packet without CRC
    packet[2] = 0x16; // CRSF Packet Types - 0x16 - Channels data 
    
    int byteIndex = 3;  // Byte index of chanels packets
    int bitShift = 0;   // Bit shift for correct packing

    for (uint8_t i = 0; i < 16; i++) {
        packet[byteIndex] |= (channels[i] << bitShift) & 0xFF;
        packet[byteIndex + 1] = (channels[i] >> (8 - bitShift)) & 0xFF;

        bitShift += 3;
        if (bitShift >= 8) {
            byteIndex++;
            bitShift -= 8;
        }
        byteIndex++;
    }

    packet[25] = _crc8(&packet[2], packet[1] - 1); // CRC
}

void btn_release() {
  while(lcdbtn!=1023) {
    lcdbtn = analogRead(0);
  }
}

void setup()
{
  DEBUG.begin(9600);
  TX.begin(115200);

  // set flight sticks to middle
  channels[0] = channels[1] = channels[3] = 992;
  // set throttle to CRSF null
  channels[2] = 172;
  // set AUX6 to CRSF null (Disarm)
  channels[9] = 172;

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Start handshaking!");
  delay(2000);
  lcd.clear();
}

void loop()
{
  lcdbtn = analogRead(0);
  lcd.setCursor(0, 0);
  lcd.print("Hub Working >>>");
  lcd.setCursor(0, 1);
  lcd.print(status);
  // preparing packets
  ch2crsf(crsf_packet, channels);
  // sending data to ELRS TX
  TX.write(crsf_packet, 26);

  // read data from serial monitor
  if (DEBUG.available() > 0) {
    char inp = DEBUG.read();
    switch (inp) {
      case '1':
        // Arm
        channels[9] = 992; // 988 (172) - disable, 1500 (992) - middle, 2012 (1811) - enable
        DEBUG.println("Disarm");
        break;
      case '2':
        // Disarm
        channels[9] = 172;
        DEBUG.println("ARM");
        break;            
    }
  }

  // reading a shield keypad pressing
  if (lcdbtn < 60) { // right
      status = "Right           ";
  } else if (lcdbtn < 200) { // up
      btn_release();
      channels[2] += 10;
      sprintf(status, "ThrtUP: %u   ", channels[2]);
  } else if (lcdbtn < 400) { // down
      btn_release();
      channels[2] -= 10;
      sprintf(status, "ThrtDW: %u   ", channels[2]);
  } else if (lcdbtn < 600) { // left
      status ="Left           ";
  } else if (lcdbtn < 800) { // select
      btn_release();
      channels[9] = (channels[9]==992 || channels[9]==0) ? 172 : 992;
      status = (channels[9]==172) ? "ARM            " : "Disarm         ";
  }
}