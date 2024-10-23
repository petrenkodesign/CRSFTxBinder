// All packets are in the CRSF format [sync] [len] [type] [payload] [crc8] with a maximum total size of 64 bytes.
// more on https://github.com/crsf-wg/crsf/wiki

#include <SoftwareSerial.h>
SoftwareSerial TX(2, 3);

#define DEBUG Serial
// CRSF packet format [sync] [len] [type] [payload] [crc8]
// 0xC8 - CRSF Addresses { Flight Controller }
// 0x04 - length of packet without CRC
// 0x28 - CRSF Packet Types { Sender requesting DEVICE_INFO from all destination devices }
// 0x00 - CRSF_ADDRESS_BROADCAST (extended destination) 0xEA - CRSF_ADDRESS_RADIO_TRANSMITTER (extended source)
// 0x54 - CRC of packet
uint8_t bind_packet[] = {0xC8, 0x04, 0x28, 0x00, 0xEA, 0x54}; // https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_DEVICE_PING
int packet_len = sizeof(bind_packet)/sizeof(uint8_t);

void setup()
{
  DEBUG.begin(9600);
  TX.begin(115200);
  DEBUG.println("Start testing...");
}

void loop()
{
  // loop sending packet to eLRS TX for binding
  TX.write(bind_packet, packet_len);

  DEBUG.print("Sending packet: ");
  for(int i=0; i<packet_len; i++) {
    DEBUG.print(bind_packet[i] , HEX);
  }
  DEBUG.print(", with length: ");
  DEBUG.println(packet_len);
  delay(100); // 100ms - is maximum for stable connection
}