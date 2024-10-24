// All packets are in the CRSF format [sync] [len] [type] [payload] [crc8] with a maximum total size of 64 bytes.
// more on https://github.com/crsf-wg/crsf/wiki

#include <LiquidCrystal.h>
#include <LCDKeypad.h>
LCDKeypad lcd;

#include <SoftwareSerial.h>
// connect eLRS TX, S.Port directly to Arduino Rx input (2), to Arduino Tx (3) through a 10k resistor or diode
SoftwareSerial TX(2, 3); 
// defines hardware serial for debugging through serial monitor
#define DEBUG Serial

// defines a channels (AUX), which statuses TX sends to drone
uint16_t channels[16] = {0};
// defines CRSF data packet, which TX sends to drone
uint8_t crsf_packet[26];
// defines CRSF data packet, which RX return to TX
char info[64] = {0};
// defines LCD information vars
char header[16] = "Hub linking >>>";
char status[16] = "Press button...";

// CRC calculation function for packets that TX will send to the drone
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
// function checks the CRC of the packet received from the RX (drone)
bool crsf_validate_frame(uint8_t *frame, int length) {
    uint8_t crc = 0;
    for (int i = 2; i < length - 1; i++) {
        crc ^= frame[i];
        for (int ii = 0; ii < 8; ii++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc = crc << 1;
            }
        }
    }
    return (crc & 0xFF) == frame[length - 1];
}
// function generate a CSRF packet to send to the drone (RX)
void ch2crsf(uint8_t packet[], int16_t channels[]) {
    packet[0] = 0xEE; // CRSF Address - 0xEE - Transmitter module, not handset
    packet[1] = 24; //  length of packet without CRC
    packet[2] = 0x16; // CRSF Packet Types - 0x16 - Channels data 
    
    int byteIndex = 3;  // Byte index of channels packets
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
// drone (RX) incoming packet handler
void handleCrsfPacket(uint8_t *data, uint8_t length) {
    uint8_t ptype = data[2];  // Packet type is at index 2 https://github.com/crsf-wg/crsf/wiki/Packet-Types
    // drone GPS data
    if(ptype==0x02){
      int32_t lat = (int32_t)((data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]);
      int32_t lon = (int32_t)((data[7] << 24) | (data[8] << 16) | (data[9] << 8) | data[10]);
      int16_t gspd = (int16_t)((data[11] << 8) | data[12]);
      int16_t hdg = (int16_t)((data[13] << 8) | data[14]);
      int16_t alt = (int16_t)((data[15] << 8) | data[16]) - 1000;
      uint8_t sats = data[17];
      sprintf(info,"Pos: %d.%d %d.%d Gspd: %d m/s Hdg: %d Alt: %dm Sats:", lat/1e7, lat%10000000, lon/1e7, lon%10000000, gspd/36, hdg/100, alt, sats);
      DEBUG.print(info);
    }
    //drone speed
    if(ptype==0x07){
      int16_t vspd = (int16_t)((data[3] << 8) | data[4]);
      sprintf(info,"VSpd: %d.%d m/s", vspd/10, vspd%10);
      DEBUG.println(info);
    }
    // drone position
    if(ptype==0x1E){
      int16_t pitch = (int16_t)((data[3] << 8) | data[4]);
      int16_t roll = (int16_t)((data[5] << 8) | data[6]);
      int16_t yaw = (int16_t)((data[7] << 8) | data[8]);
      sprintf(info, "Pit:%d.%d Rol:%d.%d Yaw:%d.%d rad", pitch/1000, pitch%1000, roll/1000, roll%1000, yaw/1000, yaw%1000);
      DEBUG.println(info);
    }
    // drone barometr data
    if(ptype==0x09){
      int32_t altBaro = (int32_t)((data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]);
      sprintf(info, "BarAlt:%d.%dm", altBaro/100, altBaro%100);
      DEBUG.println(info);
    }
    // drone RSSI
    if(ptype==0x14){
      int8_t rssi1 = (int8_t)data[3];
      int8_t rssi2 = (int8_t)data[4];
      uint8_t lq = data[5];
      int8_t snr = (int8_t)data[6];
      sprintf(info, "RS:%d/%d LQ%u%%", rssi1, rssi2, lq);
      sprintf(header, info);
      DEBUG.println(info);
    }
    // drone battery status
    if(ptype==0x08){
      int16_t vbat = (int16_t)((data[3] << 8) | data[4]);
      int16_t curr = (int16_t)((data[5] << 8) | data[6]);
      uint32_t mah = (data[7] << 16) | (data[8] << 8) | data[9];
      uint8_t pct = data[10];
      sprintf(info, "B:%d.%dV %d.%dA %u%%", vbat/10, vbat%10, curr/10, curr%10, pct);
      sprintf(header, info);
      DEBUG.println(info);
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
  // display the input information on LCD
  lcd.setCursor(0, 0);
  lcd.print(header);
  lcd.setCursor(0, 1);
  lcd.print(status);
  // preparing packets
  ch2crsf(crsf_packet, channels);

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
  switch (lcd.buttonBlocking()) {
    case KEYPAD_LEFT:
      channels[3] = ( channels[3] > 1810 ) ? 1811 : channels[3]+10;
      sprintf(status, "Left : %u     ", channels[3]);
      break;
    case KEYPAD_RIGHT:
      channels[3] = ( channels[3] < 180 ) ? 172 : channels[3]-10;
      sprintf(status, "Right: %u     ", channels[3]);
      break;
    case KEYPAD_UP:
      channels[2] = ( channels[2] > 1810 ) ? 1811 : channels[2]+10;
      sprintf(status, "ThrtUP: %u     ", channels[2]);
      break;
    case KEYPAD_DOWN:
      channels[2] = ( channels[2] < 180 ) ? 172 : channels[2]-10;
      sprintf(status, "ThrtDW: %u     ", channels[2]);
      break;
    case KEYPAD_SELECT:
      channels[9] = (channels[9]==992 || channels[9]==0) ? 172 : 992;
      sprintf(status, (channels[9]==172) ? "ARM            " : "Disarm         ");
      break;
  }
  // Read RX CRSF data
  static uint8_t buffer[64];  // Buffer to store CRSF data
  static uint8_t input_buffer[64];  // Store incoming data
  static uint8_t input_pos = 0;  // Position in the input buffer
  if (TX.available() > 0) {
    // char txinp = TX.read();
    // DEBUG.println(txinp, HEX);
    input_buffer[input_pos++] = TX.read();
    if (input_pos > 2) {  // Minimum packet size is 3
            uint8_t expected_len = input_buffer[1] + 2;  // Packet length is at index 1 + header and CRC

            if (expected_len > 64 || expected_len < 4) {
              input_pos = 0;  // Reset buffer on invalid packet size
            } else if (input_pos >= expected_len) {
              // save the data in a separate buffer for further use
              uint8_t single_packet[64];
              memcpy(single_packet, input_buffer, expected_len);
              // keep the buffer clean
              memmove(input_buffer, input_buffer + expected_len, input_pos - expected_len);
              input_pos -= expected_len;

              if (crsf_validate_frame(single_packet, expected_len)) {
                // Handle valid CRSF packet
                handleCrsfPacket(input_buffer, expected_len);
              } else {
                // DEBUG.println("CRC error!");
              }

            }
    }
  } else {
    // sending data to ELRS TX, cyclic sending of the packet binds the devices
    TX.write(crsf_packet, 26);
  }
}