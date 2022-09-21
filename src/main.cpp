#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>
#include <SoftwareSerial.h>
#include "CRC.h"


static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
PacketSerial rfd_PacketSerial;
PacketSerial lora_PacketSerial;
SoftwareSerial gpsSerial(8, 9);
HardwareSerial lora(2);
HardwareSerial rfd(1);

typedef struct packet {
  double lat;
  double lng;
  float alt;
  float speed;
  int sats;
  int packetcount;
  int cutdown_time;
  bool timer_running;
  bool cutdown_status;
  bool parachute_status;
  int bad_packet;
} packet;
packet myPacket;

typedef struct recieved_data{
    int cutdown_time;
    bool update_cutdown_time;
    bool RunTimer;
    bool trigger_cutdown;
    bool trigger_parachute;   
} recieved_data;
recieved_data rx_data;




void check_for_commands(){
  if(rx_data.trigger_cutdown){
    myPacket.cutdown_status = true;
  }
  if(rx_data.trigger_parachute){
    myPacket.parachute_status = true;
  }
  if(rx_data.RunTimer = true){
    myPacket.timer_running = true;
  }
  if(rx_data.RunTimer = false){
    myPacket.timer_running = false;
  }
  if(rx_data.update_cutdown_time){
    myPacket.cutdown_time = rx_data.cutdown_time;
  }
}

void read_gps(){
  if(gpsSerial.available() > 0){
    if(gpsSerial.available() > 0){
      if(gps.encode(gpsSerial.read())){
        if(gps.location.isValid()){
          myPacket.lat = gps.location.lat();
          myPacket.lng = gps.location.lng();
        }
        if(gps.altitude.isValid()){
          myPacket.alt = gps.altitude.meters();
        }
        if(gps.satellites.isValid()){
          myPacket.sats = gps.satellites.value();
        }
        if(gps.speed.isValid()){
          myPacket.speed = gps.speed.mph();
        }
      }
    }
  }  
}

void rfd_PacketReceived(const uint8_t* buffer, size_t size)
{
  uint32_t crc1 = CRC::Calculate(buffer, size, CRC::CRC_32());
  uint32_t crc2;
  memcpy(&crc2, &buffer[size], sizeof(crc2));
  if(crc1 == crc2){
    memcpy(&rx_data, &buffer, sizeof(rx_data)); 
  }
  else{
    myPacket.bad_packet++;
  }
}

void lora_PacketReceived(const uint8_t* buffer, size_t size)
{
  uint32_t crc1 = CRC::Calculate(buffer, size, CRC::CRC_32());
  uint32_t crc2;
  memcpy(&crc2, &buffer[size], sizeof(crc2));
  if(crc1 == crc2){
    memcpy(&rx_data, &buffer, sizeof(rx_data)); 
  }
  else{
    myPacket.bad_packet++;
  }
}

void Send_packet(){
    myPacket.packetcount++;
    uint32_t crc = CRC::Calculate(&myPacket, sizeof(myPacket), CRC::CRC_32());
    uint8_t payload[sizeof(myPacket)+sizeof(crc)];
    memcpy(&payload[0], &myPacket, sizeof(myPacket));
    memcpy(&payload[sizeof(myPacket)], &crc, sizeof(crc));
    rfd_PacketSerial.send(payload, sizeof(payload));
    lora_PacketSerial.send(payload, sizeof(payload));
}

void setup() {
  rfd.begin(57600, SERIAL_8N1, 19, 18);
  lora.begin(9600, SERIAL_8N1, 17, 16);
  gpsSerial.begin(GPSBaud, SWSERIAL_8N1, 13, 12, false, 64);
  //Serial.begin(115200);
  rfd_PacketSerial.setStream(&rfd);
  rfd_PacketSerial.setPacketHandler(&rfd_PacketReceived);
  lora_PacketSerial.setStream(&lora);
  lora_PacketSerial.setPacketHandler(&lora_PacketReceived);


}

unsigned long MillisCount1 = 0;
unsigned long MillisCount2 = 0;
 
void loop() {

  //run at 10hz
  unsigned long currentMillis = millis();
  if(currentMillis - MillisCount1 >= 100){
    MillisCount1 = currentMillis;
    Send_packet();
  }
  //run at 1hz
  if(currentMillis - MillisCount2 >= 1000){
    MillisCount2 = currentMillis;
  }
    
}