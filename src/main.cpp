#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>
#include <SoftwareSerial.h>
#include "CRC.h"

#define CutDownNichromeTime 5000 //milli sec
#define CutDownPin 25
#define ParachutePin 26
bool nichromeON = false;
unsigned long CutDownStart = 0;


static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
PacketSerial rfd_PacketSerial;
HardwareSerial gpsSerial(2);
HardwareSerial rfd(1);

unsigned long MillisCount1 = 0;
unsigned long MillisCount2 = 0;

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
  int lora_bad_packet;
  int rfd_bad_packet;
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



void parachuteUpdate(){
  if(myPacket.parachute_status == true){
      digitalWrite(ParachutePin, HIGH);
  }
  else{
    digitalWrite(ParachutePin, LOW);
  }
}

void cutdownUpdate(){
  if(myPacket.cutdown_status == true){
    if(nichromeON = false){
      CutDownStart = millis();
      digitalWrite(CutDownPin, HIGH);
      nichromeON = true;
    }
    if(millis() > (CutDownStart + CutDownNichromeTime)){
      digitalWrite(CutDownPin, LOW);
      nichromeON = false;
      myPacket.cutdown_status = false;
    }
  }
}

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
  if(myPacket.timer_running == true){
    myPacket.cutdown_time--;
  }
  if(myPacket.cutdown_time < 1){
    myPacket.cutdown_status = true;
  }
}

void read_gps(){
  if(gpsSerial.available() > 0){
    Serial.println("gps avalible");
    if(gps.encode(gpsSerial.read())){
      Serial.println("gps decoded");
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

void rfd_PacketReceived(const uint8_t* buffer, size_t size)
{
  uint32_t crc1 = CRC::Calculate(buffer, size, CRC::CRC_32());
  uint32_t crc2;
  memcpy(&crc2, &buffer[sizeof(rx_data)], sizeof(crc2));
  if(crc1 == crc2){
    memcpy(&rx_data, &buffer, sizeof(rx_data)); 
  }
  else{
    myPacket.rfd_bad_packet++;
  }
}



void Send_packet(){
    myPacket.packetcount++;
    uint32_t crc = CRC::Calculate(&myPacket, sizeof(myPacket), CRC::CRC_32());
    uint8_t payload[sizeof(myPacket)+sizeof(crc)];
    memcpy(&payload, &myPacket, sizeof(myPacket));
    memcpy(&payload[sizeof(myPacket)], &crc, sizeof(crc));
    
    /*
    for(int i = 0; i < sizeof(payload); i++){
      Serial.print(i);
      Serial.print(" ");
      Serial.println(payload[i]);
    }
    
    Serial.print("packet size ");
    Serial.println(sizeof(myPacket));
    Serial.print("payload ");
    Serial.println(sizeof(payload));
    */
    rfd_PacketSerial.send(&payload[0], sizeof(payload));
}

void setup() {
  rfd.begin(57600, SERIAL_8N1, 33, 32);
  gpsSerial.begin(GPSBaud, SERIAL_8N1, 17, 16);
  Serial.begin(115200);
  rfd_PacketSerial.setStream(&rfd);
  rfd_PacketSerial.setPacketHandler(&rfd_PacketReceived);

  myPacket.cutdown_status = false;
  myPacket.cutdown_time = 3600;
  myPacket.parachute_status = false;
  myPacket.packetcount = 0;

  pinMode(CutDownPin, OUTPUT);
  pinMode(ParachutePin, OUTPUT);
}

void loop() {
  rfd_PacketSerial.update();
  cutdownUpdate();
  parachuteUpdate();
  
  //run at 10hz
  unsigned long currentMillis = millis();
  if(currentMillis - MillisCount1 >= 100){
    MillisCount1 = currentMillis;
    
    check_for_commands();
  }

  //run at 1hz
  if(currentMillis - MillisCount2 >= 1000){
    MillisCount2 = currentMillis;
    read_gps();
    Send_packet();
    Serial.print("lat: ");
    Serial.println(myPacket.lat);
    Serial.print("lng: ");
    Serial.println(myPacket.lng);
  }
    
}