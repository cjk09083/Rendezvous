#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial xbee(3,2); //Tx, Rx
Servo servo;
int read_analog;
int analog_to_angle;
int angle = 0;

// id2에서 Co로 전송 "D2:3(40),4(24)E"
// 7E 00 1D 10 01 00 7D 33 A2 00 41 8F 2C 29 FF FE 00 00 44 32 3A 33 28 34 30 29 2C 34 28 32 34 29 45 23


//7E 00 7D 33 10 01 00 7D 33 A2 00 41 8F 1B 15 FF FE 00 00 44 73 65 74 45 67
// 00 13 A2 00 41 8F 1B 15
// 44 73 65 74 45


//int id=2;
int id = 3;
int rssiDur;  // Variable for RSSI
String data = "";
int rssiflag = 0;
int state = 0;
int parents_id = 0;
int send_time = 0;
int receive_time = 0;
uint8_t r[] = {0x7E, 0x00, 0x13, 0x10, 0x01, 0x00, 0x13, 0xA2,
               0x00, 0x41, 0X8F, 0x2C, 0x29, 0xFF, 0xFE, 0x00,
               0x00, 0x52, 0X69, 0x64, 0x33, 0x45, 0x80
              };
uint8_t parents_ad[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x44, 0X42, 0x70};
uint8_t rssi[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x44, 0X42, 0x70};

void breakdata();
void readrssi();

void setup()
{
  Serial.begin(9600);   // this is the connection for your Arduino to your PC/MAC
  Serial.println("ready 1 sec");
  xbee.begin(9600);   // this is the connection of your Xbee to your Arduino MEGA!!
  delay(1000);
  Serial.println("start");
  servo.attach(9);
  servo.write(0);

}

void loop()
{
//  if (rssiflag > 3000) {
//    rssiflag = 0;
//    Serial.println("send : Rid:3E");
//    for (int i = 0; i < sizeof(r); i++) {
//      xbee.write(r[i]);
//    }
//    send_time = millis();
//  }

//if (rssiflag > 3000) {
//  angle+=5;
//  if(angle>180)angle=0;
//  servo.write(angle);
//  rssiflag=0;
//}
//
  if (xbee.available() > 0) {
    receive_time = millis();
    int k = 0;
    data = "";
    while (xbee.available() > 0) {
      char myChar = (char)xbee.read();
      if (k == 16) {
        data = "";
      }
      data += myChar;
      k++;
      delay(1);
    }
    if (data.substring(1).startsWith("D")) {
      data = data.substring(1);
    }
    breakdata();
  }

  rssiflag++;

   
  delay(1);
}


void breakdata() {
  if (data.startsWith("D")) {
    int endpoint = 0;
    endpoint = data.indexOf('E');
    data = data.substring(1, endpoint);
    Serial.print("Data(");
    Serial.print(id);
    Serial.print(") : ");
    Serial.print(data);
    readrssi();
  }

  else {
    int interval_time = receive_time - send_time;
    Serial.print("Data(ex) : ");
    Serial.print(data);
    Serial.print("  Time : ");
    Serial.print(interval_time);
    Serial.print(" vs Flag : ");
    Serial.println(rssiflag);
  }

}

void readrssi() {
  for (int i = 0; i < sizeof(rssi); i++) {
    //      Serial.write(rssi[i]);
    xbee.write(rssi[i]);
  }
  delay(100);
  data = "";
  int k = 0;
  int rssival = 0;
  String rssichar;
  while (xbee.available() > 0) {
    char myChar = (char)xbee.read();
    if (k == 8) {
      rssival = (int) myChar;
      if (rssival == 125)rssival = 19;
      rssichar = String(rssival, HEX);

    }
    delay(5);
    k++;
  }
  //  if(rssival>=100){rssival=rssival/10;}
//  Serial.print("  angle: ");
//  Serial.print(angle);
  Serial.print("  RSSI : -");
  Serial.print(rssival);
  Serial.print("(");
  Serial.print(rssichar);
  Serial.println(") dBm");
 
  if (rssival < 20) {
    Serial.println("Rendezvous");
  }

}
