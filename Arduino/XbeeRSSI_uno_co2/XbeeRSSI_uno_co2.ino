

int rssiDur;  // Variable for RSSI
int led = 13;  // LED connected to Pin 13
String data = "";
int xdata = "";
int ydata = "";
int rssiflag = 0;
int scanflag = 100;
int state = 0;
int childindex = 0;
int devicesindex = 0;
int mode = 0 ; // 0 : 스캔시도, 1 : 스캔완료, 2: 정상작동
int level = 0;
int id = 10;
int add0 = 0;
int add1 = 0;
int add2 = 0;
int add3 = 0;

uint8_t a[] = {0x7E, 0x00, 0x16, 0x10, 0x01, 0x00, 0x7D, 0x33,
               0xA2, 0x00, 0X41, 0x95, 0x21, 0x6C, 0xFF, 0xFE,
               0x00, 0x00, 0X44, 0x30, 0x49, 0x44, 0x3A, 0x31,
               0x30, 0x45, 0x2E
              };

//uint8_t a[] = {0x7E, 0x00, 0x15, 0x10, 0x01, 0x00, 0x7D, 0x33,
//               0xA2, 0x00, 0X41, 0x95, 0x21, 0x6C, 0xFF, 0xFE,
//               0x00, 0x00, 0X44, 0x49, 0x44, 0x3A, 0x31, 0x30,
//               0x45, 0x2E
//              };


//m1: 전원 on 메세지
//m2: 스캔 start 메세지
//m3: 랑데부 메세지
//m4: 오류 메세지

uint8_t m[] = {0x7E, 0x00, 0x16, 0x10, 0x01, 0x00, 0x13, 0xA2,
                0x00, 0x41, 0X95, 0x73, 0xA4, 0xFF, 0xFE, 0x00, 0x00,
                0x4D, 0x30, 0X49, 0x44, 0x3A, 0x31, 0x30, 0x45, 0x2C
               };

uint8_t devices[20]; //스캐닝으로 획득한 디바이스들 주소

uint8_t child_ad[20]; //자식 요청받은 디바이스들 주소
uint8_t child_id[10];  //자식 요청받은 디바이스들 id
uint8_t act[10];       //D메세지 성공여부 저장


// ATND
uint8_t scan[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x4E, 0X44, 0x64};

// ATDB
uint8_t rssi[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x44, 0X42, 0x70};

void scaning();
void sendD(int index);
void breakdata();
int readrssi();
void delchild();
void sendM();           // Co에게 => 상태메세지 전송 m1~4

#include <SoftwareSerial.h>
SoftwareSerial swSerial = SoftwareSerial(3, 2) ;

void setup()
{
  Serial.begin(9600);   // this is the connection for your Arduino to your PC/MAC
  Serial.print("Ready 1 sec");
  swSerial.begin(9600);   // this is the connection of your Xbee to your Arduino MEGA!!
  delay(1000);

  Serial.println(" start");
  sendM(1);

}

void loop()
{

  if (scanflag > 2000) { // 스캔 대기시간 2000 * 5 ms = 10s
    Serial.println("sendM(4)");
    sendM(4);
    scanflag=0;
  }

  if (rssiflag > 20) { // RSSI 측정 메세지 전송 주기 20 * 5 ms  = 100ms
    if(childindex>0){
      Serial.print("sendD start : ");
      Serial.print (childindex);
      Serial.print(" ");
      for (int j = 0; j < sizeof(child_ad); j += 3) {
        if (child_ad[j] == 0) break;
        Serial.print("sendD(");
        Serial.print(child_id[j / 3]);
        Serial.print("):");
        sendD(j);
        delay(1);
      }
      if (child_ad[0] != 0) Serial.println();
      rssiflag = 0;
    }
  }
  if (swSerial.available() > 0) {
    breakdata();
  }
  

  rssiflag++;
    scanflag++;
  delay(5);
}


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


void sendD2(int add0, int add1, int add2) { //D 메세지 보내기
  Serial.print("Send(D2) ");
  a[11] = add0;
  a[12] = add1;
  a[13] = add2;
  long sum = 0;
  int lastindex = sizeof(a) - 2;
  for (int i = 0; i < sizeof(a); i++) {
    Serial.print(" ");
    Serial.print(String(a[i], HEX));
    swSerial.write(a[i]);
    if (i > 2) {
      sum += a[i];
    }

    if (i == lastindex) {
      if (sum > 157)sum = sum - 157;
      sum &= 0xFF;
      int chk = 0xff - sum;
      a[sizeof(a) - 1] = chk;
    }
  }   

  delay(50);

  for (int i = 0; i < sizeof(a); i++) {
    swSerial.write(a[i]);
  }
  Serial.println(" ");

}

void breakdata() { // 시리얼 데이터 분석
  int type = 0;
  int k = 0;
  String datat = "";
  char datati[80] = {0, 0};

  char datai_pre[30];
  //    Serial.print("Data(hex) : ");
  while (swSerial.available() > 0) {
    char myChar = (char)swSerial.read();
    datat += myChar;
    datati[k] = (int)myChar;
    k++;
    delay(1);
  }
  //    Serial.println();
  int startbit = 0;
  int bitarr[5] = {0, 0, 0, 0, 0};
  int arridx = 0;
  for (int i = 0; i < k; i++) {
    //  Serial.print(" ");
    //  Serial.print((int)datat[i]);
    if ((int)datati[i] == 126) {
      startbit++;
      bitarr[arridx] = i;
      arridx++;
    }
  }
  //  Serial.print("Total data : ");
  //  Serial.println(datat);
  //  Serial.print(" k : ");
  //  Serial.println(k);
  Serial.print(" start bit : ");
  Serial.println(startbit);
  for (int i = 0; i < startbit; i++ ) {
    char datai[30] = {0, 0};
    data = "";
    if (bitarr[i + 1] != 0) {
      for (int x = bitarr[i]; x < bitarr[i + 1]; x++) {
        if (x > sizeof(datai)) break;
        datai[x] = datati[x];
        datai_pre[x] = datai[x];
      }
      //      Serial.print("data start : ");
      //      Serial.print(bitarr[i]);
      //      Serial.print(", data end : ");
      //      Serial.println(bitarr[i + 1]);
    } else {
      for (int x = bitarr[i]; x < sizeof(datati); x++) {
        if (x > sizeof(datai)) break;
        datai[x] = datati[x];
        datai_pre[x] = datai[x];
      }
      //      Serial.print("data start : ");
      //      Serial.println(bitarr[i]);
    }

    if (2 > sizeof(datai)) break;
    add0 = (int)datai[10];
    add1 = (int)datai[11];
    add2 = (int)datai[12];
    if (add0 < 0)  add0 = add0 - int(0xff00);
    if (add1 < 0)  add1 = add1 - int(0xff00);
    if (add2 < 0)  add2 = add2 - int(0xff00);
    if ( (int)datai[5] == 78) {
      type = 1;
      Serial.print("scan response");
      add0 = (int)datai[15];
      add1 = (int)datai[16];
      add2 = (int)datai[17];
      if (add0 < 0)  add0 = add0 - int(0xff00);
      if (add1 < 0)  add1 = add1 - int(0xff00);
      if (add2 < 0)  add2 = add2 - int(0xff00);
      Serial.print(" [");
      Serial.print(add0, HEX);
      Serial.print(" ");
      Serial.print(add1, HEX);
      Serial.print(" ");
      Serial.print(add2, HEX);
      Serial.print("] ");
    }

    //    Serial.print("add1  : ");
    //    Serial.println(add1, HEX);
    //    Serial.print("add2  : ");
    //    Serial.println(add2, HEX);
    arridx++;



    if (type != 1) {
      if ((int)datai[16] > 0) {
        for (int x = 16; x < sizeof(datai); x++) {
          data += datai[x];
        }
      } else {
        for (int x = 0; x < sizeof(datai); x++) {
          data += datai[x];
        }
      }
    }


    if (data.substring(1).startsWith("D")) {
      data = data.substring(1);
    } else if (data.substring(1).startsWith("R")) {
      data = data.substring(1);
    } else if (data.substring(2).startsWith("R")) {
      data = data.substring(2);
    } else if (data.substring(1).startsWith("P")) {
      data = data.substring(1);
    } else if (data.substring(1).startsWith("S")) {
      data = data.substring(1);
    }
    Serial.print("First : ");
    Serial.print(data.substring(0,1));
    Serial.print(" ");

    if (data.startsWith("D")) {
      int endpoint = 0;
      endpoint = data.indexOf('E');
      data = data.substring(1, endpoint);
      Serial.print("Data(");
      Serial.print("C");
      Serial.print(") : ");
      Serial.println(data);
//      readrssi();
    } else if (data.startsWith("S")) {
      int endpoint = 0;
      endpoint = data.indexOf('E');
      data = data.substring(1, endpoint);
      Serial.print("Tree(");
      Serial.print("C");
      Serial.print(") : ");
      Serial.println(data);
    } else if (data.startsWith("R")) {
      int endpoint = 0;
      endpoint = data.indexOf('E');
      data = data.substring(1, endpoint);
      Serial.print("Requst(");
      Serial.print("C");
      Serial.print(") : ");
      Serial.println(data);
      sendD2(add0, add1, add2);
     
    } else if (data.startsWith("P")) {
      int endpoint = 0;
      endpoint = data.indexOf('E');
      data = data.substring(1, endpoint);
      Serial.print("Permit(");
      Serial.print("C");
      Serial.print(") : ");
      Serial.print(data);
      int addindex = childindex * 3;
      int chks = 0;
      for (int i = 0; i < sizeof(child_ad); i += 3) {
        if ((add0 == child_ad[i]) && (add1 == child_ad[i + 1])&& (add2 == child_ad[i + 2])) {
          chks = 1;
          Serial.print(" already in ");
          Serial.print(add0, HEX);
          Serial.print(",");
          Serial.print(add1, HEX);
          Serial.print(",");
          Serial.println(add2, HEX);
          break;
        }
      }
      if (chks == 0) {
        child_id[childindex] = data.substring(2).toInt();
        child_ad[addindex] = add0;
        child_ad[addindex + 1] = add1;
        child_ad[addindex + 2] = add2;
        act[childindex]=0;
        Serial.print(" id : ");
        Serial.print(child_id[childindex]);
        Serial.print(", add : ");
        Serial.print(add0, HEX);
        Serial.print(",");
        Serial.print(add1, HEX);
        Serial.print(",");
        Serial.println(add2, HEX);
        childindex++;
      }
      Serial.print("childindex : ");
      Serial.println(childindex);
    }
    else {
      Serial.print("Data(ex) : ");
      Serial.print(data);
      Serial.print(" ");
    }

  }
  Serial.println(" ");
}



void sendD(int index) { //D 메세지 보내기
  int id_d=child_id[index/3];
  a[11] = child_ad[index];
  a[12] = child_ad[index + 1];
  a[13] = child_ad[index + 2];
  long sum = 0;
  int lastindex = sizeof(a) - 2;
  for (int i = 0; i < sizeof(a); i++) {
    //    Serial.print(" ");
    //    Serial.print(String(a[i], HEX));
    swSerial.write(a[i]);
    if (i > 2) {
      sum += a[i];
      //          Serial.print(a[i]);
      //          Serial.print("+");
    }

    if (i == lastindex) {
      if (sum > 157)sum = sum - 157;
      sum &= 0xFF;
      //          Serial.println();
      int chk = 0xff - sum;
      //          Serial.print("sum = ");
      //          Serial.print(sum);
      //          Serial.print(", chk = ");
      //          Serial.print(chk);
      a[sizeof(a) - 1] = chk;
    }
  }

  int z = 0;
  int k = 0;
  int getstate = 0;
  //응답 메세지 [8]이 0이면 성공 36(0x24)이면 실패
  while (swSerial.available() < 1) {
    z++;
    if (z > 200) {
      getstate = 2;
      act[index/3]++;
      break;
    }
    delay(1);
  }
  while (swSerial.available() > 0) {
    char myChar = (char)swSerial.read();
    if (k == 8) {
      if ((int)myChar == 00) {
        Serial.print("[");
        Serial.print(a[11], HEX);
        Serial.print(",");
        Serial.print(a[12], HEX);
        Serial.print(",");
        Serial.print(a[13], HEX);
        Serial.print("]ss ");
        act[index/3]=0;
      } else {
        act[index/3]++;
      }
    }
    k++;
    delay(1);
  }
  if (act[index/3] == 5) {
    Serial.print("[");
    Serial.print(a[11], HEX);
    Serial.print(",");
    Serial.print(a[12], HEX);
    Serial.print(",");
    Serial.print(a[13], HEX);
    Serial.print("]fl ");
    child_ad[index] = 0;
    child_ad[index + 1] = 0;
    child_ad[index + 2] = 0;
    child_id[index / 3] = 0;
    act[index/3]=0;
    while (index < sizeof(child_ad)) {
      child_id[index / 3] = child_id[(index / 3) + 1];
      act[index / 3] =  act[(index / 3) + 1];
      child_ad[index] = child_ad[index + 3];
      child_ad[index + 1] = child_ad[index + 4];
      child_ad[index + 2] = child_ad[index + 5];
      index += 3;
    }
    childindex--;
    if(childindex<0) childindex=0;
    if(id_d>90){      
      delchild(id_d-90);
    } else {      
      delchild(id_d+90);
    }
  }
}

void delchild(int fromid) {
  int cindex = 100;
  int c = 0;
  while (c < sizeof(child_id)) {
    if (child_id[c] == fromid) {
      cindex = c;
    }
    c++;
    if (c > 1000) break;
  }
  if (cindex < 10) {
    Serial.print("delete child_id[");
    Serial.print(cindex);
    Serial.print("] : ");
    Serial.print(fromid);
    child_ad[cindex * 3] = 0;
    child_ad[(cindex * 3) + 1] = 0;
    child_ad[(cindex * 3) + 2] = 0;
    child_id[cindex] = 0;
    act[cindex] = 0;
    int index = cindex / 3;
    while (index < sizeof(child_ad)) {
      child_id[index / 3] = child_id[(index / 3) + 1];
      act[index / 3] =  act[(index / 3) + 1];
      child_ad[index] = child_ad[index + 3];
      child_ad[index + 1] = child_ad[index + 4];
      child_ad[index + 2] = child_ad[index + 5];
      index += 3;
      if (index > 1000) break;
    }
    Serial.print("now child_ad =");
    index = 0;
    while (index < sizeof(child_ad)) {
      Serial.print(child_ad[index]);
      Serial.print(" ");
      index++;
      if (index > 1000) break;
    }

    Serial.println();

    childindex--;
    if(childindex<0) childindex=0;
  } else {
    Serial.println("already deleted");
  }
}


void sendM(int type) {
  m[18] = 48+type;
  
  long sum = 0;
  int lastindex = sizeof(m) - 2;
  for (int i = 0; i < sizeof(m); i++) {
    //      Serial.print(" ");
    //      Serial.print(String(s[i], HEX));
    swSerial.write(m[i]);
    if (i > 2) {
      sum += m[i];
    }
    if (i == lastindex) {
      //        if (sum > 157)sum = sum - 157;
      sum &= 0xFF;
      int chk = 0xff - sum;
      m[sizeof(m) - 1] = chk;
    }
  }

}
