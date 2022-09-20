#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
#define trigPin 37                   // trigPin을 13으로 설정합니다.
#define echoPin 36                // echoPin을 12로 설정합니다.

int Dir1Pin_A = A3;      // 제어신호 1핀
int Dir2Pin_A = A2;      // 제어신호 2핀
int Dir1Pin_B = A1;      // 제어신호 3핀
int Dir2Pin_B = A0;      // 제어신호 4핀
int pwmpin_A = 5;
int pwmpin_B = 6;

//int id = 1;
int id = 3;
int level = 100;
int p_level = 100;
int max_rssi = 60;
int rang_rssi = 31; // Router끼리 랑데뷰 거리
int rang_rssi2 = 31;  // Co와의 랑데뷰 거리
int ref = 3;  // 좌우 rssi값 차이 허용범위
int addint = 0; // 보정치. L = L+ addint

int rssiDur;  // Variable for RSSI
String data = "";
int rssiflag = 0;
int scanflag = 5000;
int err_flag = 0;
int childindex = 0;
int devicesindex = 0;
int mode = 0 ; // 0 : 스캔시도, 1 : 스캔완료, 2: 정상작동
int add0 = 0;
int add1 = 0;
int add2 = 0;
int padd0 = 0;
int padd1 = 0;
int padd2 = 0;
long duration, distance;

int pre_rssi_val = 30;
int pre_rssi_val2 = 30;
int read_rssi_flag = 0;  // maf 초기화될때까지
int read_rssi_flag2 = 0;  // 이동후 신호안정화대기
int rotate_flag = 0;
int goback_flag = 0;

int state = 0;
int parents_id = 0;
int pre_dif = 0;
int com = 0;
int right_way = 0;
int r_index = 0;
int ran_flag = 0;
int nopa = 0;
long parent_flag = 0;
int getstate = 0; //D메세지 전송성공시 0 실패시 ++
int doubleflag = 0; // 회전 신호안정화 대기 // 1 : 사용안함. 2: 사용함
int doubleint = 5; // 신호 안정화를 위한 측정 횟수
int first_go = 0;


//lpf관련
int sampling_time = 10;
int fc = 5; // cutoff frequency 5~10 Hz
double dt = 1000 / 1000; // sampling time[ms] 10ms
double lambda = 2 * PI * fc * dt;
double x = 30.0;
double x_f = 0.0;
double x_fold = 0.0;
double y = 30.0;
double y_f = 0.0;
double y_fold = 0.0;

//maf관련
int stacksize = 6;
int stackx[6] = {40, 40, 40, 40, 40, 40};
int stacky[6] = {40, 40, 40, 40, 40, 40};


uint8_t parents_ad[] = {0x7E, 0x00, 0x00};
uint8_t devices[30]; //스캐닝으로 획득한 디바이스들 주소
uint8_t child_ad[30]; //자식 요청받은 디바이스들 주소
uint8_t child_id[10]; //자식 요청받은 디바이스들 id
uint8_t act[10];       //D메세지 성공여부 저장

uint8_t child[30]; //자식 요청받은 디바이스들 주소
uint8_t scan[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x4E, 0X44, 0x64};
uint8_t rssi[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x44, 0X42, 0x70};

// D메시지 [12],[13]에 주소 [19]에 level, [23] [24]에 id
uint8_t d[] = {0x7E, 0x00, 0x16, 0x10, 0x01, 0x00, 0x7D, 0x33,
               0xA2, 0x00, 0X41, 0x95, 0x21, 0x6C, 0xFF, 0xFE,
               0x00, 0x00, 0X44, 0x30, 0x49, 0x44, 0x3A, 0x31,
               0x30, 0x45, 0x2E
              };


// id2에서 Co로 전송 "Rid2E"
// 7E 00 13 10 01 00 13 A2
// 00 41 95 2C 29 FF FE 00
// 00 52 69 64 32 45 81
uint8_t r[] = {0x7E, 0x00, 0x14, 0x10, 0x01, 0x00, 0x13, 0xA2,
               0x00, 0x41, 0X95, 0x2C, 0x29, 0xFF, 0xFE, 0x00,
               0x00, 0x52, 0X69, 0x64, 0x30, 0x32, 0x45, 0x81
              };

// 부모후보로 전송 "Pid2E"
uint8_t p[] = {0x7E, 0x00, 0x14, 0x10, 0x01, 0x00, 0x13, 0xA2,
               0x00, 0x41, 0X95, 0x2C, 0x29, 0xFF, 0xFE, 0x00,
               0x00, 0x50, 0X69, 0x64, 0x30, 0x32, 0x45, 0x81
              };

// 부모로 전송 "Cid2E"
uint8_t c[] = {0x7E, 0x00, 0x14, 0x10, 0x01, 0x00, 0x13, 0xA2,
               0x00, 0x41, 0X95, 0x2C, 0x29, 0xFF, 0xFE, 0x00,
               0x00, 0x43, 0X69, 0x64, 0x30, 0x32, 0x45, 0x81
              };

//7e 0 14 10 1 0 13 a2 0 41 95 21 6c ff fe 0 0 4e 2c 29 45 f7
// 새부모주소 전송 "N???E"
uint8_t n[] = {0x7E, 0x00, 0x13, 0x10, 0x01, 0x00, 0x7D, 0x33, 0xA2,
               0x00, 0x41, 0X95, 0x2C, 0x29, 0xFF, 0xFE, 0x00,
               0x00, 0x4E, 0X69, 0x64, 0x64, 0x45, 0x81
              };
// 7E 00 19 10 01 00 7D 33 A2
// 00 41 95 2C 29 FF FE 00 00
// 53 30 32 3A 31 30 28 33 32
// 29 45 CC


// Co에게 자기 부모연결정보 전송
uint8_t s[] = {0x7E, 0x00, 0x19, 0x10, 0x01, 0x00, 0x13, 0xA2,
               0x00, 0x41, 0X95, 0x73, 0xA4, 0xFF, 0xFE, 0x00, 0x00,
               0x53, 0x30, 0X32, 0x3A, 0x31, 0x30, 0x28, 0x30, 0x32,
               0x29, 0x45, 0xCC
              };

//7E 00 16 10 01 00 13 A2
//00 41 8F 2C 29 FF FE 00 00
//4D 31 49 44 3A 30 31 45 2C

//m1: 전원 on 메세지
//m2: 스캔 start 메세지
//m3: 랑데부 메세지
//m4: 오류 메세지

uint8_t m[] = {0x7E, 0x00, 0x16, 0x10, 0x01, 0x00, 0x13, 0xA2,
               0x00, 0x41, 0X95, 0x73, 0xA4, 0xFF, 0xFE, 0x00, 0x00,
               0x4D, 0x30, 0X49, 0x44, 0x3A, 0x30, 0x31, 0x45, 0x2C
              };


void scaning();
void sendD(int index);  // RSSI 측정용 자식에게
void sendR(int index);           // 부모 정보 요청
void sendP();           // 자식 요청
void sendC();           // 랑데뷰완료 => 새 부모주소 요청
void sendN(int fromid);           // 자식에게 => 부모의 부모주소 전송
void sendM(int type);           // Co에게 => 상태메세지 전송 m1~4
void sendD2(int add0, int add1, int add2);


void breakdata();
void delchild(int fromid);

int readrssi();
void turnangle(int r1, int r2);
int lpf_m();
int lpf_s();

int maf_m(int newx);
int maf_s(int newy);

void setup()
{
  pinMode(Dir1Pin_A, OUTPUT);
  pinMode(Dir2Pin_A, OUTPUT);
  pinMode(Dir1Pin_B, OUTPUT);
  pinMode(Dir2Pin_B, OUTPUT);
  pinMode(pwmpin_A, OUTPUT);
  pinMode(pwmpin_B, OUTPUT);
  pinMode(trigPin, OUTPUT);   // trigPin 핀을 출력핀으로 설정합니다.
  pinMode(echoPin, INPUT);    // echoPin 핀을 입력핀으로 설정합니다.
  analogWrite(pwmpin_A, 253);          //모터 속도를 1/5로 설정
  analogWrite(pwmpin_B, 253);          //모터 속도를 1/5로 설정
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  if (id > 9) {
    int idt = (int) (id / 10);
    d[23] = idt + 48;
    d[24] = id - (idt * 10) + 48;
  } else {
    d[23] = 48;
    d[24] = id + 48;
  }
  r[21] = id +  48;
  p[21] = id + 48;
  c[21] = id + 48;
  Serial.print("MY ID : ");
  Serial.print(p[21] - 48);

  Serial.print(" ready 3");

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);          // 0번째 줄 0번째 셀부터 입력하게 합니다.
  lcd.print("Wait 3");       // 문구를 출력합니다.

  delay(1000);
  lcd.print(" 2");       // 문구를 출력합니다.
  Serial.print(" 2");
  delay(1000);
  lcd.print(" 1");       // 문구를 출력합니다.
  Serial.println(" 1");
  delay(1000);
  sendM(1);
  digitalWrite(Dir1Pin_A, LOW);
  digitalWrite(Dir2Pin_A, LOW);
  digitalWrite(Dir1Pin_B, LOW);
  digitalWrite(Dir2Pin_B, LOW);
  lcd.clear();
  lcd.print("Not connected");       // 문구를 출력합니다.

  Serial.println("start");

}

void loop()
{
  if (scanflag > 5000) { // 스캔 대기시간 1200 * 5 ms = 6s

    Serial.print("!");
    scaning();
    sendM(4);
  }

  if (rssiflag > 150) { // RSSI 측정 메세지 전송 주기 n * 1 ms  = nms
    int sizeofm = sizeof(child_ad);
    for (int j = 0; j < sizeofm; j += 3) {
      if (child_ad[j] == 0) break;
      digitalWrite(Dir1Pin_A, LOW);         //모터가 정지
      digitalWrite(Dir2Pin_A, LOW);
      digitalWrite(Dir1Pin_B, LOW);         //모터가 정지
      digitalWrite(Dir2Pin_B, LOW);
      Serial.print("sendD(");
      Serial.print(child_id[j / 3]);
      Serial.print("):");
      sendD(j);
      delay(1);
    }
    if (child_ad[0] != 0) Serial.println();
    rssiflag = 0;
  }
  if (Serial2.available() > 0) {
    //    Serial.print("****dataIn****  ");
    breakdata();
  }

  rssiflag++;
  scanflag++;
  if (childindex < 0)childindex = 0;
  if (mode == 1) {
    if ((scanflag % 1000) == 0) {
      Serial.print("scanflag : ");
      Serial.println(scanflag);
    }
  }
  if (mode > 2) {
    parent_flag++;
  }
  delay(1);
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void scaning() {
  if (mode == 0) {  //시작시
    devicesindex = 0;
    memset(devices, 0, sizeof(devices));
    Serial.println("scan start");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("scan start ");
    int sizeofscan = sizeof(scan);
    for (int i = 0; i < sizeofscan; i++) {
      if (first_go == 1)Serial2.write(scan[i]);

    }
    p_level = 100;
    scanflag = 0;
    mode = 1;
    read_rssi_flag = 0;
    digitalWrite(Dir1Pin_A, LOW);
    digitalWrite(Dir2Pin_A, LOW);
    digitalWrite(Dir1Pin_B, LOW);
    digitalWrite(Dir2Pin_B, LOW);
    sendM(2);
    if (first_go == 0) {
      Serial.println("first go");
      if (id == 1) {
        devices[0] = 149;
        devices[1] = 115;
        devices[2] = 183;        
      } else if (id==2){
        devices[0] = 149;
        devices[1] = 115;
        devices[2] = 216;
      } else if (id==3){
        devices[0] = 143;
        devices[1] = 33;
        devices[2] = 108;
      }
      devicesindex=3;
      scanflag = 3000;
      first_go = 1;
    }
  } else if (mode == 1) {   //스캔 결과
    if (devices[0] == 0) {
      mode = 0;
      Serial.println("scan nothing : try again");
    }
    else {
      scanflag = 4500;
      Serial.print("scan complete : ");
      Serial.println((devicesindex + 1) / 3);
      lcd.setCursor(0, 1);
      lcd.print("scan result: ");
      lcd.print((devicesindex + 1) / 3);

      mode = 2;
    }

    digitalWrite(Dir1Pin_A, LOW);
    digitalWrite(Dir2Pin_A, LOW);
    digitalWrite(Dir1Pin_B, LOW);
    digitalWrite(Dir2Pin_B, LOW);

  } else if (mode == 2) {   // 부모 후보 찾음

    digitalWrite(Dir1Pin_A, LOW);
    digitalWrite(Dir2Pin_A, LOW);
    digitalWrite(Dir1Pin_B, LOW);
    digitalWrite(Dir2Pin_B, LOW);
    if (devices[r_index] == 0) {
      Serial.print("p_level : ");
      Serial.println(p_level);
      if (p_level < 100) {
        level = p_level;
        Serial.print("Parent id(level) : ");
        Serial.print(parents_id );
        Serial.print("(");
        Serial.print(level - 1);
        Serial.print("), add [");
        Serial.print(parents_ad[0], HEX );
        Serial.print(",");
        Serial.print(parents_ad[1], HEX );
        Serial.print(",");
        Serial.print(parents_ad[2], HEX );
        Serial.print("] My level : ");
        Serial.println(level );
        for (int i = 0; i < 2; i++) {
          sendP();
          delay(50);
        }
        mode = 3;
      } else {
        Serial.println("No connected Parent try again ");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Not connected");
        nopa++;
        if (nopa > 3) {
          mode = 0;
          nopa = 0;
        }
        r_index = 0;
        scanflag = 4500;
      }
    } else {
      sendR(r_index);
      r_index += 3;
      scanflag = 4500;
    }

  } else if (mode == 3) {
    if (childindex == 0) {    // M메세지 받아서 자식 디바이스가 모두 삭제되면
      Serial.println("No child, GTG mode");
      parent_flag = 0;
      mode = 4;               // 주행모드로
    } else {
      Serial.print("have ");
      Serial.print(childindex);
      Serial.print(" children ");
      for (int i = 0; i < childindex; i++) {
        Serial.print(child_id[i]);
        Serial.print(", ");
      }
      Serial.println();
      lcd.setCursor(0, 1);
      lcd.print("have ");
      lcd.print(childindex);
      lcd.print(" children");
      if (childindex == 1) {
        Serial.print("ERROR childindex is 1 ");
        childindex = 0;
        child_ad[0] = 0;
      }
      if (child_id[0] == 0) {
        Serial.print("ERROR child_id[0] is 0");
        Serial.println("INIT children");
        childindex = 0;
      }
    }
  } else if (mode == 4) {
    if (childindex != 0) {
      if (childindex == 1) {
        Serial.print("ERROR childindex is 1 ");
        childindex = 0;
      } else {
        Serial.print("have ");
        Serial.print(childindex);
        Serial.println(" children");

        lcd.setCursor(0, 1);
        lcd.print("have ");
        lcd.print(childindex);
        lcd.print(" childrens");
        mode = 3;
      }
      if (child_id[0] == 0) {
        Serial.print("ERROR child_id[0] is 0");
        Serial.println("INIT children");
        childindex = 0;
      }
    }
  }

  if (mode > 2) {
    int maxpflag = 10000;
    if (mode == 4) {
      maxpflag = 1500;
    }
    if ( parent_flag > maxpflag) {
      Serial.print("parent_flag = ");
      Serial.print(parent_flag);
      Serial.println(" Parents disconnected");
      mode = 0;
      p_level = 100;
      level = 100;
    } else {
      if (id > 9) {
        int idt = (int) (id / 10);
        s[18] = idt + 48;
        s[19] = id - (idt * 10) + 48;
      } else {
        s[18] = 48;
        s[19] = id + 48;
      }

      if (parents_id > 9) {
        int pat = (int) (parents_id / 10);
        s[21] = pat + 48;
        s[22] = parents_id - (pat * 10) + 48;
      } else {
        s[21] = 48;
        s[22] = parents_id + 48;
      }

      if (rssiDur > 9) {
        int rst = (int) (rssiDur / 10);
        s[24] = rst + 48;
        s[25] = rssiDur - (rst * 10) + 48;
      } else {
        s[24] = 48;
        s[25] = rssiDur + 48;
      }
      Serial.print("send(S) MY : ");
      Serial.print(s[18] - 48);
      Serial.print(s[19] - 48);
      Serial.print(", PA : ");
      Serial.print(s[21] - 48);
      Serial.print(s[22] - 48);
      Serial.print(", RS : ");
      Serial.print(s[24] - 48);
      Serial.print(s[25] - 48);
      Serial.print(", LV : ");
      Serial.println(level);
      Serial.print("parent_flag = ");
      Serial.println(parent_flag);
      long sum = 0;
      int lastindex = sizeof(s) - 2;
      int sizeofs = sizeof(s);
      for (int i = 0; i < sizeofs; i++) {
        //      Serial.print(" ");
        //      Serial.print(String(s[i], HEX));
        Serial2.write(s[i]);
        if (i > 2) {
          sum += s[i];
        }
        if (i == lastindex) {
          //        if (sum > 157)sum = sum - 157;
          sum &= 0xFF;
          int chk = 0xff - sum;
          s[sizeof(s) - 1] = chk;
        }
      }
    }

    scanflag = 3000;
  }
}


void breakdata() {
  int type = 0;
  int k = 0;
  char data_total[100];
  String datat = "";

  int startbit = 0;
  int bitarr[5] = {0, 0, 0, 0, 0};
  int arridx = 0;
  while (Serial2.available() > 0) {
    char myChar = (char)Serial2.read();
    data_total[k] = (int)myChar;
    datat += myChar;
    k++;
    if ((int)myChar == 126) {
      startbit++;
      bitarr[arridx] = k;
      arridx++;
    }
    if (k > 100) {
      Serial.print(" Too long data! ");
      break;
    }
    delay(1);
  }

  //  Serial.print(k);
  Serial.print(" start bit : ");
  Serial.print(startbit);

  if (startbit > 20) {
    Serial.print(" Error!!");
  } else {
    Serial.print(", ");
    int sizeoftotal = sizeof(data_total);

    for (int i = 0; i < startbit; i++ ) {
      char data_arr[30] = {0, 0};
      int sizeofarr = sizeof(data_arr);

      data = "";
      type = 0;
      if (bitarr[i + 1] != 0) {
        for (int x = bitarr[i]; x < bitarr[i + 1]; x++) {
          if (x > sizeofarr) break;
          data_arr[x] = data_total[x];
        }
        //    Serial.print("data start : ");
        //      Serial.print(bitarr[i]);
        //      Serial.print(", data end : ");
        //      Serial.println(bitarr[i + 1]);
      } else {

        for (int x = bitarr[i]; x < sizeoftotal; x++) {
          if (x > sizeofarr) break;
          data_arr[x] = data_total[x];
        }
        //    Serial.print("data start : ");
        //      Serial.println(bitarr[i]);
      }


      if ( (int)data_arr[5] == 78) {
        type = 1;
        Serial.print("scan response");
        add0 = (int)data_arr[16];
        add1 = (int)data_arr[17];
        add2 = (int)data_arr[18];
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
      } else {
        type = 0;
        add0 = (int)data_arr[10];
        add1 = (int)data_arr[11];
        add2 = (int)data_arr[12];
        padd0 = (int)data_arr[13];
        padd1 = (int)data_arr[14];
        padd2 = (int)data_arr[15];
        if (add0 < 0)  add0 = add0 - int(0xff00);
        if (add1 < 0)  add1 = add1 - int(0xff00);
        if (add2 < 0)  add2 = add2 - int(0xff00);
        if (padd0 < 0)  padd0 = padd0 - int(0xff00);
        if (padd1 < 0)  padd1 = padd1 - int(0xff00);
        if (padd2 < 0)  padd2 = padd2 - int(0xff00);
      }


      if ( type == 1) {
        Serial.print("RSSI :  ");
        int rsval = readrssi();
        Serial.print(rsval);
        if ((id > 5) && (add1 == 115) && (add2 == 183)) {
          Serial.print(" Result : denied!! ");
        } else if ((id == 8) && (add1 == 115) && (add2 == 216)) {
          Serial.print(" Result : denied!!! ");
        } else {
          if ((rsval < max_rssi) && (rsval != 0)) {
            Serial.print(" Result : permit! ");
            devices[devicesindex] = add0;
            devicesindex++;
            devices[devicesindex] = add1;
            devicesindex++;
            devices[devicesindex] = add2;
            devicesindex++;
          } else {
            Serial.print(" Result : denied! ");
          }
        }
        Serial.print(" ");

        scanflag = 0;
        int divdot = 0;
        int sizeofdevices = sizeof(devices);
        for (int i = 0; i < sizeofdevices; i++) {
          if (divdot == 0) {
            Serial.print("[");
            Serial.print(String(devices[i], HEX));
            divdot++;
          } else if (divdot == 1) {
            Serial.print(String(devices[i], HEX));
            divdot++;
          } else {
            Serial.print(String(devices[i], HEX));
            Serial.print("]");
            divdot = 0;
          }
          Serial.print(" ");
        }
        Serial.println();
      } else {
        if ((int)data_arr[16] > 0) {
          for (int x = 16; x < sizeofarr; x++) {
            data += data_arr[x];
          }
        } else {
          for (int x = 0; x < sizeofarr; x++) {
            data += data_arr[x];
          }
        }

        if (data.substring(1).startsWith("D")) {
          data = data.substring(1);
        } else if (data.substring(1).startsWith("R")) {
          data = data.substring(1);
        } else if (data.substring(1).startsWith("P")) {
          data = data.substring(1);
        }  else if (data.substring(1).startsWith("C")) {
          data = data.substring(1);
        }  else if (data.substring(1).startsWith("N")) {
          data = data.substring(1);
        }
        Serial.print("RECEIVE ");
        Serial.print(data.substring(0, 1));
        Serial.print(" messeage, mode : ");
        Serial.println(mode);
        //      if (data.startsWith("D")) {
        //        mode=2;
        //      }
        if (mode > 1) {
          if (data.startsWith("D")) {
            int endpoint = 0;
            endpoint = data.indexOf('E');
            data = data.substring(1, endpoint);
            int lv = (int)data.charAt(0) - 48;
            if (lv > (-1)) {
              if (lv < (p_level - 1)) {
                p_level = lv + 1;
                Serial.print("New Parents!(");
                Serial.print(lv);
                Serial.print(")");
                Serial.print(", p_level : ");
                Serial.print(p_level);

                parents_id = data.substring(4).toInt();

                parents_ad[0] = add0;
                parents_ad[1] = add1;
                parents_ad[2] = add2;
                Serial.print(",  Parent_ad : ");
                Serial.print(parents_ad[0], HEX);
                Serial.print(", ");
                Serial.print(parents_ad[1], HEX);
                Serial.print(", ");
                Serial.print(parents_ad[2], HEX);
              }
              Serial.print("Data(");
              Serial.print(id);
              Serial.print(") : ");
              Serial.print(data);

              rssiDur = readrssi();
              if (mode < 4) {
                Serial.print("  RS : ");
                Serial.print(rssiDur);
                Serial.print(" ");
              }
            }
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
            int sizeofchildad = sizeof(child_ad);
            for (int i = 0; i < sizeofchildad; i += 3) {
              if ((add0 == child_ad[i]) && (add1 == child_ad[i + 1]) && (add2 == child_ad[i + 2])) {
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
              act[childindex] = 0;
              child_ad[addindex] = add0;
              child_ad[addindex + 1] = add1;
              child_ad[addindex + 2] = add2;
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
          } else if (data.startsWith("C")) {
            int endpoint = 0;
            endpoint = data.indexOf('E');
            data = data.substring(1, endpoint);
            String from = data.substring(2);
            int fromid = from.toInt();
            Serial.print("New parent_add request from id : ");
            Serial.println(fromid);
            sendN(fromid);
            delay(10);
            sendN(fromid);
            delay(10);
            sendN(fromid);
            delay(10);
            sendN(fromid);
            delay(10);


            delchild(fromid);
            delchild(fromid + 90);

            //          sendD2(add1, add2);
          } else if (data.startsWith("N")) {
            int endpoint = 0;
            endpoint = data.indexOf('E');
            data = data.substring(1, endpoint);
            Serial.print("New parent_add from parrent : ");
            padd0 = (int)data_arr[18];
            padd1 = (int)data_arr[19];
            padd2 = (int)data_arr[20];
            if (padd0 < 0)  padd0 = padd0 - int(0xff00);
            if (padd1 < 0)  padd1 = padd1 - int(0xff00);
            if (padd2 < 0)  padd2 = padd2 - int(0xff00);
            parents_ad[0] = padd0;
            parents_ad[1] = padd1;
            parents_ad[2] = padd2;
            Serial.print(padd0, HEX);
            Serial.print(", ");
            Serial.print(padd1, HEX);
            Serial.print(", ");
            Serial.print(padd2, HEX);
            level--;
            //          Serial.print(" data : ");
            //          for (int i = 0; i < sizeof(data_arr); i++ ) {
            //             Serial.print((int)data_arr[i],HEX);
            //             Serial.print(" ");
            //          }
            Serial.println(" ");
            for (int i = 0; i < 1; i++) {
              sendP();
              delay(50);
            }

          } else {
            Serial.print("Data(ex) : ");
            Serial.print(data);
            Serial.print(" ");
          }

        }
        else {
          Serial.print("Data(");
          Serial.print("ex2");
          Serial.print(") : ");
          Serial.println(data);
        }
      }
    }
    Serial.println(" ");
  }
}

int readrssi() {
  //  Serial.print(" parent_flag : ");
  //  Serial.println(parent_flag);
  parent_flag = 0;
  int sizeofrssi = sizeof(rssi);
  for (int i = 0; i < sizeofrssi; i++) {
    //      Serial.write(rssi[i]);
    Serial2.write(rssi[i]);
    Serial3.write(rssi[i]);
  }
  delay(25);

  data = "";
  int k = 0;
  int rssival = 0;
  String rssichar;
  while (Serial2.available() > 0) {
    char myChar = (char)Serial2.read();
    if (k == 8) {
      rssival = (int) myChar;
      if (rssival == 125)rssival = 19;
    }
    //    data += myChar;
    delay(1);
    k++;
    if (k > 1000) break;
  }

  int k2 = 0;
  int rssival2 = 0;
  String rssichar2;
  while (Serial3.available() > 0) {
    char myChar = (char)Serial3.read();
    if ( myChar == 'B') { // 데이터 시작 ASC:~ => HEX:7E => DEC:126
      k2 = 0;
      data = "";
    }
    if (k2 == 2) {
      rssival2 = (int) myChar;
      if (rssival2 == 125)rssival2 = 19;
    }
    data += myChar;
    delay(1);
    k2++;
    if (k2 > 1000) break;
  }

  if (mode > 3) {
    //  if ((rssival < 0) || (rssival2 < 0)) {
    //    Serial.println("Parents disconnected");
    //    mode = 0;
    //    p_level = 100;
    //    level = 100;
    //    rssival = 0;
    //    rssival2 = 0;
    //  }
    if (rssival < 0) {
      Serial.print("rssival = ");
      Serial.print(rssival);
      Serial.print("(");
      Serial.print(rssival, HEX);
      Serial.print(") <0");
      rssival = rssival * (-1);
    }
    if (rssival2 < 0) rssival2 = rssival2 * (-1);
    x = rssival;
    y = rssival2;
    if (rssival < 10) x = pre_rssi_val;
    if (rssival2 < 10) y = pre_rssi_val2;
    if (read_rssi_flag > stacksize) {

      rssival = maf_m(x);
      rssival2 = maf_s(y);
    } else {
      stackx[read_rssi_flag] = x;
      stacky[read_rssi_flag] = y;
      read_rssi_flag++;
      Serial.print("  Not yet : ");
      Serial.print(read_rssi_flag);
    }


    //  rssival = lpf_m();
    //  if (y != 0) {
    //    rssival2 = lpf_s();
    //  } else {
    //    rssival2 = 0;
    //  }
    rssichar = String(rssival, HEX);
    rssichar2 = String(rssival2, HEX);

    pre_rssi_val = rssival;
    pre_rssi_val2 = rssival2;

    //보정

    rssival2 += addint;


    //    Serial.println();
    //    Serial.print(", R : ");
    //    Serial.print(data);
    //    Serial.print("  ");

    Serial.print("  RSSI L : -");
    Serial.print(rssival);
    Serial.print("(");
    Serial.print(rssichar);
    Serial.print(")");
    Serial.print(":");
    Serial.print(x);

    Serial.print("  R : -");
    Serial.print(rssival2);
    Serial.print("(");
    Serial.print(rssichar2);
    Serial.print(")  ");
    Serial.print(":");
    Serial.print(y);

    lcd.clear();
    lcd.setCursor(0, 0);          // 0번째 줄 0번째 셀부터 입력하게 합니다.
    lcd.print("L:");
    lcd.print(rssival);       // 문구를 출력합니다.
    lcd.setCursor(5, 0);          // 0번째 줄 5번째 셀부터 입력하게 합니다.
    lcd.print("R:");
    lcd.print(rssival2);       // 문구를 출력합니다.
    if ((15 < rssival) && (rssival * rssival2 > 0)) {
      err_flag = 0;
      int max_rang_rssi = 0;
      if (s[21] = 58) {
        max_rang_rssi = rang_rssi2;
      }
      else {
        max_rang_rssi = rang_rssi;
      }
      if  ((rssival < max_rang_rssi) || (rssival2 < max_rang_rssi)) {
        Serial.println("Rendezvous");
        lcd.setCursor(0, 1);          // 0번째 줄 0번째 셀부터 입력하게 합니다.
        lcd.print(parents_id);       // 문구를 출력합니다.
        lcd.print(" Rendezvous");       // 문구를 출력합니다.
        ran_flag++;
        digitalWrite(Dir1Pin_A, LOW);         //모터가 정지
        digitalWrite(Dir2Pin_A, LOW);
        digitalWrite(Dir1Pin_B, LOW);         //모터가 정지
        digitalWrite(Dir2Pin_B, LOW);
        analogWrite(pwmpin_A, 253);
        analogWrite(pwmpin_B, 253);
        if (ran_flag > 2) {
          sendM(3);
          if (parents_id != 10) {
            for (int i = 0; i < 2; i++) {
              sendC();
              delay(50);
            }
          } else {
            if (rssival < 21) {
              Serial.println(" Parents disconnected");
              mode = 0;
              p_level = 100;
              level = 100;
              //              parent_flag=2000;
            }
          }
          ran_flag = 0;
        }
      } else {
        if ((read_rssi_flag2 > doubleint) || (doubleflag == 1)) {
          turnangle(rssival, rssival2);
          read_rssi_flag2 = 0;
        }
        read_rssi_flag2++;
      }
    } else {
      Serial.println("Error");
      lcd.setCursor(0, 1);          // 0번째 줄 0번째 셀부터 입력하게 합니다.
      lcd.print(parents_id);       // 문구를 출력합니다.
      lcd.print(" Error");       // 문구를 출력합니다.
      digitalWrite(Dir1Pin_A, LOW);
      digitalWrite(Dir2Pin_A, LOW);
      digitalWrite(Dir1Pin_B, LOW);
      digitalWrite(Dir2Pin_B, LOW);
      analogWrite(pwmpin_A, 253);
      analogWrite(pwmpin_B, 253);
      err_flag++;
      if (err_flag > 3) {
        sendP();
        err_flag = 0;
      }
    }
  } else {
    pre_rssi_val = rssival;
    pre_rssi_val2 = rssival2;
    x_fold = rssival;
    y_fold = rssival2;
  }
  return rssival;
}

void sendR(int index) { //R 메세지 보내기
  Serial.print("send(R) ");
  r[10] = devices[index];
  r[11] = devices[index + 1];
  r[12] = devices[index + 2];
  long sum = 0;
  int lastindex = sizeof(r) - 2;
  int sizeofr = sizeof(r);
  for (int i = 0; i < sizeofr; i++) {
    Serial.print(" ");
    Serial.print(String(r[i], HEX));
    Serial2.write(r[i]);
    if (i > 2) {
      sum += r[i];
    }
    if (i == lastindex) {
      sum &= 0xFF;
      int chk = 0xff - sum;
      r[sizeof(r) - 1] = chk;
    }
  }
  Serial.println();
}



void sendP() { //P 메세지 보내기
  Serial.print("send(P) ");
  err_flag = 0;
  p[20] = 0x30;
  p[10] = parents_ad[0];
  p[11] = parents_ad[1];
  p[12] = parents_ad[2];
  long sum = 0;
  int lastindex = sizeof(p) - 2;
  int sizeofp = sizeof(p);
  for (int i = 0; i < sizeofp; i++) {
    Serial.print(" ");
    Serial.print(String(p[i], HEX));
    Serial2.write(p[i]);
    if (i > 2) {
      sum += p[i];
    }
    if (i == lastindex) {
      sum &= 0xFF;
      int chk = 0xff - sum;
      p[sizeof(p) - 1] = chk;
    }
  }
  delay(50);
  for (int i = 0; i < sizeofp; i++) {
    Serial2.write(p[i]);
  }
  delay(50);
  for (int i = 0; i < sizeofp; i++) {
    Serial2.write(p[i]);
  }


  delay(100);

  Serial.println();
  Serial.print("send(Psub) ");
  p[20] = 0x39;
  sum = 0;
  for (int i = 0; i < sizeofp; i++) {
    Serial3.write(p[i]);
    Serial.print(" ");
    Serial.print(String(p[i], HEX));
    if (i > 2) {
      sum += p[i];
    }
    if (i == lastindex) {
      sum &= 0xFF;
      int chk = 0xff - sum;
      p[sizeof(p) - 1] = chk;
    }
  }
  delay(50);
  for (int i = 0; i < sizeofp; i++) {
    Serial3.write(p[i]);
  }
  delay(50);
  for (int i = 0; i < sizeofp; i++) {
    Serial3.write(p[i]);
  }
  Serial.println();
}

void sendP2() { // 새로받은 부모에게 P메세지 보내기
  Serial.print("send(R) ");
  r[10] = parents_ad[0];
  r[11] = parents_ad[1];
  r[12] = parents_ad[2];
  long sum = 0;
  int lastindex = sizeof(r) - 2;
  int sizeofr = sizeof(r) ;
  for (int i = 0; i < sizeofr; i++) {
    Serial.print(" ");
    Serial.print(String(r[i], HEX));
    Serial2.write(r[i]);
    if (i > 2) {
      sum += r[i];
    }
    if (i == lastindex) {
      sum &= 0xFF;
      int chk = 0xff - sum;
      r[sizeof(r) - 1] = chk;
    }
  }
  Serial.println();
}

void sendC() { //C 메세지 보내기
  Serial.print("send(c) ");
  c[20] = 0x30;
  c[10] = parents_ad[0];
  c[11] = parents_ad[1];
  c[12] = parents_ad[2];
  long sum = 0;
  int lastindex = sizeof(c) - 2;
  int sizeofc = sizeof(c) ;
  for (int i = 0; i < sizeofc; i++) {
    Serial.print(" ");
    Serial.print(String(c[i], HEX));
    Serial2.write(c[i]);
    if (i > 2) {
      sum += c[i];
    }
    if (i == lastindex) {
      sum &= 0xFF;
      int chk = 0xff - sum;
      c[sizeof(c) - 1] = chk;
    }
  }
  delay(50);
  for (int i = 0; i < sizeofc; i++) {
    Serial2.write(c[i]);
  }
  Serial.println();
}

void turnangle(int r1, int r2) {
  int dif = r1 - r2;


  int delay_time = 100;
  int go_delay_time = 350;

  int speed_val = 253;
  //    if (dif > 20 && dif < -20) {
  ////      speed_val = 150;
  //      delay_time = 200;
  //    } else if (dif > 10 && dif < -10) {
  ////      speed_val = 150;
  //      delay_time = 150;
  //    } else {
  ////      speed_val = 127;
  //    }
  analogWrite(pwmpin_A, speed_val);
  analogWrite(pwmpin_B, speed_val);
  lcd.setCursor(10, 0);
  //  Serial.print("Difference : L-R = ");
  //  Serial.print(dif);
  //  if (pre_dif < abs(dif)) {
  //    pre_dif = abs(dif);
  //    //    dif = ~dif;s
  //  } else {
  //    pre_dif = abs(dif);
  //  }

  if (dif >= -ref && dif <= ref) {
    if (com == 1) {
      Serial.println("  turn stop!");
      lcd.print("ok!");

      digitalWrite(trigPin, LOW);                 // trigPin에 LOW를 출력하고
      delayMicroseconds(2);                    // 2 마이크로초가 지나면
      digitalWrite(trigPin, HIGH);                // trigPin에 HIGH를 출력합니다.
      delayMicroseconds(10);                  // trigPin을 10마이크로초 동안 기다렸다가
      digitalWrite(trigPin, LOW);                // trigPin에 LOW를 출력합니다.
      duration = pulseIn(echoPin, HIGH);   // echoPin핀에서 펄스값을 받아옵니다.
      distance = duration * 17 / 1000;

      if (id == 1)distance = 30;
      if (distance > 20) {
        //      int speed_val = map(r1, 0, 100, 100, 250);
        analogWrite(pwmpin_A, 253);
        analogWrite(pwmpin_B, 253);
        digitalWrite(Dir1Pin_A, LOW);
        digitalWrite(Dir2Pin_A, HIGH);
        digitalWrite(Dir1Pin_B, HIGH);
        digitalWrite(Dir2Pin_B, LOW);
        for (int i = 0; i < 5; i++) {
          delay(go_delay_time / 5);
          duration = pulseIn(echoPin, HIGH);   // echoPin핀에서 펄스값을 받아옵니다.
          distance = duration * 17 / 1000;
          if (distance < 20) {
            digitalWrite(Dir1Pin_A, LOW);         //모터가 정회전
            digitalWrite(Dir2Pin_A, LOW);
            digitalWrite(Dir1Pin_B, LOW);         //모터가 정회전
            digitalWrite(Dir2Pin_B, LOW);
          }
        }
        digitalWrite(Dir1Pin_A, LOW);         //모터가 정회전
        digitalWrite(Dir2Pin_A, LOW);
        digitalWrite(Dir1Pin_B, LOW);         //모터가 정회전
        digitalWrite(Dir2Pin_B, LOW);
        delay(10);

        lcd.setCursor(0, 1);          // 0번째 줄 0번째 셀부터 입력하게 합니다.
        lcd.print("Go ");
        lcd.print(speed_val);
        goback_flag = 0;
      } else {
        Serial.print("Stop : ");
        Serial.print(distance);                         // distance를 시리얼 모니터에 출력합니다.
        Serial.println(" cm");
        lcd.setCursor(0, 1);          // 0번째 줄 0번째 셀부터 입력하게 합니다.
        lcd.print("Stop : ");
        lcd.print(distance);
        lcd.print("cm");
        if (goback_flag == 1) {
          analogWrite(pwmpin_A, 253);
          analogWrite(pwmpin_B, 253);
          digitalWrite(Dir1Pin_A, HIGH);
          digitalWrite(Dir2Pin_A, LOW);
          digitalWrite(Dir1Pin_B, LOW);
          digitalWrite(Dir2Pin_B, HIGH);
          delay(go_delay_time / 2);
          digitalWrite(Dir1Pin_A, LOW);         //모터가 정회전
          digitalWrite(Dir2Pin_A, LOW);
          digitalWrite(Dir1Pin_B, LOW);         //모터가 정회전
          digitalWrite(Dir2Pin_B, LOW);
          delay(10);
        }
        goback_flag = 1;

        if (pre_rssi_val < 41) {
          if (ran_flag > 2) {
            sendM(3);
            if (parents_id != 10) {
              for (int i = 0; i < 2; i++) {
                sendC();
                delay(50);
              }
            } else {
              if (pre_rssi_val < 21) {
                Serial.println(" Parents disconnected");
                mode = 0;
                p_level = 100;
                level = 100;
                //              parent_flag=2000;
              }
            }
            ran_flag = 0;
          }
        }

      }

      rotate_flag = 0;
    }
    com = 1;
  } else if (dif > ref) {
    if (com == 3) {

      Serial.println("  turn right!");
      digitalWrite(Dir1Pin_A, LOW);         //모터가 역회전
      digitalWrite(Dir2Pin_A, HIGH);
      digitalWrite(Dir1Pin_B, LOW);         //모터가 역회전
      digitalWrite(Dir2Pin_B, HIGH);

      delay(delay_time);

      digitalWrite(Dir1Pin_A, LOW);         //모터가 정회전
      digitalWrite(Dir2Pin_A, LOW);
      digitalWrite(Dir1Pin_B, LOW);         //모터가 정회전
      digitalWrite(Dir2Pin_B, LOW);
      delay(10);
      lcd.print("right!");
      rotate_flag++;
    }
    com = 3;
  } else {
    if (com == 2) {

      Serial.println("  turn left!");
      digitalWrite(Dir1Pin_A, HIGH);         //모터가 정회전
      digitalWrite(Dir2Pin_A, LOW);
      digitalWrite(Dir1Pin_B, HIGH);         //모터가 정회전
      digitalWrite(Dir2Pin_B, LOW);

      delay(delay_time);

      digitalWrite(Dir1Pin_A, LOW);         //모터가 정회전
      digitalWrite(Dir2Pin_A, LOW);
      digitalWrite(Dir1Pin_B, LOW);         //모터가 정회전
      digitalWrite(Dir2Pin_B, LOW);
      delay(10);
      lcd.print("left!");
      rotate_flag++;
    }
    com = 2;
  }

  if (rotate_flag > 10) {  //계속 한방향으로 돌면 180도 회전
    analogWrite(pwmpin_A, 253);
    analogWrite(pwmpin_B, 253);
    digitalWrite(Dir1Pin_A, LOW);
    digitalWrite(Dir2Pin_A, HIGH);
    digitalWrite(Dir1Pin_B, HIGH);
    digitalWrite(Dir2Pin_B, LOW);
    delay(go_delay_time);
    digitalWrite(Dir1Pin_A, LOW);         //모터가 정회전
    digitalWrite(Dir2Pin_A, LOW);
    digitalWrite(Dir1Pin_B, LOW);         //모터가 정회전
    digitalWrite(Dir2Pin_B, LOW);
    rotate_flag = 0;
  }
  lcd.setCursor(0, 1);          // 0번째 줄 0번째 셀부터 입력하게 합니다.
  lcd.print(parents_id);       // 문구를 출력합니다.
}

void sendN(int fromid) {
  int nindex = 100;
  int nflag = 0;
  while (nflag < sizeof(child_id)) {
    if (child_id[nflag] == fromid) {
      nindex = nflag;
    }
    nflag++;
    if (nflag > 1000) break;
  }
  if (nindex == 100) {
    Serial.println("SendN failed : already send");
  } else {
    Serial.print("SendN parent_add : ");
    Serial.print(parents_ad[0], HEX);
    Serial.print(", ");
    Serial.print(parents_ad[1], HEX);
    Serial.print(", ");
    Serial.print(parents_ad[2], HEX);
    Serial.print(" to ");
    Serial.print(child_ad[nindex * 3], HEX);
    Serial.print(", ");
    Serial.print(child_ad[(nindex * 3) + 1], HEX);
    Serial.print(", ");
    Serial.println(child_ad[(nindex * 3) + 2], HEX);
    n[11] = child_ad[nindex * 3];
    n[12] = child_ad[(nindex * 3) + 1];
    n[13] = child_ad[(nindex * 3) + 2];
    n[19] = parents_ad[0];
    n[20] = parents_ad[1];
    n[21] = parents_ad[2];
    long sum = 0;
    int lastindex = sizeof(n) - 2;
    int sizeofn = sizeof(n);
    for (int j = 0; j < 5; j++) {
      for (int i = 0; i < sizeofn; i++) {
        Serial.print(" ");
        Serial.print(String(n[i], HEX));
        Serial2.write(n[i]);
        if (j == 0) {
          if (i > 2) {
            sum += n[i];
          }
          if (i == lastindex) {
            if (sum > 157)sum = sum - 157;
            sum &= 0xFF;
            int chk = 0xff - sum;
            n[sizeof(n) - 1] = chk;
          }
        }
      }
      Serial.println();
      delay(50);
    }
  }
}

void sendM(int type) {
  m[18] = 48 + type;
  m[23] = id + 48;

  long sum = 0;
  int lastindex = sizeof(m) - 2;
  for (int i = 0; i < sizeof(m); i++) {
    //      Serial.print(" ");
    //      Serial.print(String(s[i], HEX));
    Serial2.write(m[i]);
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
    Serial.print(" now child_ad =");
    index = 0;
    while (index < sizeof(child_ad)) {
      Serial.print(child_ad[index], HEX);
      Serial.print(" ");
      index++;
      if (index > 1000) break;
    }

    Serial.println();

    childindex--;
  } else {
    Serial.println("already deleted");
  }
}
void sendD(int index) { //D 메세지 보내기
  int id_d = child_id[index / 3];
  d[11] = child_ad[index];
  d[12] = child_ad[index + 1];
  d[13] = child_ad[index + 2];
  d[19] = level + 48;
  long sum = 0;
  int lastindex = sizeof(d) - 2;
  int sizeofm = sizeof(d);
  for (int i = 0; i < sizeofm ; i++) {
    Serial2.write(d[i]);
    if (i > 2) {
      sum += d[i];
    }

    if (i == lastindex) {
      if (sum > 157)sum = sum - 157;
      sum &= 0xFF;
      int chk = 0xff - sum;
      d[sizeof(d) - 1] = chk;
    }
  }
  Serial.print("[");
  Serial.print(d[11], HEX);
  Serial.print(",");
  Serial.print(d[12], HEX);
  Serial.print(",");
  Serial.print(d[13], HEX);
  Serial.print("] ");
  /*
    int z = 0;
    int k = 0;
    getstate = 0;
    while (Serial2.available() < 1) {
    z++;
    if (z > 200) {
      act[index / 3]++;
      Serial.print("act++;");
      break;
    }
    delayMicroseconds(100);
    }
    //  delay(10);
    //응답 메세지 [8]이 0이면 성공 36(0x24)이면 실패
    while (Serial2.available() > 0) {
    char myChar = (char)Serial2.read();
    if((int)myChar == 126) k=0;
    if((int)myChar == 139) k=3;
    if (k == 8) {
      if ((int)myChar == 36) {
        Serial.print("[");
        Serial.print(d[11], HEX);
        Serial.print(",");
        Serial.print(d[12], HEX);
        Serial.print(",");
        Serial.print(d[13], HEX);
        Serial.print("]");
        Serial.print("act++");
        act[index / 3] ++;
        break;
      } else {
        Serial.print("[");
        Serial.print(d[11], HEX);
        Serial.print(",");
        Serial.print(d[12], HEX);
        Serial.print(",");
        Serial.print(d[13], HEX);
        Serial.print("]");
        Serial.print("ss");
        act[index / 3] = 0;
        break;
      }
    }
    k++;
    if (k > 1000) break;
    delay(1);
    }

    Serial.print(" {z=");
    Serial.print(z);
    Serial.print(",");
    Serial.print("k=");
    Serial.print(k);
    Serial.print("} ");

    if ( act[index / 3] > 2) {
    Serial.print("[");
    Serial.print(d[11], HEX);
    Serial.print(",");
    Serial.print(d[12], HEX);
    Serial.print(",");
    Serial.print(d[13], HEX);
    Serial.print("]fl ");
    child_ad[index] = 0;
    child_ad[index + 1] = 0;
    child_ad[index + 2] = 0;
    child_id[index / 3] = 0;
    act[index / 3] = 0;      //D메세지 실패여부 저장
    int sizeofm = sizeof(child_ad);
    while (index < sizeofm) {
      child_id[index / 3] = child_id[(index / 3) + 1];
      act[index / 3] =  act[(index / 3) + 1];
      child_ad[index] = child_ad[index + 3];
      child_ad[index + 1] = child_ad[index + 4];
      child_ad[index + 2] = child_ad[index + 5];
      index += 3;
      if (index > 1000) break;
    }
    childindex--;
    if (id_d > 90) {
      delchild(id_d - 90);
    } else {
      delchild(id_d + 90);
    }
    getstate = 0;
    }
  */

}

void sendD2(int add0, int add1, int add2) { //D 메세지 보내기
  Serial.print("Send(D2) ");
  d[11] = add0;
  d[12] = add1;
  d[13] = add2;
  d[19] = level + 48;

  long sum = 0;
  int lastindex = sizeof(d) - 2;
  int sizeofm = sizeof(d);

  for (int i = 0; i < sizeofm; i++) {
    Serial.print(" ");
    Serial.print(String(d[i], HEX));
    Serial2.write(d[i]);
    if (i > 2) {
      sum += d[i];
    }

    if (i == lastindex) {
      if (sum > 157)sum = sum - 157;
      sum &= 0xFF;
      int chk = 0xff - sum;
      d[sizeofm - 1] = chk;
    }
  }
  Serial.println(" ");
}

int lpf_m() {
  x_f = lambda / (1 + lambda) * x + 1 / (1 + lambda) * x_fold; //필터된 값
  x_fold = x_f; // 센서 필터 이전값 업데이트
  return x_f;
}

int lpf_s() {
  y_f = lambda / (1 + lambda) * y + 1 / (1 + lambda) * y_fold; //필터된 값
  y_fold = y_f; // 센서 필터 이전값 업데이트
  return y_f;
}

int maf_m(int newx) {
  //  Serial.println("");
  //  Serial.print("newx = ");
  //  Serial.print(newx);
  int sumx = 0;
  stackx[stacksize - 1] = newx;
  //  Serial.print(" sumx = ");
  for (int i = 0; i < (stacksize - 1); i++) {
    sumx += stackx[i + 1];
    stackx[i] = stackx[i + 1];
    //    Serial.print(stackx[i]);
    //    Serial.print(" + ");
  }

  int value = sumx / (stacksize - 1);
  //  Serial.print("=> ");
  //  Serial.print(sumx);
  //  Serial.print(" stacksize = ");
  //  Serial.print(stacksize);
  //  Serial.print(" sumx/(stacksize-1) = ");
  //  Serial.print(value);
  return value;
}

int maf_s(int newy) {
  int sumy = 0;
  stacky[stacksize - 1] = newy;
  for (int i = 0; i < (stacksize - 1); i++) {
    sumy += stacky[i + 1];
    stacky[i] = stacky[i + 1];
  }
  int value = sumy / (stacksize - 1);
  return value;
}
