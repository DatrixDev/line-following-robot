#include <SoftwareSerial.h>
SoftwareSerial HC05(2, 3);
const int GREEN = 0;  //khi test rút dây
const int RED = 1;    //khi test rút dây
const int BLUE = 13;

// const BUZZER = 4; //Chưa mua BUZZER

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 10;

int Speed = 200;
bool modeLine = false;
int lastPosition;
int steering = 0;
int previousError = 0;
float kp = 1;
float kd = 5;
int line = 0;
int rememberLine = 0;
unsigned char sensor;
unsigned int isCalib = 0;
unsigned int sensorValue[8];
unsigned int sensorPID[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];




//phụ
unsigned long prevMillis = 0;
bool state = false;
bool toggleColor = false;

void setup() {
  Serial.begin(9600);
  HC05.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
}

void loop() {

  if (modeLine) {
    read_sensor();  //doccambien
    if (HC05.available()) {
      char c = HC05.read();
      Serial.println(c);
      if (c == 'x') {
        Stop();
        modeLine = false;
      }
    }
  } else {
    if (HC05.available()) {
      char c = HC05.read();
      Serial.println(c);
      switch (c) {
        case 'F': tien(); break;
        case 'B': lui(); break;
        case 'R': phai(); break;
        case 'L': trai(); break;
        case 'G': tien_trai(); break;
        case 'I': tien_phai(); break;
        case 'H': lui_trai(); break;
        case 'J': lui_phai(); break;
        case 'S': Stop(); break;
        case 'X':
          Stop();
          modeLine = true;
          break;
      }
    }
  }
}
void read_sensor()  // hàm đọc cảm biến
{
  unsigned char tempBit = 0;
  unsigned int sum = 0;
  unsigned long avg = 0;
  int i, iP, iD;  // độ lệch // kéo về theo độ lệch // giảm rung
  int iRet;
  sensorValue[0] = 1023 - analogRead(A0);
  sensorValue[1] = 1023 - analogRead(A1);
  sensorValue[2] = 1023 - analogRead(A2);
  sensorValue[3] = 1023 - analogRead(A3);
  sensorValue[4] = 1023 - analogRead(A4);
  sensorValue[5] = 1023 - analogRead(A5);
  sensorValue[6] = 1023 - analogRead(A6);
  sensorValue[7] = 1023 - analogRead(A7);

  for (int j = 0; j < 8; j++) {
    if (isCalib == 0) {
      if (sensorValue[j] < black_value[j])
        sensorValue[j] = black_value[j];
      if (sensorValue[j] > white_value[j])
        sensorValue[j] = white_value[j];
      sensorPID[j] = map(sensorValue[j], black_value[j], white_value[j], 0, 1000);
    }
    tempBit = tempBit << 1;
    if (sensorValue[j] > compare_value[j]) {
      tempBit |= 0x01;
    } else {
      tempBit &= 0xfe;
    }
    sensor = tempBit;
  }
  for (int j = 0; j < 8; j++) {
    avg += (long)(sensorPID[j]) * ((j)*1000);
    sum += sensorPID[j];
  }
  if(sum!=0)
  i = (int)((avg / sum) - 3500);
  
  kp = 1;
  kd = 5;
  iP = kp * i;
  iD = kd * (lastPosition - i);
  iRet = (iP - iD);
  if ((iRet < -4000)) {
    iRet = 0;
  }
  steering = iRet / 30;

  lastPosition = i;
}
void speed_run(int speedLeft, int speedRight) {
  if (speedLeft >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speedLeft);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speedLeft);
  }
  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speedRight);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speedRight);
  }
}
void handleAndSpeed(int angle, int speedHAS) {
  int speedLeft;
  int speedRight;
  if ((speedHAS + angle) > 255) {
    speedHAS = 255 - angle;
  }
  if (speedHAS - angle > 255) {
    speedHAS = 255 + angle;
  }
  speedLeft = speedHAS + angle;
  speedRight = speedHAS - angle;
  speed_run(speedLeft, speedRight);
}
void learnLine() {
  if (sensor == 0xff) {
    int color[] = { RED, BLUE };
    Blink(color, 2);  //line full đen  = nhảy màu đỏ - xanh
  }

  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValue[i]);
    Serial.print("  ");
    if (black_value[i] == 0) {
      black_value[i] = 1100;
    }
    if (sensorValue[i] < black_value[i]) {
      black_value[i] = sensorValue[i];
    }
    if (sensorValue[i] > white_value[i]) {
      white_value[i] = sensorValue[i];
    }
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
  Serial.println();
}
// void beep(int timer) {
//   digitalWrite(BUZZER, 1);
//   delay(timer);
//   digitalWrite(BUZZER, 0);
// }

void led(int color) {
  switch (color) {
    case 0:  //đèn đỏ sáng
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 0);
      break;
    case 1:  // xanh lá sáng
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 0);
      break;
    case 2:  //xanh dương sáng
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 1);
      break;
    default:  //tắt hết
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 0);
      break;
  }
}
void Blink(int ledList[], int count) {
  static int currentIndex = 0;   

  if (millis() - prevMillis >= 200) {
    prevMillis = millis();

    for (int i = 0; i < count; i++) {
      digitalWrite(ledList[i], LOW);
    }

    digitalWrite(ledList[currentIndex], HIGH);
    currentIndex++;
    if (currentIndex >= count) {
      currentIndex = 0;
    }
  }
}
void runforwardline(int tocdo)  // hàm chạy bám line
{
  switch (sensor) {
    case 0b00000000:
      // kd = 12;
      // if (RememberLine == 1) line = 3;
      // if (RememberLine == -1) line = -3;
      // if (line > 0) {
      //   // handleAndSpeed(90, 0);
      //   speed_run(100, -40);
      //   RGB(1);
      // } else if (line < 0) {
      //   //  handleAndSpeed(-90, 0);
      //   speed_run(-40, 100);
      //   RGB(2);
      // } else {
      //   speed_run(0, 0);
      //   RGB(0);
      // }
      //speed_run(0, 0);
      handleAndSpeed(steering, tocdo);
      break;
    case 0b00011000:
    case 0b00001000:
    case 0b00010000:
    case 0b00111000:
    case 0b00011100:

      // RGB(0);
      line = 0;
      handleAndSpeed(steering, tocdo);
      break;
    case 0b00111100:
    case 0b00111110:
    case 0b01111110:

      // RGB(4);
      line = 0;
      handleAndSpeed(steering, tocdo);
      break;


      ///////////////////////////////////////////////////////////////////////
    case 0b00001100:
    case 0b00000100:
    case 0b00001110:
    case 0b00011110:

      line = 1;
      handleAndSpeed(steering, tocdo);
      break;
    case 0b00000110:
    case 0b00000010:
    case 0b00000111:

      line = 2;
      handleAndSpeed(steering, tocdo);
      break;
    case 0b00000011:
    case 0b00000001:
    case 0b00001111:
    case 0b00011111:
    case 0b00111111:

      line = 3;
      handleAndSpeed(steering, tocdo);
      break;
      /////////////////////////////////////////////////////////////////////

    case 0b00110000:
    case 0b00100000:
    case 0b01110000:
    case 0b01111000:

      line = -1;
      handleAndSpeed(steering, tocdo);
      break;
    case 0b01100000:
    case 0b01000000:
    case 0b11100000:

      line = -2;
      handleAndSpeed(steering, tocdo);
      break;
    case 0b11000000:
    case 0b10000000:
    case 0b11110000:
    case 0b11111000:
    case 0b11111100:

      line = -3;
      handleAndSpeed(steering, tocdo);
      break;

      /////////////////////////////////////////////////////////////////
    default:
      handleAndSpeed(steering, tocdo);
      break;
  }
}
void tien() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void lui() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void phai() {
  analogWrite(ENA, Speed / 5);
  analogWrite(ENB, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void trai() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed / 5);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void tien_trai() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed / 4);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void tien_phai() {
  analogWrite(ENA, Speed / 4);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void lui_trai() {
  analogWrite(ENA, Speed / 4);
  analogWrite(ENB, Speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void lui_phai() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed / 4);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void Stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}