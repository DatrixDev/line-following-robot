#include <SoftwareSerial.h>
SoftwareSerial HC05(2, 3);
const int GREEN = 11;  //khi test rút dây
const int RED = 12;    //khi test rút dây
const int BLUE = 13;

// const BUZZER = 4; //Chưa mua BUZZER

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 10;

int Speed = 250;
bool modeLine = false;
int lastPosition;
int steering = 0;
float kp = 1;
float kd = 5;
int line = 0;
int cnt = 0;
int rememberLine = 0;
int speed_run_forward;
unsigned char pattern, start;
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


void timer_init() {
  ASSR = (0 << EXCLK) | (0 << AS2);
  TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (0 << WGM20);
  TCCR2B = (0 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
  TCNT2 = 0xB2;
  OCR2A = 0x00;
  OCR2B = 0x00;
  TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (1 << TOIE2);
}
ISR(TIMER2_OVF_vect) {
  TCNT2 = 0xB2;
  read_sensor();
  cnt++;
}
// PID cập nhật 200 lần/giây
void startLineTimer() {
  TCNT2 = 0xB2;
  TIMSK2 |= (1 << TOIE2);  // bật ngắt overflow
}
void stopLineTimer() {
  TIMSK2 &= ~(1 << TOIE2);  // tắt ngắt overflow
}
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
  pattern = 10;
  start = 0;
  isCalib = 0;
  timer_init();
  stopLineTimer();
  speed_run(0, 0);
  RGB(3);
}

void loop() {

  handleBluetooth();

  if (modeLine) {
    runStateMachine();
  }
}
void handleBluetooth() {

  if (!HC05.available()) return;

  char cmd = HC05.read();
  Serial.println(cmd);

  if (cmd == 'X') {
    Serial.println("Bat che do LINE FOLLOW");
    RGB(2);
    rememberLine = 0;
    cnt = 0;
    Stop();
    modeLine = true;
    startLineTimer();
    pattern = 10;
    start = 0;
    isCalib = 0;
    Serial.println("San sang calibration (bam F)");
    return;
  }

  if (cmd == 'x') {
    Serial.println("Tat che do LINE FOLLOW");
    Serial.println("Robot dung");
    RGB(0);
    Stop();
    modeLine = false;
    stopLineTimer();
    return;
  }

  if (modeLine) {

    if (start == 0) {
      if (isCalib == 0 && cmd == 'F') {
        Serial.println("Bat dau hoc mau line (Calibration)");
        RGB(0);
        isCalib = 1;
      } else if (isCalib == 1 && cmd == 'F') {
        Serial.println("Hoan tat calibration");
        Serial.println("San sang chon toc do");
        RGB(1);
        start = 1;
        isCalib = 0;
        speed_run_forward = 0;
      }
      return;
    }

    if (pattern == 10) {
      if (cmd == 'B') {
        Serial.println("Bat dau chay line - Toc do cham");
        RGB(1);
        pattern = 11;
        speed_run_forward = 100;
        cnt = 0;
      }
      if (cmd == 'L') {
        Serial.println("Bat dau chay line - Toc do trung binh");
        RGB(2);
        pattern = 11;
        speed_run_forward = 150;
        cnt = 0;
      }
      if (cmd == 'R') {
        Serial.println("Bat dau chay line - Toc do nhanh");
        RGB(0);
        pattern = 11;
        speed_run_forward = 200;
        cnt = 0;
      }
    }

    return;
  }

  switch (cmd) {

    case 'F': tien(); break;
    case 'B': lui(); break;
    case 'R': phai(); break;
    case 'L': trai(); break;
    case 'G': tien_trai(); break;
    case 'I': tien_phai(); break;
    case 'H': lui_trai(); break;
    case 'J': lui_phai(); break;
    case 'S': Stop(); break;
  }
}
void runStateMachine() {

  if (start == 0 && isCalib == 1) {
    learnLine();
    return;
  }

  if (start == 0) return;

  switch (pattern) {

    case 10:
      break;

    case 11:

      if (sensorMask(0x01) == 0x01) {
        rememberLine = 1;
        cnt = 0;
      } else if (sensorMask(0x80) == 0x80) {
        rememberLine = -1;
        cnt = 0;
      }

      if (sensor == 0b00000000) {

        if (rememberLine != 0) {

          if (rememberLine == 1) {
            pattern = 12;
            handleAndSpeed(40, speed_run_forward);
          } else if (rememberLine == -1) {
            pattern = 12;
            handleAndSpeed(-40, speed_run_forward);
          } else {
            pattern = 100;
          }

        } else {
          speed_run(0, 0);
        }

        break;
      } else {
        runforwardline(speed_run_forward);
      }

      if (sensorMask(0b00111100) != 0b00000000) {
        if (cnt > 50) rememberLine = 0;
      }

      break;

    case 12:

      if (rememberLine == 1) {
        speed_run(100, -40);
        pattern = 21;
        break;
      } else if (rememberLine == -1) {
        speed_run(-40, 100);
        pattern = 31;
        break;
      } else {
        pattern = 11;
        break;
      }

    case 21:
      speed_run(100, -40);
      RGB(1);

      if (sensorMask(0xff) != 0) {
        speed_run(60, 60 / 2);
        pattern = 22;
      }
      break;

    case 22:
      speed_run(60, 60 / 2);
      RGB(1);

      if (sensorMask(0xfc) != 0) {
        pattern = 11;
      }
      break;

    case 31:
      speed_run(-40, 60);
      RGB(2);

      if (sensorMask(0xff) != 0) {
        speed_run(60 / 2, 60);
        pattern = 32;
      }
      break;

    case 32:
      speed_run(60 / 2, 60);
      RGB(2);

      if (sensorMask(0x3f) != 0) {
        pattern = 11;
      }
      break;

    case 100:
      speed_run(0, 0);
      RGB(millis() / 100 % 3);
      break;

    default:
      pattern = 11;
      break;
  }
}
void waitForStart() {
  int led[] = { GREEN, RED };   
  learnLine();
  Blink(led, 2, 300);
  speed_run_forward = 0;
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
  if (sum != 0)
    i = (int)((avg / sum) - 3500);
  else
    i = 0;

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
  static unsigned long lastMotor = 0;

//   if (modeLine && millis() - lastMotor > 1000) {
//     lastMotor = millis();

//     Serial.print("Dong co trai (ENA): ");
//     Serial.print(speedLeft);
//     Serial.print(" | Dong co phai (ENB): ");
//     Serial.println(speedRight);
//   }

}
// void speed_run(int speedLeft, int speedRight) {

//   static unsigned long lastMotor = 0;

//   if (modeLine && millis() - lastMotor > 1000) {
//     lastMotor = millis();

//     Serial.print("Dong co trai (ENA): ");
//     Serial.print(speedLeft);
//     Serial.print(" | Dong co phai (ENB): ");
//     Serial.println(speedRight);
//   }

//   analogWrite(ENA, 0);
//   analogWrite(ENB, 0);
// }
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

static unsigned long lastMotorLog = 0;  
if (millis() - lastMotorLog > 1000) {
    lastMotorLog = millis();

    Serial.print("Tinh toan dong co -> Trai: ");
    Serial.print(speedLeft);
    Serial.print(" | Phai: ");
    Serial.println(speedRight);
  }

  speed_run(speedLeft, speedRight);
}
void learnLine() {
  if (sensor == 0xff) {
    int color[] = { RED, BLUE };
    Blink(color, 2, 300);  //line full đen  = nhảy màu đỏ - xanh
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

void RGB(int color) {
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
void Blink(int ledList[], int count, int time) {
  static int currentIndex = 0;

  if (millis() - prevMillis >= time) {
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
  static unsigned long lastLog = 0;

  if (millis() - lastLog >= 1000) {
    lastLog = millis();

    Serial.print("Cam bien: ");
    for (int i = 7; i >= 0; i--) {
      Serial.print((sensor >> i) & 1);
    }

    Serial.print(" | PID: ");
    Serial.print(steering);

    Serial.print(" | Line: ");
    Serial.println(line);
  }
}
unsigned char sensorMask(unsigned char mask) {
  return (sensor & mask);
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
  analogWrite(ENB, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void trai() {
  analogWrite(ENA, Speed);

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