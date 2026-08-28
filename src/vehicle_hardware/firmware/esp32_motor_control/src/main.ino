/*
 * ESP32 差速小车固件 — N20 编码器电机 + L298N 驱动
 *
 * 通信协议（串口 115200 baud）：
 *   PC → ESP32:  "M <左PWM> <右PWM>\n"    例: "M 150 150\n"（前进）
 *   ESP32 → PC:  "E <左脉冲> <右脉冲>\n"   每 50ms 发送一次
 *   ESP32 → PC:  "I <ax> <ay> <az> <gx> <gy> <gz>\n"  IMU 数据（如有）
 *
 * 引脚接线：
 *   L298N: ENA=26, IN1=27, IN2=14, IN3=32, IN4=33, ENB=25
 *   编码器: 左A=34, 左B=35, 右A=18, 右B=19
 */

//蓝牙
 #include<BluetoothSerial.h>
 BluetoothSerial SerialBT;

// ===== 电机引脚 =====
const int ENA = 26, IN1 = 27, IN2 = 14;  // 左电机
const int ENB = 25, IN3 = 32, IN4 = 33;  // 右电机

// ===== 编码器引脚 =====
const int ENC_L_A = 34, ENC_L_B = 35;
const int ENC_R_A = 18, ENC_R_B = 19;

// ===== 编码器计数器 =====
volatile long enc_left  = 0;
volatile long enc_right = 0;

// ===== 电机参数 =====
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;   // 0-255
const int PWM_MAX  = 255;

// ===== 定时发送 =====
unsigned long last_send = 0;
const unsigned long SEND_INTERVAL = 50;  // ms

// ===== 前置声明（Arduino 预处理会重排函数顺序） =====
enum Motor { LEFT, RIGHT };
void set_motor(Motor side, int pwm);
void stop();

void IRAM_ATTR on_enc_left() {
  if (digitalRead(ENC_L_B)) enc_left++; else enc_left--;
}

void IRAM_ATTR on_enc_right() {
  if (digitalRead(ENC_R_B)) enc_right++; else enc_right--;
}


void setup() {
  SerialBT.begin("car");

  // ---- 电机引脚 ----
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(ENA, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(ENB, 1);

  // ---- 编码器引脚 ----
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), on_enc_left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), on_enc_right, CHANGE);

  stop();
  SerialBT.println("ESP32 ready");
}


void loop() {
  // ---- 1) 接收 PC 指令 ----
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("M ")) {
      int sp1 = cmd.indexOf(' ', 2);
      int left  = cmd.substring(2, sp1).toInt();
      int right = cmd.substring(sp1 + 1).toInt();
      set_motor(LEFT,  left);
      set_motor(RIGHT, right);
    }
  }

  // ---- 2) 定时发送编码器数据 ----
  if (millis() - last_send >= SEND_INTERVAL) {
    last_send = millis();
    SerialBT.print("E ");
    SerialBT.print(enc_left);
    SerialBT.print(" ");
    SerialBT.println(enc_right);
  }
}


// ==================== 电机控制 ====================

void set_motor(Motor side, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  int en, in1, in2;
  if (side == LEFT)  { en = 0; in1 = IN1; in2 = IN2; }
  else               { en = 1; in1 = IN3; in2 = IN4; }

  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  // 刹车
  }
  ledcWrite(en, abs(pwm));
}

void stop() {
  set_motor(LEFT, 0);
  set_motor(RIGHT, 0);
}
