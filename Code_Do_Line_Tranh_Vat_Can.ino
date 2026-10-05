#include <Wire.h> //thư viện giao tiếp I2C
#include <LiquidCrystal.h>  // Thư viện LCD 4-bit
#include <EEPROM.h>//thư viện đọc/ghi dữ liệu vào bộ nhớ vĩnh viễn Adruino

// LCD 1602 sử dụng giao tiếp 4-bit
LiquidCrystal lcd(0, 1, 2, 3, 4, 5);  // RS=0, EN=1, D4=2, D5=3, D6=4, D7=5

//L298N
#define IN1 11   // Motor trái
#define IN2 10   // chân điều khiển tốc độ/hướng motor trái
#define IN3 9  // Motor phải
#define IN4 6 // chân điều khiển tốc độ/hướng motor phải

// EEPROM address
#define EEPROM_ADDR 0

// CB HCSR04
#define TRIG_PIN 7// chân phát sóng siêu âm
#define ECHO_PIN 8// chân nhận sóng siêu âm phản hồi


//TCRT5000
int threshold[5] = {500, 500, 500, 500, 500}; // tùy chỉnh theo ánh sáng
const int pinSensor[5] = {A0, A1, A2, A3, A4}; //5 chân analog đọc dữ liệu từ cảm biến
float weight[5] = {-2, -1, 0, 1, 2};  // trọng số gán cho mỗi mắt để tính toán độ lệch PID
bool sensorValue[5]; // lưu trạng thái line (0:ngoài vạch,1:trên vạch)
unsigned long pressStartTime = 0;
bool buttonState;

#define BTN 12// chân nút điều khiển
#define COI 13 // chân còi báo(buzzer)

// ---- Lưu threshold vào EEPROM ----
void saveThreshold(){
  for(int i=0;i<5;i++){
    //Serial.print(threshold[i]);
    //Serial.print("  |  ");
    EEPROM.put(EEPROM_ADDR + i*2, threshold[i]); // mỗi int 2 byte vào bộ nhớ
  }  //Serial.println();
}
// ---- Đọc threshold từ EEPROM ----
void loadThreshold(){
  for(int i=0;i<5;i++){
    EEPROM.get(EEPROM_ADDR + i*2, threshold[i]);// lấy dữ liệu từ địa chỉ tương ứng
  }
}
void readSensor() { //đọc giá trị từ 5 cảm biến hồng ngoại
  int count = 0;
  int sumArr[] = {0,0,0,0,0,0,0,0};
  while(count < 1) {
    for(int i=0; i<5; i++){
      sumArr[i] += analogRead(pinSensor[i]);
    }
    count++;
  }
  for(int i=0; i<5; i++) {
    if((sumArr[i]/count) > threshold[i]) sensorValue[i] = 1;// nếu giá trị> ngưỡng thì trên vạch
    else sensorValue[i] = 0;// ngược lại là ngoài vạch
    //Serial.print(sensorValue[i]);
    //Serial.print("  |  ");
  } //Serial.println();
}
// Hàm trả về khoảng cách (cm)
long readDistance() { //đo khoảng cách bằng sóng siêu âm
  long duration;
  long distance;

  // Tạo xung trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);// phát xung 10 micro giây
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Đọc echo với TIMEOUT 20ms (~3.4m)
  duration = pulseIn(ECHO_PIN, HIGH, 20000);// đo thời gian xung phản hồi

  // Không có phản xạ → bỏ qua
  if (duration == 0) {
    return -1;
  }

  // Tính khoảng cách (cm)
  distance = duration * 0.034 / 2;// vận tốc âm thanh.thời gian/2

  // Lọc giá trị nhiễu
  if (distance < 2 || distance > 200) {
    return -1;
  }

  return distance;
}

void beep(int n) {
  for(int i = 0; i < n; i++) {
    digitalWrite(COI, HIGH);  // Bật buzzer
    delay(70);               // thời gian kêu 200ms
    digitalWrite(COI, LOW);   // Tắt buzzer
    delay(70);               // nghỉ giữa các beep 200ms
  }
}
float getError() { // thuật toán điều khiển PID
  readSensor();
  int sum = 0;
  int count = 0;
  for(int i = 0; i < 5; i++) {
    sum += weight[i] * sensorValue[i];// tổng trọng số các mắt đang thấy vạch
    count += sensorValue[i];// số lượng mắt đang thấy vạch
  }
  return (count>0) ? (float)sum/count : NAN; // outline thì trả về NAN
}
// Hàm điều khiển bánh trái
void motorLeft(int speed) {
  if (speed > 0) {
    analogWrite(IN1, speed);
    digitalWrite(IN2, LOW);
  } 
  else if (speed < 0) {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, -speed);
  } 
  else { // speed = 0
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
// ---- Calibrate threshold ----
void calibrateThreshold() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" Press BTN > 2s");
  lcd.setCursor(0,1);
  lcd.print("  To CALIBRATE");

  bool skip = true;
  bool longPress = false;

  int inlineValue[5];
  int outlineValue[5];

  for (int i = 0; i < 5; i++) {
    inlineValue[i] = 1023;
    outlineValue[i] = 0;
  }

  /* ====== CHỜ NHẤN NÚT ====== */
  while (true) {
    buttonState = digitalRead(BTN);
    if (buttonState == LOW) {
      delay(20); // debounce
      pressStartTime = millis();
      beep(1);

      while (digitalRead(BTN) == LOW) {
        if (millis() - pressStartTime >= 2000) {
          skip = false;
          longPress = true;
          break;
        }
      }
      break;
    }
  }

  if (longPress) {
    beep(3);
  }

  delay(800);

  /* ====== LOAD / CALIB ====== */
  if (!skip) {

    /* ====== IN-LINE ====== */
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  Put ON line");
    lcd.setCursor(0,1);
    lcd.print("   Press BTN");

    while (digitalRead(BTN) == LOW);
    while (digitalRead(BTN) != LOW);

    digitalWrite(COI, HIGH);
    delay(300);

    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 5; j++) {
        inlineValue[j] = min(inlineValue[j], analogRead(pinSensor[j]));
        //Serial.print(inlineValue[j]);
        //Serial.print("  |  ");
      } //Serial.println();
    }

    digitalWrite(COI, LOW);

    /* ====== OUT-LINE ====== */
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  Put OUT line");
    lcd.setCursor(0,1);
    lcd.print("   Press BTN");

    while (digitalRead(BTN) == LOW);
    while (digitalRead(BTN) != LOW);

    digitalWrite(COI, HIGH);
    delay(300);

    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 5; j++) {
        outlineValue[j] = max(outlineValue[j], analogRead(pinSensor[j]));
        //Serial.print(outlineValue[j]);
        //Serial.print("  |  ");
      } //Serial.println();
    }

    digitalWrite(COI, LOW);

    /* ====== TÍNH THRESHOLD ====== */
    for (int i = 0; i < 5; i++) {
      threshold[i] = (inlineValue[i] + outlineValue[i]) / 2;
    }

    saveThreshold();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  CALIB DONE!");
    lcd.setCursor(0,1);
    lcd.print("  Saved to MEM");
    delay(1200);
    beep(3);
    delay(1200);

  } else {

    /* ====== LOAD TỪ BỘ NHỚ ====== */
    loadThreshold();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   Skip Calib");
    lcd.setCursor(0,1);
    lcd.print("   Load MEMORY");
    beep(3);
    delay(1200);
  }
}
// Hàm điều khiển bánh phải
void motorRight(int speed) {
  if (speed > 0) {
    analogWrite(IN3, speed);
    digitalWrite(IN4, LOW);
  } 
  else if (speed < 0) {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, -speed);
  } 
  else { // speed = 0
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}
int limitMinPWM(int pwm, int minPWM) {
  if (pwm > 0 && pwm < minPWM) return minPWM;
  if (pwm < 30 && pwm > -minPWM) return -minPWM;
  return pwm;
}

// PID
float lastError = 0;
float baseSpeed = 70;
float kp = 45;
float kd = 7;
unsigned long lastTime = 0;
unsigned long lastLCD = 0;
// PID control

void motorControl(){
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt < 0.01) dt = 0.01;

  float error = getError();

  if (isnan(error)) {
    motorLeft(70);
    motorRight(70);
    return;
  }

  float P = kp * error;
  float D = kd * (error - lastError) / dt;
  lastError = error;

  int leftSpeed, rightSpeed;

  // ===== CUA GẮT =====
  if (abs(error) > 1.5) {
    leftSpeed  =  P;
    rightSpeed = -P;
  }
  // ===== CUA NHẸ / THẲNG =====
  else {
    leftSpeed  = baseSpeed + P + D;
    rightSpeed = baseSpeed - P - D;
  }

  // ---- ÉP PWM TỐI THIỂU ----
  leftSpeed  = limitMinPWM(leftSpeed,  70);
  rightSpeed = limitMinPWM(rightSpeed, 70);

  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motorLeft(leftSpeed);
  motorRight(rightSpeed);
}
void lcdLineMode (long distance) {
  lcd.setCursor(0, 0);
  lcd.print("FOLLOW LINE MODE");

  lcd.setCursor(0, 1);
  lcd.print("   KC: ");
  if (distance == -1) {
    lcd.print(" cm ");
  } else {
    lcd.print(distance);
    lcd.print("cm   ");
  }
}

void lcdObstacleMode(long distance) {
  lcd.setCursor(0, 0);
  lcd.print(" !!! CO VAT !!! ");

  lcd.setCursor(0, 1);
  lcd.print("   KC: ");
  lcd.print(distance);
  lcd.print("cm   ");
}


void setup() {
  lcd.begin(16, 2); 

  pinMode(BTN, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(COI, OUTPUT);

  calibrateThreshold();
}

void loop() {
  long kc = readDistance();
  // ===== HIỂN THỊ LCD (mỗi 250ms) =====
  if (millis() - lastLCD > 250) {
    lastLCD = millis();
    if (kc > 15 ) lcdLineMode(kc);
  }
  // Tránh Vật Cản
  if (kc <  15 && kc != -1) {
    lcdObstacleMode(kc);
    motorLeft(-90);
    motorRight(-90);
    beep(3); delay(200);
    motorLeft(90);
    motorRight(-90);
    delay(380);
    motorLeft(0);
    motorRight(0);
    delay(200);
    motorLeft(90);
    motorRight(90);
    delay(800);
    motorLeft(0);
    motorRight(0);
    delay(100);
    motorLeft(-80);
    motorRight(80);
    delay(270);
    motorLeft(90);
    motorRight(60);
    while(sensorValue[0] == 0 || sensorValue[1] == 0 ){
      readSensor();
    }
    motorLeft(80);
    motorRight(-80);
    delay(50);
  }
  motorControl();
}

