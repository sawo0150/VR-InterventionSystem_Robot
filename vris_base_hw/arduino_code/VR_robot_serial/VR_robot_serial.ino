// ==== 모터 드라이버 핀 정의 ====
// 왼쪽 모터 드라이버
const int L_RPWM = 11;   // 왼쪽 RPWM (PWM)
const int L_LPWM = 10;   // 왼쪽 LPWM (PWM)
const int L_REN  = 4;    // 왼쪽 R_EN
const int L_LEN  = 5;    // 왼쪽 L_EN

// 오른쪽 모터 드라이버
const int R_RPWM = 9;    // 오른쪽 RPWM (PWM)
const int R_LPWM = 8;    // 오른쪽 LPWM (PWM)
const int R_REN  = 2;    // 오른쪽 R_EN
const int R_LEN  = 3;    // 오른쪽 L_EN

// 속도 값 (0~255)
const int SPEED_FORWARD = 255;
const int SPEED_BACK    = 255;
const int SPEED_TURN    = 255;

void setup() {
  Serial.begin(9600);   // ROS2 노드와 동일하게 맞출 것
  Serial.println("Arduino MOTOR DRIVER START");

  // 모터 핀 모드 설정
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_REN,  OUTPUT);
  pinMode(L_LEN,  OUTPUT);

  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_REN,  OUTPUT);
  pinMode(R_LEN,  OUTPUT);

  // 드라이버 Enable 활성화
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);

  // 처음엔 정지
  stopMotors();
}

void loop() {
  // PC(ROS2)에서 시리얼로 보내는 명령 처리
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("CMD: ");
    Serial.println(cmd);

    switch (cmd) {
      case 'F':   // 앞으로
        moveForward();
        break;
      case 'B':   // 뒤로
        moveBackward();
        break;
      case 'L':   // 왼쪽 회전
        turnLeft();
        break;
      case 'R':   // 오른쪽 회전
        turnRight();
        break;
      case 'S':   // 정지
      default:
        stopMotors();
        break;
    }
  }
}

// ---------------- 모터 제어 함수들 ----------------

// 왼쪽 모터: 정/역/정지
void leftMotorForward(int speed) {
  analogWrite(L_RPWM, speed);
  analogWrite(L_LPWM, 0);
}

void leftMotorBackward(int speed) {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, speed);
}

void leftMotorStop() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
}

// 오른쪽 모터: 정/역/정지
void rightMotorForward(int speed) {
  analogWrite(R_RPWM, speed);
  analogWrite(R_LPWM, 0);
}

void rightMotorBackward(int speed) {
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, speed);
}

void rightMotorStop() {
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
}

// ---------------- 동작 패턴 함수들 ----------------

void moveForward() {
  leftMotorForward(SPEED_FORWARD);
  rightMotorForward(SPEED_FORWARD);
}

void moveBackward() {
  leftMotorBackward(SPEED_BACK);
  rightMotorBackward(SPEED_BACK);
}

void turnLeft() {
  // 왼쪽 멈추고 오른쪽 앞으로
  leftMotorStop();
  rightMotorForward(SPEED_TURN);
}

void turnRight() {
  // 오른쪽 멈추고 왼쪽 앞으로
  rightMotorStop();
  leftMotorForward(SPEED_TURN);
}

void stopMotors() {
  leftMotorStop();
  rightMotorStop();
}
