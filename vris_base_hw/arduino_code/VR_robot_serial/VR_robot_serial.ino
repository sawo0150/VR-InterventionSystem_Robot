#include <Arduino.h>

// ---- 모터 드라이버 핀 정의 (기존 그대로) ----
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

// 최대 PWM 값
const int MAX_PWM = 255;

// 디버깅용 LED (보드 내장 LED)
const int LED_PIN = LED_BUILTIN;

// 디버깅/워치독용 상태 변수
unsigned long lastCmdTime = 0;
unsigned long lastStatusPrint = 0;
int lastLeftPWM = 0;
int lastRightPWM = 0;
long cmdCount = 0;

// ---------------- 모터 제어 함수들 (기존 그대로) ----------------

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

// 두 바퀴 모두 정지
void stopMotors() {
  leftMotorStop();
  rightMotorStop();
}

// ---------------- PWM 적용 함수 ----------------

// 왼/오 PWM(-255~255)을 받아서 방향+세기로 변환
void setMotorPWM(int leftPWM, int rightPWM) {
  // 안전 범위 제한
  leftPWM  = constrain(leftPWM,  -MAX_PWM, MAX_PWM);
  rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

  // 왼쪽 모터
  if (leftPWM > 0) {
    leftMotorForward(leftPWM);
  } else if (leftPWM < 0) {
    leftMotorBackward(-leftPWM);  // 절댓값
  } else {
    leftMotorStop();
  }

  // 오른쪽 모터
  if (rightPWM > 0) {
    rightMotorForward(rightPWM);
  } else if (rightPWM < 0) {
    rightMotorBackward(-rightPWM);
  } else {
    rightMotorStop();
  }
}

void setup() {
  Serial.begin(115200);   // ROS2 노드와 동일한 baudrate로 맞추기

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

  // parseInt 타임아웃 (ms)
  // 너무 짧으면 두 번째 숫자를 0으로 읽는 경우가 있어서 약간 여유를 줌
  Serial.setTimeout(50);

  // 디버깅용 LED 설정
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  stopMotors();

  Serial.println("Arduino motor controller started");
}

void loop() {
  
  unsigned long now = millis();

  // ----- 워치독: 일정 시간 동안 명령이 없으면 정지 -----
  if ((now - lastCmdTime) > 500 && (lastLeftPWM != 0 || lastRightPWM != 0)) {
    setMotorPWM(0, 0);
  }

  // 주기적으로 상태 출력 (1초마다)
  if (now - lastStatusPrint > 1000) {
    lastStatusPrint = now;
    Serial.print("[STATUS] t=");
    Serial.print(now);
    Serial.print(" ms, cmds=");
    Serial.print(cmdCount);
    Serial.print(", last L=");
    Serial.print(lastLeftPWM);
    Serial.print(", R=");
    Serial.println(lastRightPWM);
  }

  // 시리얼로부터 "왼PWM 오른PWM\n" 형식으로 값 수신
  if (Serial.available() > 0) {
    // 숫자 2개 읽기 (parseInt는 숫자/마이너스 외엔 무시)
    int leftPWM  = Serial.parseInt();   // 첫 번째 숫자
    int rightPWM = Serial.parseInt();   // 두 번째 숫자

    // 줄 끝까지 버퍼 비우기 (옵션)
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
    }

    // 디버깅용 상태 업데이트
    lastCmdTime = now;
    lastLeftPWM = leftPWM;
    lastRightPWM = rightPWM;
    cmdCount++;

    // 값이 실제로 들어왔는지 간단 체크 (필요시 추가)
    setMotorPWM(leftPWM, rightPWM);

    // 디버그 출력
    Serial.print("Set PWM L: ");
    Serial.print(leftPWM);
    Serial.print("  R: ");
    Serial.println(rightPWM);

    // 명령 한 번 받을 때마다 LED 토글 (깜빡임으로 상태 확인)
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
  }
}
