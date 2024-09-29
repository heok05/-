// LED 핀이 연결된 GPIO 번호
const int ledPin = 7;

void setup() {
  // LED 핀을 출력으로 설정
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // 1초 동안 LED 켜기 (LED가 LOW일 때 켜짐)
  digitalWrite(ledPin, LOW);
  delay(1000);

  // 1초 동안 LED 5회 깜빡이기
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPin, HIGH);  // LED 끄기
    delay(100);                  // 0.1초 대기
    digitalWrite(ledPin, LOW);   // LED 켜기
    delay(100);                  // 0.1초 대기
  }

  // LED 끄기 (LED가 HIGH일 때 꺼짐)
  digitalWrite(ledPin, HIGH);

  // 무한 루프
  while (1) {
    // 무한히 대기 상태
 
  }
}
