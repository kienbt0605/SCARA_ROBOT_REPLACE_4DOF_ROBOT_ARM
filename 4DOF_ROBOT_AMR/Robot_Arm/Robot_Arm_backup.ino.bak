#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Khởi tạo đối tượng PCA9685 ở địa chỉ 0x41 (đã hàn A0)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Thông số xung cho MG996R 
#define USMIN  500  // Độ rộng xung cho góc 0 độ
#define USMAX  2400 // Độ rộng xung cho góc 180 độ
#define SERVO_FREQ 50 // Tần số 50Hz cho servo

// Mảng kênh cắm servo (kênh 4 đến 7)
#define NUM_SERVOS 4
uint8_t servoChannels[NUM_SERVOS] = {4, 5, 6, 7};
/*
  servo 4: trục xoay tổng
  servo 5: khớp 1
  servo 6: khớp 2
  servo 7: khớp 3
*/

// Mảng góc mặc định cho từng servo (độ)
int servoAngles[NUM_SERVOS] = {0, 0, 0, 90};

//
void setup() {
  Serial.begin(115200);
  Serial.println("Khoi dong PCA9685 - 4 Servo (kenh 4-7)!");
  Wire.begin(21, 22);

  pwm.begin();
  // pwm.setOscillatorFrequency(27000000); // Bạn có thể thêm // vào đầu dòng này nếu servo bị rung
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(10);
  
  // Set góc cho cả 4 servo cùng lúc
  setAllServos(servoAngles);
}

// Hàm đổi từ Góc (0-180) sang Độ rộng xung (us)
int angleToPulse(int angle) {
  return map(angle, 0, 180, USMIN, USMAX);
}

// Hàm set góc cho cả 4 servo cùng lúc
void setAllServos(int angles[]) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    int pulse = angleToPulse(angles[i]);
    pwm.writeMicroseconds(servoChannels[i], pulse);
    Serial.print("Servo kenh ");
    Serial.print(servoChannels[i]);
    Serial.print(" -> ");
    Serial.print(angles[i]);
    Serial.println(" do");
  }
}

void loop() {
  // Ví dụ: thay đổi góc cho cả 4 servo
  // int newAngles[NUM_SERVOS] = {45, 90, 135, 0};
  // setAllServos(newAngles);
}