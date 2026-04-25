#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---- PCA9685 (địa chỉ 0x41, giống Robot_Arm) ----
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

#define USMIN      500
#define USMAX      2400
#define SERVO_FREQ 50

// ---- Servo channels ----
#define NUM_SERVOS 4
uint8_t servoChannels[NUM_SERVOS] = {4, 5, 6, 7};

// =====================================================
// Set góc cho 1 servo (0-180°)
// =====================================================
void setServoAngle(uint8_t channel, float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  int pulse = (int)((angle / 180.0f) * (USMAX - USMIN) + USMIN);
  pwm.writeMicroseconds(channel, pulse);
}

// =====================================================
// Set góc cho 4 servo cùng lúc
// servoAngles[0] → ch4, [1] → ch5, [2] → ch6, [3] → ch7
// =====================================================
void setAllServoAngles(float servoAngles[NUM_SERVOS]) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(servoChannels[i], servoAngles[i]);
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  pwm.begin(); 
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Set servo 4,5,6,7 = 0°, 180°, 0°, 0°
  float initAngles[NUM_SERVOS] = { 90, 180, 0, 0 };
  setAllServoAngles(initAngles);

  Serial.println("Servo set: ch4=0, ch5=180, ch6=0, ch7=0");
}

void loop() {
  // Không làm gì thêm
}
