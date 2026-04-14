
#include <Wire.h>
#include <SCServo.h>
#include <HardwareTimer.h>

SMS_STS st;
#define button PB10
#define LED_CHANNEL 1

HardwareSerial Serial2(PA3, PA2);
HardwareTimer *timer2 = new HardwareTimer(TIM4);
HardwareTimer *timerled = new HardwareTimer(TIM2);


void setup() {
  Serial1.setTx(PA9);
  Serial1.enableHalfDuplexRx();
  Serial1.setHalfDuplex();
  Serial1.begin(1000000);
  st.pSerial = &Serial1;  

  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.setClock(400000);
  Wire.begin();
  
  Serial2.begin(57600);
  
  
  pinMode(button, INPUT_PULLDOWN);

  HardwareTimer *timer1 = new HardwareTimer(TIM1);
  timer1->setOverflow(1400, HERTZ_FORMAT);
  timer1->attachInterrupt(UpdateEncoder);
  timer1->resume();

  timer2->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PB9);
  timer2->setPrescaleFactor(8);
  timer2->setOverflow(20000, MICROSEC_FORMAT);
  timer2->setCaptureCompare(4, 1500, MICROSEC_COMPARE_FORMAT);
  timer2->refresh();
  timer2->resume();

  timerled->setMode(LED_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, PA15);
  timerled->setPrescaleFactor(100 * 50);
  timerled->setOverflow(20000, TICK_FORMAT);
  timerled->setCaptureCompare(LED_CHANNEL, 10000, TICK_COMPARE_FORMAT);
  timerled->refresh();
  timerled->resume();
}

int angle = 0;
int accumulation = 0;
int last_angle = 0;

void UpdateEncoder() {

  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 2);
  int new_angle = 0;
  if (2 <= Wire.available()) {
    new_angle = Wire.read() & 0x0F;
    new_angle = new_angle << 8;
    new_angle |= Wire.read();
  }

  if (abs(new_angle - last_angle) > 3000) {
    if (new_angle > last_angle) {
      accumulation -= 4096;
    } else {
      accumulation += 4096;
    }
  }
  last_angle = new_angle;
  angle = accumulation + new_angle;
}

int num = 0;

int wait_for_packet(uint8_t *data) {
  while (true) {
    if (Serial2.available()) {
      uint8_t byte = Serial2.read();

      for (int i = 0; i < 2; i++) {
        if (byte == 0x35 && num == 0) {
          num++;
          break;
        }
        else if (byte == 0x80 && num == 1) {
          num++;
          break;
        }
        else if (num == 2) {
          data[0] = byte;
          num++;
          break;
        }
        else if (num == 3) {  
          data[1] = byte;
          num++;
          break;
        }
        else if (num == 4) {
          
          data[2] = byte;
          uint8_t response[7];
          response[0] = 0x35;
          response[1] = 0x01;
          memcpy(response + 2, &angle, sizeof(int));
          response[6] = digitalRead(button) ? 255 : 0; 
          Serial2.write(response, 7);
          num = 0;
          return 0x80;  
        
        }
        else {
          num = 0;
        }
      }
    }
  }
}

void loop() {
  
  uint8_t data[3] = { 127, 127, 127};
  int cmd = wait_for_packet(data);
  if (cmd == 0x80) {
    int steer_pos = 1400 + data[1] * (2600 - 1400) / 256;
    steer_pos = std::clamp(steer_pos, 1300, 2700);
    st.WritePosEx(1, steer_pos, 4096, 250);

    int drive_speed = 1060 + (1860 - 1060) * data[0] / 256;
    drive_speed = std::clamp(drive_speed, 1060, 1860);
    timer2->setCaptureCompare(4, drive_speed, MICROSEC_COMPARE_FORMAT);
    timerled->setPrescaleFactor(100 * data[2]);
    timerled->refresh();
    timerled->resume();
  }

}
