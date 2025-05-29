#include <Wire.h>
#include <MPU6050_tockn.h>
#include <PID_v1.h>
#include "motorcontrol.h"
#include "IRController.h"

#define MIN_ABS_SPEED_A 0
#define MIN_ABS_SPEED_B 0

#define IN1 8
#define IN2 9
#define ENA 6
#define ENB 5
#define IN3 13
#define IN4 12

MPU6050 mpu6050(Wire);
IRController ir(3);

double originalSetpoint = -2.42;
double input, output, setpoint = originalSetpoint;
double Kp = 40;
double Ki = 280;
double Kd = 0.3;

double speedOffset = 0, trunOffset = 0, realOutput;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

motorcontrol my_motor(ENA, IN1, IN2, ENB, IN3, IN4, 0.9, 1.0);  // SpeedFactorA, SpeedFactorB

unsigned long commandTimeout = 100;  // สำหรับเข็คว่ามี IRcommand ไหมใน 0.2 วิ

double x_offset = -3.47, y_offset = 0.98, z_offset = -1.44;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(x_offset, y_offset, z_offset);
  // mpu6050.calcGyroOffsets(true);  // วางหุ่นยนต์นิ่งบนพื้นราบ แล้วเรียกฟังก์ชันนี้


  my_motor.begin();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  ir.begin();
  Serial.println("Ready!");
  delay(2000);
}

void loop() {
  // อ่านข้อมูลจากเซนเซอร์
  mpu6050.update();
  input = mpu6050.getAngleX();

  // if (input > originalSetpoint + 10 || input < originalSetpoint-10) {
  //    input = originalSetpoint;
  // }

  // คำนวณค่า PID
  pid.Compute();
  
  unsigned long now = millis();
  
  if (ir.available()) {
      IRCommand cmd = ir.getCommand();
      ir.resume();
      ir.updateLastCommandTime();

      switch (cmd) {
          case IRCommand::Forward:
              // เดินหน้า
              if (setpoint < originalSetpoint + 6)
                setpoint += 6;
              break;

          case IRCommand::Backward:
              // ถอยหลัง 
              if (setpoint > originalSetpoint - 7.4)
                setpoint -= 7.4;
              break;

          case IRCommand::Left:
              // เลี้ยวซ้าย
              // my_motor.turnLeft(255);
              trunOffset = -120;
              break;

          case IRCommand::Right:
              // เลี้ยวขวา
              // my_motor.turnRight(255);
              trunOffset = 120;
              break;

          // case IRCommand::Boot:
          //     // ถอยหลัง 
          //     if (setpoint > originalSetpoint + 10)
          //       setpoint += 10;
          //     break;

          default:
              break;
      }
  }

  // กลับสู่สภาวะ Default ถ้าไม่มีคำสั่ง IR มานานเกินไป
  if (now - ir.getLastCommandTime() > commandTimeout) {
     setpoint = originalSetpoint;
     trunOffset = 0;
  }
  my_motor.move(output, speedOffset, trunOffset, MIN_ABS_SPEED_A, MIN_ABS_SPEED_B);

  // แสดงข้อมูลใน Serial Monitor
  Serial.print(" | Angle: ");
  Serial.print(input);
  Serial.print(" | Output: ");
  Serial.print(output);
  Serial.print(" | Setpoint: ");
  Serial.println(setpoint);
  delay(10);
}