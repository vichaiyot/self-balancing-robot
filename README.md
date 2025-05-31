# self-balancing-robot

โปรเจกต์หุ่นยนต์ทรงตัวด้วยตัวเอง (Self-Balancing Robot) ที่ใช้ MPU6050 สำหรับตรวจจับการเอียง, มอเตอร์ DC พร้อมไดรเวอร์ L298N สำหรับควบคุมการเคลื่อนที่ และใช้ PID ควบคุมสมดุล พร้อมควบคุมการเคลื่อนที่ผ่านรีโมต IR

## 🔧 Features

- ควบคุมสมดุลด้วย MPU6050 และอัลกอริธึม PID
- ควบคุมทิศทางด้วยรีโมตอินฟราเรด (IR Remote)
- รองรับการปรับค่า PID แบบง่ายในโค้ด
- ใช้งานร่วมกับ Arduino Uno

## 🧰 Hardware Requirements

- Arduino Uno
- MPU6050 (Gyroscope + Accelerometer)
- Motor Driver L298N
- DC Motors x2
- ล้อ (Wheels)
- แบตเตอรี่ 7.4V หรือ 12V
- รีโมต IR และตัวรับ IR
- ตัวถังหุ่นยนต์ (Chassis)

## 🚀 Getting Started

1. ต่ออุปกรณ์ตามวงจรใน `images/robot_diagram.png`
2. เปิด `main.ino` ใน Arduino IDE
3. อัปโหลด Library ที่เกียวข้อง
4. ปรับค่าพินตามที่เชื่อมต่อในโค้ด PID , offset , setpoint
5. อัปโหลดโค้ดไปยัง Arduino Uno
6. ทดสอบการทรงตัวบนพื้นเรียบ และใช้รีโมตควบคุม

## ⚙️ PID Tuning

ค่าเริ่มต้นในโค้ด:

```cpp
double Kp = 40;
double Ki = 280;
double Kd = 0.3;
