// Nama: Tegar Raditya Hikmawan
// NIM: 24/535872/SV/24321

#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// FOLLOWING-AXIS SERVO MOTOR AND MOTION DETECTION

// ESP32 mengontrol 5 servo berdasarkan data sensor MPU6050 dan deteksi gerakan dari sensor PIR.

// Komponen yang digunakan:
//  - ESP32 Dev Board
//  - 1x MPU6050 (accelerometer + gyroscope)
//  - 1x PIR Sensor
//  - 5x Servo Motor

// Fitur utama:
//  - Servo bergerak mengikuti kemiringan (roll, pitch, yaw)
//  - Saat PIR mendeteksi gerakan, semua servo bergerak ke arah yang sama kemudian kembali ke posisi semula.

Adafruit_MPU6050 mpu;
Servo servo1, servo2, servo3, servo4, servo5;

// PIN servo
#define pinServo1 19
#define pinServo2 5
#define pinServo3 18
#define pinServo4 17
#define pinServo5 16
#define pinPIR 13      // Pin sensor PIR (output)

float roll, pitch, yaw;  // Variabel untuk sudut orientasi
int servoAwal = 90;      // Posisi awal servo (netral)

// Setup
void setup() {
  Serial.begin(115200);

  // Inisialisasi MPU6050
  if (!mpu.begin()) {
    Serial.println("Gagal inisialisasi MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 Siap.");

  // Set PIR sensor sebagai input
  pinMode(pinPIR, INPUT);

  // Hubungkan setiap servo dengan pin-nya
  servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  servo3.attach(pinServo3);
  servo4.attach(pinServo4);
  servo5.attach(pinServo5);

  // Set posisi awal semua servo ke tengah
  servo1.write(servoAwal);
  servo2.write(servoAwal);
  servo3.write(servoAwal);
  servo4.write(servoAwal);
  servo5.write(servoAwal);
}

// Main Loop
void loop() {
  // Baca data sensor dari MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // HITUNG ORIENTASI
  // Rumus untuk menghitung kemiringan (dalam derajat)
  // roll  = sudut kemiringan kiri/kanan
  // pitch = sudut kemiringan depan/belakang
  // yaw   = arah rotasi (dari gyro)
  roll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  yaw = g.gyro.z;

  // BACA SENSOR PIR
  int pirState = digitalRead(pinPIR);  // HIGH jika ada gerakan

  // LOGIKA KEMIRINGAN (MPU6050)
  // Roll
  if (roll > 10) {
    // Miring ke kanan jika roll positif
    servo1.write(servoAwal - 90);
    servo2.write(servoAwal - 90);
  } else if (roll < -10) {
    // Miring ke kiri ke kiri jika roll negatif
    servo1.write(servoAwal + 90);
    servo2.write(servoAwal + 90);
  } else {
    // Netral
    servo1.write(servoAwal);
    servo2.write(servoAwal);
  }

  // Pitch
  if (pitch > 10) {
    // Miring ke depan jika roll positf
    servo3.write(servoAwal + 90);
    servo4.write(servoAwal + 90);
  } else if (pitch < -10) {
    // Miring ke belakang jika roll negatif
    servo3.write(servoAwal - 90);
    servo4.write(servoAwal - 90);
  } else {
    // Netral
    servo3.write(servoAwal);
    servo4.write(servoAwal);
  }

  // Yaw
  if (abs(yaw) > 1) {
    // Jika ada rotasi
    int arahYaw = (yaw > 0) ? 1 : -1;
    servo5.write(servoAwal + (90 * arahYaw));
    delay(1000);
    servo5.write(servoAwal);
  }

  // LOGIKA SENSOR PIR
  if (pirState == HIGH) {
    // Jika PIR mendeteksi gerakan, semua servo bergerak ke titik sama
    int targeted = 127; // Titik target yang sama (bisa disesuaikan)

    // Gerakkan semua servo bersamaan
    servo1.write(targeted);
    servo2.write(targeted);
    servo3.write(targeted);
    servo4.write(targeted);
    servo5.write(targeted);

    delay(1000); // Agar servo diam sejenak

    // Mengembalikan semua servo ke posisi awal
    servo1.write(servoAwal);
    servo2.write(servoAwal);
    servo3.write(servoAwal);
    servo4.write(servoAwal);
    servo5.write(servoAwal);

    delay(500); // Delay sebentar
  }
}
