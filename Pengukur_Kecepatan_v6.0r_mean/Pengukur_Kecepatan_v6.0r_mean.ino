/*******************************************************************************#
#                                                                               #
# Pengukur kecepatan menggunakan Arduino Leonardo, modul sensor reflective IR   #
# dan reflective incremental rotary encoder                                     #
#                                                                               #
# Copyright (C) 2018 Muhammad Shalahuddin Yahya Sunarko                         #
#                                                                               #
# This program is free software: you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation, either version 3 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
# You should have received a copy of the GNU General Public License             #
# along with this program.  If not, see <http://www.gnu.org/licenses/>          #
#                                                                               #
********************************************************************************/

/*
   Encoder: 20 degree each: alternating black and white stripes
   Diameter roda: 2.47 cm
   Mode: change
   Periode pulsa frekuensi tinggi (timer), Thf = 1/100000 s, fhf = 100000 Hz
   Rentang pengukuran kecepatan (omega): min. 1 rad/s, max. 200 rad/s
   Rentang pengukuran kecepatan (v): min. 2.5 cm/s, max. 500 cm/s
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define a 0.742667756402082 // faktor kalibrasi f(x)=ax+b
#define b 0 // faktor kalibrasi f(x)=ax+b
#define sensor 7 // pin sensor
#define r_roda 2.47 // jari-jari roda (cm)
#define Thf 0.00001 // s
#define omegamin 0.1 // rad/s
#define omegamax 250 // rad/s
#define BAUDRATE 115200 // baudrate B115200

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // atur alamat LCD I2C
const float dtheta = 20 * PI / 180; // panjang pola hitam/putih (rad). jika mode rising/falling, maka dtheta = panjang pola hitam + putih
const long n_omegamin = dtheta / (omegamin * Thf);
const long n_omegamax = dtheta / (omegamax * Thf);
volatile int i = 0;
volatile unsigned long n = 0;
volatile unsigned long n_copy = 0;
volatile unsigned long n_before = 0;
volatile unsigned long n_diff = 0;
volatile float buf[10] = {0};
volatile float omega = 0;
volatile float v = 0;
float v_buf = 0;
int omega_dummy = 123; // dummy data
int i_lcd = 0;
char charbuf[8];

void setup() {
  /* Inisiasi pin */
  pinMode(sensor, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensor), hitungomega, CHANGE);

  /* Inisiasi Timer1*/
  noInterrupts();
  TCCR1A = 0; // bersihkan register A timer1
  TCCR1B = 0; // bersihkan register B timer1
  TCNT1 = 65536 - (16000000 * Thf); // muat 65536-(16MHz/prescaler)*Thf agar timer memiliki Thf = 1/100000 s
  TCCR1B |= (1 << CS10); // tanpa prescaler (prescaler = 1)
  TIMSK1 |= (1 << TOIE1); // aktifkan timer overflow interrupt
  interrupts();

  /* Inisiasi komunikasi serial */
  Serial.begin(BAUDRATE);

  /* Inisiasi LCD I2C */
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Pengukur Kecepa-");
  lcd.setCursor(0, 1);
  lcd.print("tan Gerak (cm/s)");
  delay(3000);
  TCNT1 = 65536 - (16000000 * Thf); // muat ulang
}

void loop() {
  if (n > n_omegamin) {
    n = 0; // reset n
    n_copy = 0; // reset n_copy
    v = 0;
    omega = 0;
    for (int k = 0; k < 10; k++) buf[k] = 0;
    i = 0;
  }
  v_buf = v; // kopi nilai v sehingga tidak berubah saat penampilan
  Serial.print("V"); // header data v
  sprintf(charbuf, "%04d", round(v_buf)); // penambahan angka 0 di depan nilai omega sehingga menjadi 4 digit
  Serial.println(charbuf); // kirim sebaris data melalui serial
  //Serial.println(round(v_buf));
  if (i_lcd >= 100) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("v (cm/s) :");
    lcd.setCursor(0, 1);
    lcd.print(charbuf);
    //lcd.print(round(v_buf));
    i_lcd = 0;
  }
  i_lcd++;
  delay(2);
}

void hitungomega() {
  n_before = n_copy; // kopi nilai n sebelumnya untuk filtrasi
  n_copy = n; // kopi nilai n sehingga tidak berubah saat perhitungan
  n_diff = n_copy - n_before;
  /* Jika 1 < omega < 200 rad/s (band pass filter kecepatan), maka hitung omega */
  if (n_diff < n_omegamax) {}
  else if (n_diff <= n_omegamin && n_diff >= n_omegamax) {
    n = 0; // reset n
    buf[i] = dtheta / (n_copy * Thf); // rad/s
    n_copy = 0; // reset n_copy
    i++;
    if (i > 9) {
      omega = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5]
               + buf[6] + buf[7] + buf[8] + buf[9]) / 10; // rad/s
      v = a * (omega * r_roda) + b; // cm/s
      i = 0;
    }
  }
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = 65536 - (16000000 * Thf); // muat ulang
  n++;
}
