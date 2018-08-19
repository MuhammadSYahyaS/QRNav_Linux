/*******************************************************************************#
#                                                                               #
# Moving QR Identification Software                                             #
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

#include <vector>
#include <sstream>
#include <string>
#include <chrono>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <pthread.h>
#include "withrobot_camera.hpp"
#include "tldpSerial.hpp"

#define EXP_MANUAL 1
#define EXP_AUTO 3

using namespace std;
using namespace chrono;

typedef high_resolution_clock Clock;

bool flag_jalan = 0;

int v_sens = 0;
const unsigned int bufsize = 16;
char data[bufsize];
tldpSerial kecepatan("/dev/ttyACM0");

void *serialReceive(void *) {
  while(flag_jalan){
      do kecepatan.receive(data, bufsize);
      while(data[0] != 'V');
      for(int i = 1; i <= 4; i++) data[i] -= '0';
      v_sens = data[1] * 1000 + data[2] * 100 + data[3] * 10 + data[4];
      std::fill(data, data + sizeof(data)/sizeof(data[0]), 0);
      usleep(100);
    }
  kecepatan.close();
  pthread_exit(NULL);
}

int main(void) {
  cv::Mat kernel = (cv::Mat_<char>(3, 3) << -1, -1, -1,
                                            -1, 9, -1,
                                            -1, -1, -1);
  cv::Mat frame_olahan;
  vector<cv::Point> vp;
  vector<cv::Point> matrixLokasiQR;
  vector<string> matrixDataQR;
  vector<double> matrixRotasiQR;
  system_clock::time_point t_ref, t_frame;
  stringstream ss;
  string dataQR, tanggal_jam, namaFile, namaFrame;
  duration<double> t_stamp, t_stamp_lama;
  double v_diinginkan, lux_diinginkan_buf[3], lux_diinginkan, t_exp_diinginkan, t_proses, rotasi, beta;
  unsigned long frame_ke = 0;
  int i, n, size, lebar, tinggi, fps;
  unsigned int iQR;
  char tombol, opsi_v, opsi_t;
  flag_jalan = 1;

  /* Inisialisasi Komunikasi serial */
  kecepatan.begin(B115200);
  pthread_t getData;
  int rc;
  rc = pthread_create(&getData, NULL, serialReceive, NULL);
  if (rc) {
      cout << "Error: unable to create thread," << rc << endl;
      exit(-1);
    }
  /*const unsigned int bufsize = 32;
  char data[bufsize];
  tldpSerial kecepatan("/dev/ttyACM0"); // alamat port komunikasi serial Arduino
  kecepatan.begin(B115200); // inisialisasi komunikasi dengan baud rate 115200 baud/s
  int v_sens = 0;*/

  /* Inisialisasi kamera */
  const char* devPath = "/dev/video1"; //lokasi perangkat kamera
  Withrobot::Camera camera(devPath);
  lebar = 1280;
  tinggi = 960;
  fps = 45;
  camera.set_format(lebar, tinggi, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, fps);
  Withrobot::camera_format camFormat;
  camera.get_current_format(camFormat);
  vector<std::pair<const char*, unsigned int>> valid_control_list;
  camera.valid_controls(valid_control_list);
  int brightness = camera.get_control("Gain");
  double t_exp = camera.get_control("Exposure (Absolute)");
  int wb_merah = camera.get_control("White Balance Red Component");
  int wb_biru = camera.get_control("White Balance Blue Component");
  brightness = 64;
  wb_biru = 125;
  wb_merah = 133;
  camera.set_control("Exposure, Auto", EXP_MANUAL);
  camera.set_control("Gain", brightness);
  camera.set_control("Exposure (Absolute)", t_exp);
  camera.set_control("White Balance Red Component", wb_merah);
  camera.set_control("White Balance Blue Component", wb_biru);
  if (!camera.start()) {
      cout << "Gagal menginisiasi kamera: " << strerror(errno) << "\n";;
      exit(0);
    }
  cv::Mat frameBayer(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
  cv::Mat frame(cv::Size(camFormat.width, camFormat.height), CV_8UC3);

  /* Pesan pembuka */
  cout << "\nPerangkat lunak lokalisasi berbasis kode QR\n"
          "Oleh Muhammad Shalahuddin Yahya Sunarko\n\n";

  /* Nilai variabel independen pada saat pengambilan data */
  cout << "a. 0 m/s\te. 2.0 m/s\n"
          "b. 0.5 m/s\tf. 2.5 m/s\n"
          "c. 1.0 m/s\tg. 0.25 m/s\n"
          "d. 1.5 m/s\th. 0.75 m/s\n"
          "Pilih kecepatan v: ";
  cin >> opsi_v;
  switch (opsi_v) {
    case 'a':
      v_diinginkan = 0;
      break;
    case 'b':
      v_diinginkan = 0.5;
      break;
    case 'c':
      v_diinginkan = 1.0;
      break;
    case 'd':
      v_diinginkan = 1.5;
      break;
    case 'e':
      v_diinginkan = 2.0;
      break;
    case 'f':
      v_diinginkan = 2.5;
      break;
    case 'g':
      v_diinginkan = 0.25;
      break;
    case 'h':
      v_diinginkan = 0.75;
      break;
    default:
      cout << "Error! Tidak terdapat dalam pilihan!\n";
      exit(0);
      break;
    }

  cout << "\nMasukkan iluminansi E_0 (lux): ";
  cin >> lux_diinginkan_buf[0];
  cout << "\nMasukkan iluminansi E_1 (lux): ";
  cin >> lux_diinginkan_buf[1];
  cout << "\nMasukkan iluminansi E_2 (lux): ";
  cin >> lux_diinginkan_buf[2];
  lux_diinginkan = (lux_diinginkan_buf[0] + lux_diinginkan_buf[1] + lux_diinginkan_buf[2]) / 3.0;
  cout << "\n"
          "a. 22.2 milisec\td. 2.7 milisec\n"
          "b. 10.5 milisec\te. 1.3 milisec\n"
          "c. 5.6 milisec\tf. 0.7 milisec\n"
          "Pilih waktu exposure t_exp: ";
  cin >> opsi_t;
  switch (opsi_t) {
    case 'a':
      t_exp_diinginkan = 22.2;
      break;
    case 'b':
      t_exp_diinginkan = 10.5;
      break;
    case 'c':
      t_exp_diinginkan = 5.6;
      break;
    case 'd':
      t_exp_diinginkan = 2.7;
      break;
    case 'e':
      t_exp_diinginkan = 1.3;
      break;
    case 'f':
      t_exp_diinginkan = 0.7;
      break;
    default:
      cout << "Error! Tidak terdapat dalam pilihan!\n";
      exit(0);
      break;
    }

  /* Atur waktu exposure kamera */
  t_exp = t_exp_diinginkan * 10.0;
  camera.set_control("Exposure (Absolute)", t_exp);

  /* Inisialisasi pemindai kode QR */
  zbar::ImageScanner pemindai;
  zbar::Image dipindai;
  zbar::Image::SymbolIterator symbol;
  pemindai.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0); // matikan seluruh pemindai
  pemindai.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1); // nyalakan hanya pemindai kode QR
  dipindai.set_size(lebar, tinggi);
  dipindai.set_format("GREY");

  /* Tanggal dan jam pengambilan data */
  time_t rawtime = time(0);   // get time now
  struct tm *now = localtime(&rawtime);
  ss << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' << now->tm_mday << '_' << now->tm_hour << '.' << now->tm_min << '.' << now->tm_sec;
  tanggal_jam = ss.str();
  cout << "\nTime: " << tanggal_jam;

  /* Inisiasi tampilan */
  ss << "_v" << v_diinginkan << "_E" << lux_diinginkan << "_t" << t_exp_diinginkan;
  namaFile = ss.str();
  cout << "\n\nBerkas log (format tab-separated value) akan disimpan dengan nama " << namaFile;

  cout << "\n\nProgram sukses terinisiasi!\nKamu siap? Tekan ENTER untuk memulai...\n\n"
       << "Frame | t_stamp | t_rekog | v | t_exp_aktl | Jml_QR_terdtksi | QR_dat_i | QR_pos_i | QR_rot_i\n";
  cin.ignore();
  cin.ignore();
  cout.flush();
  sleep(3);
  cv::namedWindow("QR Code Reader with oCam 1CGN-U | Press 'q' to quit...", CV_WINDOW_KEEPRATIO|CV_WINDOW_NORMAL);

  /* Inisiasi berkas untuk penyimpanan data */
  ofstream fileCatatan;
  fileCatatan.open((namaFile + ".txt").c_str(), ios::binary | ios::out | ios::trunc);
  struct stat st;
  if (stat(namaFile.c_str(), &st) == -1) mkdir(namaFile.c_str(), 0700);
  else exit(0);
  vector<int> save_params;
  save_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  save_params.push_back(80);
  //fps_kamera = camFormat.frame_rate;
  //cv::VideoWriter dataVideo;
  //dataVideo.open(namaFile + ".avi", CV_FOURCC('F', 'F', 'V', '1'), fps_kamera, cv::Size(lebar, tinggi));
  ss.clear();
  fileCatatan << "Waktu: \t" << tanggal_jam << "\tv [m/s]: " << v_diinginkan << "\tIlluminansi [lux]: \t" << lux_diinginkan << "\tWaktu exposure [milisec]: \t" << t_exp_diinginkan << "\n";
  fileCatatan << "Frame no.\tTime stamp\tt rekognisi (ms)\tv sensor (m/s)\tt exposure aktual (s)\tJumlah kode QR terdeteksi\tData kode QR ke-i\tLokasi kode QR ke-i\tRotasi kode QR ke-i\n";

  /* Kalang pengambilan data penelitian */
  while (flag_jalan) {
      /* Dapatkan citra (frame) dari kamera */
      t_exp = camera.get_control("Exposure (Absolute)") / 10000.0; // Dapatkan nilai waktu exposure kamera
      size = camera.get_frame(frameBayer.data, camFormat.image_size, 1); // Dapatkan 1 frame dari kamera (blocking function)
      t_frame = Clock::now(); // Dapatkan waktu pada saat pengambilan frame
      /* Jika terjadi galat, mulai ulang kamera. */
      if (size == -1) {
          cout << "Cannot get image from camera: " << strerror(errno) << "\n";
          camera.stop();
          camera.start();
          continue;
        }

      /* Dapatkan kecepatan sesaat pada saat pengambilan frame */
      /*do kecepatan.receive(data, bufsize);
      while(data[0] != 'V');
      for(i = 1; i <= 4; i++) data[i] -= '0';
      v_sens = data[1] * 1000 + data[2] * 100 + data[3] * 10 + data[4];
      std::fill(data, data + sizeof(data)/sizeof(data[0]), 0);*/

      /* Hitung waktu saat pengambilan frame */
      if (frame_ke == 0) t_ref = t_frame;
      t_stamp_lama = t_stamp;
      t_stamp = t_frame - t_ref;
      t_proses = (t_stamp - t_stamp_lama).count() * 1000;

      /* Pindai kode QR pada frame */
      cv::cvtColor(frameBayer, frame, cv::COLOR_BayerGB2BGR); // Konversi citra Bayer ke RGB
      cv::cvtColor(frameBayer, frame_olahan, cv::COLOR_BayerGB2GRAY); // Konversi citra RGB ke keabu-abuan
      cv::filter2D(frame_olahan, frame_olahan, -1, kernel);
      dipindai.set_data((uchar *)frame_olahan.data, lebar * tinggi); // Siapkan data frame yang akan dipindai
      n = pemindai.scan(dipindai); // Pindai frame untuk kode QR
      iQR = 0; // Reset penghitung kode QR

      /* Ekstraksi kode QR dari hasil pemindaian */
      for (symbol = dipindai.symbol_begin(); symbol != dipindai.symbol_end(); ++symbol) {
          dataQR = symbol->get_data();
          iQR++;
          n = symbol->get_location_size(); //menentukan jumlah titik sudut kode QR yang terdeteksi
          vp.clear();
          /* Dapatkan koordinat titik-titik sudut kode QR */
          for (i = 0; i < n; i++) {
              vp.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
            }
          /* Gambar persegi di tepi area kode QR */
          cv::RotatedRect r = minAreaRect(vp);
          for (i = 0; i < 4; i++) cv::line(frame, vp[i], vp[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
          /* Dapatkan rotasi kode QR terhadap sumbu x */
          beta = 90 - abs(r.angle);
          if (beta >= 45 && beta < 90) {
                  rotasi = ((beta - 45) + symbol->get_orientation() * 90) - 45;
          }
          else if (beta >= 0 && beta < 45) {
                  rotasi = ((beta + 45) + symbol->get_orientation() * 90) - 45;
          }
          if (rotasi < 0) {
                  rotasi = 360 + rotasi;
          }
          rotasi = roundf(rotasi * 10) / 10.0;
          /* Simpan data dan lokasi kode QR */
          matrixDataQR.push_back(dataQR);
          matrixLokasiQR.push_back(r.center);
          matrixRotasiQR.push_back(rotasi);
          putText(frame, to_string(iQR) + ": " + dataQR, vp[2], cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 0, 0), 1); // Beri tulisan isi data di dekat kode QR
        }

      /* Tampilkan dan simpan hasil */
      putText(frame, "Frame: " + to_string(frame_ke), cv::Point(5, tinggi - 35), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 0, 0), 1);
      putText(frame, "Time (s): " + to_string(t_stamp.count()), cv::Point(5, tinggi - 20), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 0, 0), 1);
      putText(frame, "Processing time (ms): " + to_string(t_proses), cv::Point(5, tinggi - 5), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 0, 0), 1);
      cout << frame_ke << "\t" << t_stamp.count() << "\t" << t_proses << "\t" << v_sens << "\t" << t_exp << "\t" <<  matrixDataQR.size() << "\t";
      fileCatatan << frame_ke << "\t" << t_stamp.count() << "\t" << t_proses << "\t" << v_sens << "\t" << t_exp << "\t" << matrixDataQR.size() << "\t";
      for (iQR = 1; iQR < matrixDataQR.size() + 1; iQR++) {
          cout << matrixDataQR.at(iQR - 1) << "\t";
          cout << matrixLokasiQR.at(iQR - 1) << "\t";
          cout << matrixRotasiQR.at(iQR - 1) << "\t";
          fileCatatan << matrixDataQR.at(iQR - 1) << "\t";
          fileCatatan << matrixLokasiQR.at(iQR - 1) << "\t";
          fileCatatan << matrixRotasiQR.at(iQR - 1) << "\t";
        }
      cout << "\n";
      fileCatatan << "\n";
      matrixDataQR.clear();
      matrixLokasiQR.clear();
      matrixRotasiQR.clear();
      dipindai.set_data(NULL, 0); // clean up
      namaFrame = namaFile + "/" + string(6 - to_string(frame_ke).length(), '0') + to_string(frame_ke) + ".jpg";
      cv::imwrite(namaFrame, frame, save_params);
      //dataVideo << frame;
      cv::imshow("QR Code Reader with oCam 1CGN-U | Press 'q' to quit...", frame);
      tombol = cv::waitKey(1);
      if (tombol == 'q' || tombol == 'Q') flag_jalan = 0;
      frame_ke++;
    }

  /* Program selesai atau ditutup */
  sleep(3);
  cv::destroyAllWindows();
  camera.stop();
  //dataVideo.release();
  fileCatatan << "END OF FILE\n";
  fileCatatan.close();
  kecepatan.close();
  flag_jalan = 0;
  cout << "\nProgram berhasil ditutup!\n";
  return 0;
}
