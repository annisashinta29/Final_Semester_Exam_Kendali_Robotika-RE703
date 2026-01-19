# UAS Delivery Robot FSM using ROS 2 & Gazebo

**Nama Kelompok :**
- Annisa Shinta Dewi (422220105)
- Muhammad Hasmar (4222201016)
-------------  
Project ini merupakan simulasi **robot delivery** dengan menggunakan model articubot berbasis **ROS 2 Humble** yang berjalan di **Gazebo**, menggunakan **Finite State Machine (FSM)** untuk mengatur perilaku robot dalam melakukan pengiriman barang.

Robot akan:
- Bergerak dari **titik A → B → C → kembali ke A**
- **Berhenti sementara** di setiap titik 
- **Menghindari obstacle** menggunakan sensor LiDAR

Model robot yang digunakan adalah **Articubot One**.

---

## Konsep Finite State Machine (FSM)

FSM digunakan untuk mengatur alur kerja robot delivery.

### State Diagram (Logika FSM)

| State | Deskripsi |
|------|----------|
| `GO_TO_A` | Robot menuju titik A |
| `STOP_AT_A` | Robot berhenti (ambil barang) |
| `GO_TO_B` | Robot menuju titik B |
| `STOP_AT_B` | Robot berhenti (taruh barang) |
| `GO_TO_C` | Robot menuju titik C |
| `STOP_AT_C` | Robot berhenti (taruh barang) |
| `BACK_TO_A` | Robot kembali ke titik A |
| `STOP_AT_A_FINAL` | Berhenti sebelum siklus baru |
| `AVOID_OBSTACLE` | Menghindari rintangan |

---

## Navigasi Robot

Robot bergerak menggunakan:
- **Linear velocity** untuk maju
- **Angular velocity** untuk mengoreksi arah menuju target

Robot akan:
- Berputar jika sudut error besar
- Maju lurus jika sudah menghadap target

---

## Obstacle Avoidance

Obstacle dideteksi menggunakan **sensor LiDAR (`/scan`)**.

Kriteria obstacle:
- Jarak < **0.8 meter**
- Di area depan robot (±20 indeks laser)

Saat obstacle terdeteksi:
- Robot masuk state `AVOID_OBSTACLE`
- Robot berhenti maju dan berputar
- Setelah obstacle hilang, robot kembali ke waypoint terdekat

---
