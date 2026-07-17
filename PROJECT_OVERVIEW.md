# Project Overview: ROS 2 Jazzy PX4 Global Waypoint Mission

## 1. Project ini sebenarnya apa?

Project ini adalah **sistem mission waypoint untuk multicopter PX4 yang
dijalankan dari companion computer menggunakan ROS 2 Jazzy dan mode Offboard**.

Operator menentukan sepuluh target dalam koordinat global LLA:

```text
Latitude, Longitude, Altitude
```

Companion computer kemudian:

1. membaca state estimasi dari PX4 melalui uXRCE-DDS;
2. memvalidasi waypoint, estimator, home, battery, dan batas mission;
3. mengubah waypoint LLA menjadi local NED terhadap referensi EKF PX4;
4. mengatur urutan takeoff, rotasi, translasi, dan hold;
5. menerbitkan setpoint posisi Offboard secara kontinu;
6. memonitor program mission dan kondisi vehicle;
7. meminta PX4 melakukan Auto Land ketika terjadi kegagalan yang dikonfigurasi.

Project ini memindahkan bagian **mission sequencing** dari Navigator mission
onboard PX4 ke program ROS 2, tetapi tetap memakai estimator dan seluruh
controller penerbangan PX4.

```text
ROS 2 bertanggung jawab:
mission definition, sequencing, validation, watchdog, target generation

PX4 bertanggung jawab:
EKF2, position control, attitude control, rate control, allocation, motor output
```

## 2. Masalah yang ingin diselesaikan

Project dibuat untuk kebutuhan berikut:

- merekam posisi drone sebagai waypoint tanpa mengetik latitude/longitude;
- menyimpan sepuluh waypoint global yang dapat dimodifikasi;
- menerbangkan multicopter melalui waypoint secara deterministik;
- melakukan takeoff vertikal tanpa command rotasi yaw;
- mengarahkan nose menuju jalur berikutnya ketika arah berubah;
- berhenti dan menahan posisi pada setiap waypoint;
- mempertahankan waypoint terakhir setelah mission selesai;
- melakukan Auto Land ketika mission node gagal, estimator invalid, komunikasi
  target berhenti, atau battery di bawah batas;
- menyediakan fondasi untuk dikembangkan menjadi sistem mission produksi.

Project ini tidak hanya mengirim satu koordinat tujuan. Ia merupakan state
machine mission dengan health gate dan supervisor terpisah.

## 3. Tujuan utama

### 3.1 Tujuan fungsional

1. Menyediakan tiga perintah launch yang jelas:
   - membaca/merekam posisi;
   - mengaktifkan dan mengawasi Offboard;
   - menjalankan mission sepuluh waypoint.
2. Memakai waypoint global LLA agar target tetap mempunyai arti geografis.
3. Memakai proyeksi global-to-local yang konsisten dengan PX4.
4. Menahan yaw awal selama takeoff vertikal.
5. Melakukan rotate-before-translate untuk perubahan arah.
6. Memastikan hold dimulai setelah posisi dan kecepatan memenuhi batas.
7. Memisahkan executor mission dan safety supervisor.
8. Menghasilkan Auto Land, bukan RTL, untuk kegagalan yang berada dalam scope
   supervisor.

### 3.2 Tujuan keselamatan

- jangan masuk Offboard tanpa local position dan battery status valid;
- jangan menjalankan mission dengan global reference/home yang invalid;
- jangan menerima waypoint dengan format atau range salah;
- jangan meneruskan mission ketika estimator masuk dead-reckoning atau accuracy
  melewati batas;
- jangan membiarkan hilangnya heartbeat mission menjadi command terakhir tanpa
  batas waktu;
- jangan menerbangkan lintasan yang melewati batas altitude, distance-from-home,
  atau panjang segmen yang dikonfigurasi;
- jangan menganggap build sukses sebagai bukti flight safety.

### 3.3 Tujuan engineering

- source utama menggunakan C++;
- utility/interface internal menggunakan `.h`;
- launch menggunakan ROS 2 XML;
- konfigurasi waypoint dan threshold berada di YAML;
- komponen mempunyai tanggung jawab yang terpisah;
- parameter keselamatan dapat diaudit;
- perilaku dapat diuji melalui SITL, HITL, dan fault injection.

## 4. Non-goals

Project ini saat ini **bukan**:

- clone penuh QGroundControl;
- pengganti PX4 EKF2 atau flight controller;
- MAVLink Mission Protocol uploader;
- mission yang disimpan ke dataman PX4;
- SLAM, obstacle avoidance, atau path planner berbasis occupancy map;
- terrain-following tervalidasi;
- custom geofence polygon planner;
- photogrammetry, mapping grid generator, atau camera trigger system;
- sistem yang menjamin zero error atau zero bug;
- sistem yang telah disertifikasi untuk operasi real-world.

Jika companion computer atau DDS mati, mission ROS tidak dapat melanjutkan
sequencing. Lapisan native Offboard-loss PX4 wajib dikonfigurasi sebagai backup.

## 5. Baseline teknologi

| Komponen | Baseline |
|---|---|
| OS middleware | ROS 2 Jazzy |
| Autopilot | PX4 1.16.1 |
| ROS interface | official `px4_msgs` branch `release/1.16`, commit `392e831` |
| Transport | PX4 uXRCE-DDS + Micro XRCE-DDS Agent |
| Vehicle utama | multicopter, termasuk Hexacopter-X |
| Control mode | PX4 Offboard position control |
| Mission coordinate | WGS-84 LLA + altitude frame |
| Controller coordinate | PX4 local NED |
| Implementasi | C++17 melalui `ament_cmake` |

## 6. Arsitektur sistem

```text
                 ┌─────────────────────────────┐
                 │ waypoints.yaml              │
                 │ 10 × LLA/frame/yaw/hold     │
                 └──────────────┬──────────────┘
                                │ ROS parameters
                                ▼
┌──────────────────┐    ┌──────────────────────────────┐
│ Position Recorder│    │ Waypoint Mission Node        │
│                  │    │                              │
│ PX4 global/local │    │ feasibility + health gate    │
│ → wp:=1..10      │    │ LLA → NED                    │
│ → YAML update    │    │ takeoff/align/navigate/hold  │
└──────────────────┘    └──────────────┬───────────────┘
                                       │ /mission/target_ned
                                       │ /mission/abort
                                       ▼
                         ┌──────────────────────────────┐
                         │ Offboard Supervisor          │
                         │                              │
                         │ pre-stream + mode request    │
                         │ heartbeat + battery watchdog │
                         │ single owner PX4 setpoint    │
                         │ failure → NAV_LAND           │
                         └──────────────┬───────────────┘
                                        │ /fmu/in/*
                                        ▼
                         ┌──────────────────────────────┐
                         │ PX4                          │
                         │ EKF2 → MC position control   │
                         │ attitude/rate → allocator    │
                         │ → enam motor Hexa-X          │
                         └──────────────────────────────┘
```

Desain menggunakan satu publisher PX4 trajectory: hanya supervisor yang
menerbitkan `TrajectorySetpoint`. Mission node mengirim target internal. Ini
mencegah dua node berebut setpoint PX4.

## 7. Tiga workflow utama

### 7.1 Membaca atau merekam posisi

Report-only:

```bash
ros2 launch px4_waypoint_mission 01_position.launch.xml
```

Rekam posisi aktual sebagai waypoint tertentu:

```bash
ros2 launch px4_waypoint_mission 01_position.launch.xml wp:=1
```

`wp:=1` dapat diganti sampai `wp:=10`. Recorder:

1. membaca `/fmu/out/vehicle_global_position` dan local position;
2. menunggu validity dan accuracy memenuhi batas;
3. menolak dead-reckoning;
4. mengumpulkan 20 sampel sehat;
5. menghitung rata-rata latitude/longitude;
6. menolak batch yang spread-nya terlalu besar;
7. hanya mengganti latitude/longitude waypoint terpilih;
8. mempertahankan altitude, frame, yaw policy, dan hold;
9. membuat `.bak` lalu mengganti file secara atomik.

### 7.2 Mengaktifkan supervisor Offboard

```bash
ros2 launch px4_waypoint_mission 02_offboard.launch.xml
```

Supervisor:

1. menunggu local position dan battery valid;
2. menangkap target safe-hold dari posisi dan yaw aktual;
3. melakukan pre-stream `OffboardControlMode` dan `TrajectorySetpoint`;
4. meminta mode Offboard;
5. opsional meminta arm jika `auto_arm=true`;
6. menjadi satu-satunya publisher trajectory ke PX4;
7. memonitor target mission, explicit abort, position, battery, dan status PX4;
8. mengirim `VEHICLE_CMD_NAV_LAND` ketika watchdog gagal.

`auto_arm` default `false` agar perpindahan mode tidak otomatis menghidupkan
motor tanpa keputusan operator.

### 7.3 Menjalankan mission

```bash
ros2 launch px4_waypoint_mission 03_mission.launch.xml
```

Mission node memuat sepuluh item, menunggu estimator/home valid, menjalankan
feasibility check, lalu mengaktifkan state machine penerbangan.

## 8. Kontrak waypoint

Format setiap waypoint:

```text
latitude_deg,longitude_deg,altitude_m,alt_frame,yaw_rad,hold_seconds
```

Contoh:

```yaml
- "47.3977420,8.5455940,5.0,REL_HOME,nan,5.0"
```

Maknanya:

- latitude `47.3977420°`;
- longitude `8.5455940°`;
- altitude 5 m di atas home;
- `yaw=nan`: heading otomatis mengikuti bearing lintasan;
- hold 5 detik setelah reach condition terpenuhi.

Altitude frame:

| Frame | Makna |
|---|---|
| `REL_HOME` | altitude di atas `HomePosition.alt` |
| `AMSL` | altitude absolut terhadap datum altitude estimator |

Terrain-relative belum didukung.

## 9. Transformasi LLA menjadi NED

Waypoint tetap disimpan sebagai koordinat global. Sebelum masuk position
controller, mission node memakai referensi EKF:

```text
VehicleLocalPosition.ref_lat
VehicleLocalPosition.ref_lon
VehicleLocalPosition.ref_alt
```

Latitude/longitude diproyeksikan menggunakan Azimuthal Equidistant dengan radius
bumi yang sama dengan `PX4::MapProjection`:

```text
LLA waypoint + global reference EKF
              ↓
North, East

REL_HOME:
altitude_amsl = home_alt + altitude_relative

Down = -(altitude_amsl - ref_alt)
```

Hasil akhirnya adalah `LocalWaypoint [North, East, Down]`. PX4 position
controller bekerja terhadap local state EKF, bukan langsung terhadap angka LLA.

## 10. State machine mission

```text
WAIT_FOR_OFFBOARD
       │
       ▼
TAKEOFF_VERTICAL
  XY = posisi awal
  yaw = heading awal
  Z → altitude WP1
       │ altitude reached + stopped
       ▼
ALIGN_HEADING
  XYZ ditahan
  yaw → bearing segment
       │ yaw aligned
       ▼
NAVIGATE
  XYZ → waypoint
  yaw = bearing segment
       │ position reached + stopped
       ▼
HOLD
  target posisi/yaw dipertahankan
       │ hold time selesai
       ├────────────→ waypoint berikutnya / ALIGN_HEADING
       ▼
FINISHED
  WP10 terus diterbitkan
```

### 10.1 Takeoff tanpa command yaw rotation

Ketika Offboard mulai, program menangkap heading aktual. Selama takeoff:

- North/East tetap posisi awal;
- yaw tetap heading awal;
- hanya Down target yang berubah menuju altitude WP1.

Jadi program tidak meminta nose menuju utara atau menuju WP berikutnya selama
vertical takeoff.

### 10.2 Rotasi antar-waypoint

Untuk yaw `nan`, bearing dihitung:

```text
yaw_target = atan2(delta_east, delta_north)
```

Sudut dibungkus ke `[-π, π]` agar mengambil arah pendek. Program menahan XYZ,
meramp yaw setpoint sesuai `max_yaw_rate_rad_s`, lalu baru bertranslasi ketika
heading error masuk `yaw_acceptance_rad`.

Perilaku saat ini adalah:

```text
berhenti → berputar → bergerak → berhenti
```

Bukan rounded-corner atau fly-through trajectory.

### 10.3 Reach dan hold

Waypoint dianggap siap memasuki hold jika:

```text
horizontal error <= acceptance_xy_m
vertical error   <= acceptance_z_m
3D speed         <= stopped_speed_mps
```

Timer hold berjalan selama kondisi tetap terpenuhi. Jika vehicle keluar dari
batas atau bergerak lagi, awal timer diperbarui. Setelah WP10, node tidak keluar;
ia terus mengirim WP10 agar hold dan heartbeat supervisor tetap aktif.

## 11. Health dan feasibility gate

Mission tidak langsung menerbangkan seluruh input YAML. Sebelum takeoff diperiksa:

- jumlah item tepat sepuluh;
- range latitude/longitude;
- altitude frame dikenali;
- altitude dan hold finite;
- home horizontal/altitude valid;
- local/global reference valid;
- global position fresh;
- estimator tidak dead-reckoning;
- `eph/epv` di bawah batas;
- altitude di atas home berada dalam range;
- waypoint tidak terlalu jauh dari home;
- panjang segment tidak melebihi batas.

Selama mission dipantau:

- local/global validity;
- failsafe flags;
- geofence breach aktual;
- estimator accuracy;
- reset counter XY/Z/heading/global;
- perubahan reference timestamp;
- perubahan home;
- heartbeat mission.

Estimator reset menyebabkan target LLA diproyeksikan ulang dan alignment diulang.
Home berubah ketika mission aktif menghasilkan abort karena dapat mengubah makna
altitude relative.

## 12. Failsafe dan Auto Land

Supervisor meminta Auto Land ketika:

- mission node menerbitkan `/mission/abort`;
- heartbeat/target mission timeout;
- mission tidak mulai dalam startup timeout;
- target mengandung NaN/Inf;
- local position invalid atau timeout;
- battery ≤15% selama jumlah sampel debounce;
- battery status invalid/timeout saat armed atau Offboard.

Action yang dikirim:

```text
VEHICLE_CMD_NAV_LAND
```

Supervisor tidak mengirim RTL. Command Land diulang secara periodik. Namun bila
supervisor sendiri mati, hanya native PX4 Offboard-loss yang masih bekerja.
Karena itu `COM_OF_LOSS_T` dan `COM_OBL_RC_ACT` wajib diatur dan diuji agar aksi
native juga Land.

Battery `remaining` merupakan estimasi PX4 0–1. Kalibrasi power module, kapasitas,
cell count, dan parameter battery menentukan apakah angka 15% dapat dipercaya.

## 13. Struktur workspace

```text
ros2_px4_waypoint_ws/
├── README.md
├── PROJECT_OVERVIEW.md
├── GAP_IMPLEMENTASI_DAN_ROADMAP.md
├── src/
│   ├── px4_msgs/
│   └── px4_waypoint_mission/
│       ├── config/
│       │   ├── offboard.yaml
│       │   └── waypoints.yaml
│       ├── include/px4_waypoint_mission/
│       │   ├── waypoint.h
│       │   ├── waypoint_mission.h
│       │   └── offboard_supervisor.h
│       ├── launch/
│       │   ├── 01_position.launch.xml
│       │   ├── 02_offboard.launch.xml
│       │   └── 03_mission.launch.xml
│       └── src/
│           ├── core/
│           │   ├── waypoint_mission.cpp
│           │   └── offboard_supervisor.cpp
│           └── node/
│               ├── position_reporter_node.cpp
│               ├── waypoint_mission_node.cpp
│               └── offboard_supervisor_node.cpp
```

### Tanggung jawab file

| File | Tanggung jawab |
|---|---|
| `waypoint.h` | data global/local waypoint, parsing, validasi, proyeksi |
| `waypoint_mission.cpp` | feasibility, health, state machine dan progress |
| `offboard_supervisor.cpp` | ownership setpoint PX4 dan safety watchdog |
| `position_reporter_node.cpp` | report dan perekaman waypoint |
| `waypoints.yaml` | target dan threshold mission |
| `offboard.yaml` | timing, battery, arming, dan supervisor policy |

## 14. Topic ROS/PX4 utama

### Input dari PX4

```text
/fmu/out/vehicle_local_position
/fmu/out/vehicle_global_position
/fmu/out/home_position
/fmu/out/vehicle_status
/fmu/out/vehicle_command_ack
/fmu/out/failsafe_flags
/fmu/out/battery_status
/fmu/out/vehicle_land_detected
/fmu/out/vehicle_angular_velocity      # butuh dds_topics.yaml + firmware rebuild, lihat GAP §4.1
/fmu/out/control_allocator_status      # butuh dds_topics.yaml + firmware rebuild, lihat GAP §4.4
```

### Internal project

```text
/mission/target_ned
/mission/target_feedforward   # TwistStamped: yaw-rate selalu; velocity NED hanya saat fly-through
/mission/current_waypoint
/mission/abort
```

### Output menuju PX4

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

QoS PX4 disesuaikan dengan pola best-effort/volatile pada uXRCE-DDS, sedangkan
target dan abort internal memakai reliable; abort dibuat transient-local agar
supervisor yang baru aktif dapat menerima status abort yang masih berlaku.

## 15. Cara menjalankan

Build:

```bash
cd /home/udinkicau/masterpiece_ws/ros2_px4_waypoint_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Urutan operasi:

```bash
# Terminal 1: lihat/rekam waypoint
ros2 launch px4_waypoint_mission 01_position.launch.xml

# Terminal 2: supervisor dan Offboard
ros2 launch px4_waypoint_mission 02_offboard.launch.xml

# Terminal 3: executor 10 waypoint
ros2 launch px4_waypoint_mission 03_mission.launch.xml
```

Micro XRCE-DDS Agent dan PX4 client harus sudah terhubung sebelum topic `/fmu`
tersedia.

## 16. Penggunaan real-world yang benar

Urutan pengembangan yang diwajibkan:

1. unit test fungsi parsing, projection, angle wrapping, dan state transition;
2. PX4 SITL nominal;
3. SITL fault injection;
4. HITL dengan flight controller dan link aktual;
5. bench test tanpa propeller;
6. tether/controlled hover jika prosedur mengizinkan;
7. penerbangan waypoint pendek di area steril;
8. perluasan envelope bertahap.

Sebelum flight simpan:

- firmware version/hash;
- `px4_msgs` hash;
- parameter dump PX4;
- airframe dan actuator geometry;
- calibration sensor/power module;
- waypoint file dan hash;
- threshold supervisor/mission;
- hasil test sebelumnya.

## 17. Kriteria keberhasilan

Project dianggap berhasil secara fungsional jika:

1. sepuluh waypoint direkam dan dibaca dengan benar;
2. LLA-to-NED sesuai test vector PX4;
3. takeoff tidak mengubah yaw di luar tolerance;
4. rotasi selesai sebelum translasi;
5. setiap waypoint mencapai error dan hold SLA;
6. WP10 tetap ditahan;
7. seluruh fault yang ditentukan menghasilkan Auto Land;
8. tidak ada fault test yang menghasilkan RTL;
9. reset estimator tidak menghasilkan translasi liar;
10. log menunjukkan tidak ada saturation yang melampaui batas operasi.

Metric yang harus ditentukan dan diukur:

```text
horizontal/vertical waypoint error P95/P99
yaw error dan yaw-rate P95/P99
takeoff XY drift dan yaw drift
settling time dan hold drift
watchdog-to-Land latency
battery-trigger-to-Land latency
DDS packet age/dropout
control allocation saturation duration
```

## 18. Goal akhir project

Goal akhirnya adalah menghasilkan **mission waypoint multicopter berbasis LLA
yang dapat diprogram, diawasi, diuji, dan dihentikan secara aman dari ROS 2**,
dengan PX4 tetap menjadi sumber estimasi dan flight-control authority.

Versi produksi yang diinginkan harus:

- deterministik;
- fail-safe terhadap matinya mission process dan link Offboard;
- mempunyai batas error yang terukur;
- mempunyai konfigurasi yang dapat diaudit;
- menghasilkan log yang cukup untuk membuktikan perilaku;
- bekerja konsisten pada SITL, HITL, dan vehicle target;
- tidak bergantung pada klaim “presisi” tanpa measurement.

Daftar gap menuju goal tersebut terdapat pada
[`GAP_IMPLEMENTASI_DAN_ROADMAP.md`](GAP_IMPLEMENTASI_DAN_ROADMAP.md).
