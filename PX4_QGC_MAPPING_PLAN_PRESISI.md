# Cara Kerja Mapping Plan QGroundControl 5.0.8 dan PX4 1.16.1

> **Baseline yang diaudit:** source lokal `qgroundcontrol-5.0.8` dan
> `PX4-Autopilot-1.16.1` di workspace ini. Dokumen ini membahas **Survey/Mapping
> Mission**: perencanaan lintasan, upload mission, eksekusi waypoint, estimasi
> posisi, dan pemicu kamera. Ini bukan algoritma SLAM atau fotogrametri pembuat
> orthomosaic.

Kamera **tidak wajib**. Seluruh pipeline waypoint, estimator, Navigator,
controller, dan failsafe tetap bekerja tanpa kamera. Bagian kamera hanya berlaku
jika mission dipakai untuk akuisisi citra; untuk coverage flight tanpa payload,
abaikan seluruh camera command dan tentukan spacing berdasarkan kebutuhan jalur.

## 1. Batas jaminan dan definisi presisi

Tidak ada sistem penerbangan real-world yang dapat dijamin “tidak pernah miss,
tidak ada error, dan tidak ada bug”. QGC dan PX4 juga tidak mengoreksi hasil
orthomosaic. Sistem yang benar harus menetapkan batas error, mendeteksi ketika
batas itu dilanggar, lalu membatalkan atau mengulang akuisisi.

Pisahkan empat jenis ketelitian berikut:

1. **Ketelitian rencana:** seberapa tepat grid matematis menutup polygon.
2. **Ketelitian navigasi:** selisih posisi kendaraan sebenarnya terhadap jalur.
3. **Ketelitian waktu/posisi exposure:** selisih titik foto sebenarnya terhadap
   titik foto ideal.
4. **Ketelitian produk peta:** error orthomosaic/DSM terhadap datum atau GCP.

QGC terutama menangani (1), PX4 menangani (2) dan menjalankan trigger untuk
(3), sedangkan aplikasi fotogrametri, RTK/PPK, GCP, dan kalibrasi kamera
menentukan (4). Angka latitude dengan resolusi protokol sekitar 1 cm tidak
berarti posisi pesawat akurat 1 cm.

## 2. Arsitektur end-to-end

```text
Operator
  │ polygon WGS-84, kamera, GSD/tinggi, overlap, angle
  ▼
QGC SurveyComplexItem
  │ proyeksi geodetik → bidang lokal; potong grid; urutkan transect
  ▼
QGC TransectStyleComplexItem
  │ expand ComplexItem → NAV_WAYPOINT + camera commands
  ▼
QGC PlanManager ───── MAVLink 2 Mission Protocol ───── PX4 mavlink_mission
  │   MISSION_COUNT / REQUEST_INT / ITEM_INT / ACK         │
  │                                                        ▼
  │                                                  dataman storage
  │                                                        │
  │                                                        ▼
  │                                                  Navigator/Mission
  │                                                        │ global setpoint
  │                                                        ▼
  │                              EKF2 → local/global position → controller
  │                                                        │
  │                                                        ▼
  └──── telemetry, mission progress, CAMERA_TRIGGER ◀ vehicle + camera
```

Hal penting: **QGC menghasilkan semua waypoint survey sebelum upload**. PX4
tidak menerima polygon, GSD, atau persentase overlap dan tidak membuat grid.
PX4 menerima daftar mission item yang sudah “diratakan” menjadi perintah MAVLink.

## 3. Kontrak koordinat

### 3.1 Datum dan frame

| Lapisan | Representasi | Satuan/arah | Fungsi |
|---|---|---|---|
| Polygon dan waypoint QGC | geodetik WGS-84 | latitude/longitude derajat | penyimpanan dan tampilan global |
| Kalkulasi grid QGC | bidang lokal bertangen | meter | intersection dan spacing stabil |
| MAVLink mission global | `MISSION_ITEM_INT.x/y` | derajat × `10^7` | transport lat/lon presisi integer |
| PX4 global position | WGS-84 lat/lon + altitude | derajat, meter | referensi mission/global telemetry |
| PX4 local navigation | NED | x North, y East, z Down; meter | EKF dan position control |
| Body FRD | x Forward, y Right, z Down | meter/radian | sensor dan dinamika kendaraan |

Pada QGC, `QGeoCoordinate` menampung `(latitude, longitude, altitude)`. Saat
geometri survey dibuat, titik polygon diproyeksikan ke bidang lokal dengan
origin/tangent point dekat polygon. Di source QGC variabel planar diperlakukan
seperti N/E dalam meter, kemudian hasil intersection dikonversi kembali ke
geodetik. Ini menghindari kesalahan fatal berupa memperlakukan satu derajat
longitude sebagai jarak yang sama di semua latitude.

Untuk implementasi kompatibel, gunakan proyeksi lokal yang sama untuk area
kecil. Untuk wilayah besar, dekat kutub, atau melintasi antimeridian, gunakan
GeographicLib/ENU yang terdefinisi dan pecah misi; jangan memakai rumus
`111111 m/degree` secara universal.

### 3.2 Latitude/longitude pada MAVLink

QGC mengirim mission melalui `MISSION_ITEM_INT`. Untuk item posisi global:

```text
x = round(latitude_deg  × 10^7)   // int32
y = round(longitude_deg × 10^7)   // int32
z = altitude_m                   // float
```

Resolusi kuantisasi latitude adalah kira-kira 1.11 cm. Resolusi longitude
adalah `1.11 cm × cos(latitude)`. Ini hanya resolusi encoding, bukan akurasi
GNSS. Pada source QGC 5.0.8, `PlanManager` mengalikan `param5/param6` dengan
`1e7` saat mengemas `MISSION_ITEM_INT`, dan membaginya dengan `1e7` saat
membaca kembali.

Catatan interoperabilitas: source QGC ini menyimpan frame internal seperti
`MAV_FRAME_GLOBAL_RELATIVE_ALT`, lalu tetap meng-encode x/y × `1e7`. Spesifikasi
MAVLink modern menganjurkan varian frame berakhiran `_INT` ketika dipakai dengan
`MISSION_ITEM_INT`. Reimplementasi baru sebaiknya mengikuti spesifikasi `_INT`
dan memiliki compatibility test terhadap PX4 1.16.1, bukan menyalin asumsi
secara buta.

### 3.3 Arti altitude

Altitude adalah sumber kesalahan paling sering. Bedakan:

| Frame | Makna `z` |
|---|---|
| `MAV_FRAME_GLOBAL(_INT)` | altitude absolut AMSL yang dipakai stack |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT(_INT)` | meter di atas home altitude |
| `MAV_FRAME_GLOBAL_TERRAIN_ALT(_INT)` | meter di atas terrain |

“AMSL” tidak otomatis identik dengan tinggi ellipsoid GNSS. GNSS dapat memberi
ellipsoid height dan geoid separation; terrain service mempunyai datum dan
resolusi sendiri; barometer bersifat relatif serta dapat drift. Sistem clone
wajib menuliskan datum untuk setiap field dan melakukan transformasi geoid
secara eksplisit.

QGC terrain following yang direncanakan bukan terrain avoidance real-time.
QGC mengambil sampel elevasi terrain, lalu membentuk waypoint altitude agar
jarak ke permukaan mendekati target dengan batas tolerance/climb/descent.
DEM yang tua, kasar, atau tidak memuat kabel/pohon/bangunan tetap dapat
menghasilkan lintasan berbahaya.

## 4. Cara QGC membentuk survey grid

### 4.1 Dari kamera ke footprint dan GSD

Untuk orientasi landscape, source `CameraCalc` menggunakan:

```text
GSD_cm_per_px = H_m × sensor_width_mm × 100
                --------------------------------
                image_width_px × focal_length_mm

H_m = image_width_px × GSD_cm_per_px × focal_length_mm
      -------------------------------------------------
      sensor_width_mm × 100

footprint_side_m    = image_width_px  × GSD_cm_per_px / 100
footprint_frontal_m = image_height_px × GSD_cm_per_px / 100

line_spacing_m = footprint_side_m × (1 - side_overlap_pct/100)
trigger_dist_m = footprint_frontal_m × (1 - front_overlap_pct/100)
```

Untuk portrait, width dan height pada dua footprint ditukar. Model ini adalah
model pinhole ideal dan permukaan planar tegak lurus optical axis. Distorsi
lensa, rolling shutter, kemiringan kamera, relief terrain, dan blur tidak
dikoreksi oleh rumus tersebut.

Contoh: footprint frontal 40 m dan front overlap 75% memberi trigger distance
10 m. Jika cross-track error 1 m atau AGL berubah, overlap nyata tidak lagi
persis 75%.

### 4.2 Pembentukan garis

Urutan algoritma `SurveyComplexItem` adalah:

1. Validasi polygon minimal dan pilih tangent origin dekat polygon.
2. Konversi seluruh vertex WGS-84 ke koordinat planar lokal dalam meter.
3. Ambil `gridSpacing = AdjustedFootprintSide`.
4. Normalisasi angle grid. Jika opsi refly aktif, pass kedua memakai angle + 90°.
5. Bentuk sekumpulan garis paralel yang cukup panjang untuk melampaui bounding
   rectangle polygon.
6. Potong setiap garis terhadap polygon. Untuk polygon cekung, satu garis dapat
   menghasilkan beberapa segmen.
7. Jika split-concave aktif, pecah/kelompokkan bagian polygon agar koneksi antar
   transect tidak memotong area secara keliru.
8. Tambahkan turnaround di luar batas pada kedua ujung sesuai konfigurasi.
9. Urutkan boustrophedon: arah segmen berselang-seling agar jalur membentuk
   lawnmower dan jarak transit minimum.
10. Balik urutan/arah sesuai entry point (top-left, top-right, bottom-left,
    bottom-right). Fixed-wing dapat memakai alternate transects agar belokan
    tidak terlalu tajam.
11. Konversi titik planar kembali ke `QGeoCoordinate`.
12. Jika terrain follow aktif, query elevasi dan densifikasi/ubah altitude.
13. Expand menjadi simple mission items.

Secara geometri, intersection sebuah garis grid dengan setiap edge polygon
dicari. Intersection diurutkan sepanjang garis dan dipasangkan menjadi segmen
inside-polygon. Implementasi clone harus mempunyai aturan eksplisit untuk
vertex tepat pada garis, edge collinear, duplicate intersection, polygon
self-intersecting, hole, dan floating-point epsilon.

### 4.3 Perintah mission hasil ekspansi

Hasil akhir umumnya terdiri dari:

- `MAV_CMD_NAV_WAYPOINT` untuk ujung transect/turnaround dan titik terrain.
- `MAV_CMD_DO_SET_CAM_TRIGG_DIST` untuk capture berbasis jarak.
- `MAV_CMD_IMAGE_START_CAPTURE` untuk mode hover-and-capture/interval tertentu.
- perintah stop capture ketika keluar dari bagian yang harus difoto.
- opsional speed dan gimbal commands dari section terkait.

`DO_SET_CAM_TRIGG_DIST.param1` adalah jarak trigger dalam meter; nilai nol
mematikan triggering. QGC dapat mengaktifkan trigger hanya di bagian survey
atau juga pada turnaround. Pada hover-and-capture, kendaraan berhenti di titik
capture; jumlah mission item dan durasi naik drastis.

Trigger “setiap N meter” menggunakan jarak navigasi kendaraan, bukan feedback
bahwa file foto benar-benar tersimpan. Untuk integritas dataset, cocokkan setiap
command/event trigger dengan acknowledgement kamera atau file kamera.

## 5. Upload mission QGC ↔ PX4

### 5.1 State machine upload

```text
QGC                                      PX4
 |--- MISSION_COUNT(count,type) --------->|
 |<-- MISSION_REQUEST_INT(seq=0,type) ----|
 |--- MISSION_ITEM_INT(seq=0,...) -------->|
 |<-- MISSION_REQUEST_INT(seq=1,type) ----|
 |                 ...                    |
 |--- MISSION_ITEM_INT(seq=N-1,...) ------>|
 |<-- MISSION_ACK(ACCEPTED,type) ----------|
```

PX4 adalah pihak yang meminta setiap sequence. Permintaan ulang sequence yang
sama harus idempotent: QGC mengirim item yang sama lagi. QGC memiliki timeout,
retry counter, expected-ACK state, pengecekan `mission_type`, range sequence,
dan hanya menganggap transaksi sukses setelah `MISSION_ACK: ACCEPTED` serta
semua item sudah diminta.

Download adalah kebalikannya:

```text
QGC -- MISSION_REQUEST_LIST --> PX4
QGC <-- MISSION_COUNT --------- PX4
QGC -- MISSION_REQUEST_INT(i) -> PX4
QGC <-- MISSION_ITEM_INT(i) --- PX4
QGC -- MISSION_ACK ------------> PX4
```

Link USB, serial telemetry, Wi-Fi, atau UDP hanya carrier. MAVLink menyediakan
sequence number frame dan checksum untuk mendeteksi korupsi paket; mission
microservice menangani loss/retry. MAVLink signing memberi autentikasi, bukan
enkripsi. Sistem clone harus menangani duplicate, out-of-order, stale packet,
reconnect, target system/component salah, dan mission type yang berbeda.

### 5.2 Verifikasi yang wajib

ACK sukses hanya berarti mission diterima, bukan membuktikan isi identik secara
semantik. Setelah upload:

1. Download mission kembali.
2. Canonicalize setiap item.
3. Bandingkan `seq`, `command`, `frame`, params, lat/lon integer, altitude,
   `autocontinue`, dan `mission_type`.
4. Izinkan hanya normalisasi yang terdokumentasi, misalnya representasi frame
   INT/non-INT yang ekuivalen.
5. Hash representasi canonical dan simpan bersama versi firmware/QGC.

Jangan membandingkan JSON ComplexItem dengan mission PX4 secara langsung;
ComplexItem hanya hidup di file `.plan` QGC, sedangkan vehicle menyimpan daftar
SimpleItem hasil ekspansinya.

### 5.3 Call flow upload yang benar-benar dijalankan source QGC 5.0.8

Urutan objek dan fungsi saat operator menekan **Upload** adalah:

```text
MissionController::sendToVehicle()
  → MissionController::sendItemsToVehicle()
    → MissionController::_convertToMissionItems()
      → setiap VisualMissionItem::appendMissionItems()
        ├─ SimpleMissionItem: salin satu MissionItem
        ├─ TransectStyleComplexItem: bangun banyak waypoint/command
        └─ MissionSettingsItem: dapat menambah end-of-mission action
    → Vehicle::missionManager()->writeMissionItems()
      → PlanManager::_writeMissionItemsWorker()
        → PlanManager::_writeMissionCount()
```

`_convertToMissionItems()` mengiterasi seluruh `visualItems`, bukan mengirim
objek UI atau JSON `.plan`. Untuk survey yang belum mempunyai cached loaded
items, `TransectStyleComplexItem::appendMissionItems()` memanggil
`_buildAndAppendMissionItems()`. Helper `_appendWaypoint()` menghasilkan item
berikut secara konkret:

```text
command      = MAV_CMD_NAV_WAYPOINT
frame        = frame hasil pilihan altitude mode
param1       = holdTime
param2       = 0          // PX4 memakai acceptance default
param3       = 0          // pass-through
param4       = NaN        // yaw tidak ditentukan QGC
param5/6/7   = latitude/longitude/altitude
autocontinue = true
```

Jadi waypoint survey standar QGC memang tidak meminta yaw tertentu dan tidak
meminta hold kecuali builder memberi `holdTime`. Hover-and-capture, condition
gate, serta distance-trigger menambah item tersendiri; mereka bukan field
tersembunyi dari waypoint.

Ada perlakuan sequence yang mudah terlewat ketika membuat clone. Bila firmware
plugin menyatakan home tidak perlu dikirim, `PlanManager::writeMissionItems()`:

1. membuang visual mission item pertama yang mewakili home;
2. mengurangi sequence semua item tersisa sebanyak satu;
3. mengurangi target `MAV_CMD_DO_JUMP.param1` sebanyak satu;
4. menandai item pertama hasil akhir sebagai `current`.

Karena itu sequence pada UI/JSON tidak selalu identik dengan sequence yang
masuk PX4. QGC kemudian membuat `_itemIndicesToWrite`, mengirim
`MISSION_COUNT`, dan menunggu `MISSION_REQUEST_INT`. Untuk setiap request,
`PlanManager::_handleMissionRequest()` memeriksa `mission_type`, expected ACK
state, dan batas sequence. Request duplikat dilayani lagi secara idempotent.

Packing aktual `MISSION_ITEM_INT` dilakukan di fungsi itu:

```text
target_system/component = vehicle / MAV_COMP_ID_AUTOPILOT1
seq, frame, command      = MissionItem
current                  = (requested sequence == 0)
autocontinue             = MissionItem.autoContinue
param1..4                = nilai float item
x/y                      = param5/6 × 1e7, kecuali MAV_FRAME_MISSION
z                        = param7
mission_type             = jenis PlanManager
```

Sesudah setiap kirim, QGC kembali memasang ACK timeout. Transaksi belum sukses
hanya karena seluruh paket pernah dikirim; `_handleMissionAck()` masih
memvalidasi `mission_type`, hasil ACK, dan state transaksi sebelum daftar write
dipindah menjadi daftar mission aktif pada sisi QGC.

## 6. Eksekusi di PX4

### 6.1 Dari MAVLink ke Navigator

`mavlink_mission` memvalidasi message, command/frame/sequence dan menyimpan
mission ke `dataman`. Metadata mission menunjuk bank penyimpanan aktif sehingga
mission yang belum lengkap tidak boleh menggantikan mission valid secara
setengah jadi. Navigator membaca `mission_item_s`, menjalankan item non-posisi,
dan menerjemahkan item posisi menjadi `position_setpoint_triplet`:

```text
previous setpoint → current setpoint → next setpoint
```

Triplet memungkinkan controller membentuk jalur dan transisi lebih halus.
Navigator menentukan kapan waypoint dianggap reached berdasarkan tipe vehicle,
acceptance radius, altitude acceptance, loiter, yaw, dan state item. Karena itu
kendaraan tidak harus melewati tepat satu titik matematis agar mission maju.

Untuk mapping, acceptance radius tidak boleh disamakan dengan cross-track
accuracy. Radius terlalu besar dapat memotong sudut; terlalu kecil dapat membuat
fixed-wing terus mengorbit atau multicopter lambat. Turnaround distance harus
memperhitungkan kecepatan, angin, bank angle maksimum, dan response controller.

### 6.2 Trigger kamera

Navigator meneruskan camera command menjadi vehicle command. Driver
`camera_trigger` dapat memicu interface PWM, GPIO, Seagull, atau MAVLink sesuai
konfigurasi. `CAMERA_TRIGGER`/`CAMERA_IMAGE_CAPTURED` dapat membawa waktu dan
posisi event, tetapi event trigger bukan jaminan sensor membuka shutter pada
waktu persis sama.

Error posisi exposure secara kasar:

```text
along_track_error ≈ ground_speed × total_trigger_latency
```

Pada 10 m/s dan latency+jitter 80 ms, error sepanjang jalur sekitar 0.8 m.
Gunakan hardware trigger, timestamp clock yang tersinkronisasi, feedback hotshoe
bila tersedia, dan ukur latency distribution—jangan hanya memakai nilai rata-rata.

### 6.3 State machine penerimaan aktual di PX4 1.16.1

`MavlinkMissionManager::handle_mission_count()` hanya memulai upload ketika
state `IDLE`, target system/component cocok, dan tidak ada transfer pada instance
MAVLink lain. Ia menolak count di atas `current_max_item_count()` dengan
`MAV_MISSION_NO_SPACE`. Count nol bukan no-op: PX4 mengosongkan mission dan
tetap menukar ID bank dataman agar Navigator mengetahui bahwa mission berubah.

Untuk mission non-kosong, PX4 memilih **bank yang sedang tidak aktif**:

```text
aktif OFFBOARD_0 → upload ditulis ke OFFBOARD_1
aktif OFFBOARD_1 → upload ditulis ke OFFBOARD_0
```

Lalu state menjadi `GETLIST`, `_transfer_seq=0`, count dan partner sysid/compid
dikunci, dan PX4 mengirim request sequence nol. Setiap `MISSION_ITEM_INT` masuk
melewati `handle_mission_item()` dan `parse_mavlink_mission_item()` untuk:

- memeriksa partner, state, sequence, mission type, frame, dan command;
- mengubah field MAVLink menjadi `mission_item_s`;
- mengonversi latitude/longitude integer kembali menjadi derajat;
- memetakan param command-specific seperti hold, acceptance, yaw, radius;
- mencatat marker land/land-start dan menghitung CRC mission;
- menulis item ke bank dataman transfer, bukan bank mission aktif.

PX4 baru memanggil `update_active_mission()` setelah sequence terakhir berhasil
ditulis. Pergantian metadata berisi dataman ID, count, current sequence, dan
CRC. Dengan demikian upload terputus tidak meninggalkan separuh mission baru
sebagai mission aktif. ACK `MAV_MISSION_ACCEPTED` dikirim setelah commit ini;
error parsing/storage menghasilkan ACK error dan bank lama tetap menjadi acuan.

Ini adalah sifat transaksi yang perlu direplikasi: **stage → validate each item
→ compute CRC → atomic metadata switch**, bukan menimpa array mission aktif
item demi item.

### 6.4 Call flow eksekusi setiap waypoint di PX4

Saat Commander memilih `NAVIGATION_STATE_AUTO_MISSION`, Navigator mengaktifkan
objek `Mission`. Alur pentingnya adalah:

```text
Mission::on_activation()
  → MissionBase::update_mission()
  → feasibility/validity checks
  → loadCurrentMissionItem()
  → Mission::setActiveMissionItems()
    → getNextPositionItems()
    → handleTakeoff()/handleLanding()
    → mission_item_to_position_setpoint(current)
    → mission_item_to_position_setpoint(next), bila diizinkan
    → publish position_setpoint_triplet

MissionBase::on_active(), berulang
  → update vehicle/global/geofence/dataman state
  → is_mission_item_reached_or_completed()
  → jalankan command atau tunggu position/yaw/time_inside
  → set_mission_item_reached()
  → jika autocontinue: advance_mission() → set_mission_items()
```

`getNextPositionItems()` melewati item non-posisi untuk mencari target geometris
berikutnya. Command seperti camera trigger, speed, gimbal, ROI, dan sebagian DO
command diproses sebagai action mission; command tersebut tidak otomatis menjadi
current position target. Gate mempunyai cabang khusus: posisi sesudah gate dapat
dipakai sebagai current target dan posisi berikutnya sebagai next target.

Untuk current item berposisi, `mission_item_to_position_setpoint()` mengisi
antara lain `valid`, `type`, `lat`, `lon`, altitude absolut, yaw, acceptance
radius, cruising speed, loiter radius/direction, dan weather-vane permission.
Takeoff dan landing dapat menyisipkan internal work item, sehingga satu sequence
MAVLink tidak selalu sama dengan satu fase trajectory controller.

Pada setiap cycle aktif, completion bukan satu boolean jarak sederhana.
`MissionBlock::is_mission_item_reached_or_completed()` bercabang menurut command
dan vehicle type: horizontal distance, vertical distance, passed-waypoint test,
loiter geometry, landing/takeoff state, yaw reach/timeout, `time_inside`, atau
completion command non-posisi. Baru sesudah itu `autocontinue` mengizinkan
`advance_mission()`.

### 6.5 Transformasi triplet sampai menjadi motor output

`FlightTaskAuto::_evaluateTriplets()` menerima `position_setpoint_triplet`.
Jika current invalid atau altitude tidak finite, ia mengganti waypoint internal
dengan posisi sekarang dan tipe loiter sebagai fallback. Jika valid, fungsi ini:

1. memilih cruising speed item atau `MPC_XY_CRUISE`, lalu membatasinya dengan
   `MPC_XY_VEL_MAX`;
2. memproyeksikan lat/lon current melalui `_reference_position.project()` ke
   local x/y;
3. menghitung local z Down sebagai
   `-(setpoint_altitude - local_reference_altitude)`;
4. memproyeksikan previous dan next jika valid; jika tidak, menyamakannya dengan
   current target;
5. mendeteksi perubahan target dengan toleransi komponen 0,001 m serta perubahan
   flag validitas previous/next;
6. membentuk internal waypoint/trajectory sesuai state off-track, target-behind,
   previous-current-next geometry, waypoint type, dan yaw policy.

Konsekuensi source-level-nya jelas: `next.valid=true` memberi informasi corner
ke trajectory generator, sedangkan `next.valid=false` membuat next internal sama
dengan target current sehingga vehicle merencanakan berhenti. Inilah mekanisme
yang dipakai `brake_for_hold`, bukan command motor khusus bernama “hold”.

Trajectory menghasilkan `trajectory_setpoint_s`. Di
`PositionControl::setInputSetpoint()` data itu masuk ke cascade berikut:

```text
PositionControl::update(dt)
  → _positionControl()       // position error → velocity setpoint
  → _velocityControl(dt)     // velocity PID + constraints → acceleration
  → _accelerationControl()   // gravity/tilt/hover thrust → thrust vector
  → vehicle_attitude_setpoint + vehicle_local_position_setpoint
  → mc_att_control → rate controller
  → vehicle_torque_setpoint + vehicle_thrust_setpoint
  → control_allocator → actuator_motors[0..5] pada Hexa-X
```

Setpoint global tidak pernah langsung menjadi PWM. Semua koreksi terjadi
terhadap state local NED hasil estimator; perubahan referensi/reset EKF dan
saturation pada lapisan bawah karena itu harus masuk analisis presisi.

## 7. Estimasi posisi dan koreksi error PX4 EKF2

### 7.1 Apa yang diestimasi

EKF2 mempropagasi attitude, velocity, dan position dengan IMU pada rate tinggi,
lalu mengoreksi drift memakai measurement yang lebih lambat: GNSS position dan
velocity, barometer, magnetometer/GNSS yaw, range finder, optical flow, external
vision, airspeed, dan sumber lain sesuai konfigurasi.

State relevan mencakup quaternion attitude, velocity NED, posisi global/local,
bias gyro, bias accelerometer, medan magnet, dan state tambahan sesuai build.
Output predictor mengompensasi delay estimator agar controller mendapat output
rate tinggi dan rendah latency.

### 7.2 Koreksi Kalman secara ringkas

Untuk measurement `z` dan prediksi measurement `h(x)`:

```text
innovation       ν = z - h(x)
innovation cov   S = H P Hᵀ + R
Kalman gain      K = P Hᵀ S⁻¹
state update     x = x + Kν
covariance       P = (I - KH)P
test ratio       = ν² / (gate² × S)       // bentuk skalar konseptual
```

Jika innovation terlalu besar terhadap uncertainty dan innovation gate,
measurement ditolak atau fusion dihentikan/reset sesuai aid source dan fault
logic. Jadi EKF bukan “merata-ratakan sensor”; bobot bergantung covariance,
noise model, delay, health checks, dan konsistensi innovation.

### 7.3 GNSS quality gate, bias, dan reset

Sebelum GNSS dipakai, EKF memeriksa hal seperti fix/quality, satellite count,
horizontal/vertical accuracy, speed accuracy, drift saat diam, dan consistency.
Nama serta default parameter harus dibaca dari firmware 1.16.1 yang benar;
jangan menyalin tuning versi lain.

Jika fusion normal, error kecil dikoreksi bertahap. Jika estimator kehilangan
observability atau sumber kembali dengan offset besar, EKF dapat melakukan
state reset. PX4 mempublikasikan reset counter dan delta agar downstream tidak
menafsirkan loncatan origin/position sebagai gerak fisik kendaraan.

Masalah yang tidak dapat diselesaikan EKF sendirian:

- multipath GNSS atau spoofing yang terlihat konsisten;
- lever-arm GNSS/camera/IMU yang tidak dimodelkan dengan benar;
- magnetometer terganggu arus listrik;
- rolling shutter dan shutter latency;
- datum altitude/geoid salah;
- DEM salah atau obstacle yang tidak ada di DEM;
- kamera gagal menyimpan frame.

### 7.4 Standard GNSS, RTK, dan PPK

Standard GNSS biasanya tidak cukup untuk produk centimeter-level. RTK memakai
base/correction stream dan carrier phase untuk solusi float/fixed; tetap pantau
fix type, baseline, age of corrections, number of satellites, reported hacc/vacc,
dan cycle slip. Jangan menerima label “RTK fixed” sebagai satu-satunya gate.

PPK dapat memperbaiki trajectory/camera center setelah penerbangan jika raw GNSS
dan timestamp exposure tersedia. GCP/check point tetap penting untuk mengukur
akurasi absolut produk akhir dan menangkap datum/bias sistematis.

## 8. Sumber error dan mitigasi

| Sumber | Dampak | Deteksi | Mitigasi |
|---|---|---|---|
| GNSS noise/multipath | grid bergeser/bergelombang | innovation, hacc, repeat pass | lokasi base baik, RTK/PPK, reject gate |
| Angin/controller | cross-track dan sudut terpotong | log setpoint vs estimate | kurangi speed, tune controller, tambah margin |
| Trigger latency/jitter | foto bergeser/tidak seragam | hotshoe/event timestamp | hardware trigger, time sync, kalibrasi latency |
| Camera interval limit | frame miss | feedback file/capture sequence | `speed ≤ trigger_distance / min_interval` |
| Relief terrain | GSD/overlap berubah | DEM/range/flight log | terrain follow, overlap margin, AGL validation |
| Datum/geoid mismatch | vertical bias konstan | benchmark/GCP | transform datum eksplisit |
| Lever arm/boresight | geotag bias saat attitude berubah | calibration flight | ukur transform IMU→antenna→camera |
| Rolling shutter/motion blur | geometry foto cacat | image QC | global shutter, exposure cepat, speed turun |
| Packet loss | upload tidak lengkap | retry/ACK/read-back | mission state machine + canonical compare |
| Polygon degenerat | jalur silang/segmen kecil | geometry validator | repair/reject, epsilon dan minimum segment |

Constraint kamera yang wajib dihitung:

```text
photo_rate_hz = ground_speed_mps / trigger_distance_m
photo_interval_s = trigger_distance_m / ground_speed_mps

requirement:
photo_interval_s >= camera_min_trigger_interval_s + safety_margin_s
```

Overlap operasional harus diberi margin terhadap expected position, altitude,
attitude, dan timing error. Jika toleransi maksimum cross-track adalah `e_xy`,
spacing efektif tidak boleh dipilih tepat pada batas teori. Lakukan Monte Carlo
atau worst-case envelope dengan distribusi error yang benar-benar diukur.

## 9. Blueprint membangun sistem kompatibel

### 9.1 Data model

Pisahkan immutable input dan derived output:

```text
SurveyInput
  polygon_wgs84[]
  camera_intrinsics + orientation
  target_gsd OR target_agl
  front_overlap, side_overlap
  grid_angle, entry_corner, turnaround
  altitude_mode + vertical_datum
  terrain_source + terrain_dataset_version

CompiledMission
  generator_version
  source_input_hash
  items[] {seq, frame, command, params[1..7], autocontinue}
  canonical_hash
  assumptions + warnings
```

Jangan gunakan float32 selama geometri polygon/proyeksi. Gunakan float64 dan
baru lakukan quantization sekali saat membentuk `MISSION_ITEM_INT`.

### 9.2 Pipeline deterministik

1. Validasi range lat/lon, winding, self-intersection, duplicate vertex, area,
   edge minimum, dan antimeridian.
2. Bekukan camera calibration, firmware version, terrain dataset/version, datum,
   dan unit.
3. Proyeksikan ke local tangent plane.
4. Hitung footprint, spacing, trigger distance, dan rate feasibility.
5. Generate, clip, sort, dan orient transects secara deterministik.
6. Tambahkan turnaround dengan model kinematika vehicle.
7. Apply terrain dengan sampling cukup rapat dan vertical constraints.
8. Expand ke mission command.
9. Quantize lat/lon; jalankan feasibility checks.
10. Upload dengan protocol state machine, download kembali, canonical compare.
11. Arm hanya jika estimator, GNSS/RTK, storage, camera, time sync, battery,
    geofence, wind, dan home/datum memenuhi gate.
12. Saat terbang, monitor deviation dan completeness; abort/retry bila keluar SLA.

### 9.3 Invariant penting

- `seq` kontigu dari 0 sampai N−1.
- Semua lat/lon finite dan berada dalam range legal.
- Tidak ada mission item parsial yang menjadi aktif.
- Compile input yang sama menghasilkan byte/canonical mission yang sama.
- Read-back sama dengan compiled mission dalam aturan normalisasi yang eksplisit.
- Jarak antar transect memenuhi spacing setelah quantization.
- Setiap capture region mempunyai start dan stop yang seimbang.
- Photo interval selalu di atas camera minimum plus margin.
- Altitude clearance minimum terpenuhi pada seluruh segmen, bukan hanya waypoint.
- Setiap timestamp mempunyai clock domain dan uncertainty.

## 10. Fokus Multicopter dan Hexacopter-X

### 10.1 Batas tanggung jawab QGC dan PX4

QGC tidak perlu mengetahui jumlah motor untuk membuat waypoint. Plan yang sama
dapat dikirim ke quad-X atau hexa-X selama tipe vehicle dan kemampuan firmware
kompatibel. Perbedaannya berada setelah Navigator:

```text
QGC mission: lat/lon/alt/yaw
             │
             ▼
PX4 Navigator → trajectory setpoint NED
             │
             ▼
FlightTaskAuto / trajectory smoothing
             │ position, velocity, acceleration, yaw, yaw-rate
             ▼
mc_pos_control → attitude setpoint + collective thrust
             │
             ▼
mc_att_control → body-rate setpoint
             │
             ▼
mc_rate_control → torque setpoint
             │
             ▼
Control Allocator → enam perintah actuator
             │
             ▼
PWM/DShot/CAN driver → ESC → motor
```

Navigator tidak langsung menghasilkan PWM. Ketelitian lintasan ditentukan oleh
seluruh rantai estimator, trajectory generator, position/velocity loop,
attitude/rate loop, allocator, ESC, motor, propeller, massa, CG, dan gangguan.

### 10.2 Definisi airframe Generic Hexarotor X

Source `ROMFS/px4fmu_common/init.d/airframes/6001_hexa_x` menetapkan:

```text
MAV_TYPE       = 13   // MAV_TYPE_HEXAROTOR
CA_ROTOR_COUNT = 6

rotor  PX     PY      KM eksplisit
0      0.00   0.50   -0.05
1      0.00  -0.50    default lawan tanda
2      0.43  -0.25   -0.05
3     -0.43   0.25    default lawan tanda
4      0.43   0.25    default lawan tanda
5     -0.43  -0.25   -0.05
```

`PX/PY` adalah posisi rotor ternormalisasi pada body frame FRD: x forward,
y right, z down. `KM` adalah koefisien momen yaw terhadap thrust; tanda yang
berlawanan merepresentasikan arah putaran berlawanan. Nilai yang tidak ditulis
airframe memakai default parameter, sehingga audit konfigurasi nyata harus
mengambil parameter dump dari flight controller, bukan hanya script airframe.

Control Allocator membangun effectiveness matrix secara konseptual:

```text
wrench = B × actuator

wrench = [torque_x, torque_y, torque_z, force_x, force_y, force_z]ᵀ
actuator = [motor0, motor1, motor2, motor3, motor4, motor5]ᵀ
```

Allocator mencari actuator command yang menghasilkan torque/thrust diminta
dengan batas minimum/maksimum dan penanganan saturation. Redundansi enam rotor
memberi lebih banyak actuator daripada empat axis utama, tetapi **tidak otomatis
menjamin** penerbangan aman setelah satu motor gagal. Itu bergantung geometry,
thrust reserve, arah propeller, failure detection, allocator method, dan tuning.

Jangan menebak nomor motor dari urutan fisik. Verifikasi satu per satu dengan
Actuator Test QGC dalam kondisi propeller dilepas, lalu pastikan:

- posisi `CA_ROTORn_PX/PY/PZ` sesuai pusat thrust motor terhadap CG;
- arah axis `CA_ROTORn_AX/AY/AZ` sesuai orientasi thrust;
- tanda `CA_ROTORn_KM` sesuai CW/CCW aktual;
- output function `Motor 1..6` terhubung ke ESC yang benar;
- minimum, maximum, disarmed, protocol, dan direction ESC benar;
- semua motor memiliki thrust response yang seimbang.

Kesalahan mapping satu motor dapat tampak stabil saat disarmed tetapi menyebabkan
flip saat takeoff. Failure detector attitude aktif khusus pada takeoff bahkan
ketika flight termination in-flight dinonaktifkan, tetapi itu bukan pengganti
pengujian actuator tanpa propeller.

### 10.3 Loop kendali multicopter

Untuk position control, error utama secara konseptual:

```text
e_p = position_setpoint - estimated_position
velocity_sp = feedforward_velocity + Kp_position × e_p

e_v = velocity_sp - estimated_velocity
acceleration_sp = feedforward_acceleration
                + Kp_velocity × e_v
                + Ki_velocity × integral(e_v)
                + Kd_velocity × derivative/filter(e_v)

thrust_vector = function(acceleration_sp, gravity, hover_thrust)
attitude_sp = direction(thrust_vector) + yaw_sp
```

Implementasi mempunyai limit velocity, acceleration, jerk, tilt, thrust, dan
anti-windup. Parameter mission multicopter yang sangat relevan antara lain:

| Kelompok | Parameter penting | Efek |
|---|---|---|
| Reach | `NAV_ACC_RAD`, `NAV_MC_ALT_RAD` | radius horizontal dan vertikal target |
| Cruise | `MPC_XY_CRUISE` | kecepatan horizontal auto nominal |
| Vertical | `MPC_Z_V_AUTO_UP/DN` | climb/descent auto |
| Dynamics | `MPC_ACC_HOR`, `MPC_JERK_AUTO` | percepatan dan kehalusan trajectory |
| Error | `MPC_XY_ERR_MAX` | batas error horizontal pada trajectory control |
| Position gains | `MPC_XY_P`, `MPC_Z_P` | position error ke velocity target |
| Velocity gains | `MPC_XY_VEL_*`, `MPC_Z_VEL_*` | velocity loop ke acceleration/thrust |
| Attitude limit | `MPC_TILTMAX_AIR`, `MPC_TILTMAX_LND` | tilt saat flight/landing |
| Thrust | `MPC_THR_MIN/MAX/HOVER`, `MPC_USE_HTE` | authority dan hover model |

Nilai default di source bukan rekomendasi otomatis untuk setiap hexa. Tuning
rate/attitude wajib sehat sebelum position tuning; thrust-to-weight ratio,
inertia, propeller, dan CG hexa berbeda dari reference airframe.

### 10.4 Cara multicopter membelok dan berotasi antar-waypoint

QGC mengirim titik diskret, bukan stream kurva belok. Untuk Survey, QGC dapat
menambahkan endpoint/turnaround sebagai waypoint; untuk waypoint biasa QGC
tetap hanya menentukan urutan dan coordinate. PX4 yang membuat trajectory
kontinu dari triplet:

```text
previous = WP1
current  = WP2
next     = WP3
```

Misalnya jalur tidak lurus:

```text
WP1 ─────────► WP2
               │
               │
               ▼ WP3
```

Ada dua rotasi independen:

1. **Rotasi vektor kecepatan/lintasan:** vehicle mengubah arah gerak dari timur
   ke selatan. Ini memerlukan acceleration horizontal dan tilt roll/pitch.
2. **Rotasi heading/yaw badan:** nose dapat ikut menghadap WP3, menghadap arah
   velocity, menghadap home, atau tetap pada heading lama sesuai yaw policy.

Multicopter tidak harus menghadap arah perjalanan. Ia dapat bergerak menyamping
menuju WP3 sambil mempertahankan yaw tetap.

#### Pembentukan kurva translasi

`FlightTaskAuto` mengubah triplet global WGS-84 menjadi waypoint lokal NED,
lalu mengirim array `{previous, current, next}` ke `PositionSmoothing`.
Smoother:

1. menghitung arah dan jarak terhadap current/next;
2. menghitung kecepatan yang masih memungkinkan belok dengan batas dynamic;
3. memakai crossing point/L1-style point saat trajectory sedang berbelok;
4. menghasilkan position, velocity, acceleration, dan jerk setpoint kontinu;
5. memperlambat waktu trajectory bila vehicle tertinggal terlalu jauh dari
   moving setpoint.

Source mendeteksi kondisi “turning” jika velocity lebih dari 0,2 m/s, sudut
antara velocity dan arah target lebih dari kira-kira 10° (`cos < 0.98`), dan
trajectory belum masuk acceptance radius. Kecepatan di dekat corner dihitung
dari sudut WP1–WP2–WP3, acceptance radius, cruise speed, acceleration, dan jerk.
Model dasarnya adalah kurva singgung dengan kebutuhan centripetal:

```text
a_c = v² / r

v_turn ≤ sqrt(a_available × r_effective)
```

Karena itu belokan tajam, acceptance radius kecil, atau acceleration limit kecil
memaksa kecepatan lebih rendah. Acceptance radius besar mengizinkan corner cut
lebih besar. PX4 multicopter menyatakan waypoint posisi reached ketika horizontal
distance masuk acceptance radius dan vertical error masuk altitude acceptance
radius; ia tidak harus menyentuh coordinate matematis tepat.

Parameter dominan:

| Parameter | Dampak pada belokan |
|---|---|
| `NAV_ACC_RAD` atau waypoint `param2` | radius pergantian/reach; besar berarti corner dapat dipotong lebih jauh |
| `NAV_MC_ALT_RAD` | toleransi vertikal saat memajukan item |
| `MPC_XY_CRUISE` | kecepatan auto nominal |
| `MPC_ACC_HOR` | acceleration horizontal yang direncanakan |
| `MPC_JERK_AUTO` | seberapa cepat acceleration boleh berubah |
| `MPC_XY_TRAJ_P` | scaling acceleration yang dipakai dalam perhitungan radius/speed |
| `MPC_XY_ERR_MAX` | batas error sebelum trajectory time-stretch/perlambatan |
| `MPC_TILTMAX_AIR` | batas tilt, sehingga membatasi acceleration horizontal aktual |

Position controller mengubah acceleration horizontal menjadi thrust vector.
`ControlMath::thrustToAttitude()` membentuk body-z sesuai arah thrust dan
menggabungkannya dengan yaw target. Jadi untuk berbelok, vehicle miring; komponen
thrust horizontal mengubah velocity, sedangkan komponen vertikal menahan berat.
Jika tilt/thrust saturated, jalur aktual melebar atau tertinggal dari trajectory.

#### Pemilihan yaw saat belok

Jika mission item mempunyai `param4` yaw finite, yaw itu menjadi target item dan
weather-vane dipaksa nonaktif. Jika `param4=NaN`, `FlightTaskAuto` memakai
`MPC_YAW_MODE`:

| Nilai | Perilaku heading |
|---:|---|
| `0` | nose menuju current waypoint |
| `1` | nose menuju home |
| `2` | nose menjauhi home |
| `3` | nose mengikuti velocity/trajectory |
| `4` | yaw menuju waypoint terlebih dahulu, kemudian translasi |
| `5` | yaw fixed; tidak otomatis mengikuti arah jalur |

Untuk mode 0, vector `target - position` menghasilkan bearing yaw. Setelah
masuk acceptance radius, yaw dikunci untuk mencegah perubahan heading berlebihan
ketika vector menjadi sangat kecil. Untuk mode 3, heading hanya dihitung jika
velocity setpoint XY > 0,1 m/s dan jarak target > 2 m; jika tidak, previous yaw
ditahan. Mode 4 membuat velocity setpoint nol sampai yaw dianggap aligned.

Perubahan yaw target tidak diberikan sebagai step tanpa batas. `_limitYawRate()`
membatasi perubahan per-cycle dengan `MPC_YAWRAUTO_MAX`, membuat yaw-rate
feed-forward, dan menilai alignment memakai `MIS_YAW_ERR`. Attitude controller
mengubah quaternion yaw error menjadi body-rate target; rate controller mengubah
gyro-z error menjadi torque-z. `MC_YAW_P`, `MC_YAWRATE_*`, filter, dan saturation
menentukan tracking aktual.

#### Realisasi belokan pada Hexacopter-X

Control Allocator memisahkan kebutuhan wrench:

```text
roll/pitch torque → beda thrust berdasarkan lengan PX/PY rotor
yaw torque        → beda kelompok CW dan CCW berdasarkan KM
collective thrust → kenaikan/penurunan keenam motor bersama
```

Saat WP1→WP2 berubah menjadi WP2→WP3, position controller meminta acceleration
baru, attitude controller meminta tilt baru, dan—jika yaw policy menghendaki—
torque yaw secara paralel. Yaw dapat tertinggal karena authority yaw lebih kecil
dan diprioritaskan di bawah roll/pitch untuk stabilitas. Belokan agresif dapat
menyaturasi motor sehingga translasi, altitude, dan yaw saling berebut authority.

Untuk lintasan presisi, log dan bandingkan:

```text
previous/current/next global setpoint
trajectory position/velocity/acceleration/yaw/yawspeed
vehicle local position/velocity/heading
attitude dan body-rate setpoint versus aktual
thrust/torque setpoint
control allocation saturation dan enam actuator output
```

Uji minimal sudut 15°, 45°, 90°, 135°, dan balik arah 180° pada beberapa speed
dan acceptance radius. Ukur corner-cut distance, cross-track P95/P99, minimum
speed di corner, yaw error/rate, altitude loss, dan saturation. Jangan menilai
presisi hanya dari garis QGC karena garis itu adalah rencana, bukan trajectory
setpoint internal atau lintasan aktual.

### 10.5 Cara PX4 menahan posisi di waypoint

#### QGC hanya mengirim definisi hold, bukan mengendalikan motor

Pada `MAV_CMD_NAV_WAYPOINT`, QGC mengirim `param1` sebagai waktu hold/time
inside (detik), `param2` sebagai acceptance radius, `param3` sebagai pass
radius, `param4` sebagai yaw (`NaN` berarti kebijakan yaw PX4), serta
latitude/longitude/altitude. Setelah upload, QGC tidak men-stream koreksi motor.
PX4 menyimpan item dan menjalankan closed loop onboard; hilangnya QGC hanya akan
mengubah perilaku jika kebijakan data-link failsafe memerintahkannya.

#### Tiga perilaku yang sering tertukar

```text
param1 = 0, autoContinue = true
  next valid → boleh fly-through/corner smoothing; belum tentu berhenti

param1 > 0
  brake_for_hold → next invalid → rem menuju current waypoint
  → tunggu time_inside → lanjut ke waypoint berikutnya

waypoint terakhir
  tidak ada next → pertahankan target terakhir sebagai loiter/hold
  → sampai mode, mission, operator, atau failsafe mengubahnya
```

Untuk rotary-wing, `Mission::setActiveMissionItems()` sengaja membuat
`position_setpoint_triplet.next.valid = false` ketika `time_inside > 0`.
`FlightTaskAuto` lalu tidak memakai waypoint berikutnya untuk fly-through dan
dapat mengerem menuju target. Current item masih bertipe
`SETPOINT_TYPE_POSITION`; ini berbeda secara internal dari Loiter eksplisit.
Jika `autoContinue=false`, sequence juga tidak maju otomatis, tetapi kelanjutan
membutuhkan perintah eksternal sehingga bukan pengganti dwell terukur.

#### Kapan timer hold mulai

PX4 terlebih dahulu memeriksa:

```text
horizontal_distance <= acceptance_radius
AND |altitude_error| <= altitude_acceptance_radius
AND yaw_reached, bila yaw item menjadi syarat
```

Acceptance horizontal memakai waypoint `param2` bila finite dan positif, atau
default seperti `NAV_ACC_RAD`; vertikal memakai konfigurasi seperti
`NAV_MC_ALT_RAD`. Setelah reach, PX4 mencatat `_time_wp_reached`. Item selesai
saat `elapsed_since_reached >= max(time_inside, 0)`.

Jadi hold 5 detik berarti lima detik setelah kriteria reach dianggap terpenuhi,
**bukan** jaminan kecepatan tepat nol selama lima detik. `brake_for_hold`
membantu berhenti, tetapi speed aktual harus dibuktikan dari log. Acceptance
radius adalah ambang pemajuan mission, bukan jaminan error hover.

#### Dari koordinat global ke hover lokal

```text
mission lat/lon/alt WGS-84
  → Navigator current global setpoint
  → proyeksi terhadap referensi local/global EKF
  → FlightTaskAuto target local NED
  → trajectory position/velocity/acceleration setpoint
  → position controller: thrust vector + yaw
  → attitude/rate controller: torque/thrust
  → control allocator Hexa-X → enam motor
```

Pada target diam, trajectory akhirnya meminta velocity mendekati nol. Loop
kontrol secara ringkas:

```text
velocity_sp = Kp_pos × (position_sp - estimated_position)

acceleration_sp = Kp_vel × (velocity_sp - estimated_velocity)
                + velocity_integrator
                - Kd_vel × velocity_derivative
                + feed-forward
```

Jika angin menggeser vehicle ke timur, error posisi meminta velocity ke barat;
velocity loop meminta tilt/thrust ke barat. Integrator membangun bias untuk
menolak angin menetap, sementara anti-windup membatasinya saat thrust/tilt
saturated. Yaw ditahan paralel oleh attitude/rate loop; allocator memberi beda
thrust pada kelompok motor CW/CCW sambil mempertahankan collective thrust.

#### Penentu presisi hold nyata

| Lapisan | Faktor dominan |
|---|---|
| Estimator | GNSS/RTK, multipath, vibration, compass/yaw, barometer, EKF innovation/reset |
| Reach | `param2`, `NAV_ACC_RAD`, `NAV_MC_ALT_RAD`, `MIS_YAW_ERR` |
| Trajectory | `MPC_XY_CRUISE`, `MPC_ACC_HOR`, `MPC_JERK_AUTO` |
| Controller | keluarga `MPC_XY_P`, `MPC_Z_P`, `MPC_XY_VEL_*`, `MPC_Z_VEL_*` |
| Authority | `MPC_THR_HOVER`/hover-thrust estimator, `MPC_THR_MAX`, `MPC_TILTMAX_AIR` |
| Hexa-X | geometri/direction rotor, coefficient, CG, motor-prop matching, saturation |

Nama dan range parameter harus mengikuti metadata firmware yang dipakai; gain
tidak boleh disalin buta. RTK dapat memperkecil error global, tetapi tidak
menggantikan tuning, yaw yang benar, thrust reserve, dan mitigasi angin. Reset
posisi/yaw EKF juga dapat mengubah error yang dilihat controller walaupun gerak
fisik vehicle berbeda, sehingga reset counters wajib dipantau.

#### Uji hold secara terukur

Uji waypoint tengah dengan `param1` 5–10 detik dan waypoint terakhir, tanpa
angin lalu dengan gangguan aman. Rekam:

```text
mission sequence/result/reached dan position_setpoint_triplet next.valid
trajectory_setpoint position/velocity/acceleration/yaw
vehicle_local_position, global_position, attitude, angular_velocity
thrust/torque setpoint, actuator outputs, allocation saturation
estimator innovations dan reset counters
```

Hitung error horizontal/vertikal, speed saat timer mulai, settling time, drift
P95/P99 selama dwell, yaw error, motor headroom, dan respons terhadap gust.
Tidak ada desain yang jujur menjamin nol error: kualitas hold dibatasi estimator,
tuning, authority aktuator, dan lingkungan. Tetapkan abort/failsafe jika accuracy
EKF, tilt, thrust saturation, atau geofence melewati batas yang divalidasi.

## 11. Takeoff Multicopter dalam Mission

### 11.1 Yang dilakukan QGC

Di QGC 5.0.8, `MissionController::insertTakeoffItem()` membuat
`TakeoffMissionItem` dengan `MAV_CMD_NAV_TAKEOFF` untuk multicopter. QGC
menempatkannya dalam sequence mission dan mengirimnya sama seperti item lain.
Untuk PX4 multicopter, `TakeoffMissionItem` berusaha menjaga coordinate takeoff
sama dengan launch/home dari sudut pandang pengguna kecuali launch atau takeoff
sengaja dipindahkan. Altitude dan frame tetap wajib diperiksa pada simple item
hasil ekspansi.

Tiga operasi QGC yang tampak serupa tetapi kontraknya berbeda:

```text
Plan: Takeoff item  → tersimpan sebagai bagian mission, dieksekusi Navigator
Fly: Takeoff action → command guided takeoff sekali ke PX4
Offboard takeoff    → external controller men-stream setpoint; bukan Plan QGC
```

Demikian juga tombol **Land** dan **RTL** pada Fly View adalah guided/mode
commands real-time, bukan otomatis menambahkan `NAV_LAND`/RTL ke file mission.
Sistem clone harus memisahkan editor mission, command transaksi sekali, mode
request, dan Offboard stream.

### 11.2 Preflight dan arming

Sebelum arm, Commander menjalankan Health and Arming Checks terhadap sensor,
estimator, home/global position sesuai mode, power/battery, ESC telemetry bila
diwajibkan, mission feasibility, geofence, RC/data link, SD/logger, dan checks
lain sesuai konfigurasi. Arming berhasil tidak membuktikan mission aman; itu
hanya berarti checks aktif melewati threshold saat tersebut.

Untuk auto mission dari keadaan landed, minimal pastikan:

- home dan global position valid;
- yaw aligned dan tidak melompat saat throttle naik;
- mission memiliki takeoff yang valid atau PX4 dapat menyisipkan initial climb;
- first waypoint distance lulus `MIS_DIST_1WP` bila check diaktifkan;
- altitude frame dan planned home konsisten dengan home PX4 aktual;
- takeoff volume bebas obstacle termasuk drift saat spool-up.

### 11.3 Mission takeoff versus takeoff state machine

Ada dua lapisan terpisah:

1. Navigator memproses `MAV_CMD_NAV_TAKEOFF` atau membuat work item climb.
2. `mc_pos_control` mengelola state motor/takeoff agar thrust tidak melonjak.

State takeoff controller:

```text
disarmed → spoolup → ready_for_takeoff → rampup → flight
```

Setelah arm, spool-up hysteresis menunggu motor siap. Saat `want_takeoff` aktif,
velocity limit ke atas diramp secara bertahap selama `MPC_TKO_RAMP_T` menuju
`MPC_TKO_SPEED`. Karena NED memakai z positif ke bawah, climb velocity adalah
nilai z negatif.

Jika mission dimulai tanpa takeoff yang memadai ketika landed, Mission logic
dapat membentuk climb pada posisi lat/lon kendaraan saat ini menuju initial
climb altitude, kemudian melanjutkan waypoint sebenarnya. Jika kendaraan sudah
terbang dan menemukan takeoff item rotary-wing, item dapat diperlakukan sebagai
waypoint. Perilaku exact juga bergantung state mission/work item; karena itu
mission generator baru tetap sebaiknya menghasilkan takeoff eksplisit.

Parameter relevan:

- `MIS_TAKEOFF_ALT`: minimum/initial mission takeoff height yang dipakai logic
  mission pada kondisi terkait;
- `MPC_TKO_RAMP_T`: durasi ramp velocity/thrust takeoff;
- `MPC_TKO_SPEED`: climb speed selama automatic takeoff;
- `COM_SPOOLUP_TIME`: waktu spool-up sebelum takeoff;
- failure detector attitude/roll/pitch timeouts untuk mendeteksi flip.

Jangan membuat waypoint pertama horizontal jauh dengan altitude sangat rendah.
Lebih aman: takeoff vertikal ke clear altitude, stabil, baru transit horizontal.

### 11.4 Mengapa takeoff mission tidak memerintahkan rotasi yaw

Pada QGC 5.0.8, `MAV_CMD_NAV_TAKEOFF.param4` memang didefinisikan sebagai yaw
dalam derajat, tetapi default multicopter adalah `NaN` (`nanUnchanged=true`).
Untuk multicopter, QGC juga menjaga coordinate takeoff sama dengan launch/home,
sehingga bagian awal takeoff adalah climb vertikal, bukan penerbangan horizontal
menuju titik lain.

Di PX4 1.16.1, `mavlink_mission` mula-mula mengubah `param4` derajat menjadi
`mission_item.yaw` radian. Namun jalur multicopter takeoff kemudian sengaja
mengirim yaw tak-tertentu:

```text
QGC TAKEOFF param4 = NaN
       │ MISSION_ITEM_INT
       ▼
PX4 mission_item.yaw = NaN
       │ Navigator NAV_CMD_TAKEOFF/work-item climb
       ▼
position_setpoint_triplet.current.yaw = NaN
       │
       ▼
FlightTaskAuto memilih/menahan heading yang kontinu
```

`Mission::handleTakeoff()` mengeset `_mission_item.yaw = NAN` dengan komentar
bahwa `FlightTaskAuto` menangani yaw secara langsung. Ketika takeoff work item
selesai dan diubah menjadi waypoint, yaw kembali diabaikan agar vehicle tidak
berputar sebelum heading logic mengambil alih. `MissionBlock` juga tidak
menerbitkan yaw tertentu untuk takeoff karena Navigator sendiri tidak menangani
yaw reset.

`NaN` di sini **bukan berarti motor yaw dibiarkan bebas**. Saat Auto task aktif:

1. `FlightTaskAuto::activate()` menginisialisasi `_yaw_setpoint` dan
   `_yaw_sp_prev` dari yaw estimate kendaraan saat itu.
2. Bila triplet tidak memberikan yaw, `_set_heading_from_mode()` memakai
   `MPC_YAW_MODE`. Pada climb vertikal ke coordinate yang sama, vector XY target
   sangat kecil/di dalam acceptance radius, sehingga logic mengunci yaw saat
   ini alih-alih menghitung bearing yang tidak stabil.
3. Jika yaw target dan yaw-rate sama-sama tidak finite, Auto task mencoba
   heading sepanjang trajectory; bila tidak ada gerakan XY yang cukup, ia
   menahan `_yaw_sp_prev`.
4. Perubahan yaw setpoint dibatasi `MPC_YAWRAUTO_MAX`; attitude controller
   mengubah yaw error menjadi yaw-rate target; rate controller menghasilkan
   torque-z untuk melawan rotasi fisik.
5. Jika EKF melakukan heading reset, FlightTask dan attitude controller
   menggeser setpoint dengan `delta_psi` agar perubahan koordinat estimator
   tidak diterjemahkan menjadi perintah putar mendadak.

Jadi perilaku yang dituju adalah **heading hold pada arah saat arm/takeoff**,
bukan yaw absolut nol dan bukan jaminan sudut tidak berubah satu derajat pun.
Setelah masuk waypoint horizontal, `MPC_YAW_MODE=0` default dapat memutar nose
menuju waypoint berikutnya. Bila seluruh mission harus mempertahankan heading,
opsi `MPC_YAW_MODE=5` (`yaw fixed`) tersedia pada source ini, tetapi harus diuji
terhadap kebutuhan mission dan cara yaw command/manual override diberikan.

Pada Hexacopter-X, heading hold hanya berhasil bila yaw authority fisik benar.
Airframe `6001_hexa_x` membuat tiga rotor `KM=-0.05` (CW) dan tiga rotor memakai
default `KM=+0.05` (CCW). Control Allocator menaikkan kelompok satu arah dan
menurunkan kelompok lawannya untuk menghasilkan torque yaw sambil menjaga
collective thrust. Rotor direction atau tanda `KM` yang tertukar membuat
feedback bertindak ke arah salah atau kehilangan authority.

Jika vehicle tetap berputar saat takeoff, urutan diagnosis yang benar:

1. Lepas propeller; cocokkan Motor 1–6, posisi `PX/PY`, arah putar, propeller,
   dan tanda `KM` dengan actuator test.
2. Periksa `vehicle_attitude.yaw`, `quat_reset_counter`, `delta_q_reset`, EKF yaw
   innovations, magnetometer/GNSS-yaw health. Bedakan badan benar-benar berputar
   dari estimate yaw yang melompat.
3. Bandingkan `trajectory_setpoint.yaw/yawspeed`,
   `vehicle_attitude_setpoint`, `vehicle_rates_setpoint.yaw`, actual gyro-z,
   torque setpoint, actuator outputs, dan allocation saturation.
4. Periksa thrust/ESC/motor/propeller tidak seimbang, CG, frame twist, ground
   friction, satu motor lambat spool-up, serta thrust margin.
5. Baru evaluasi `MC_YAW_P`, `MC_YAWRATE_K/P/I/D`, filter, dan
   `MPC_YAWRAUTO_MAX`; jangan menutupi wiring/geometry salah dengan tuning.
6. Pastikan weather-vane/ROI/manual yaw override tidak aktif dan QGC takeoff
   `param4` tetap `NaN` jika yang diinginkan adalah mempertahankan heading awal.

Kriteria uji sebaiknya numerik: maksimum yaw excursion, yaw-rate P95/P99,
torque/allocation saturation, dan tidak ada heading reset dari arm sampai clear
altitude. “Tidak terlihat berputar” saja tidak cukup untuk menyatakan sistem
benar.

## 12. Landing Multicopter

Pada Plan View multicopter, landing biasa direpresentasikan sebagai simple
`MAV_CMD_NAV_LAND`; landing-pattern complex item yang kaya terutama mempunyai
perlakuan khusus untuk fixed-wing/VTOL. Jangan menerapkan glide-slope fixed-wing
ke multicopter. QGC menentukan coordinate/altitude command, sedangkan profil
descent, land detection, dan disarm tetap diputuskan PX4.

### 12.1 Mission land

`MAV_CMD_NAV_LAND` menjadi landing setpoint. Untuk multicopter, vehicle menuju
lokasi land, turun dengan profil kecepatan terbatas, mendeteksi kontak/landed,
kemudian Commander dapat auto-disarm sesuai parameter. `MAV_CMD_NAV_RETURN_TO_LAUNCH`
berbeda: RTL adalah mode/prosedur yang dapat climb, return, descend, lalu land.

Profil vertical landing PX4 1.16.1 menggunakan parameter:

| Parameter | Fungsi |
|---|---|
| `MPC_LAND_ALT1` | mulai transisi dari auto descent menuju land speed |
| `MPC_LAND_ALT2` | di bawah ini descent dibatasi `MPC_LAND_SPEED` |
| `MPC_LAND_ALT3` | jika range valid, di bawah ini gunakan crawl speed |
| `MPC_LAND_SPEED` | kecepatan landing normal |
| `MPC_LAND_CRWL` | crawl speed dekat tanah |
| `MPC_TILTMAX_LND` | tilt maksimum selama landing |

Altitude threshold dekat tanah bergantung validitas `dist_bottom`/range sensor.
Tanpa observasi jarak tanah yang valid, logic memakai sumber local altitude dan
sebagian close-to-ground check dapat dilewati; hasil di terrain miring atau
vegetasi harus diuji khusus.

### 12.2 Land detector

Land detector multicopter tidak hanya membaca altitude. Ia menggabungkan:

- commanded descent;
- vertical dan horizontal motion dari local position;
- angular motion roll/pitch;
- thrust setpoint relatif terhadap min/hover thrust;
- distance-to-ground bila observable;
- hysteresis waktu agar noise singkat tidak dianggap landed.

State konseptualnya:

```text
flying → ground_contact → maybe_landed → landed
```

Ground contact memerlukan low thrust, descent command, dan tidak ada gerak yang
berarti. `maybe_landed` memperketat thrust dan rotation/motion. `landed` baru
aktif setelah condition bertahan selama hysteresis. False landed berbahaya
karena dapat menurunkan thrust/disarm; false not-landed membuat motor lama aktif.

Setelah `landed=true`, auto-disarm dikendalikan parameter Commander seperti
`COM_DISARM_LAND`. Selama pengujian, jangan mengandalkan auto-disarm sebagai
kill switch; operator tetap harus mempunyai prosedur abort/kill yang aman.

### 12.3 Precision landing

Normal `NAV_LAND` memakai target global/local dan estimator biasa. Precision
landing merupakan subsystem berbeda (`precland`) yang membutuhkan target
measurement seperti `LANDING_TARGET`. Tanpa beacon/vision target, PX4 tidak
mengoreksi landing ke marker hanya dari QGC waypoint. GNSS RTK memperbaiki
global position tetapi tidak otomatis mendeteksi obstacle atau permukaan aman.

## 13. Failsafe yang Dapat Menginterupsi Mission

### 13.1 Prinsip arbiter failsafe

Failsafe bukan satu `if`. Commander mengumpulkan health/failure flags, kondisi
mode, armed/landed state, ketersediaan position/manual control, lalu memilih
aksi yang layak. Jika beberapa failure aktif, aksi yang lebih berat dapat
menggantikan yang lebih ringan. Aksi juga dapat berubah ketika position menjadi
invalid sehingga RTL/Hold tidak lagi mungkin.

```text
failure detector/health flags
        │
        ▼
Commander failsafe state machine
        │ pilih aksi yang tersedia
        ├── Warning / tetap pada mode
        ├── Hold
        ├── Return
        ├── Land
        ├── Disarm (kondisi tertentu)
        └── Terminate / lockdown
```

`Terminate` mematikan output yang dikendalikan dan dapat menjatuhkan vehicle.
Jangan mengaktifkannya tanpa analisis keselamatan, recovery system, area uji,
dan pemahaman `CBRK_FLIGHTTERM`.

### 13.2 Failure utama dan parameter kebijakan

| Kondisi | Deteksi/timeout | Kebijakan utama | Catatan mission |
|---|---|---|---|
| Manual/RC loss | `COM_RC_LOSS_T` | `NAV_RCL_ACT` | dapat diabaikan dalam kondisi/mode tertentu sesuai config |
| GCS/data-link loss | `COM_DL_LOSS_T` | `NAV_DLL_ACT` | mission onboard tidak butuh QGC, tetapi kebijakan dapat tetap menginterupsi |
| Offboard signal loss | `COM_OF_LOSS_T` | `COM_OBL_RC_ACT` | khusus Offboard, bukan Mission onboard |
| Low/critical battery | threshold battery | `COM_LOW_BAT_ACT` | dapat Warning, Return, Land sesuai level |
| Position/estimator loss | validity/accuracy timeout | fallback mode/action Commander | RTL tak mungkin tanpa global position valid |
| Geofence breach | fence evaluator | `GF_ACTION` | Warn, Hold, Return, Land, atau Terminate sesuai pilihan |
| Excess attitude | `FD_FAIL_R/P`, `FD_FAIL_R_TTRI` dan terkait | lockdown/termination | takeoff check memiliki perlakuan khusus |
| Motor failure | ESC/current/RPM criteria bila dikonfigurasi | failure flag/termination atau allocator handling | bukan jaminan hexa tetap terbang |
| Mission invalid | feasibility/mission result | reject start/loiter | upload ACK saja tidak cukup |

Nama dan enum action harus dibaca dari parameter metadata firmware terpasang.
Jangan menyalin angka enum dari artikel atau versi PX4 lain karena opsi/default
dapat berubah. Simpan parameter dump yang diuji bersama firmware hash.

### 13.3 Data-link loss bukan hilangnya mission

Setelah mission berhasil disimpan di PX4, kehilangan QGC tidak menghapus daftar
waypoint. Secara teknis Navigator dapat melanjutkan mission onboard. Namun
Commander dapat menjalankan `NAV_DLL_ACT` setelah `COM_DL_LOSS_T`, sehingga
perilaku nyata ditentukan kebijakan failsafe. Untuk operasi tanpa link:

1. tentukan secara eksplisit apakah mission harus Continue, Hold, RTL, atau Land;
2. uji loss di SITL/HITL dan area aman;
3. perhitungkan bahwa QGC tidak lagi menampilkan keadaan aktual;
4. pastikan RTL/home/geofence dan battery reserve tetap valid.

RC loss dan data-link loss adalah dua channel berbeda. Radio kontrol hilang
tidak sama dengan telemetry QGC hilang.

### 13.4 Position failure

Mission global membutuhkan global horizontal position. Bila GNSS/EKF gagal:

- current waypoint tetap tersimpan, tetapi controller tidak mempunyai state
  global yang cukup untuk mengikutinya;
- Hold/RTL berbasis position mungkin tidak tersedia;
- fallback dapat menjadi Altitude, Stabilized, Land, atau termination tergantung
  sensor yang masih valid dan konfigurasi;
- jika hanya global reference terganggu tetapi local position masih valid,
  mode lokal tertentu mungkin tetap tersedia, namun mission global tidak boleh
  diasumsikan aman.

Uji GNSS dropout, spoofed jump, yaw reset, baro failure, dan estimator reset.
Pantau reset counters; loncatan estimate dapat menggeser error terhadap waypoint
tanpa vehicle benar-benar berpindah.

### 13.5 RTL bukan garis langsung sederhana

RTL mempunyai state dan tipe strategi: direct RTL, return melalui mission/land
pattern, atau varian fast/reverse sesuai `RTL_TYPE` serta mission yang tersedia.
Direct multicopter RTL secara umum memilih safe return altitude, climb bila
perlu, kembali ke home/destination, descend, lalu land/loiter sesuai parameter.

Parameter penting mencakup keluarga `RTL_*`, misalnya return altitude, descend
altitude, landing delay, minimum distance, cone/safe altitude, serta `RTL_TYPE`.
Validasi clearance sepanjang segmen RTL; altitude aman terhadap home belum tentu
aman terhadap bukit di tengah jalur.

## 14. Mission versus Offboard

### 14.1 Perbedaan kontrak

| Aspek | Mission | Offboard |
|---|---|---|
| Sumber target | daftar item tersimpan di PX4 | komputer eksternal mengirim setpoint live |
| QGC/link putus | mission masih tersimpan; DLL policy berlaku | proof-of-life hilang memicu Offboard-loss |
| Koordinat umum | global WGS-84 mission items | local NED trajectory setpoint atau level lain |
| Pemajuan target | Navigator `current_seq` + reach logic | aplikasi eksternal |
| Rate komunikasi | transaksi upload lalu telemetry | stream kontinu >2 Hz untuk availability |
| Failsafe utama | mission/DLL/RC/position/battery | semua itu + Offboard-loss |

QGC standard Plan View menjalankan **Mission**, bukan men-stream semua waypoint
sebagai Offboard setpoint. Guided goto/reposition juga bukan identik dengan
mission ataupun streaming Offboard.

QGC Fly View menyediakan guided takeoff, goto, altitude change, pause, land,
RTL, orbit, dan ROI melalui `Vehicle`/PX4 firmware plugin. Operasi itu mengirim
mode atau vehicle command saat diminta; operasi tersebut tidak berarti QGC
mengambil alih closed-loop controller dan bukan proof-of-life Offboard.

### 14.2 Offboard control hierarchy

Offboard memilih level kontrol melalui `OffboardControlMode`. Field pertama yang
aktif dari atas menentukan estimator requirement dan loop yang dilewati:

```text
position → velocity → acceleration → attitude → body_rate → thrust_and_torque
```

Untuk multicopter position Offboard, external computer biasanya mengirim
`TrajectorySetpoint` dalam **local NED**:

```text
position = [x_North, y_East, z_Down] meter
velocity = [vx_North, vy_East, vz_Down] m/s
yaw      = radian, sekitar z Down
yawspeed = rad/s
```

NaN dipakai untuk field yang tidak dikendalikan/untuk memilih kombinasi
setpoint dan feed-forward yang didukung. Jangan mengirim nol untuk “unused”:
nol berarti target riil nol dan dapat menarik vehicle ke origin.

Pada MAVLink, receiver menerjemahkan pesan seperti
`SET_POSITION_TARGET_LOCAL_NED`, `SET_ATTITUDE_TARGET`, atau actuator-related
setpoint menjadi topic uORB yang sesuai. Pada ROS 2/uXRCE-DDS,
`OffboardControlMode` adalah proof-of-life terpisah dan setpoint topic membawa
nilai target.

### 14.3 Proof-of-life dan kehilangan Offboard

PX4 mensyaratkan signal Offboard kontinu lebih dari sekitar 2 Hz dan hadir
sebelum masuk/arm pada Offboard. Jika rate turun dan melewati
`COM_OF_LOSS_T`, Commander keluar dari Offboard dan memilih aksi
`COM_OBL_RC_ACT` dengan mempertimbangkan ketersediaan RC/mode. Jangan mendesain
tepat 2 Hz; gunakan rate dengan margin, monotonic timestamp, watchdog lokal,
sequence monitoring, dan bounded network jitter.

External planner harus mempunyai state machine:

```text
DISCONNECTED
 → stream proof-of-life + safe initial setpoint
 → verify estimator and PX4 mode eligibility
 → request Offboard
 → verify mode actually accepted
 → arm/takeoff only when authorised
 → stream setpoint continuously
 → on local fault: command safe state or relinquish intentionally
```

Mode request atau arm ACK tidak cukup; verifikasi `vehicle_status.nav_state`,
arming state, estimator validity, dan actual setpoint tracking.

### 14.4 Frame dan origin Offboard

Local NED origin dibuat estimator dan dapat berubah/reset. External system harus
mengonsumsi reference timestamp/reset counter dan menangani estimator resets.
Jika planner bekerja dalam WGS-84, lakukan transformasi ke origin PX4 yang
aktual; jangan menganggap `[0,0,0]` selalu home permanen. Jika memakai ROS frame
ENU/FLU, transformasi wajib eksplisit:

```text
ENU [East, North, Up]  ↔ NED [North, East, Down]
FLU [Forward, Left, Up] ↔ FRD [Forward, Right, Down]
```

Kesalahan ENU/NED paling berbahaya adalah z: target `+10` yang dimaksud 10 m
ke atas dalam ENU berarti 10 m **ke bawah** bila langsung dibaca sebagai NED.

## 15. Interupsi, Pause, Resume, dan Perubahan Mission

Saat Mission dinonaktifkan karena pilot override, Hold, RTL, atau failsafe,
Navigator menyimpan indeks terkait. Source 1.16.1 memiliki logic resume survey
yang dapat kembali ke waypoint sebelumnya agar coverage/citra tidak hilang.
Karena ada item non-posisi, resume index tidak selalu berarti waypoint visual
yang sama di QGC.

Kasus yang harus didefinisikan sistem clone:

- pause di tengah segmen: lanjut dari current, previous, atau nearest segment;
- resume setelah position reset: pertahankan global target atau local path;
- upload mission baru saat armed: ditolak atau atomic replacement;
- `DO_JUMP`: sequence execution bukan lagi monoton;
- item kamera/speed sebelum resume: replay stateful command yang diperlukan;
- takeoff/land item saat resume: jangan dieksekusi seperti waypoint biasa;
- failsafe cleared: kembali ke user-intended mode hanya lewat policy eksplisit.

QGC menampilkan current item berdasarkan telemetry mission index. QGC tidak
menentukan sendiri bahwa waypoint reached; PX4 menerbitkan progress setelah
Navigator mengubah current sequence. Tampilan terlambat tidak mengubah target
yang sedang dijalankan PX4.

## 16. Validasi dan test matrix

### 16.1 Unit/property tests geometri

Uji rectangle, triangle, concave U/C, hampir collinear, vertex tepat pada grid,
polygon sangat kecil, koordinat negatif, latitude tinggi, dekat ±180°, winding
CW/CCW, duplicate vertex, dan self-intersection. Property yang diperiksa:

- semua segmen capture berada dalam polygon, kecuali turnaround yang ditandai;
- tidak ada NaN/Inf atau segmen nol yang tak disengaja;
- perubahan entry hanya mengubah urutan/arah, bukan coverage;
- refly 90° menghasilkan orientasi ortogonal;
- round-trip local↔WGS-84 berada di bawah tolerance;
- output deterministik lintas pengulangan.

### 16.2 Protocol fault injection

Drop, duplicate, delay, dan reorder setiap message mission; putuskan link pada
setiap sequence; kirim ACK salah, stale mission type, request di luar range, dan
reboot saat transaksi. Mission lama harus tetap valid atau mission baru lengkap;
tidak boleh ada campuran.

### 16.3 SITL/HITL dan real-world acceptance

1. SITL tanpa noise untuk kebenaran logika.
2. SITL dengan GNSS noise, delay, dropout, wind, terrain, camera latency.
3. HITL untuk timing, serial/UDP loss, dan driver kamera.
4. Ground test trigger terhadap moving timestamp/encoder reference.
5. Flight kecil di area aman dengan GCP/checkpoint terukur.
6. Bandingkan planned path, global estimate, actual RTK/PPK trajectory, exposure
   centers, file foto, dan residual photogrammetry.

Definisikan acceptance criteria numerik sebelum flight, misalnya P95/P99
cross-track, maksimum vertical error, maksimum trigger jitter, nol gap coverage,
persentase foto valid, dan RMSE check point. “Terlihat rapi di map” bukan test.

### 16.4 Test khusus Hexacopter-X

Urutan bertahap yang aman:

1. Static configuration audit terhadap airframe, parameter, output functions.
2. Actuator test tanpa propeller: nomor, arah, protocol, min/max.
3. Propeller-off closed-loop bench log untuk sensor orientation dan vibration.
4. Tether/stand hanya bila dirancang khusus dan tidak mengubah dinamika berbahaya.
5. Low hover manual/Stabilized, lalu Altitude, Position, Hold.
6. Auto takeoff/land dengan area luas dan abort pilot.
7. Mission rectangle lambat, baru grid dan kecepatan operasional.
8. Failsafe injection satu per satu; jangan menguji termination di udara tanpa
   test plan keselamatan khusus.
9. Motor-loss simulation/SITL lebih dahulu. Jangan mematikan motor real di udara
   hanya karena platform mempunyai enam rotor.

Acceptance meliputi motor saturation, control allocation status, thrust margin,
rate tracking, vibration/IMU clipping, EKF innovations, position tracking,
takeoff ramp, land detection time, dan setiap failsafe transition.

## 17. Telemetri/log yang harus direkam

- `.plan` asli dan compiled mission canonical.
- versi/hash QGC, PX4, board config, seluruh parameter, dan airframe.
- raw GNSS base/rover, correction age/fix status bila RTK/PPK.
- IMU, estimator status, innovations/test ratios, local/global position,
  reset counters, setpoints, attitude, wind estimate.
- mission result/current sequence dan seluruh camera commands/events.
- clock synchronization state dan offset uncertainty.
- daftar file kamera lengkap dengan EXIF/timestamp, checksum, dan sequence.
- terrain dataset identity, geoid model, GCP/checkpoint coordinate + datum.

Tambahan khusus multicopter/hexa:

- `vehicle_status`, `vehicle_control_mode`, `failsafe_flags`, health/arming events;
- `trajectory_setpoint`, local position setpoint, attitude/rates/thrust/torque setpoints;
- actuator motors, actuator effectiveness/allocation status, ESC RPM/current/error;
- `takeoff_status`, `vehicle_land_detected`, battery status, geofence result;
- Offboard control mode dan arrival timestamp jika Offboard digunakan.

## 18. Checklist operasi real-world

### Sebelum upload

- [ ] Camera intrinsics, orientation, focus, shutter, dan min interval tervalidasi.
- [ ] Polygon/datum/unit dan home altitude diperiksa.
- [ ] DEM cukup baru/resolusi cukup; obstacle diberi clearance terpisah.
- [ ] Kecepatan memenuhi interval kamera dan blur limit.
- [ ] Turnaround cukup untuk vehicle dan kondisi angin.
- [ ] GSD/overlap diberi uncertainty margin.

### Sesudah upload

- [ ] Mission di-download kembali dan canonical hash cocok.
- [ ] Tidak ada feasibility warning PX4.
- [ ] Home, global/local position, heading, dan altitude masuk akal.
- [ ] EKF healthy; innovation dan accuracy memenuhi flight gate.
- [ ] RTK fixed/correction age memenuhi batas jika diwajibkan.
- [ ] Camera/storage/time sync diuji dengan capture nyata.

### Sesudah flight

- [ ] Jumlah file cocok dengan event/plan dalam aturan yang didefinisikan.
- [ ] Tidak ada gap atau interval foto abnormal.
- [ ] Trajectory dan exposure center memenuhi SLA.
- [ ] Orthomosaic dinilai dengan independent checkpoints, bukan GCP training saja.
- [ ] Semua log, parameter, calibration, dan hash diarsipkan.

## 19. Jejak source yang diaudit

QGroundControl 5.0.8:

- `src/MissionManager/MissionController.cc`: konversi semua Visual/ComplexItem
  menjadi daftar `MissionItem` datar dan penyerahan daftar ke PlanManager.
- `src/MissionManager/CameraCalc.cc`: rumus GSD, footprint, overlap, spacing,
  dan trigger distance.
- `src/MissionManager/SurveyComplexItem.cc`: proyeksi polygon, grid lines,
  intersection, concave split, entry point, dan refly 90°.
- `src/MissionManager/TransectStyleComplexItem.cc`: terrain, turnaround, dan
  ekspansi ke waypoint/camera mission commands.
- `src/MissionManager/PlanManager.cc`: home-item skip/renumber, state machine
  upload/download, retry/ACK, dan packing `MISSION_ITEM_INT`.
- `src/MissionManager/MissionItem.cc`: representasi parameter/frame item.

PX4 1.16.1:

- `src/modules/mavlink/mavlink_mission.cpp`: penerimaan, validasi, storage, ACK,
  dan konversi mission protocol.
- `src/modules/navigator/mission.cpp`, `mission_base.cpp`, dan
  `mission_block.cpp`: pembacaan item, `brake_for_hold`, validitas next setpoint,
  acceptance/reach/time-inside, hold akhir mission, serta camera command.
- `src/lib/geo/geo.cpp`: jarak, bearing, dan proyeksi geodetik.
- `src/modules/ekf2/EKF/`: propagation, aid-source fusion, innovation gating,
  covariance, reset, terrain, GNSS, vision, dan output predictor.
- `src/drivers/camera_trigger/`: eksekusi trigger perangkat kamera.
- `ROMFS/px4fmu_common/init.d/airframes/6001_hexa_x`: geometry/default
  Generic Hexarotor X.
- `src/modules/mc_pos_control/`, khususnya
  `PositionControl/PositionControl.cpp`: trajectory, closed-loop
  position/velocity/integrator/anti-windup, konversi acceleration ke thrust,
  takeoff ramp, landing-speed limits, dan failsafe setpoint lokal.
- `src/modules/mc_att_control/` dan rate controller: attitude/rate cascade.
- `src/modules/control_allocator/`: effectiveness matrix dan distribusi
  torque/thrust ke enam actuator.
- `src/modules/land_detector/MulticopterLandDetector.cpp`: ground-contact,
  maybe-landed, landed, motion/thrust checks, dan hysteresis.
- `src/modules/commander/failsafe/` serta `HealthAndArmingChecks/`: arbitration,
  mode eligibility, arming, dan failsafe actions.
- `src/modules/mavlink/mavlink_receiver.cpp` dan `docs/en/flight_modes/offboard.md`:
  penerimaan setpoint dan kontrak Offboard.

Referensi spesifikasi resmi:

- [QGC Survey Plan Pattern](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/pattern_survey.html)
- [QGC Plan file format](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html)
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)
- [PX4 1.16 EKF2 tuning and concepts](https://docs.px4.io/v1.16/en/advanced_config/tuning_the_ecl_ekf)
- [PX4 1.16 camera trigger driver](https://docs.px4.io/v1.16/en/modules/modules_driver_camera)

## 20. Kesimpulan desain

Sistem yang “sama seperti QGC + PX4” membutuhkan tiga subsistem berbeda:

1. **Mission compiler deterministik** yang mengubah polygon + camera model +
   terrain menjadi mission items.
2. **Mission transport dan executor fault-tolerant** berbasis MAVLink,
   persistent storage, Navigator, dan controller.
3. **Estimator dan measurement/QC pipeline** yang mengukur uncertainty,
   menolak data buruk, mengikat exposure ke posisi/waktu, serta membuktikan
   hasil memakai RTK/PPK dan checkpoint.

Menyalin grid generator saja tidak menghasilkan presisi PX4/QGC. Presisi akhir
adalah error budget dari datum → projection → estimator → controller → trigger
→ camera → photogrammetry. Setiap komponen harus mempunyai uncertainty,
telemetry, acceptance gate, dan test yang dapat direproduksi.
