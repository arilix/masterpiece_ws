# ROS 2 Jazzy PX4 Waypoint Mission

Penjelasan lengkap mengenai identitas, tujuan, arsitektur, workflow, dan goal
project tersedia di [`PROJECT_OVERVIEW.md`](PROJECT_OVERVIEW.md).

Daftar fitur yang belum diterapkan, risiko, urutan implementasi, test matrix,
dan referensi source tersedia di
[`GAP_IMPLEMENTASI_DAN_ROADMAP.md`](GAP_IMPLEMENTASI_DAN_ROADMAP.md) (lihat
catatan "Update 2026-07-17" di bagian atas dokumen tersebut untuk ringkasan
fitur yang baru ditambahkan). Checklist parameter failsafe native PX4 yang
wajib diverifikasi manual per sesi terbang ada di
[`FAILSAFE_PARAMETER_AUDIT.md`](FAILSAFE_PARAMETER_AUDIT.md).

Workspace ini mengendalikan multicopter PX4 melalui uXRCE-DDS dan `px4_msgs`.
Konfigurasi mission menggunakan waypoint global **LLA WGS-84**, seperti daftar
mission QGC: latitude, longitude, dan altitude. Mission node kemudian memakai
referensi global EKF (`ref_lat/ref_lon/ref_alt`) untuk melakukan proyeksi
Azimuthal Equidistant yang sama dengan `PX4::MapProjection`, dan baru mengirim
hasil local NED ke PX4 melalui Offboard `TrajectorySetpoint`.

## Kontrak keselamatan

Kode ini harus diuji di SITL/HITL sebelum propeller terpasang. Supervisor adalah
satu-satunya publisher `OffboardControlMode` dan `TrajectorySetpoint`. Bila
mission node berhenti, target timeout, atau local position invalid, supervisor
mengirim `VEHICLE_CMD_NAV_LAND` berulang—tidak mengirim RTL. Pengiriman command
sekarang diikuti **state machine konfirmasi**: ACK diterima → `nav_state`
menjadi `AUTO_LAND` → `VehicleLandDetected.landed=true` → disarm (hanya setelah
landed, bukan saat masih terbang). Jika Land berulang kali ditolak PX4,
supervisor tetap retry (tidak otomatis memilih aksi lain) tetapi mencatat error
eskalasi eksplisit di log setelah `land_command_warn_after_attempts`.

Rotasi antar-segmen sekarang menunggu **yaw-rate aktual** dari
`/fmu/out/vehicle_angular_velocity` (bukan hanya heading setpoint internal)
sebelum dianggap selesai, plus watchdog timeout/progress terpisah untuk rotasi
dan takeoff (`YAW_STUCK`/`TAKEOFF_STUCK` di log). **Topic ini tidak dibridge
oleh firmware PX4 1.16.1 default** — sudah diaktifkan di
`PX4-Autopilot-1.16.1/src/modules/uxrce_dds_client/dds_topics.yaml` di
workspace ini, tetapi firmware wajib di-rebuild dan di-reflash sebelum data
mengalir. Selama belum, `require_yaw_rate_feedback=true` (default) membuat
setiap alignment timeout dan mission abort ke Auto Land — ini kegagalan aman
yang disengaja, bukan bug.

Supervisor juga memonitor `/fmu/out/battery_status`. Tiga sampel berturut-turut
dengan `remaining <= 15%` memicu Auto Land. Battery yang tidak connected,
`remaining=-1`/NaN, atau topic timeout ketika vehicle sudah Offboard/armed juga
memicu Auto Land jika `require_battery_status=true`; sebelum Offboard kondisi
tersebut memblokir perpindahan mode. Atur threshold, timeout, dan debounce di
`config/offboard.yaml`. Nilai `remaining` adalah estimasi PX4, sehingga kalibrasi
power module dan kapasitas battery tetap wajib benar.

Jika **supervisor itu sendiri** mati, keputusan pindah mode dilakukan failsafe
native PX4. Set parameter PX4 Offboard-loss ke **Land**, bukan RTL, dan validasi
nama/nilai enum terhadap metadata firmware 1.16.1 yang terpasang. Periksa minimal
`COM_OF_LOSS_T` dan `COM_OBL_RC_ACT`; jangan menyalin angka enum antarversi.
Geofence, battery, estimator, RC, dan data-link failsafe PX4 tetap dapat mengambil
alih dan harus dikonfigurasi terpisah.

`auto_arm` default `false`. Arm manual setelah preflight check, atau ubah menjadi
`true` hanya dalam lingkungan pengujian yang terkendali.

## Dependency dan build

ROS 2 Jazzy sudah harus terpasang. Workspace ini menyertakan official
`PX4/px4_msgs` branch `release/1.16` pada commit pendek `392e831`, agar definisi
message cocok dengan PX4 1.16.x. Topic DDS PX4 harus tersedia melalui Micro
XRCE-DDS Agent.

```bash
cd /home/udinkicau/masterpiece_ws/ros2_px4_waypoint_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Tiga perintah utama

Jalankan pada terminal terpisah dan selalu source workspace.

1. Lihat coordinate estimator tanpa mengubah mission:

```bash
ros2 launch px4_waypoint_mission 01_position.launch.xml
```

Untuk merekam posisi vehicle langsung sebagai latitude/longitude waypoint
tertentu, tambahkan argument `wp:=1` sampai `wp:=10`:

```bash
ros2 launch px4_waypoint_mission 01_position.launch.xml wp:=1
ros2 launch px4_waypoint_mission 01_position.launch.xml wp:=2
```

Recorder menunggu position global valid, lalu mengumpulkan 20 sampel yang lolos
validity, non-dead-reckoning, `eph`, dan `epv`. Latitude/longitude yang disimpan
adalah rata-rata sampel; seluruh batch ditolak bila penyebarannya melampaui
`max_sample_spread_m`. Altitude, altitude frame, yaw policy, dan hold time tetap
dipertahankan. Penulisan memakai file lock (`flock` exclusive non-blocking,
menolak jika recorder lain sedang berjalan), file sementara + `fsync` +
rename atomik, dan riwayat revisi bertimestamp (`waypoints.yaml.bak.<UTC>`)
selain `waypoints.yaml.bak` terbaru. Hash SHA-256 file sebelum dan sesudah
perubahan dicatat di log untuk audit. Sintaks ROS 2 launch adalah `wp:=1`,
bukan `wp=1`.

2. Mulai pre-stream, tangkap posisi hold, lalu minta mode Offboard:

```bash
ros2 launch px4_waypoint_mission 02_offboard.launch.xml
```

3. Jalankan mission 10 waypoint:

```bash
ros2 launch px4_waypoint_mission 03_mission.launch.xml
```

Ubah seluruh titik dan hold time di
`src/px4_waypoint_mission/config/waypoints.yaml`. Nilai bawaan adalah contoh
lokasi PX4 SITL Zurich dan **tidak boleh dipakai pada real vehicle di lokasi
lain**. Mission wajib tepat 10 titik. Format setiap titik:

```text
"latitude_deg,longitude_deg,altitude_m,alt_frame,yaw_rad,hold_seconds[,FLY_THROUGH]"
```

`alt_frame=AMSL` berarti altitude absolut terhadap datum altitude estimator.
`alt_frame=REL_HOME` berarti altitude di atas `HomePosition.alt`, lalu diubah
menjadi AMSL sebelum dihitung menjadi local Down:

```text
altitude_amsl = home_alt_amsl + relative_altitude
local_down    = -(altitude_amsl - ekf_ref_alt_amsl)
```

`alt_frame=TERRAIN` diparsing tapi **selalu ditolak** oleh feasibility check --
tidak ada sumber DEM/datum tervalidasi di workspace ini, jadi mission jenis ini
gagal secara eksplisit (fail-closed) alih-alih diam-diam salah. Mission tidak
menerbitkan heartbeat target bila referensi global EKF atau home altitude yang
dibutuhkan belum valid; supervisor kemudian menjalankan Auto Land jika vehicle
sudah menjalankan mission.

Field ke-7 opsional `FLY_THROUGH` (default tidak ada = perilaku lama, stop-
rotate-go) mengizinkan sebuah waypoint dilewati tanpa berhenti penuh, hanya
jika `hold_seconds=0` dan bukan waypoint terakhir. Saat aktif, node menghitung
"pass radius" (`fly_through_pass_radius_m`) dan toleransi yaw yang lebih
longgar (`fly_through_yaw_tolerance_rad`) terhadap bearing segmen berikutnya,
lalu mempublish velocity feed-forward terbatas (`fly_through_speed_mps`) lewat
topic `/mission/target_feedforward`. Ini **opt-in dan belum divalidasi flight
test** — bukan pengganti trajectory generator jerk-limited penuh seperti
`FlightTaskAuto` PX4 (lihat GAP §5.1).

Custom geofence opsional (`geofence_inclusion`/`geofence_exclusions` di
`waypoints.yaml`, format `"lat,lon;lat,lon;lat,lon"` per polygon, minimal 3
titik) diperiksa saat feasibility check di bidang lokal yang sama dengan
waypoint: setiap titik dan segmen antar-waypoint diuji point-in-polygon dan
jarak-ke-boundary (`geofence_margin_m`). Ini melengkapi, bukan menggantikan,
`failsafe_flags.geofence_breached` native PX4 yang tetap dipantau saat
runtime.

Sebelum takeoff, seluruh mission harus lolos feasibility gate:

- tepat 10 waypoint dengan LLA dan frame valid;
- altitude berada di antara batas minimum/maksimum di atas home;
- jarak horizontal dari home dan panjang segmen berada di bawah batas;
- local/global position fresh, valid, dan bukan dead-reckoning;
- `eph/epv` berada di bawah batas;
- PX4 tidak melaporkan position, home, geofence, accuracy, atau navigator failure.

Selama mission, node memonitor reset counter local/global, `ref_timestamp`, dan
`HomePosition.update_count`. Reset estimator memicu proyeksi ulang target LLA
dan alignment ulang. Perubahan home saat mission aktif atau health gate yang
gagal menerbitkan `/mission/abort`; supervisor kemudian mengirim Auto Land,
bukan RTL.

Nilai yaw `nan` memilih heading otomatis mengikuti bearing segmen. Yaw numerik
memaksa heading tersebut pada waypoint. Mission memakai state machine:

```text
TAKEOFF_VERTICAL → ALIGN_HEADING → NAVIGATE → HOLD → waypoint berikutnya
```

Pada `TAKEOFF_VERTICAL`, target horizontal ditangkap dari posisi aktual dan yaw
dikunci ke heading saat Offboard mulai; hanya target altitude yang berubah.
Setelah altitude tercapai dan vehicle berhenti, node menghitung bearing dengan
`atan2(delta_east, delta_north)`, menahan XYZ sambil melakukan rotasi, lalu baru
melakukan translasi. Perubahan yaw setpoint dibatasi `max_yaw_rate_rad_s` dan
translasi dimulai setelah error heading masuk `yaw_acceptance_rad`.
Timer hold baru dimulai setelah error horizontal dan vertikal masuk acceptance
serta kecepatan 3D berada di bawah `stopped_speed_mps`. Ini sengaja lebih ketat
daripada sekadar masuk acceptance sphere.

Setelah WP10 selesai, node tetap menerbitkan WP10 agar vehicle hold dan watchdog
tetap sehat. Hentikan mission node ketika masih terbang akan memicu Auto Land
setelah `mission_timeout_seconds`.

## Urutan uji

1. PX4 SITL + XRCE Agent, pastikan topic `/fmu/out/...` muncul.
2. Jalankan position reporter dan pastikan `xy_valid/z_valid=true`.
3. Jalankan supervisor; cek ACK Offboard dan jangan arm jika belum aman.
4. Jalankan mission; cek waypoint index dan hold.
5. Di SITL, hentikan mission node dan pastikan PX4 masuk `AUTO_LAND`, bukan RTL.
6. Hentikan supervisor dan buktikan native Offboard-loss PX4 juga memilih Land.
