# Gap Implementasi dan Roadmap Mission Waypoint ROS 2–PX4

> **Update 2026-07-17:** Fase 1 (P0), Fase 2 (P1), dan sebagian besar Fase 3
> (P2) pada §12 sudah diberi implementasi kode dan **build + smoke-test topic
> lulus** (lihat detail per bagian di bawah, ditandai "Update 2026-07-17").
> Yang **BELUM** dilakukan sama sekali: SITL fault-injection otomatis, HITL,
> dan flight test bertahap (§13 test matrix) -- semua item di bawah karena itu
> tetap berstatus **"implemented but not validated"** sesuai §14, bukan
> "selesai". Dua gap tetap TIDAK diimplementasikan secara jujur karena
> keterbatasan lingkungan, bukan diabaikan diam-diam:
> - **Battery multi-instance (§9.1):** `dds_topics.yaml` PX4 1.16.1 hanya
>   membridge SATU instance `battery_status`; aggregasi multi-battery asli
>   butuh perubahan bridge + firmware rebuild yang lebih luas. Yang
>   diimplementasikan hanya penggunaan field `faults/warning/is_required/
>   time_remaining_s` pada instance tunggal yang tersedia.
> - **Terrain-relative altitude (§8.2):** tidak ada sumber DEM/datum
>   tervalidasi di workspace ini. `alt_frame=TERRAIN` sekarang diparsing tapi
>   **selalu ditolak** oleh feasibility check (fail-closed), bukan dipalsukan
>   seolah didukung.
>
> Dua topic DDS yang dibutuhkan (`vehicle_angular_velocity`,
> `control_allocator_status`) sudah ditambahkan/diaktifkan di
> `PX4-Autopilot-1.16.1/src/modules/uxrce_dds_client/dds_topics.yaml`, tetapi
> **firmware harus di-rebuild dan di-reflash** sebelum data itu benar-benar
> mengalir -- sampai saat itu, gate yang bergantung padanya gagal secara
> fail-closed (`require_yaw_rate_feedback=true` membuat alignment timeout,
> BUKAN silent bypass) atau tetap nonaktif by design
> (`require_control_allocator_status=false`).

## 1. Tujuan dan batas dokumen

Dokumen ini menjelaskan secara jujur bagian yang **belum diterapkan atau belum
divalidasi** pada workspace `ros2_px4_waypoint_ws`, cara menerapkannya, urutan
prioritas, dan referensi source QGroundControl 5.0.8 serta PX4 1.16.1.

Workspace saat ini adalah **Offboard controller berbasis LLA**:

```text
waypoints.yaml (LLA)
  → proyeksi LLA ke local NED
  → /mission/target_ned
  → offboard_supervisor
  → /fmu/in/trajectory_setpoint
  → PX4 multicopter position/attitude/rate controller
```

Workspace ini bukan pengganti identik untuk QGC Mission Protocol + PX4
Navigator. Klaim “100% selesai” baru boleh dibuat terhadap requirement dan test
matrix yang terukur, bukan terhadap seluruh QGC/PX4.

## 2. Yang sudah diterapkan

| Area | Status saat ini |
|---|---|
| Waypoint | Tepat 10 titik LLA WGS-84, `AMSL` atau `REL_HOME` |
| Proyeksi | Azimuthal Equidistant mengikuti rumus `PX4::MapProjection` |
| Recorder | Pemilihan `wp:=1..10`, multi-sample, `eph/epv`, spread, dead-reckoning gate |
| Takeoff | XY dan yaw awal dikunci, altitude berubah vertikal |
| Rotasi | Bearing `atan2(E,N)`, shortest-angle wrap, rotate-before-translate |
| Yaw setpoint | Rate-limited dasar, numerical yaw atau `nan` untuk bearing otomatis |
| Reach/hold | XY/Z acceptance + 3D stopped speed + dwell timer |
| Estimator | Validity, freshness, uncertainty, reset counter, reference/home monitoring |
| Feasibility | Altitude, distance dari home, dan panjang segmen |
| Watchdog | Mission timeout, invalid position, explicit mission abort |
| Battery | ≤15% + warning/faults/time-reserve dengan debounce menghasilkan Auto Land (update 2026-07-17) |
| Failsafe action | Supervisor mengirim `VEHICLE_CMD_NAV_LAND`, bukan RTL, dengan konfirmasi ACK/nav-state/landed/disarm (update 2026-07-17) |
| Yaw-rate gate | `VehicleAngularVelocity` fail-closed gate + accel/jerk-shaped yaw-rate + `yawspeed` feed-forward (update 2026-07-17) |
| Watchdog rotasi/takeoff | Timeout absolut + minimum-progress window untuk alignment dan takeoff (update 2026-07-17) |
| Saturation monitor | `ControlAllocatorStatus` yaw-authority degraded-hold + Auto Land escalation, default nonaktif sampai firmware di-reflash (update 2026-07-17) |
| Reset policy | Per-jenis reset dilog terpisah + reset-storm guard (update 2026-07-17) |
| Geofence | Custom inclusion/exclusion polygon + margin di local plane (update 2026-07-17) |
| Mission persistence | Schema version, SHA-256 hash, file lock, fsync, revision history (update 2026-07-17) |

Build sukses berarti kode dapat dikompilasi. **Smoke-test topik (§2 update)
membuktikan node start, parsing config, dan file persistence bekerja seperti
dirancang** -- itu tetap belum membuktikan dinamika vehicle, estimator, DDS
loss, atau motor saturation aman pada vehicle/SITL nyata.

## 3. Ringkasan gap dan prioritas

| Prioritas | Gap | Risiko bila belum ada | Status kode (2026-07-17) |
|---:|---|---|---|
| P0 | Yaw-rate feedback, timeout, progress watchdog | vehicle dapat tertahan selamanya saat alignment | Diimplementasikan, fail-closed; butuh firmware rebuild untuk topic aktif; belum SITL/flight |
| P0 | Konfirmasi mode Land dan retry state machine | command Land dapat ditolak tanpa eskalasi terdeteksi | Diimplementasikan (LandState machine); belum SITL/flight |
| P0 | Native PX4 failsafe parameter audit | supervisor mati dapat menghasilkan aksi selain Land | Checklist tertulis di `FAILSAFE_PARAMETER_AUDIT.md` (nilai dari source 1.16.1); verifikasi manual di vehicle TETAP wajib per sesi |
| P0 | SITL fault-injection otomatis | cabang gagal hanya diyakini dari code review | **Belum dilakukan** -- di luar scope perubahan kode kali ini |
| P1 | Control-allocation saturation monitoring | rotasi/hold dapat kehilangan authority tanpa terdeteksi | Diimplementasikan, default nonaktif (topic belum dibridge default); belum SITL/flight |
| P1 | Yaw acceleration/jerk shaping | perubahan yaw-rate belum benar-benar halus | Diimplementasikan (accel-limited, measured dt); belum SITL/flight |
| P1 | Battery multi-instance dan remaining-time reserve | battery utama bukan selalu instance yang diterima | Time-reserve+faults/warning pada instance tunggal diimplementasikan; **multi-instance TIDAK** (lihat catatan update di atas) |
| P1 | Custom geofence path feasibility | titik/segmen dapat melewati exclusion zone | Diimplementasikan (point-in-polygon + segment intersection + margin); belum SITL/flight |
| P1 | LLA datum dan home-change policy lengkap | altitude dapat bergeser secara semantik | Tidak berubah pada iterasi ini |
| P2 | Previous/current/next corner trajectory | lintasan hanya stop–rotate–go | Fly-through opt-in per-waypoint (bounded, bukan full jerk-trajectory planner) diimplementasikan; default tetap stop-rotate-go |
| P2 | Terrain-relative altitude | tidak dapat mengikuti permukaan secara tervalidasi | **Tetap tidak didukung** -- sekarang fail-closed (parse+reject) alih-alih diam-diam salah |
| P2 | Mission persistence/version/CRC | perubahan file tidak punya kontrak transaksi lengkap | Diimplementasikan (schema_version, SHA-256, flock, fsync, revision history); diverifikasi dengan smoke-test |
| Arsitektural | MAVLink Mission Protocol/dataman | tidak identik dengan mission onboard QGC/PX4 | Tidak berubah (Opsi A/Offboard dipertahankan sesuai keputusan) |

## 4. Gap rotasi dan yaw

### 4.1 Yaw-rate aktual belum diperiksa

> **Update 2026-07-17:** Diimplementasikan di `WaypointMission::yaw_rate_settled()`
> (`waypoint_mission.cpp`) -- subscribe `VehicleAngularVelocity`, gate
> `abs(xyz[2]) <= yaw_rate_stopped_rad_s` + freshness, dikombinasikan dengan
> `!yaw_rate_saturated_` di kondisi selesai `kAlignHeading`. **Fail-closed**:
> topic ini tidak dibridge default di firmware 1.16.1; sampai
> `dds_topics.yaml` (sudah diaktifkan di workspace ini) di-rebuild+reflash,
> `require_yaw_rate_feedback=true` membuat alignment selalu timeout ke
> `check_yaw_alignment_watchdog()` lalu Auto Land -- bukan silent bypass.

Saat ini alignment melihat heading dan yaw setpoint internal. Program belum
memakai gyro yaw-rate aktual. Vehicle dapat mempunyai heading error kecil sesaat
tetapi masih berputar cepat, atau heading tidak berubah karena yaw authority
hilang.

Implementasi yang diperlukan:

1. Subscribe `/fmu/out/vehicle_angular_velocity`.
2. Ambil komponen body-z `xyz[2]` dan timestamp/freshness.
3. Definisikan `yaw_rate_stopped_rad_s`, misalnya hasil tuning SITL/HITL.
4. Anggap alignment selesai hanya jika seluruh kondisi bertahan selama debounce:

```text
abs(wrap_pi(yaw_target - heading)) <= yaw_acceptance
AND abs(actual_yaw_rate) <= yaw_rate_stopped
AND command_yaw sudah tidak rate-saturated
```

5. Log heading error, yaw-rate setpoint, yaw-rate aktual, dan durasi alignment.

Referensi:

- `PX4-Autopilot-1.16.1/msg/versioned/VehicleAngularVelocity.msg`
- `PX4-Autopilot-1.16.1/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp`
  fungsi `_limitYawRate()`.
- `PX4-Autopilot-1.16.1/src/modules/mc_att_control/` untuk attitude-to-rate.
- PX4 rate controller untuk gyro error menjadi torque setpoint.

### 4.2 Belum ada timeout dan progress watchdog rotasi

> **Update 2026-07-17:** Diimplementasikan sebagai
> `WaypointMission::check_yaw_alignment_watchdog()`: timeout absolut
> `yaw_alignment_timeout_s` + minimum-progress per window
> (`yaw_stuck_window_s`/`yaw_stuck_min_progress_rad`), memicu `abort_mission`
> ("YAW_STUCK") -> `/mission/abort` -> supervisor Auto Land. Belum diuji lewat
> SITL fault-injection (compass freeze/yaw torque limit) sungguhan.

Jika compass salah, motor yaw saturated, atau heading estimate macet, state
`ALIGN_HEADING` dapat tidak pernah selesai. Heartbeat mission tetap sehat sehingga
supervisor tidak mengetahui bahwa mission macet.

Implementasi:

1. Catat `alignment_started_at` dan initial absolute yaw error.
2. Tambahkan `yaw_alignment_timeout_s`.
3. Setiap window, hitung progress:

```text
progress = previous_abs_error - current_abs_error
```

4. Jika progress di bawah minimum selama beberapa window, deklarasikan
   `YAW_STUCK`.
5. Publish `/mission/abort=true`; supervisor mengirim Auto Land.
6. Bedakan timeout saat di tanah dan di udara agar tidak mengirim command yang
   tidak diperlukan.

Acceptance test:

- bekukan heading topic pada SITL;
- batasi yaw torque secara sengaja;
- injeksikan compass/yaw reset;
- pastikan Auto Land terjadi dalam timeout yang ditentukan dan tidak RTL.

Referensi PX4:

- `FlightTaskAuto.cpp:167-169`: velocity dipaksa nol ketika menunggu yaw-first.
- `FlightTaskAuto.cpp:303-335`: alignment, rate limit, yawspeed filter.
- `src/modules/navigator/mission_block.cpp`: yaw reach dan yaw timeout.
- parameter `MIS_YAW_ERR` di `src/modules/navigator/mission_params.c`.

### 4.3 Yaw acceleration/jerk belum dibatasi

> **Update 2026-07-17:** Diimplementasikan di `WaypointMission::limited_yaw()`:
> desired yaw-rate dari error dibatasi `max_yaw_rate_rad_s`, perubahan rate
> dibatasi `max_yaw_accel_rad_s2 * dt` dengan `dt` terukur (bukan selalu
> `1/publish_rate_hz`), lalu diintegrasikan jadi `commanded_yaw_`.
> `commanded_yaw_rate_rad_s_` dipublish sebagai `yawspeed` feed-forward lewat
> `/mission/target_feedforward` dan diteruskan supervisor ke
> `TrajectorySetpoint.yawspeed` (item 6 di bawah). Yaw jerk (turunan kedua)
> belum dibatasi terpisah -- hanya accel-limited, bukan full jerk-limited.

Workspace membatasi perubahan yaw angle per cycle sehingga secara efektif
membatasi yaw-rate, tetapi transisi yaw-rate dapat tetap tajam. Untuk vehicle
besar, payload sensitif, atau authority kecil, diperlukan acceleration shaping.

Langkah implementasi:

1. Simpan `yaw`, `yaw_rate`, dan `yaw_acceleration` command state.
2. Hitung desired yaw-rate dari yaw error.
3. Batasi desired yaw-rate.
4. Batasi perubahan yaw-rate per `dt` menggunakan `max_yaw_accel_rad_s2`.
5. Opsional batasi perubahan yaw acceleration menggunakan yaw jerk.
6. Isi `TrajectorySetpoint.yawspeed` secara konsisten, bukan hanya quaternion yaw
   melalui topic internal.
7. Gunakan monotonic measured `dt`, bukan selalu `1/publish_rate_hz`.

### 4.4 Saturation dan authority Hexa-X belum menjadi feedback mission

> **Update 2026-07-17:** Diimplementasikan di
> `OffboardSupervisor::update_saturation_monitor()`: subscribe
> `ControlAllocatorStatus`, debounce (`yaw_saturation_debounce_seconds`) pada
> `!torque_setpoint_achieved && abs(unallocated_torque[2]) > threshold`, lalu
> membekukan translasi (frozen position hold) + nol-kan velocity/yawspeed
> feed-forward, dan Auto Land jika tidak pulih dalam
> `yaw_saturation_land_timeout_seconds`. **Default nonaktif**
> (`require_control_allocator_status=false`) karena topic ini tidak dibridge
> firmware 1.16.1 default -- entri sudah ditambahkan ke `dds_topics.yaml` di
> workspace ini, tetapi butuh rebuild+reflash sebelum diaktifkan. Threshold
> `yaw_unallocated_torque_threshold` adalah nilai awal, WAJIB dituning per
> vehicle sebelum dipakai nyata.

Control Allocator mengetahui apakah torque yaw berhasil direalisasikan, tetapi
workspace belum membacanya.

Langkah implementasi:

1. Expose/subscribe `control_allocator_status` melalui DDS configuration PX4.
2. Pantau:
   - `torque_setpoint_achieved`;
   - `unallocated_torque[2]` untuk yaw;
   - `actuator_saturation[0..5]`;
   - handled motor-failure mask.
3. Debounce saturation; jangan abort karena satu transient sample.
4. Ketika yaw saturation menetap:
   - hentikan translasi;
   - kurangi yaw-rate command;
   - bila tidak pulih, Auto Land.
5. Korelasikan dengan altitude error dan thrust headroom.

Referensi:

- `PX4-Autopilot-1.16.1/msg/ControlAllocatorStatus.msg`
- `src/modules/control_allocator/ControlAllocator.cpp:597-645`
- `ROMFS/px4fmu_common/init.d/airframes/6001_hexa_x`

### 4.5 Weather-vane, ROI, dan external yaw ownership belum ada

PX4 Auto mempunyai weather-vane dan ROI/mount interactions. Workspace sekarang
selalu menjadi pemilik yaw ketika mengirim quaternion yaw. Jika nantinya ROI,
gimbal-coupled yaw, atau weather-vane digunakan, ownership harus eksplisit.

Pilihan desain:

- `MISSION_YAW`: workspace menentukan yaw;
- `PX4_WEATHERVANE`: yaw dikirim NaN dan PX4 menentukan yaw-rate;
- `ROI_YAW`: bearing menuju ROI;
- `FIXED_YAW`: heading eksplisit.

Jangan mencampur dua owner dalam satu cycle. Referensi:

- `FlightTaskAuto.cpp` bagian `_weathervane.update()`.
- `MissionBase::heading_sp_update()` pada `mission_base.cpp`.
- QGC ROI command di `MissionController` dan metadata MAV_CMD.

## 5. Gap trajectory antar-waypoint

### 5.1 Workspace masih stop–rotate–go

> **Update 2026-07-17:** Fly-through opt-in per-waypoint diimplementasikan
> (field ke-7 `FLY_THROUGH`, hanya valid jika `hold_s=0` dan bukan waypoint
> terakhir; `WaypointMission::pass_condition_met()` + `publish_feedforward()`).
> Ini SENGAJA bukan corner-smoothing jerk-limited penuh seperti
> `FlightTaskAuto::_evaluateTriplets()` -- hanya pass-radius + velocity
> feed-forward terbatas. Default tidak ada waypoint contoh yang memakainya;
> opt-in, belum divalidasi flight test.

Semua waypoint mempunyai hold, sehingga berhenti merupakan perilaku yang masuk
akal. Namun ini tidak sama dengan corner smoothing PX4 ketika next waypoint
valid dan tidak ada hold.

PX4 menggunakan previous/current/next:

```text
Navigator position_setpoint_triplet
  → FlightTaskAuto::_evaluateTriplets()
  → internal previous/current/next
  → jerk/acceleration/velocity constrained trajectory
```

Implementasi opsional:

1. Tambah field per waypoint: `fly_through`, acceptance, cruise speed.
2. Untuk `hold_s > 0`, next dianggap invalid dan vehicle wajib berhenti.
3. Untuk `fly_through=true`, bangun trajectory segment dengan informasi next.
4. Gunakan library trajectory yang mempunyai constraint velocity, acceleration,
   jerk, serta corner radius; jangan mengirim step posisi mentah.
5. Pisahkan position-reached dari segment-passed logic.
6. Ukur corner cut dan cross-track P95/P99.

Referensi:

- `src/modules/navigator/mission.cpp`, `Mission::setActiveMissionItems()` dan
  `brake_for_hold`.
- `src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp`, terutama
  `_evaluateTriplets()` dan update trajectory.
- `src/modules/mc_pos_control/` untuk trajectory/position constraints.

### 5.2 Feed-forward velocity/acceleration belum digunakan

Saat ini supervisor mengirim position finite dan velocity/acceleration NaN.
PX4 tetap mengontrol posisi, tetapi planner eksternal belum memberikan lintasan
kinematis yang eksplisit.

Untuk trajectory eksternal penuh:

1. Bangun time-parameterized path.
2. Pastikan position, velocity, acceleration konsisten secara kinematis.
3. Isi feed-forward hanya jika planner telah tervalidasi.
4. Terapkan continuity check pada setiap cycle.
5. Jika planner invalid, jangan mengganti NaN dengan nol secara sembarang;
   publish abort dan Land.

## 6. Gap takeoff dan landing

### 6.1 Takeoff belum memakai TakeoffState/PX4 takeoff ramp secara eksplisit

> **Update 2026-07-17:** Sebagian diimplementasikan:
> `WaypointMission::check_takeoff_watchdog()` menambah timeout absolut
> (`takeoff_timeout_s`) + minimum climb-progress per window
> (`takeoff_stuck_window_s`/`takeoff_stuck_min_progress_m`), dan transisi
> keluar `kTakeoffVertical` sekarang mensyaratkan `VehicleLandDetected.landed
> == false` (bila topic fresh) selain position/speed. **Belum**: state
> spool-up/ramp/ground-contact-release granular seperti
> `src/modules/navigator/takeoff.cpp` -- gate yang ditambah baru boolean
> landed, bukan replikasi penuh `TakeoffState`.

Workspace menaikkan position target Z sambil mengunci XY/yaw. PX4 position
controller tetap melakukan tracking, tetapi workspace tidak mereplikasi semua
state `MAV_CMD_NAV_TAKEOFF` atau `TakeoffState` Navigator.

Yang perlu ditambah:

1. Subscribe `vehicle_land_detected` dan `takeoff_status`.
2. Jangan menganggap takeoff selesai hanya dari position/velocity.
3. Bedakan spool-up, ramp, ground-contact release, airborne, dan altitude reached.
4. Tambah takeoff timeout serta minimum climb progress.
5. Abort bila land detector tetap landed tetapi thrust/time melewati batas.
6. Pastikan yaw lock diuji saat heading awal bukan nol dan setelah yaw reset.

Referensi:

- `src/modules/navigator/mission.cpp` fungsi `handleTakeoff()`.
- `src/modules/navigator/takeoff.cpp`.
- `src/modules/mc_pos_control/` takeoff handling/ramp.
- `src/modules/land_detector/MulticopterLandDetector.cpp`.

### 6.2 Auto Land belum mempunyai completion state yang kuat

> **Update 2026-07-17:** Diimplementasikan sebagai `OffboardSupervisor::LandState`
> (`kNone -> kRequested -> kAckConfirmed -> kNavStateConfirmed -> kLanded ->
> kDisarmed`) di `update_land_state_machine()`: mencocokkan
> `VehicleCommandAck.result`, `VehicleStatus.nav_state == AUTO_LAND`,
> `VehicleLandDetected.landed`, lalu mengirim disarm setelah
> `disarm_after_land_delay_seconds` sejak landed (bukan saat masih terbang).
> ACK rejected berulang dicatat sebagai error eskalasi eksplisit
> (`land_command_warn_after_attempts`) tanpa memilih aksi lain otomatis, sesuai
> item 7 di bawah.

Supervisor mengulang `VEHICLE_CMD_NAV_LAND`, tetapi belum memverifikasi seluruh
urutan ACK → `NAVIGATION_STATE_AUTO_LAND` → landed → disarmed.

Implementasi P0:

1. Saat meminta Land, masuk state `LAND_REQUESTED`.
2. Cocokkan `VehicleCommandAck.command` dan `result`.
3. Jika rejected, log reason dan retry dengan bounded count.
4. Verifikasi `VehicleStatus.nav_state == AUTO_LAND`.
5. Subscribe `VehicleLandDetected`; tunggu `landed=true`.
6. Verifikasi auto-disarm atau jalankan kebijakan disarm yang eksplisit setelah
   landed, bukan saat masih terbang.
7. Jika ACK terus ditolak, gunakan escalation policy yang telah diuji—jangan
   otomatis memilih termination.

## 7. Gap estimator dan koordinat

### 7.1 Reset handling perlu uji fisik dan kebijakan per jenis reset

> **Update 2026-07-17:** `WaypointMission::check_estimator_resets()` sekarang
> membedakan dan mencatat `xy_local`, `z_local`, `heading`, `global_latlon`,
> `global_alt`, `reference_changed` sebagai flag terpisah (log eksplisit per
> jenis + `delta_xy/delta_z/delta_heading`), dan menambah **reset-storm guard**
> (`reset_storm_window_s`/`max_resets_per_window`) yang meng-abort mission bila
> reset terlalu sering. Tindakan korektif tetap sama untuk semua jenis reset
> (reproject + re-align) -- differensiasi ada di deteksi/logging/guard, bukan
> di cabang tindakan terpisah per jenis. **Belum diuji fisik** (kombinasi
> reset asli di SITL/HITL).

Workspace memonitor counter dan melakukan reproject/alignment, tetapi belum
dibuktikan untuk kombinasi:

- local XY reset tanpa global reset;
- global lat/lon reset;
- altitude reset;
- heading reset;
- perubahan local reference;
- perubahan home.

Langkah:

1. Simpan snapshot sebelum reset: target LLA, local target, estimate, delta reset.
2. Terapkan policy berbeda untuk setiap reset.
3. Untuk LLA target, hitung ulang local target dari reference terbaru.
4. Untuk yaw reset, pastikan yaw target tetap bermakna terhadap North dan tidak
   menghasilkan step salah.
5. Freeze advancement/hold timer selama stabilization window.
6. Abort jika reset berulang atau uncertainty melewati batas.

Referensi:

- `VehicleLocalPosition.msg`: `delta_xy`, `delta_z`, `delta_heading`, counters.
- `VehicleGlobalPosition.msg`: global reset counters.
- `src/modules/ekf2/EKF/` reset dan output predictor.
- `FlightTaskAuto` global-to-local projection dan reset handling downstream.

### 7.2 Datum altitude belum dibuktikan end-to-end

`AMSL`, ellipsoid height, barometric reference, geoid, home altitude, dan DEM
tidak boleh dianggap sama.

Langkah:

1. Dokumentasikan sumber `HomePosition.alt` dan `ref_alt` pada firmware/build.
2. Rekam `alt`, `alt_ellipsoid`, geoid separation bila tersedia.
3. Bandingkan QGC altitude frame dengan hasil NED target.
4. Buat test vector AMSL dan REL_HOME dengan nilai yang diketahui.
5. Tolak mission jika datum/config tidak diketahui.

## 8. Gap geofence dan terrain

### 8.1 Custom geofence belum diperiksa sepanjang segmen

> **Update 2026-07-17:** Diimplementasikan di `geofence.h` +
> `WaypointMission::check_geofence()`: inclusion/exclusion polygon
> (`geofence_inclusion`/`geofence_exclusions` di `waypoints.yaml`) diproyeksikan
> ke bidang lokal yang sama dengan waypoint (Azimuthal Equidistant terhadap
> `ref_lat/ref_lon` EKF saat ini), lalu diuji point-in-polygon +
> point-to-boundary distance (margin) untuk setiap waypoint DAN setiap
> segmen (di-sample tiap `geofence_sample_resolution_m`) plus uji
> segment-vs-boundary intersection. Dijalankan sekali saat feasibility check
> (pra-arm), bukan pengganti `failsafe_flags.geofence_breached` runtime yang
> tetap dipantau terpisah.

Workspace baru memeriksa jarak maksimum dari home dan flag breach aktual. Dua
waypoint dapat berada di area legal tetapi garis di antaranya memotong exclusion
polygon.

Implementasi:

1. Ambil geofence inclusion/exclusion polygon dari satu sumber canonical.
2. Proyeksikan fence dan waypoint ke bidang lokal yang sama.
3. Uji setiap waypoint inside inclusion dan outside exclusion.
4. Uji intersection setiap segment dengan seluruh boundary.
5. Tambahkan margin berdasarkan navigation uncertainty, bukan hanya geometry.
6. Hash fence bersama mission configuration.
7. Di runtime, tetap monitor PX4 `geofence_result`/failsafe flags.

Referensi:

- PX4 `src/modules/navigator/geofence.cpp`, `geofence.h`, `geofence_params.c`.
- Commander `HealthAndArmingChecks/checks/geofenceCheck.cpp`.
- parameter `GF_ACTION`, `GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`.

### 8.2 Terrain-relative belum didukung

> **Update 2026-07-17:** Tetap tidak didukung, sekarang secara sengaja
> **fail-closed**: `alt_frame=TERRAIN` diparsing (`AltitudeFrame::kTerrain`)
> tapi `WaypointMission::check_mission_feasibility()` langsung menolak mission
> apa pun yang memakainya, dengan pesan eksplisit. Ini lebih aman daripada
> silent fallback ke AMSL/REL_HOME yang bisa salah datum. Semua langkah di
> bawah (DEM provider, datum, densifikasi) masih sepenuhnya belum dikerjakan.

Implementasi yang aman membutuhkan lebih dari menambahkan elevasi numerik:

1. Tentukan DEM provider, datum, resolution, age, dan no-data policy.
2. Sample terrain sepanjang segment, bukan hanya di waypoint.
3. Densifikasi ketika perubahan terrain melewati tolerance.
4. Terapkan climb/descent feasibility dan clearance margin.
5. Simpan terrain snapshot/hash agar mission reproducible.
6. Tolak mission saat query incomplete atau datum tidak cocok.
7. Terrain following tidak sama dengan obstacle avoidance; kabel/pohon/bangunan
   tetap membutuhkan sumber obstacle lain.

Referensi:

- QGC `TransectStyleComplexItem.cc`: terrain query, densification, terrain frame.
- QGC `TransectStyle.SettingsGroup.json`: terrain tolerance.
- PX4 `mission_block.cpp` terrain altitude checks.
- `MAV_FRAME_GLOBAL_TERRAIN_ALT(_INT)`.

## 9. Gap battery dan failsafe

### 9.1 Battery multi-instance belum diagregasi

> **Update 2026-07-17:** Sebagian diimplementasikan pada SATU instance yang
> tersedia: `OffboardSupervisor::on_battery_status()` sekarang memakai
> `warning` (>= `WARNING_CRITICAL`), `faults != 0`, dan `time_remaining_s`
> (`battery_time_reserve_seconds`) selain `remaining` mentah. **Multi-instance
> asli TIDAK diimplementasikan** -- diperiksa langsung ke source: default
> `dds_topics.yaml` PX4 1.16.1 hanya membridge SATU topic
> `/fmu/out/battery_status` (tidak ada mekanisme `multi_topic`/instance count
> di file itu). Item 1-2 di bawah (subscribe seluruh instance, aggregation
> policy lintas-instance) butuh perubahan bridge yang lebih besar + firmware
> rebuild, di luar scope perubahan kali ini.

PX4 mendukung beberapa battery (`MAX_INSTANCES=4`). Workspace membaca satu topic
default. Untuk redundant battery/power rail:

1. Subscribe seluruh instance yang diekspos DDS.
2. Gunakan `id`, `priority`, `is_required`, `connected`, `faults`.
3. Definisikan aggregation policy: minimum remaining required battery, bukan
   rata-rata sembarang.
4. Tambahkan time-to-land reserve menggunakan `time_remaining_s`.
5. Trigger segera untuk emergency/failed warning dan smart-battery faults yang
   dipilih.
6. Bandingkan action dengan native PX4 `COM_LOW_BAT_ACT` agar tidak konflik.

Referensi:

- `px4_msgs/msg/BatteryStatus.msg`.
- `src/modules/commander/commander_params.c`: `COM_LOW_BAT_ACT`.
- `src/modules/commander/failsafe/`.

### 9.2 Native Offboard-loss harus menjadi lapisan independen

Jika proses supervisor mati, ia tidak dapat mengirim Land. Karena itu PX4 harus
dikonfigurasi dan diuji agar Offboard loss memilih Land:

1. Verifikasi `COM_OF_LOSS_T`.
2. Verifikasi enum `COM_OBL_RC_ACT` pada metadata firmware 1.16.1 yang terpasang.
3. Uji dengan RC tersedia dan tanpa RC.
4. Kill supervisor process, putus DDS, dan hentikan XRCE Agent secara terpisah.
5. Pastikan nav state menjadi Auto Land, bukan RTL/Position/Altitude.

Referensi:

- `commander_params.c`: `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`.
- `HealthAndArmingChecks/checks/offboardCheck.hpp`.
- `src/modules/commander/failsafe/`.

## 10. Gap persistensi dan transaksi mission

> **Update 2026-07-17:** Diimplementasikan di `position_reporter_node.cpp` +
> `sha256.h`: `schema_version` (waypoints.yaml + validasi), canonical
> serialization + SHA-256 (dilog sebelum & sesudah tiap update, juga dilog
> oleh `waypoint_mission_node` untuk mission yang benar-benar dimuat/diterbangkan),
> file lock (`flock` exclusive non-blocking, menolak recorder ganda), `fsync`
> pada file sementara DAN direktori sekitar rename, dan riwayat revisi
> bertimestamp (`waypoints.yaml.bak.<UTC>`) selain `.bak` terbaru. Diverifikasi
> dengan smoke-test end-to-end (topic palsu -> record -> cek file/hash/lock).
> **Belum**: rollback command dan audit actor/time/source estimate granular
> (baru timestamp UTC di nama file backup, bukan metadata terstruktur).

Recorder mempunyai backup dan atomic file rename, tetapi belum mempunyai:

- schema version;
- canonical serialization;
- CRC/hash mission;
- file locking untuk dua recorder bersamaan;
- audit actor/time/source estimate;
- rollback command;
- read-back verification setelah launch.

Langkah implementasi:

1. Tambahkan schema version dan metadata firmware/px4_msgs.
2. Lock file selama update.
3. Parse seluruh YAML, validasi, serialize canonical, hitung SHA-256.
4. `fsync` temporary file dan directory sebelum/sekitar atomic rename.
5. Simpan revision history, bukan hanya satu `.bak`.
6. Mission node log hash yang benar-benar diterbangkan.
7. Tolak runtime parameter mutation setelah mission armed kecuali melalui
   controlled pause/reload protocol.

Referensi desain transaksi QGC/PX4:

- QGC `MissionController::_convertToMissionItems()`.
- QGC `PlanManager.cc` upload state machine dan read-back.
- PX4 `mavlink_mission.cpp`: inactive dataman bank, item validation, CRC, active
  mission metadata switch.

## 11. Gap arsitektural: Offboard bukan Mission Protocol

Untuk memperoleh perilaku mission onboard yang benar-benar sama dengan QGC/PX4,
pilih salah satu:

### Opsi A — Pertahankan Offboard

Workspace bertanggung jawab atas planner, progress, heartbeat, reach, hold,
rotation, persistence, dan fault handling. Kelebihannya fleksibel; kelemahannya
komputer pendamping dan DDS berada pada critical path.

### Opsi B — Implementasikan uploader MAVLink Mission Protocol

Bangun ROS 2 bridge/client yang mengirim `MISSION_COUNT`, melayani
`MISSION_REQUEST_INT`, mengirim `MISSION_ITEM_INT`, menunggu ACK, download
read-back, dan membandingkan canonical mission. Setelah upload, PX4 Navigator
menjadi executor utama dan mission tetap onboard saat companion mati.

Yang tidak boleh dilakukan adalah menulis langsung dataman/uORB internal dari
ROS tanpa kontrak firmware yang stabil.

Referensi utama:

- QGC `src/MissionManager/PlanManager.cc`.
- QGC `src/MissionManager/MissionController.cc`.
- PX4 `src/modules/mavlink/mavlink_mission.cpp`.
- PX4 `src/modules/navigator/mission.cpp`, `mission_base.cpp`,
  `mission_block.cpp`.

## 12. Roadmap implementasi yang disarankan

> **Update 2026-07-17:** Item kode di Fase 1-3 di bawah sudah diimplementasikan
> (lihat tanda ✅/⚠️/❌ per item). Exit criteria dan test matrix (§13) **BELUM
> terpenuhi** karena item 5 di tiap fase (SITL/HITL/flight test) belum
> dijalankan sama sekali -- itu bukan pekerjaan kode dan butuh lingkungan
> simulasi/vehicle yang tidak tersedia di iterasi ini. Fase 4 keputusan
> arsitektural tetap Opsi A (Offboard dipertahankan).

### Fase 1 — P0 keselamatan rotasi dan landing

1. ✅ Tambah `VehicleAngularVelocity` feedback (fail-closed sampai firmware direbuild).
2. ✅ Tambah yaw alignment debounce, timeout, dan progress watchdog.
3. ✅ Tambah Land ACK/nav-state/landed confirmation state machine.
4. ⚠️ Audit native Offboard-loss dan low-battery parameters -- checklist tertulis
   di `FAILSAFE_PARAMETER_AUDIT.md` dengan nilai dari source 1.16.1; verifikasi
   MANUAL di vehicle/SITL nyata tetap wajib per sesi, tidak bisa otomatis dari ROS.
5. ❌ Buat test kill mission, kill supervisor, DDS loss, frozen yaw -- **belum dijalankan**.

Exit criteria (status: **belum terpenuhi**, butuh item 5 di atas):

- seluruh fault menghasilkan Auto Land dalam batas waktu;
- tidak ada RTL;
- tidak ada false “alignment complete” saat yaw-rate tinggi;
- ACK rejected terlihat dan ditangani.

### Fase 2 — Authority, estimator, dan battery

1. ✅ Expose ControlAllocatorStatus dan VehicleAngularVelocity via DDS
   (`dds_topics.yaml` diedit; firmware belum direbuild/reflash).
2. ✅ Tambah persistent saturation response (default nonaktif sampai firmware siap).
3. ✅ Lengkapi reset-specific policy (deteksi/log/guard per jenis; aksi korektif seragam).
4. ⚠️ Tambah multi-battery dan time-to-land reserve -- time-to-land reserve ✅
   pada instance tunggal; multi-battery asli ❌ (bridge PX4 tidak mendukung
   tanpa perubahan firmware lebih besar, lihat §9.1).
5. ❌ Uji motor degraded/saturation pada Hexa-X SITL/HITL -- **belum dijalankan**.

### Fase 3 — Geofence, terrain, dan trajectory

1. ✅ Implementasi custom fence path intersection + uncertainty margin (§8.1).
2. ❌ Implementasi terrain snapshot/datum/densification -- sengaja TIDAK
   dikerjakan (fail-closed reject, §8.2) karena tidak ada sumber DEM tervalidasi.
3. ✅ Tambah fly-through hanya untuk waypoint tanpa hold (opt-in, bounded, §5.1).
4. ⚠️ Validasi jerk/acceleration/velocity continuity -- yaw accel-limited ✅;
   translational jerk-limited trajectory generator penuh ❌ (fly-through hanya
   velocity feed-forward terbatas, bukan planner jerk-continuous).

### Fase 4 — Mission persistence atau migrasi ke onboard mission

1. ✅ Versioned schema, hash, locking, history (§10) -- read-back cross-check
   otomatis terhadap PX4 belum ada (tidak relevan untuk Opsi A/Offboard).
2. ✅ Diputuskan: Offboard tetap menjadi executor (Opsi A dipertahankan).
3. N/A -- tidak berlaku karena Opsi A dipilih, bukan migrasi ke onboard mission.

## 13. Test matrix minimum

| Test | Stimulus | Hasil wajib |
|---|---|---|
| Takeoff yaw | heading awal 45°, 90°, 180° | yaw tidak berubah selama vertical takeoff |
| Corner | 15°, 45°, 90°, 135°, 180° | stop, align, lalu translate; error dalam limit |
| Yaw stuck | freeze heading/kurangi yaw authority | timeout → Auto Land |
| Yaw reset | inject heading reset | re-align tanpa translasi liar |
| XY/Z reset | inject estimator reset | LLA reproject, hold timer tidak salah maju |
| GNSS loss | invalid/dead-reckoning | abort → Auto Land |
| Battery 15% | tiga sample ≤ threshold | Auto Land, bukan RTL |
| Battery dropout | topic timeout saat armed | Auto Land |
| Mission crash | kill waypoint node | supervisor timeout → Auto Land |
| Supervisor crash | kill supervisor | native PX4 Offboard-loss → Auto Land |
| DDS loss | stop XRCE Agent | native PX4 → Auto Land |
| Geofence | lintasan memotong exclusion | ditolak sebelum arm |
| Land rejected | force command rejection | retry bounded + explicit failure log |
| Motor saturation | yaw demand tinggi | terdeteksi, rate turun/abort |

Setiap test harus merekam setidaknya:

```text
vehicle_status, vehicle_command_ack, failsafe_flags
vehicle_local_position, vehicle_global_position, estimator reset counters
trajectory_setpoint, attitude/yaw, angular velocity
torque/thrust setpoint, control_allocator_status, actuator outputs
battery_status, land_detected, mission phase/current waypoint
```

## 14. Definisi “selesai” yang dapat dipertanggungjawabkan

Fitur tidak dianggap selesai hanya karena build sukses. Selesai berarti:

1. requirement dan batas numerik tertulis;
2. code review dan static analysis bersih;
3. unit test fungsi koordinat/state machine lulus;
4. SITL nominal dan fault-injection lulus;
5. HITL dengan timing DDS nyata lulus;
6. ground test tanpa propeller lulus;
7. flight test bertahap di area aman lulus;
8. log membuktikan P95/P99 error, timeout, saturation, dan failsafe sesuai SLA;
9. firmware hash, px4_msgs hash, parameter dump, mission hash, dan calibration
   diarsipkan.

Tanpa bukti tersebut, istilah yang tepat adalah **implemented but not validated**,
bukan “100% presisi” atau “tanpa bug”.
