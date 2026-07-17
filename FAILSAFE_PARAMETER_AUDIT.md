# Audit Parameter Failsafe Native PX4 1.16.1 (GAP §4.4 / §9.2, Fase 1 P0)

> Ini adalah **audit yang harus dijalankan manual pada vehicle/SITL nyata**,
> bukan kode. Nilai di bawah diambil langsung dari source firmware yang
> disertakan di workspace ini:
> `PX4-Autopilot-1.16.1/src/modules/commander/commander_params.c`.
> **Jangan menyalin nilai enum dari versi PX4 lain** -- definisi param dapat
> berubah antar versi.

## Kenapa ini P0

`offboard_supervisor` mengirim `VEHICLE_CMD_NAV_LAND` saat mission/health gate
gagal. Tetapi jika **proses supervisor itu sendiri mati** (crash, OOM, companion
computer freeze), atau **DDS/uXRCE-Agent terputus**, tidak ada software ROS 2
yang bisa mengambil alih. Satu-satunya yang tersisa adalah failsafe *native*
PX4 yang berjalan di flight controller. Parameter berikut WAJIB diverifikasi
sebelum setiap sesi terbang real (bukan hanya sekali saat setup):

## 1. `COM_OF_LOSS_T` -- timeout kehilangan Offboard

`src/modules/commander/commander_params.c:322`

```text
PARAM_DEFINE_FLOAT(COM_OF_LOSS_T, 1.0f);
```

- Satuan detik, default firmware **1.0 s**, range 0-60 s.
- Waktu tunggu sebelum PX4 mendeklarasikan "Offboard signal loss" dan memicu
  aksi `COM_OBL_RC_ACT`.
- **Rekomendasi workspace ini:** biarkan 1.0 s atau perkecil sedikit
  (mis. 0.5 s) agar deteksi native lebih cepat daripada
  `mission_timeout_seconds`/`position_timeout_seconds` milik supervisor ROS,
  supaya kedua lapisan konsisten alih-alih saling menunggu.

## 2. `COM_OBL_RC_ACT` -- aksi saat Offboard hilang

`src/modules/commander/commander_params.c:336-351`

```text
@value 0 Position mode
@value 1 Altitude mode
@value 2 Stabilized
@value 3 Return mode
@value 4 Land mode
@value 5 Hold mode
@value 6 Terminate
@value 7 Disarm
PARAM_DEFINE_INT32(COM_OBL_RC_ACT, 0);
```

- **Default firmware adalah `0` (Position mode) -- INI SALAH untuk workspace
  ini.** Position mode menyerahkan kendali ke RC stick; jika tidak ada pilot RC
  yang mengawasi (operasi BVLOS/companion-only), vehicle bisa melayang tak
  terkendali.
- **WAJIB diset ke `4` (Land mode)** agar konsisten dengan kontrak README:
  *"Set parameter PX4 Offboard-loss ke Land, bukan RTL"*.
- Jangan pakai `3` (Return/RTL) -- bertentangan langsung dengan desain
  supervisor yang secara eksplisit tidak pernah mengirim RTL.
- Verifikasi lewat `param show COM_OBL_RC_ACT` di MAVLink console/QGC
  Parameters, BUKAN dengan asumsi dari dokumen versi lain.

## 3. `COM_LOW_BAT_ACT` -- native battery failsafe

`src/modules/commander/commander_params.c:264-275`

```text
@value 0 Warning
@value 2 Land mode
@value 3 Return at critical level, land at emergency level
PARAM_DEFINE_INT32(COM_LOW_BAT_ACT, 0);
```

- Default firmware `0` (Warning only) -- tidak memicu Land otomatis.
- Software gate `offboard_supervisor` (battery_land_threshold_percent,
  battery_time_reserve_seconds, warning/faults) adalah lapisan pertama, tetapi
  jika node ROS mati, native failsafe adalah cadangan.
- **Rekomendasi: set `2` (Land mode)**. Hindari `3` (Return) karena mengandung
  RTL di level emergency, bertentangan dengan kontrak "tidak pernah RTL".
- Perhatikan `COM_LOW_BAT_ACT=2` tetap bergantung pada `BAT_CRIT_THR`/
  `BAT_EMERGEN_THR` dan kalibrasi kapasitas battery yang benar (lihat §7.4
  GAP roadmap) -- parameter aksi yang benar tidak berguna jika estimasi
  `remaining` PX4 sendiri salah.

## 4. Parameter yang TIDAK cukup untuk companion-computer loss

`COM_OBC_LOSS_T` (`commander_params.c:353-362`) hanya **timeout peringatan**
sebelum PX4 melaporkan companion computer loss -- ini bukan action switch
dan tidak menggantikan `COM_OF_LOSS_T`/`COM_OBL_RC_ACT`. Jangan menganggap
mengatur `COM_OBC_LOSS_T` saja sudah cukup.

## 5. Cara verifikasi (wajib per sesi terbang, bukan sekali)

1. `param show COM_OF_LOSS_T COM_OBL_RC_ACT COM_LOW_BAT_ACT` di MAVLink shell
   atau QGC Vehicle Setup > Parameters, dan catat hash parameter dump.
2. Uji nyata di SITL: kill proses `offboard_supervisor` saat armed+Offboard,
   putuskan XRCE Agent, dan pastikan `vehicle_status.nav_state` berpindah ke
   `NAVIGATION_STATE_AUTO_LAND` -- BUKAN `AUTO_RTL`/`POSITION`.
3. Ulangi dengan RC tersedia dan tanpa RC (lihat GAP §9.2 test matrix).
4. Arsipkan parameter dump ini bersama firmware hash sebelum setiap
   penerbangan real (lihat GAP §16 "Definisi selesai").

Referensi kode: `PX4-Autopilot-1.16.1/src/modules/commander/commander_params.c`,
`HealthAndArmingChecks/checks/offboardCheck.hpp`,
`src/modules/commander/failsafe/`.
