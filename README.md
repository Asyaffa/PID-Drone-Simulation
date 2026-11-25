# Simulasi Kendali PID Drone (versi lokal)

Simulator real-time quadrotor 2D yang dikendalikan oleh tiga pengendali PID bertingkat (kaskade): ketinggian, posisi horizontal, dan sudut roll. Dibuat untuk praktikum/demonstrasi mata kuliah Teknik Kendali.

## Isi folder

```
pid-drone-sim/
├── main.py          # aplikasi utama (antarmuka pygame) — jalankan file ini
├── sim.py           # simulasi: kendali kaskade, mixer, gangguan, setpoint
├── pid.py           # kelas PID diskrit (filter D, anti-windup)
├── drone.py         # model dinamika quadrotor planar
├── metrics.py       # pengukur rise time, overshoot, settling time
├── config.py        # konstanta fisik & opsi simulasi
├── ui.py            # widget tombol, slider, checkbox
├── gfx.py           # lapisan gambar skala-bebas (tajam di semua resolusi)
├── requirements.txt
├── data/            # hasil CSV & screenshot disimpan di sini
├── web/index.html   # versi browser (tanpa instalasi)
└── .vscode/         # konfigurasi Run/Debug VS Code
```

## Menjalankan di VS Code

1. Pasang **Python 3.10+** dan ekstensi **Python** di VS Code.
2. Buka folder ini: *File → Open Folder… → pid-drone-sim*.
3. Buka terminal (*Terminal → New Terminal*), lalu buat lingkungan virtual dan pasang pygame:

   **Windows**
   ```
   python -m venv .venv
   .venv\Scripts\activate
   pip install -r requirements.txt
   ```
   **macOS / Linux**
   ```
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```
4. Pilih interpreter `.venv` (*Ctrl+Shift+P → Python: Select Interpreter*).
   > Proyek ini memakai **pygame-ce** (pygame Community Edition), versi pygame yang terus dirawat dan mendukung Python 3.10–3.14. Kode tetap memakai `import pygame`. Jika sebelumnya sempat memasang `pygame` biasa, hapus dulu: `pip uninstall pygame`.
5. Tekan **F5** (konfigurasi "Jalankan Simulasi PID Drone"), atau ketik `python main.py` di terminal.

Ukuran jendela bebas diubah atau dimaksimalkan; teks dan grafik digambar ulang pada resolusi asli layar. Tekan **F11** untuk layar penuh (cocok untuk proyektor).

## Kontrol

| Aksi | Cara |
|---|---|
| Pindah setpoint | klik/seret di area simulasi, atau tombol panah (0,5 m) |
| Jeda / lanjut | Spasi |
| Reset | R |
| Kecepatan 0,25× / 0,5× / 1× / 2× | 1 / 2 / 3 / 4 |
| Simpan data 10 s terakhir (CSV) | C atau tombol *Simpan CSV* |
| Simpan screenshot | S |
| Layar penuh | F11 (Esc untuk keluar) |
| Zoom area terbang | roda mouse di area simulasi (di titik kursor), tombol − / + / Pas, atau tombol keyboard + − 0 |
| Geser tampilan | seret dengan klik kanan (atau klik tengah) |
| Kamera mengikuti drone | F atau tombol *Ikuti* |
| Penyesuaian halus slider | roda mouse di atas slider |

Mode setpoint: **Manual**, **Kotak** (4 waypoint, tiap 5 s), **Lingkaran** (uji tracking). Tombol **Step Y** / **Step X** memberi masukan step untuk analisis respons.

## Model dan pengendali

```
m·x''  = −(F1+F2)·sin θ + F_angin − c·x'
m·y''  =  (F1+F2)·cos θ − m·g     − c·y'
I·θ''  =  (F2 − F1)·L
```
m = 1 kg (nominal), L = 0,25 m, I = 0,02 kg·m², F_maks = 12 N per motor, motor orde-1 (τ = 30 ms). Fisika diintegrasikan pada 1 kHz; pengendali berjalan pada frekuensi yang bisa diatur (default 250 Hz, zero-order hold).

Struktur kaskade:
- **PID x** → percepatan horizontal → sudut referensi θref = −atan(aₓ/g)
- **PID θ** (loop dalam) → percepatan sudut → torsi τ
- **PID y** → percepatan vertikal → gaya dorong total T = m(g + u)/cos θ (dengan feedforward gravitasi)
- **Mixer**: F1 = T/2 − τ/(2L), F2 = T/2 + τ/(2L)

## Saran percobaan

1. Preset *Hanya P*, tekan Step Y. Naikkan Kd perlahan dan catat overshoot.
2. Naikkan massa ke 1,5 kg dengan Ki = 0. Ukur error tunak, lalu naikkan Ki.
3. Matikan anti-windup (tab Sistem) dan bandingkan overshoot saat take-off.
4. Naikkan noise sudut ke 2°. Bandingkan gaya motor dengan filter D 5 Hz vs tanpa filter.
5. Turunkan frekuensi kendali ke 30 Hz. Kapan loop sudut mulai tidak stabil?
6. Turunkan Kp sudut sampai loop dalam lebih lambat dari loop posisi. Apa yang terjadi?

Data CSV (kolom: `t, y, y_sp, x, x_sp, theta_deg, theta_ref_deg, F1, F2`) bisa diolah di Excel, MATLAB, atau Python untuk laporan praktikum.

## Versi browser

`web/index.html` berisi versi browser dengan fitur yang sama. Buka langsung dengan klik dua kali, atau di VS Code klik kanan → *Open with Live Server*. Tidak perlu instalasi apa pun.
