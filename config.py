"""Konstanta fisik dan opsi simulasi."""
from dataclasses import dataclass

# ---- Konstanta fisik drone (model bidang vertikal 2D) ----
G = 9.81          # percepatan gravitasi [m/s^2]
L = 0.25          # panjang lengan (pusat ke motor) [m]
F_MAX = 12.0      # gaya dorong maksimum per motor [N]
M_NOM = 1.0       # massa nominal yang diasumsikan pengendali [kg]
I_NOM = 0.02      # momen inersia [kg m^2]
DRAG = 0.3        # redaman translasi linier [N/(m/s)]
ROT_DRAG = 0.004  # redaman rotasi [N m/(rad/s)]

# ---- Simulasi ----
DT_PHYS = 0.001   # langkah integrasi fisika (1 kHz)
PLOT_WINDOW = 10.0  # panjang jendela grafik [s]

# ---- Batas dunia (meter) ----
X_MIN, X_MAX = -6.0, 6.0
Y_MIN = -0.5


@dataclass
class SimOptions:
    """Parameter yang bisa diubah dari antarmuka."""
    # gangguan / fisik
    mass: float = 1.0            # massa sebenarnya [kg]
    wind: float = 0.0            # angin konstan [N]
    turbulence: float = 0.0      # intensitas turbulensi [N]
    noise_pos: float = 0.0       # std noise sensor posisi [m]
    noise_ang_deg: float = 0.0   # std noise sensor sudut [deg]
    # implementasi digital
    control_hz: float = 250.0    # frekuensi loop kendali [Hz]
    d_cutoff_hz: float = 20.0    # cutoff filter turunan [Hz] (>=200 = tanpa filter)
    max_tilt_deg: float = 30.0   # sudut kemiringan maksimum [deg]
    tau_motor: float = 0.03      # konstanta waktu motor [s]
    feedforward: bool = True     # kompensasi gravitasi
    d_on_measurement: bool = True
    anti_windup: bool = True
