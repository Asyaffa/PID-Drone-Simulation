"""Simulasi lengkap: drone + pengendali kaskade + gangguan + setpoint."""
import math
import random
from collections import deque

from config import G, L, F_MAX, M_NOM, I_NOM, DT_PHYS, PLOT_WINDOW
from pid import PID
from drone import Drone
from metrics import StepMeter

WAYPOINTS = [(-3, 2), (3, 2), (3, 5), (-3, 5)]

PRESETS = {
    # nama: ((Kp,Ki,Kd) ketinggian, (Kp,Ki,Kd) posisi x)
    "Seimbang": ((6, 1.5, 4.5), (1.2, 0.05, 1.6)),
    "Hanya P":  ((4, 0, 0),     (1.2, 0, 0)),
    "PD":       ((4, 0, 3),     (1.2, 0, 1.6)),
    "PI":       ((4, 1.5, 0),   (1.2, 0.1, 0)),
    "Agresif":  ((15, 4, 4),    (3, 0.2, 2)),
    "Lambat":   ((1.5, 0.3, 3), (0.5, 0.02, 1.4)),
}


class Simulation:
    def __init__(self, opts):
        self.opts = opts
        self.drone = Drone()
        self.pid_z = PID(6, 1.5, 4.5, -8, 15)        # keluaran: percepatan vertikal [m/s^2]
        self.pid_x = PID(1.2, 0.05, 1.6, -6, 6)      # keluaran: percepatan horizontal [m/s^2]
        self.pid_th = PID(120, 0, 18, -250, 250)     # keluaran: percepatan sudut [rad/s^2]
        self.meter_z = StepMeter()
        self.meter_x = StepMeter()
        self.history = deque()   # (t, y, ysp, x, xsp, th_deg, thref_deg, F1, F2)
        self.trail = deque(maxlen=120)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self):
        self.t = 0.0
        self.drone.reset()
        for p in (self.pid_z, self.pid_x, self.pid_th):
            p.reset()
        self.sp_x, self.sp_y = 0.0, 3.0
        self.f1_cmd = self.f2_cmd = 0.0
        self.theta_ref = 0.0
        self.saturated = False
        self.turb = self.gust = self.gust_time = self.wind_total = 0.0
        self.ctrl_acc = 1.0
        self.sample_acc = self.trail_acc = self.time_acc = 0.0
        self.mode = "manual"
        self.wp_index, self.wp_time, self.circle_time = 0, 0.0, 0.0
        self.history.clear()
        self.trail.clear()
        self.meter_z.start(0, 0, 3)
        self.meter_z.ess = self.meter_x.ess = 0.0
        self.meter_x.disable()

    def apply_preset(self, name):
        (z, x) = PRESETS[name]
        self.pid_z.kp, self.pid_z.ki, self.pid_z.kd = z
        self.pid_x.kp, self.pid_x.ki, self.pid_x.kd = x
        self.pid_z.integral = self.pid_x.integral = 0.0

    def set_setpoint(self, x, y, y_max=6.0):
        x = max(-5.5, min(5.5, x))
        y = max(0.0, min(y_max, y))
        changed_x = abs(x - self.sp_x) > 0.01
        changed_y = abs(y - self.sp_y) > 0.01
        self.sp_x, self.sp_y = x, y
        if self.mode != "circle":
            if changed_y:
                self.meter_z.start(self.t, self.drone.y, y)
            if changed_x:
                self.meter_x.start(self.t, self.drone.x, x)

    def set_mode(self, mode):
        self.mode = mode
        if mode == "square":
            self.wp_index, self.wp_time = 0, 0.0
            self.set_setpoint(*WAYPOINTS[0])
        elif mode == "circle":
            self.circle_time = 0.0
            self.meter_z.disable()
            self.meter_x.disable()

    def gust_push(self, direction):
        self.gust, self.gust_time = 6.0 * direction, 0.6

    def kick(self):
        self.drone.om += random.choice((-1, 1)) * 8.0

    # ------------------------------------------------------------------
    def control(self, dt):
        """Loop kendali kaskade (dijalankan pada frekuensi control_hz)."""
        o, d = self.opts, self.drone
        # pengukuran sensor dengan noise
        y_m = d.y + o.noise_pos * random.gauss(0, 1)
        x_m = d.x + o.noise_pos * random.gauss(0, 1)
        th_m = d.th + math.radians(o.noise_ang_deg) * random.gauss(0, 1)

        # 1) loop luar posisi x -> sudut referensi
        max_tilt = math.radians(o.max_tilt_deg)
        th_ref = 0.0
        if self.pid_x.enabled:
            a_x = self.pid_x.update(self.sp_x, x_m, dt, o)
            th_ref = max(-max_tilt, min(max_tilt, -math.atan2(a_x, G)))

        # 2) loop ketinggian -> gaya dorong total
        c = max(math.cos(th_m), 0.5)
        if self.pid_z.enabled:
            a_z = self.pid_z.update(self.sp_y, y_m, dt, o)
            T = M_NOM * ((G + a_z) if o.feedforward else a_z) / c
        else:
            T = M_NOM * G / c if o.feedforward else 0.0
        T = max(0.0, min(2 * F_MAX, T))

        # 3) loop dalam sudut -> torsi
        tau = 0.0
        if self.pid_th.enabled:
            alpha = self.pid_th.update(th_ref, th_m, dt, o)
            tau = I_NOM * alpha

        # mixer
        f1 = T / 2 - tau / (2 * L)
        f2 = T / 2 + tau / (2 * L)
        self.saturated = not (0 <= f1 <= F_MAX and 0 <= f2 <= F_MAX)
        self.f1_cmd = max(0.0, min(F_MAX, f1))
        self.f2_cmd = max(0.0, min(F_MAX, f2))
        self.theta_ref = th_ref

    def step(self, h):
        """Satu langkah fisika (1 ms)."""
        o, d = self.opts, self.drone
        self.t += h

        # setpoint otomatis
        if self.mode == "square":
            self.wp_time += h
            if self.wp_time >= 5:
                self.wp_time = 0.0
                self.wp_index = (self.wp_index + 1) % len(WAYPOINTS)
                self.set_setpoint(*WAYPOINTS[self.wp_index])
        elif self.mode == "circle":
            self.circle_time += h
            w = 0.45
            self.sp_x = 2.6 * math.cos(w * self.circle_time)
            self.sp_y = 3.3 + 1.8 * math.sin(w * self.circle_time)

        # pengendali digital (zero-order hold)
        self.ctrl_acc += h
        if self.ctrl_acc >= 1.0 / o.control_hz - 1e-9:
            self.control(self.ctrl_acc)
            self.ctrl_acc = 0.0

        # gangguan angin: konstan + turbulensi (Ornstein-Uhlenbeck) + hembusan
        self.turb += -self.turb * h / 0.6 + o.turbulence * math.sqrt(2 * h / 0.6) * random.gauss(0, 1)
        if self.gust_time > 0:
            self.gust_time -= h
            if self.gust_time <= 0:
                self.gust = 0.0
        self.wind_total = o.wind + self.turb + self.gust

        d.step(self.f1_cmd, self.f2_cmd, self.wind_total, o.mass, o.tau_motor, h)

        self.meter_z.update(self.t, d.y, self.sp_y, h)
        self.meter_x.update(self.t, d.x, self.sp_x, h)

        self.sample_acc += h
        if self.sample_acc >= 1 / 60:
            self.sample_acc = 0.0
            self.history.append((self.t, d.y, self.sp_y, d.x, self.sp_x,
                                 math.degrees(d.th), math.degrees(self.theta_ref), d.F1, d.F2))
            while self.history and self.history[0][0] < self.t - PLOT_WINDOW:
                self.history.popleft()
        self.trail_acc += h
        if self.trail_acc >= 1 / 30:
            self.trail_acc = 0.0
            self.trail.append((d.x, d.y))

    def advance(self, real_dt, speed):
        """Jalankan fisika sesuai waktu nyata yang berlalu."""
        self.time_acc += real_dt * speed
        n = 0
        while self.time_acc >= DT_PHYS and n < 400:
            self.step(DT_PHYS)
            self.time_acc -= DT_PHYS
            n += 1
        if n >= 400:
            self.time_acc = 0.0

    def status(self):
        d = self.drone
        if abs(d.th) > math.radians(70):
            return "TERBALIK - tekan R", "bad"
        if self.saturated:
            return "Motor saturasi", "warn"
        if d.on_ground:
            return "Di tanah", "mut"
        return "Terbang", "ok"
