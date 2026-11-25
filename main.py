"""Simulasi Kendali PID Drone — versi lokal (Python + pygame-ce).

Jalankan:  python main.py

Kontrol:
  Klik / seret di area simulasi : pindahkan setpoint
  Panah                         : geser setpoint 0,5 m
  Spasi                         : jeda / lanjut
  R                             : reset
  1 2 3 4                       : kecepatan 0.25x 0.5x 1x 2x
  C                             : simpan data 10 detik terakhir ke CSV (folder data/)
  S                             : simpan screenshot PNG (folder data/)
  F11                           : layar penuh
  Roda mouse di atas slider     : penyesuaian halus
"""
import csv
import math
import os
import sys
from datetime import datetime

# Windows: minta mode DPI-aware agar jendela tidak diregangkan (buram/terhimpit)
if sys.platform == "win32":
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

import pygame

import gfx
from config import SimOptions, G, L, F_MAX, PLOT_WINDOW, X_MIN, X_MAX, Y_MIN
from sim import Simulation, PRESETS, WAYPOINTS
from ui import (Button, Slider, Checkbox, BG, PANEL, PANEL2, LINE, TXT, MUT, ACC, SP,
                OK, BAD, COL_P, COL_I, COL_D, M1, M2)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "data")

WIN_W, WIN_H = gfx.BASE_W, gfx.BASE_H   # ukuran logis tata letak
SIM_RECT = pygame.Rect(10, 52, 840, 400)
PANEL_RECT = pygame.Rect(860, 52, 410, 740)
VIS = 1.7  # pembesaran visual drone agar mudah dilihat

FONT_NAMES = {"sans": "segoeui,dejavusans,arial,helvetica",
              "mono": "consolas,dejavusansmono,menlo,couriernew"}
FONT_FILES = {  # cadangan bila SysFont gagal (mis. Python 3.14 di Windows)
    "sans": [r"C:\Windows\Fonts\segoeui.ttf", r"C:\Windows\Fonts\arial.ttf",
             "/System/Library/Fonts/Supplemental/Arial.ttf",
             "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"],
    "mono": [r"C:\Windows\Fonts\consola.ttf", r"C:\Windows\Fonts\cour.ttf",
             "/System/Library/Fonts/Menlo.ttc",
             "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"],
}
FONT_BOLD_FILES = {"sans": [r"C:\Windows\Fonts\segoeuib.ttf", r"C:\Windows\Fonts\arialbd.ttf",
                            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"],
                   "mono": [r"C:\Windows\Fonts\consolab.ttf",
                            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"]}


def load_font(kind, size, bold=False):
    """Muat font pada ukuran piksel `size`. Utamakan file font asli (tebal asli,
    bukan tebal sintetis), lalu SysFont, lalu font bawaan pygame."""
    for path in (FONT_BOLD_FILES[kind] if bold else []) + FONT_FILES[kind]:
        if os.path.exists(path):
            try:
                f = pygame.font.Font(path, size)
                if bold and path not in FONT_BOLD_FILES[kind]:
                    f.set_bold(True)
                return f
            except Exception:
                continue
    try:
        return pygame.font.SysFont(FONT_NAMES[kind], size, bold=bold)
    except Exception:
        f = pygame.font.Font(None, int(size * 1.3))
        f.set_bold(bold)
        return f


class App:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Simulasi Kendali PID Drone")
        try:
            dw, dh = pygame.display.get_desktop_sizes()[0]
        except Exception:
            dw, dh = 1600, 1000
        k0 = max(0.6, min(dw * 0.94 / WIN_W, dh * 0.88 / WIN_H))
        self.screen = pygame.display.set_mode((int(WIN_W * k0), int(WIN_H * k0)), pygame.RESIZABLE)
        self.fullscreen = False
        self.clock = pygame.time.Clock()
        self._font_k = None
        self.on_resize()

        self.opts = SimOptions()
        self.sim = Simulation(self.opts)
        self.speed = 1.0
        self.paused = False
        self.tab = "PID"
        self.preset = "Seimbang"
        self.dragging_sp = False
        self.toast_text, self.toast_time = "", 0.0
        self.particles = [[(i * 0.137) % 1, (i * 0.379) % 1, 0.5 + (i * 0.618) % 1] for i in range(50)]

        # kamera area terbang (zoom & geser)
        self.base_scale = SIM_RECT.w / (X_MAX - X_MIN)   # piksel logis per meter saat zoom 100%
        self.y_max = 6.0                                  # batas atas setpoint [m]
        self.reset_view()
        self.follow = False
        self.panning = None
        self._build_widgets()

    def on_resize(self):
        self.screen = pygame.display.get_surface()
        gfx.setup(*self.screen.get_size())
        if self._font_k != gfx.K:
            self._font_k = gfx.K
            fp = gfx.font_px
            self.f_title = load_font("sans", fp(18), bold=True)
            self.f = load_font("sans", fp(14))
            self.f_s = load_font("sans", fp(12))
            self.f_xs = load_font("sans", fp(11))
            self.f_m = load_font("mono", fp(13))
            self.f_mb = load_font("mono", fp(16), bold=True)
            self._bg_cache = None

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        if self.fullscreen:
            self._win_size = self.screen.get_size()
            pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            pygame.display.set_mode(self._win_size, pygame.RESIZABLE)
        self.on_resize()

    # ------------------------------------------------------------------ UI
    def _build_widgets(self):
        sim, o = self.sim, self.opts
        self.top = []
        x = 330

        def add(w, text, cb, active=None):
            nonlocal x
            self.top.append(Button((x, 10, w, 30), text, cb, active))
            x += w + 4

        add(70, "Manual", lambda: sim.set_mode("manual"), lambda: sim.mode == "manual")
        add(70, "Kotak", lambda: sim.set_mode("square"), lambda: sim.mode == "square")
        add(84, "Lingkaran", lambda: sim.set_mode("circle"), lambda: sim.mode == "circle")
        x += 8
        add(74, "Step Y", self.step_y)
        add(74, "Step X", self.step_x)
        x += 8
        for s, lab in ((0.25, "¼×"), (0.5, "½×"), (1.0, "1×"), (2.0, "2×")):
            add(36, lab, lambda s=s: self.set_speed(s), lambda s=s: self.speed == s)
        x += 8
        add(62, "Jeda", self.toggle_pause, lambda: self.paused)
        add(62, "Reset", self.reset)
        add(100, "Simpan CSV", self.save_csv)

        bx, by = SIM_RECT.x + 10, SIM_RECT.bottom - 36
        self.sim_btns = [
            Button((bx, by, 30, 26), "−", lambda: self.set_zoom(self.zoom / 1.25)),
            Button((bx + 34, by, 30, 26), "+", lambda: self.set_zoom(self.zoom * 1.25)),
            Button((bx + 68, by, 48, 26), "Pas", self.reset_view),
            Button((bx + 120, by, 58, 26), "Ikuti", self.toggle_follow, lambda: self.follow),
        ]

        px, pw = PANEL_RECT.x + 10, PANEL_RECT.w - 20
        self.tabs = []
        tw = (pw - 12) // 4
        for i, name in enumerate(("PID", "Gangguan", "Sistem", "Bantuan")):
            self.tabs.append(Button((px + i * (tw + 4), PANEL_RECT.y + 6, tw, 28), name,
                                    lambda n=name: setattr(self, "tab", n), lambda n=name: self.tab == n))

        self.widgets = {"PID": [], "Gangguan": [], "Sistem": [], "Bantuan": []}

        # ---- Tab PID ----
        W = self.widgets["PID"]
        y = PANEL_RECT.y + 64
        bw = (pw - 5 * 4) // 6
        for i, name in enumerate(PRESETS):
            b = Button((px + i * (bw + 4), y, bw, 26), name,
                       lambda n=name: self.apply_preset(n), lambda n=name: self.preset == n)
            b.small = True
            W.append(b)
        self.loop_boxes = []
        y = PANEL_RECT.y + 100
        loops = [
            (sim.pid_z, "Ketinggian (y)", "u = percepatan vertikal [m/s²]", (20, 0.1), (10, 0.05), (10, 0.05), "{:.2f}"),
            (sim.pid_x, "Posisi horizontal (x)", "u = percepatan horizontal [m/s²]", (5, 0.05), (2, 0.01), (5, 0.05), "{:.2f}"),
            (sim.pid_th, "Sudut roll (θ) — loop dalam", "u = percepatan sudut [rad/s²]", (400, 1), (100, 0.5), (40, 0.5), "{:.1f}"),
        ]
        for pid, name, unit, rp, ri, rd, fmt in loops:
            box = pygame.Rect(px, y, pw, 150)
            self.loop_boxes.append((box, pid, unit))

            def set_en(v, p=pid):
                p.enabled = v
                p.reset()
            W.append(Checkbox((px + 10, y + 6, pw - 20, 22), name, lambda p=pid: p.enabled, set_en))
            for j, (attr, lab, (vmax, step)) in enumerate((("kp", "Kp", rp), ("ki", "Ki", ri), ("kd", "Kd", rd))):
                W.append(Slider((px + 10, y + 46 + j * 25, pw - 20, 24), lab,
                                lambda p=pid, a=attr: getattr(p, a),
                                lambda v, p=pid, a=attr: setattr(p, a, v),
                                0, vmax, step, fmt, label_w=36, value_w=70))
            y += 158

        # ---- Tab Gangguan ----
        W = self.widgets["Gangguan"]
        y = PANEL_RECT.y + 70
        defs = [
            ("Massa drone", "mass", 0.5, 2.0, 0.05, lambda v: f"{v:.2f} kg"),
            ("Angin konstan", "wind", -6, 6, 0.1, lambda v: f"{v:+.1f} N"),
            ("Turbulensi", "turbulence", 0, 3, 0.05, lambda v: f"{v:.2f} N"),
            ("Noise posisi", "noise_pos", 0, 0.1, 0.002, lambda v: f"σ {v*100:.1f} cm"),
            ("Noise sudut", "noise_ang_deg", 0, 3, 0.05, lambda v: f"σ {v:.2f}°"),
        ]
        for lab, attr, lo, hi, st, fmt in defs:
            W.append(Slider((px, y, pw, 26), lab, lambda a=attr: getattr(o, a),
                            lambda v, a=attr: setattr(o, a, v), lo, hi, st, fmt, label_w=120, value_w=88))
            y += 34
        y += 30
        hw = (pw - 6) // 2
        W.append(Button((px, y, hw, 30), "← Hembusan angin", lambda: sim.gust_push(-1)))
        W.append(Button((px + hw + 6, y, hw, 30), "Hembusan angin →", lambda: sim.gust_push(1)))
        W.append(Button((px, y + 36, hw, 30), "Tendangan rotasi", sim.kick))
        W.append(Button((px + hw + 6, y + 36, hw, 30), "Hapus gangguan", self.clear_disturbances))

        # ---- Tab Sistem ----
        W = self.widgets["Sistem"]
        y = PANEL_RECT.y + 70
        W.append(Slider((px, y, pw, 26), "Frekuensi kendali", lambda: o.control_hz,
                        lambda v: setattr(o, "control_hz", v), 10, 1000, 5,
                        lambda v: f"{v:.0f} Hz", label_w=130, value_w=78))
        W.append(Slider((px, y + 34, pw, 26), "Filter turunan", lambda: o.d_cutoff_hz,
                        lambda v: setattr(o, "d_cutoff_hz", v), 1, 200, 1,
                        lambda v: "tanpa filter" if v >= 200 else f"{v:.0f} Hz", label_w=130, value_w=78))
        W.append(Slider((px, y + 68, pw, 26), "Kemiringan maks", lambda: o.max_tilt_deg,
                        lambda v: setattr(o, "max_tilt_deg", v), 5, 45, 1,
                        lambda v: f"{v:.0f}°", label_w=130, value_w=78))
        W.append(Slider((px, y + 102, pw, 26), "Waktu respons motor", lambda: o.tau_motor * 1000,
                        lambda v: setattr(o, "tau_motor", v / 1000), 5, 150, 1,
                        lambda v: f"{v:.0f} ms", label_w=130, value_w=78))
        y += 150
        self.sys_notes = []
        for lab, attr, note in (
            ("Kompensasi gravitasi (feedforward)", "feedforward",
             "T = m(g+u)/cos θ. Jika mati, Ki harus menanggung berat drone."),
            ("Turunan pada pengukuran", "d_on_measurement",
             "Menghindari derivative kick saat setpoint melompat."),
            ("Anti-windup (integrasi bersyarat)", "anti_windup",
             "Integrator berhenti saat keluaran tersaturasi."),
        ):
            W.append(Checkbox((px, y, pw, 22), lab, lambda a=attr: getattr(o, a),
                              lambda v, a=attr: setattr(o, a, v)))
            self.sys_notes.append((px + 24, y + 22, note))
            y += 56

    # ------------------------------------------------------------ actions
    def toast(self, msg):
        self.toast_text, self.toast_time = msg, 2.0

    def set_speed(self, s):
        self.speed = s

    def toggle_pause(self):
        self.paused = not self.paused

    def reset(self):
        self.sim.reset()
        self.toast("Simulasi direset")

    def apply_preset(self, name):
        self.sim.apply_preset(name)
        self.preset = name
        self.toast("Preset: " + name)

    def step_y(self):
        self.sim.set_mode("manual")
        self.sim.set_setpoint(self.sim.sp_x, 5.0 if abs(self.sim.sp_y - 2) < 0.1 else 2.0)

    def step_x(self):
        self.sim.set_mode("manual")
        self.sim.set_setpoint(3.0 if abs(self.sim.sp_x + 3) < 0.1 else -3.0, self.sim.sp_y)

    def clear_disturbances(self):
        o = self.opts
        o.wind = o.turbulence = o.noise_pos = o.noise_ang_deg = 0.0
        self.sim.gust = self.sim.turb = 0.0

    def save_csv(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        path = os.path.join(DATA_DIR, datetime.now().strftime("pid_drone_%Y%m%d_%H%M%S.csv"))
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "y", "y_sp", "x", "x_sp", "theta_deg", "theta_ref_deg", "F1", "F2"])
            for row in self.sim.history:
                w.writerow([f"{v:.4f}" for v in row])
        self.toast(f"CSV disimpan: data/{os.path.basename(path)}")

    def save_screenshot(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        path = os.path.join(DATA_DIR, datetime.now().strftime("screenshot_%Y%m%d_%H%M%S.png"))
        pygame.image.save(self.screen, path)
        self.toast(f"Screenshot disimpan: data/{os.path.basename(path)}")

    # ------------------------------------------------------------ koordinat
    # ------------------------------------------------------------ kamera
    def reset_view(self):
        self.zoom = 1.0
        self.scale = self.base_scale
        self.cam_x = 0.0
        self.cam_y = Y_MIN + SIM_RECT.h / self.base_scale / 2

    def set_zoom(self, z, anchor=None):
        """Ubah zoom; titik `anchor` (koordinat layar logis) tetap di tempatnya."""
        z = max(0.4, min(8.0, z))
        if anchor is None:
            anchor = SIM_RECT.center
        wx0, wy0 = self.screen_to_world(anchor)
        self.zoom = z
        self.scale = self.base_scale * z
        self.cam_x = wx0 - (anchor[0] - SIM_RECT.centerx) / self.scale
        self.cam_y = wy0 + (anchor[1] - SIM_RECT.centery) / self.scale

    def toggle_follow(self):
        self.follow = not self.follow
        self.toast("Kamera mengikuti drone: " + ("ON" if self.follow else "OFF"))

    def update_camera(self, real_dt):
        if self.follow:
            a = min(1.0, real_dt * 4)
            self.cam_x += (self.sim.drone.x - self.cam_x) * a
            self.cam_y += (self.sim.drone.y + 0.3 - self.cam_y) * a

    def wx(self, x):
        return SIM_RECT.centerx + (x - self.cam_x) * self.scale

    def wy(self, y):
        return SIM_RECT.centery - (y - self.cam_y) * self.scale

    def screen_to_world(self, pos):
        return (self.cam_x + (pos[0] - SIM_RECT.centerx) / self.scale,
                self.cam_y - (pos[1] - SIM_RECT.centery) / self.scale)

    def set_sp_from_mouse(self, pos):
        x, y = self.screen_to_world(pos)
        self.sim.set_setpoint(x, y, y_max=self.y_max)

    # ------------------------------------------------------------ events
    # ------------------------------------------------------------ events
    def handle_events(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            if ev.type in (pygame.VIDEORESIZE, pygame.WINDOWSIZECHANGED):
                self.on_resize()
                continue
            if hasattr(ev, "pos") and ev.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP, pygame.MOUSEMOTION):
                d = dict(ev.dict)
                d["pos"] = gfx.to_logical(ev.pos)
                ev = pygame.event.Event(ev.type, d)
            if ev.type == pygame.KEYDOWN:
                k = ev.key
                moves = {pygame.K_LEFT: (-.5, 0), pygame.K_RIGHT: (.5, 0),
                         pygame.K_UP: (0, .5), pygame.K_DOWN: (0, -.5)}
                if k in moves:
                    self.sim.set_mode("manual")
                    dx, dy = moves[k]
                    self.sim.set_setpoint(self.sim.sp_x + dx, self.sim.sp_y + dy, y_max=self.y_max)
                elif k == pygame.K_SPACE:
                    self.toggle_pause()
                elif k == pygame.K_r:
                    self.reset()
                elif k == pygame.K_c:
                    self.save_csv()
                elif k == pygame.K_s:
                    self.save_screenshot()
                elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    self.set_zoom(self.zoom * 1.25)
                elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    self.set_zoom(self.zoom / 1.25)
                elif k in (pygame.K_0, pygame.K_KP0):
                    self.reset_view()
                elif k == pygame.K_f:
                    self.toggle_follow()
                elif k == pygame.K_F11:
                    self.toggle_fullscreen()
                elif k in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    self.speed = {pygame.K_1: .25, pygame.K_2: .5, pygame.K_3: 1., pygame.K_4: 2.}[k]
                elif k == pygame.K_ESCAPE:
                    if self.fullscreen:
                        self.toggle_fullscreen()
                    else:
                        return False
                continue
            # zoom dengan roda mouse di area terbang
            if ev.type == pygame.MOUSEWHEEL and SIM_RECT.collidepoint(gfx.mouse_pos()):
                self.set_zoom(self.zoom * (1.15 ** ev.y), anchor=gfx.mouse_pos())
                continue
            # geser tampilan: seret dengan klik kanan / tengah
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button in (2, 3) and SIM_RECT.collidepoint(ev.pos):
                self.panning = ev.pos
                self.follow = False
                continue
            if ev.type == pygame.MOUSEMOTION and self.panning:
                self.cam_x -= (ev.pos[0] - self.panning[0]) / self.scale
                self.cam_y += (ev.pos[1] - self.panning[1]) / self.scale
                self.panning = ev.pos
                continue
            if ev.type == pygame.MOUSEBUTTONUP and ev.button in (2, 3):
                self.panning = None
                continue
            handled = False
            for w in self.sim_btns + self.top + self.tabs + self.widgets[self.tab]:
                if w.handle(ev):
                    handled = True
                    break
            if handled:
                continue
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and SIM_RECT.collidepoint(ev.pos):
                self.sim.set_mode("manual")
                self.dragging_sp = True
                self.set_sp_from_mouse(ev.pos)
            elif ev.type == pygame.MOUSEMOTION and self.dragging_sp:
                self.set_sp_from_mouse(ev.pos)
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                self.dragging_sp = False
        return True

    # ------------------------------------------------------------ gambar
    @staticmethod
    def dashed_line(surf, color, p1, p2, dash=6, width=1):
        x1, y1 = p1
        x2, y2 = p2
        length = math.hypot(x2 - x1, y2 - y1)
        if length == 0:
            return
        n = max(1, int(length / dash))
        for i in range(0, n, 2):
            a, b = i / n, min(1, (i + 1) / n)
            gfx.line(surf, color, (x1 + (x2 - x1) * a, y1 + (y2 - y1) * a),
                     (x1 + (x2 - x1) * b, y1 + (y2 - y1) * b), width)

    def _sim_background(self):
        rr = gfx.R(SIM_RECT)
        if self._bg_cache is None or self._bg_cache.get_size() != rr.size:
            bg = pygame.Surface(rr.size)
            for i in range(rr.h):
                c = 15 + int(6 * i / rr.h)
                pygame.draw.line(bg, (c, c + 8, c + 21), (0, i), (rr.w, i))
            self._bg_cache = bg
        return self._bg_cache, rr.topleft

    def draw_sim(self, real_dt):
        s, sim, d = self.screen, self.sim, self.sim.drone
        gfx.clip(s, SIM_RECT)
        bg, pos = self._sim_background()
        s.blit(bg, pos)
        # batas dunia tampak (meter)
        xl, yt = self.screen_to_world(SIM_RECT.topleft)
        xr, yb = self.screen_to_world(SIM_RECT.bottomright)
        grid = (36, 44, 58)
        step = self.nice_step(75 / self.scale)          # jarak grid ~75 px
        y_ground = min(SIM_RECT.bottom, max(SIM_RECT.top, self.wy(0)))
        v = math.ceil(xl / step) * step
        while v <= xr:
            gfx.line(s, grid, (self.wx(v), SIM_RECT.y), (self.wx(v), y_ground))
            if SIM_RECT.top < self.wy(0) < SIM_RECT.bottom - 14:
                gfx.text(s, self.f_s, f"{v:g}", MUT, topleft=(self.wx(v) + 3, self.wy(0) + 4))
            v += step
        v = max(step, math.ceil(yb / step) * step)
        while v <= yt:
            gfx.line(s, grid, (SIM_RECT.x, self.wy(v)), (SIM_RECT.right, self.wy(v)))
            gfx.text(s, self.f_s, f"{v:g} m", MUT, bottomleft=(SIM_RECT.x + 4, self.wy(v) - 2))
            v += step
        # area di luar dinding dunia (x = ±6,5 m)
        for x0, x1 in ((xl - 1, -6.5), (6.5, xr + 1)):
            if x1 > xl and x0 < xr:
                a, b = max(self.wx(x0), SIM_RECT.x), min(self.wx(x1), SIM_RECT.right)
                if b > a:
                    gfx.alpha_rect(s, (0, 0, 0, 90), (a, SIM_RECT.y, b - a, SIM_RECT.h))
        # tanah
        if self.wy(0) < SIM_RECT.bottom:
            gfx.rect(s, (26, 33, 25), (SIM_RECT.x, y_ground, SIM_RECT.w, SIM_RECT.bottom - y_ground))
            gfx.line(s, (59, 74, 53), (SIM_RECT.x, self.wy(0)), (SIM_RECT.right, self.wy(0)), 2)

        # partikel angin
        wt = sim.wind_total
        if abs(wt) > 0.15:
            a = min(1.0, abs(wt) * 0.18)
            col = (int(15 + 165 * a * 0.6), int(23 + 187 * a * 0.6), int(36 + 219 * a * 0.6))
            for p in self.particles:
                p[0] = (p[0] + wt * 0.035 * real_dt * p[2]) % 1.0
                px = SIM_RECT.x + p[0] * SIM_RECT.w
                py = SIM_RECT.y + p[1] * (self.wy(0.1) - SIM_RECT.y)
                ln = max(6, min(60, abs(wt) * 6 * p[2])) * (1 if wt > 0 else -1)
                gfx.line(s, col, (px, py), (px - ln, py))

        # lintasan acuan
        dim_sp = (110, 85, 45)
        if sim.mode == "square":
            pts = [(self.wx(x), self.wy(y)) for x, y in WAYPOINTS]
            for i in range(4):
                self.dashed_line(s, dim_sp, pts[i], pts[(i + 1) % 4])
        elif sim.mode == "circle":
            w, h = 2 * 2.6 * self.scale, 2 * 1.8 * self.scale
            gfx.ellipse(s, dim_sp, (self.wx(0) - w / 2, self.wy(3.3) - h / 2, w, h), 1)

        # setpoint
        sx, sy = self.wx(sim.sp_x), self.wy(sim.sp_y)
        self.dashed_line(s, (80, 66, 40), (sx, SIM_RECT.y), (sx, self.wy(0)))
        self.dashed_line(s, (80, 66, 40), (SIM_RECT.x, sy), (SIM_RECT.right, sy))
        gfx.circle(s, SP, (sx, sy), 9, 2)
        for a, b in (((-14, 0), (-5, 0)), ((5, 0), (14, 0)), ((0, -14), (0, -5)), ((0, 5), (0, 14))):
            gfx.line(s, SP, (sx + a[0], sy + a[1]), (sx + b[0], sy + b[1]), 2)
        gfx.text(s, self.f_m, f"({sim.sp_x:.2f}, {sim.sp_y:.2f})", SP, bottomleft=(sx + 12, sy - 10))

        # jejak
        tr = list(sim.trail)
        for i in range(1, len(tr)):
            f = i / len(tr) * 0.6
            col = (int(15 + (ACC[0] - 15) * f), int(23 + (ACC[1] - 23) * f), int(36 + (ACC[2] - 36) * f))
            gfx.line(s, col, (self.wx(tr[i - 1][0]), self.wy(tr[i - 1][1])),
                     (self.wx(tr[i][0]), self.wy(tr[i][1])), 2)

        # drone (koordinat badan dalam meter, sumbu y ke atas)
        c, sn = math.cos(d.th), math.sin(d.th)

        def B(bx, by):
            return (self.wx(d.x + (bx * c - by * sn) * VIS), self.wy(d.y + (bx * sn + by * c) * VIS))

        for sgn in (-1, 1):
            gfx.line(s, (107, 116, 136), B(sgn * 0.11, 0.045), B(sgn * 0.15, 0), 2)
        gfx.line(s, (154, 164, 184), B(-L, 0.08), B(L, 0.08), 3)
        gfx.polygon(s, (214, 219, 229), [B(-0.11, 0.045), B(0.11, 0.045), B(0.11, 0.115), B(-0.11, 0.115)])
        gfx.polygon(s, ACC, [B(-0.025, 0.1), B(0.025, 0.1), B(0.025, 0.115), B(-0.025, 0.115)])
        for bx, F, col, dirn in ((-L, d.F1, M1, -1), (L, d.F2, M2, 1)):
            gfx.polygon(s, (91, 100, 120), [B(bx - 0.025, 0.08), B(bx + 0.025, 0.08),
                                            B(bx + 0.025, 0.13), B(bx - 0.025, 0.13)])
            gfx.line(s, (70, 80, 98), B(bx - 0.2, 0.14), B(bx + 0.2, 0.14), 3)
            pw = 0.2 * abs(math.cos(d.prop_phase * dirn))
            gfx.line(s, (230, 235, 245), B(bx - pw, 0.14), B(bx + pw, 0.14), 2)
            ln = F / F_MAX * 0.7
            if ln > 0.01:
                gfx.line(s, col, B(bx, 0.16), B(bx, 0.16 + ln), 3)
                gfx.polygon(s, col, [B(bx, 0.16 + ln + 0.06), B(bx - 0.04, 0.16 + ln), B(bx + 0.04, 0.16 + ln)])
        # penunjuk arah bila drone di luar tampilan
        dx_, dy_ = self.wx(d.x), self.wy(d.y)
        if not SIM_RECT.inflate(-20, -20).collidepoint(dx_, dy_):
            cx, cy = SIM_RECT.center
            vx, vy = dx_ - cx, dy_ - cy
            f = min((SIM_RECT.w / 2 - 22) / max(abs(vx), 1e-6), (SIM_RECT.h / 2 - 22) / max(abs(vy), 1e-6))
            ex, ey = cx + vx * f, cy + vy * f
            ang = math.atan2(vy, vx)
            tip = (ex + 10 * math.cos(ang), ey + 10 * math.sin(ang))
            l_ = (ex + 8 * math.cos(ang + 2.4), ey + 8 * math.sin(ang + 2.4))
            r_ = (ex + 8 * math.cos(ang - 2.4), ey + 8 * math.sin(ang - 2.4))
            gfx.polygon(s, BAD, [tip, l_, r_])
            gfx.text(s, self.f_s, "drone", BAD, center=(ex - 22 * math.cos(ang), ey - 16 * math.sin(ang)))

        # HUD
        lines = [f"t     {sim.t:6.1f} s",
                 f"y     {d.y:6.2f} m    x {d.x:6.2f} m",
                 f"θ     {math.degrees(d.th):6.1f}°    θref {math.degrees(sim.theta_ref):5.1f}°",
                 f"F1    {d.F1:6.2f} N    F2 {d.F2:6.2f} N",
                 f"angin {sim.wind_total:6.2f} N"]
        hw = max(gfx.text_size(self.f_m, ln_)[0] for ln_ in lines) + 18
        gfx.alpha_rect(s, (13, 16, 21, 200), (SIM_RECT.x + 10, SIM_RECT.y + 10, hw, 17 * len(lines) + 12))
        for i, ln_ in enumerate(lines):
            gfx.text(s, self.f_m, ln_, TXT, topleft=(SIM_RECT.x + 19, SIM_RECT.y + 16 + i * 17))
        text, kind = sim.status()
        if self.paused:
            text, kind = "DIJEDA", "warn"
        col = {"bad": BAD, "warn": SP, "mut": MUT, "ok": OK}[kind]
        tw, th = gfx.text_size(self.f_m, text)
        box = pygame.Rect(0, 0, tw + 16, th + 10)
        box.topright = (SIM_RECT.right - 10, SIM_RECT.y + 10)
        gfx.rect(s, BG, box, border_radius=5)
        gfx.rect(s, LINE, box, 1, border_radius=5)
        gfx.text(s, self.f_m, text, col, center=box.center)
        gfx.text(s, self.f_s, "Klik kiri: setpoint · Roda: zoom · Klik kanan+seret: geser · F: ikuti drone · 0: reset tampilan",
                 MUT, bottomright=(SIM_RECT.right - 10, SIM_RECT.bottom - 5))
        gfx.clip(s, None)
        gfx.rect(s, LINE, SIM_RECT, 1, border_radius=6)

    @staticmethod
    def nice_step(x):
        p = 10 ** math.floor(math.log10(x))
        f = x / p
        return (1 if f < 1.5 else 2 if f < 3.5 else 5 if f < 7.5 else 10) * p

    def draw_plot(self, rect, title, legend, series, min_span, hline=None):
        s = self.screen
        gfx.rect(s, PANEL, rect, border_radius=8)
        gfx.rect(s, LINE, rect, 1, border_radius=8)
        x = rect.x + 10
        gfx.text(s, self.f_s, title, MUT, midleft=(x, rect.y + 13))
        x += gfx.text_size(self.f_s, title)[0] + 12
        for col, name in legend:
            gfx.line(s, col, (x, rect.y + 13), (x + 14, rect.y + 13), 2)
            gfx.text(s, self.f_s, name, MUT, midleft=(x + 18, rect.y + 13))
            x += gfx.text_size(self.f_s, name)[0] + 30

        area = pygame.Rect(rect.x + 46, rect.y + 26, rect.w - 56, rect.h - 46)
        hist = self.sim.history
        vals = [h[i] for h in hist for i, _, _ in series]
        if hline is not None:
            vals.append(hline)
        lo, hi = (min(vals), max(vals)) if vals else (0.0, 1.0)
        mid = (lo + hi) / 2
        span = max(hi - lo, min_span) * 1.15
        lo, hi = mid - span / 2, mid + span / 2
        t1 = max(self.sim.t, PLOT_WINDOW)
        t0 = t1 - PLOT_WINDOW

        def X(tv):
            return area.x + (tv - t0) / PLOT_WINDOW * area.w

        def Y(v):
            return area.y + (hi - v) / (hi - lo) * area.h

        st = self.nice_step(span / 4)
        v = math.ceil(lo / st) * st
        while v <= hi:
            yy = Y(v)
            gfx.line(s, (52, 60, 78) if abs(v) < 1e-9 else (32, 38, 52), (area.x, yy), (area.right, yy))
            gfx.text(s, self.f_s, f"{v:g}", MUT, midright=(area.x - 5, yy))
            v += st
        tv = math.ceil(t0 / 2) * 2
        while tv <= t1:
            xx = X(tv)
            gfx.line(s, (32, 38, 52), (xx, area.y), (xx, area.bottom))
            gfx.text(s, self.f_s, f"{tv:g}s", MUT, midtop=(xx, area.bottom + 3))
            tv += 2
        gfx.clip(s, area)
        if hline is not None:
            self.dashed_line(s, MUT, (area.x, Y(hline)), (area.right, Y(hline)))
        for idx, col, width in series:
            if len(hist) >= 2:
                gfx.lines(s, col, False, [(X(h[0]), Y(h[idx])) for h in hist], width)
        gfx.clip(s, None)

    def fmt_s(self, v):
        return "—" if v is None else f"{v:.2f} s"

    def draw_metrics(self):
        s = self.screen
        for i, (meter, title) in enumerate(((self.sim.meter_z, "Respons step — ketinggian y"),
                                            (self.sim.meter_x, "Respons step — posisi x"))):
            r = pygame.Rect(10 + i * 425, 458, 415, 54)
            gfx.rect(s, PANEL, r, border_radius=8)
            gfx.rect(s, LINE, r, 1, border_radius=8)
            gfx.text(s, self.f_s, title, MUT, topleft=(r.x + 10, r.y + 4))
            os_ = meter.overshoot()
            items = [("Rise time", self.fmt_s(meter.rise_time())),
                     ("Overshoot", "—" if os_ is None else f"{os_:.1f} %"),
                     ("Settling 2%", self.fmt_s(meter.settling_time(self.sim.t))),
                     ("Error tunak", f"{meter.ess * 100:+.1f} cm")]
            cw = (r.w - 20) / 4
            for j, (lab, val) in enumerate(items):
                cx = r.x + 10 + j * cw
                gfx.text(s, self.f_s, lab, MUT, topleft=(cx, r.y + 20))
                gfx.text(s, self.f_mb, val, TXT, topleft=(cx, r.y + 33))

    def draw_panel(self):
        s = self.screen
        gfx.rect(s, PANEL, PANEL_RECT, border_radius=8)
        gfx.rect(s, LINE, PANEL_RECT, 1, border_radius=8)
        gfx.line(s, LINE, (PANEL_RECT.x, PANEL_RECT.y + 40), (PANEL_RECT.right - 1, PANEL_RECT.y + 40))
        for b in self.tabs:
            b.draw(s, self.f)
        px = PANEL_RECT.x + 10

        if self.tab == "PID":
            gfx.text(s, self.f_s, "PRESET LOOP LUAR (KETINGGIAN & POSISI)", MUT, topleft=(px, PANEL_RECT.y + 46))
            for box, pid, unit in self.loop_boxes:
                gfx.rect(s, PANEL2, box, border_radius=8)
                gfx.rect(s, LINE, box, 1, border_radius=8)
                gfx.text(s, self.f_s, unit, MUT, topleft=(box.x + 34, box.y + 27))
                lim = max(abs(pid.out_min), pid.out_max)
                vals = (("P", pid.p, COL_P), ("I", pid.integral, COL_I), ("D", pid.d, COL_D),
                        ("u", max(pid.out_min, min(pid.out_max, pid.u)), TXT))
                cw = (box.w - 20) / 4
                for j, (n, v, col) in enumerate(vals):
                    cx = box.x + 10 + j * cw
                    gfx.text(s, self.f_s, f"{n}: {v:.0f}" if abs(v) >= 100 else f"{n}: {v:.2f}", MUT,
                             topleft=(cx, box.y + 121))
                    tr = pygame.Rect(cx, box.y + 139, cw - 8, 5)
                    gfx.rect(s, BG, tr, border_radius=2)
                    r = max(-1, min(1, v / lim))
                    w = abs(r) * tr.w / 2
                    mid = tr.centerx
                    if w >= 0.5:
                        gfx.rect(s, col, (mid if r > 0 else mid - w, tr.y, w, tr.h), border_radius=2)
                    gfx.line(s, LINE, (mid, tr.y), (mid, tr.bottom))
                if not pid.enabled:
                    gfx.alpha_rect(s, (21, 26, 34, 120), box)
            gfx.text(s, self.f_s, "Tip: putar roda mouse di atas slider untuk penyesuaian halus.", MUT,
                     topleft=(px, PANEL_RECT.y + 576))

        elif self.tab == "Gangguan":
            gfx.text(s, self.f_s, "PARAMETER FISIK", MUT, topleft=(px, PANEL_RECT.y + 48))
            gfx.text(s, self.f_s, "GANGGUAN SESAAT", MUT, topleft=(px, PANEL_RECT.y + 252))
            self.wrap_text(["Pengendali selalu memakai massa nominal 1,0 kg untuk kompensasi",
                            "gravitasi. Ubah massa untuk melihat mengapa aksi integral (Ki)",
                            "dibutuhkan."], px, PANEL_RECT.y + 360, plain=True)

        elif self.tab == "Sistem":
            gfx.text(s, self.f_s, "IMPLEMENTASI DIGITAL", MUT, topleft=(px, PANEL_RECT.y + 48))
            for x, y, note in self.sys_notes:
                gfx.text(s, self.f_s, note, MUT, topleft=(x, y + 2))

        elif self.tab == "Bantuan":
            self.wrap_text([
                "STRUKTUR KASKADE",
                "  PID x → θref → PID θ → torsi τ",
                "  PID y → gaya dorong total T",
                "  (T, τ) → Mixer → F1, F2 → drone",
                "",
                "MODEL DRONE",
                "  m·x'' = −(F1+F2)·sin θ + F_angin − c·x'",
                "  m·y'' =  (F1+F2)·cos θ − m·g − c·y'",
                "  I·θ'' =  (F2 − F1)·L",
                "  m=1 kg, L=0,25 m, I=0,02 kg·m², Fmaks=12 N/motor",
                "",
                "PID DISKRIT",
                "  u[k] = Kp·e[k] + Ki·Ts·Σe + Kd·Δe/Ts",
                "",
                "SARAN PERCOBAAN",
                "1. Preset 'Hanya P', Step Y. Naikkan Kd, catat overshoot.",
                "2. Massa 1,5 kg dengan Ki = 0. Ukur error tunak, lalu",
                "    naikkan Ki.",
                "3. Matikan anti-windup, bandingkan overshoot saat take-off.",
                "4. Noise sudut 2°: bandingkan filter D 5 Hz vs tanpa filter.",
                "5. Turunkan frekuensi kendali ke 30 Hz. Kapan tidak stabil?",
                "6. Turunkan Kp sudut sampai loop dalam lebih lambat dari",
                "    loop posisi. Apa yang terjadi?",
                "",
                "PINTASAN",
                "  Spasi jeda · R reset · 1–4 kecepatan · F11 layar penuh",
                "  Roda mouse zoom · klik kanan+seret geser · + − 0 · F ikuti",
                "  C simpan CSV · S screenshot (folder data/)",
            ], px, PANEL_RECT.y + 50)

        for w in self.widgets[self.tab]:
            if isinstance(w, Button):
                w.draw(s, self.f_xs if getattr(w, "small", False) else self.f_s)
            else:
                w.draw(s, self.f)

    def wrap_text(self, lines, x, y, plain=False):
        for ln in lines:
            col = ACC if (ln.isupper() and not plain) else TXT
            gfx.text(self.screen, self.f_s, ln, col, topleft=(x, y))
            y += 19

    def draw(self, real_dt):
        s = self.screen
        s.fill(BG)
        gfx.text(s, self.f_title, "Simulasi Kendali PID Drone", TXT, topleft=(12, 5))
        gfx.text(s, self.f_s, "Quadrotor 2D · kendali kaskade · real-time", MUT, topleft=(12, 28))
        gfx.line(s, LINE, (0, 46), (WIN_W, 46))
        for b in self.top:
            b.draw(s, self.f_s)
        self.update_camera(real_dt)
        self.draw_sim(0 if self.paused else real_dt)
        for b in self.sim_btns:
            b.draw(s, self.f)
        gfx.text(s, self.f_s, f"Zoom {self.zoom * 100:.0f}%", MUT,
                 midleft=(self.sim_btns[-1].rect.right + 10, self.sim_btns[-1].rect.centery))
        self.draw_metrics()
        pw, ph = 415, 134
        o = self.opts
        self.draw_plot(pygame.Rect(10, 518, pw, ph), "Ketinggian y (m)", [(ACC, "y"), (SP, "setpoint")],
                       [(2, SP, 1), (1, ACC, 2)], 1)
        self.draw_plot(pygame.Rect(435, 518, pw, ph), "Posisi x (m)", [(ACC, "x"), (SP, "setpoint")],
                       [(4, SP, 1), (3, ACC, 2)], 1)
        self.draw_plot(pygame.Rect(10, 658, pw, ph), "Sudut θ (°)", [(ACC, "θ"), (SP, "θ ref")],
                       [(6, SP, 1), (5, ACC, 2)], 10)
        self.draw_plot(pygame.Rect(435, 658, pw, ph), "Gaya motor (N)",
                       [(M1, "F1 kiri"), (M2, "F2 kanan"), (MUT, "hover")],
                       [(7, M1, 2), (8, M2, 2)], 4, hline=o.mass * G / 2)
        self.draw_panel()
        if self.toast_time > 0:
            self.toast_time -= real_dt
            tw, th = gfx.text_size(self.f, self.toast_text)
            box = pygame.Rect(0, 0, tw + 24, th + 14)
            box.midbottom = (WIN_W // 2, WIN_H - 14)
            gfx.rect(s, PANEL2, box, border_radius=8)
            gfx.rect(s, ACC, box, 1, border_radius=8)
            gfx.text(s, self.f, self.toast_text, TXT, center=box.center)

    # ------------------------------------------------------------ loop
    def run(self, max_frames=None):
        frames = 0
        running = True
        while running:
            real_dt = min(self.clock.tick(60) / 1000.0, 0.05)
            running = self.handle_events()
            if not self.paused:
                self.sim.advance(real_dt, self.speed)
            self.draw(real_dt)
            pygame.display.flip()
            frames += 1
            if max_frames and frames >= max_frames:
                break
        pygame.quit()


if __name__ == "__main__":
    App().run()
    sys.exit(0)
