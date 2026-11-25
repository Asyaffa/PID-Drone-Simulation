"""Model dinamika quadrotor pada bidang vertikal (planar).

    m*x''  = -(F1+F2)*sin(th) + F_angin - c*x'
    m*y''  =  (F1+F2)*cos(th) - m*g     - c*y'
    I*th'' =  (F2 - F1)*L

th positif = berlawanan arah jarum jam (miring ke kiri).
F1 = motor kiri, F2 = motor kanan. Motor dimodelkan orde-1.
"""
import math
from config import G, L, I_NOM, DRAG, ROT_DRAG


class Drone:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x = self.y = 0.0
        self.vx = self.vy = 0.0
        self.th = self.om = 0.0
        self.F1 = self.F2 = 0.0
        self.on_ground = True
        self.prop_phase = 0.0

    def step(self, f1_cmd, f2_cmd, wind, mass, tau_motor, h):
        # dinamika motor orde-1
        self.F1 += (f1_cmd - self.F1) * h / tau_motor
        self.F2 += (f2_cmd - self.F2) * h / tau_motor
        T = self.F1 + self.F2

        ax = (-T * math.sin(self.th) + wind - DRAG * self.vx) / mass
        ay = (T * math.cos(self.th) - DRAG * self.vy) / mass - G
        alpha = ((self.F2 - self.F1) * L - ROT_DRAG * self.om) / I_NOM

        # Euler semi-implisit
        self.vx += ax * h
        self.vy += ay * h
        self.om += alpha * h
        self.x += self.vx * h
        self.y += self.vy * h
        self.th += self.om * h
        if self.th > math.pi:
            self.th -= 2 * math.pi
        elif self.th < -math.pi:
            self.th += 2 * math.pi

        # kontak dengan tanah
        if self.y <= 0:
            self.y = 0.0
            self.vy = max(0.0, self.vy)
            self.vx *= 0.02 ** h
            if T * math.cos(self.th) < mass * G:
                self.om *= 0.95
                self.th *= 0.99
            self.on_ground = True
        else:
            self.on_ground = False

        # dinding dunia
        if self.x < -6.5:
            self.x, self.vx = -6.5, max(0.0, self.vx)
        if self.x > 6.5:
            self.x, self.vx = 6.5, min(0.0, self.vx)
        if self.y > 9:
            self.y, self.vy = 9.0, min(0.0, self.vy)

        self.prop_phase += h * (T + 4) * 6
