"""Pengukur karakteristik respons step."""


class StepMeter:
    def __init__(self):
        self.active = False
        self.ess = 0.0  # error tunak (rata-rata eksponensial)

    def start(self, t, start_value, target):
        self.active = abs(target - start_value) > 0.05
        self.t0, self.start_value, self.target = t, start_value, target
        self.t10 = self.t90 = None
        self.peak = start_value
        self.last_outside = t

    def disable(self):
        self.active = False

    def update(self, t, value, setpoint, h):
        self.ess += (setpoint - value - self.ess) * min(1.0, h / 0.5)
        if not self.active:
            return
        s = 1 if self.target > self.start_value else -1
        A = abs(self.target - self.start_value)
        progress = (value - self.start_value) * s / A
        if self.t10 is None and progress >= 0.1:
            self.t10 = t
        if self.t90 is None and progress >= 0.9:
            self.t90 = t
        if (value - self.peak) * s > 0:
            self.peak = value
        band = max(0.02 * A, 0.02)
        if abs(value - self.target) > band:
            self.last_outside = t

    def rise_time(self):
        if self.active and self.t10 is not None and self.t90 is not None:
            return self.t90 - self.t10
        return None

    def overshoot(self):
        if not self.active:
            return None
        s = 1 if self.target > self.start_value else -1
        A = abs(self.target - self.start_value)
        return max(0.0, (self.peak - self.target) * s) / A * 100

    def settling_time(self, t):
        if self.active and t - self.last_outside > 1.5:
            return self.last_outside - self.t0
        return None
