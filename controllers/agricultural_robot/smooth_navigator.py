"""
smooth_navigator.py - Wall-sensing, smooth-motion lane navigator
  * IN ROW  : steers from the side distance sensors (centres between the two crop walls,
              aligns heading to the walls). Falls back to one wall / IMU if a wall is missing.
  * ROW END : detected by the side sensors going 'open' (no wall) -> starts a continuous arc.
  * TURN    : smooth semicircular arc (pure-pursuit on a generated path), never stops or pivots.
  * OBSTACLE: front sensors see it -> lateral offset is smoothly ramped, steering off the far wall.
  * All commands pass through slew-rate limiters + low-pass, so v and omega change smoothly.
Pure Python - no Webots needed to test.
"""
import math

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def clamp(v, lo, hi): return max(lo, min(hi, v))

class SmoothNavigator:
    LANES_Y = [0.0, 2.2, -2.2]
    HALF_GAP = 0.68          # wall distance seen by a side sensor when centred (0.9 - 0.22)
    OPEN = 1.30              # side reading above this = no wall
    V_CRUISE, V_MIN, V_ARC = 0.50, 0.25, 0.40
    W_MAX = 0.80
    ACC_MAX = 0.45           # m/s^2   (smooth speed changes)
    ALPHA_MAX = 1.2          # rad/s^2 (smooth steering changes)
    SENSOR_BASE = 0.40
    DETOUR = 0.50            # sideways shift around an obstacle (m)
    OBS_TRIGGER = 1.50
    KH, KE = 1.8, 1.3        # heading gain, cross-track gain

    def __init__(self, laps=2):
        self.total = 3 * laps
        self.lane_i, self.done, self.dir = 0, 0, +1
        self.state = "IN_ROW_TRACKING"
        self.v, self.w, self.w_f = 0.30, 0.0, 0.0
        self.open_cnt = self.wall_cnt = 0
        self.offset = self.offset_t = 0.0
        self.detour = None            # dict(side, start_odo, dist)
        self.odo, self._last = 0.0, None
        self.path, self.pi, self.arc_end = [], 0, 0
        self.end_odo = None

    # ------------------------------------------------------------------ in-row wall following
    def _wall_errors(self, d, yaw):
        """Per-sensor validity: a sensor reading > OPEN means 'no wall' and is ignored
        (at row ends the front sensors go open before the rear ones)."""
        lf, lr, rf, rr = d["left_front"], d["left_rear"], d["right_front"], d["right_rear"]
        href = 0.0 if self.dir > 0 else math.pi
        psi_yaw = wrap(yaw - href)
        use_left = not (self.detour and self.detour["side"] < 0)    # stone side is unreliable
        use_right = not (self.detour and self.detour["side"] > 0)
        es, psis = [], []
        for use, f, r, sgn in ((use_left, lf, lr, +1), (use_right, rf, rr, -1)):
            if not use: continue
            ok = [v for v in (f, r) if v < self.OPEN]
            if not ok: continue
            dist = sum(ok) / len(ok)
            es.append(sgn * (self.HALF_GAP - dist))          # +ve = shifted left
            if len(ok) == 2:
                psis.append(math.atan2((r - f) if sgn > 0 else (f - r), self.SENSOR_BASE))
        e = sum(es) / len(es) if es else 0.0
        psi = sum(psis) / len(psis) if psis else psi_yaw
        return e, psi

    def _in_row(self, x, y, yaw, d, dt):
        e, psi = self._wall_errors(d, yaw)
        front = min(d["front_center"], d["front_left"], d["front_right"])
        in_rows = abs(x) < 7.4

        # obstacle -> start smooth detour
        if (self.detour is None and in_rows and front < self.OBS_TRIGGER and abs(psi) < 0.3):
            side = -1 if d["front_left"] < d["front_right"] - 0.02 else +1   # shift away from stone
            self.detour = dict(side=side, start=self.odo, dist=front)
            self.offset_t = side * self.DETOUR
        if self.detour and self.odo - self.detour["start"] > self.detour["dist"] + 0.9:
            self.offset_t = 0.0
            if abs(self.offset) < 0.03: self.detour = None
        self.offset += clamp(self.offset_t - self.offset, -0.3 * dt, 0.3 * dt)   # ramp 0.3 m/s

        ep = e - self.offset
        v = self.V_CRUISE / (1.0 + 1.5 * abs(ep) + 0.8 * abs(psi))
        v = clamp(v, self.V_MIN, self.V_CRUISE)
        w = -(self.KH * psi + self.KE * math.atan2(ep, 0.5 + v))

        # row end: both sides open (debounced) -> arc turn
        both_open = d["left_front"] > self.OPEN and d["right_front"] > self.OPEN
        self.open_cnt = self.open_cnt + 1 if (both_open and self.dir * x > 7.0) else 0
        if self.open_cnt >= 3 or self.dir * x > 9.2:
            self._start_turn(x, y)
        return v, w

    # ------------------------------------------------------------------ smooth headland arc
    def _start_turn(self, x, y):
        self.done += 1
        if self.done >= self.total:
            self.state, self.end_odo = "ENDING", self.odo
            return
        self.lane_i += 1
        y2 = self.LANES_Y[self.lane_i % 3]
        dy = y2 - y
        left = (self.dir * dy) > 0
        R = abs(dy) / 2.0
        th0 = 0.0 if self.dir > 0 else math.pi
        ln = (-math.sin(th0), math.cos(th0))              # left normal
        pts, step = [], 0.05
        # straight lead-out
        n0 = int(0.35 / step)
        for i in range(n0 + 1):
            pts.append((x + math.cos(th0) * step * i, y + math.sin(th0) * step * i))
        sx, sy = pts[-1]
        sgn = 1 if left else -1
        cx, cy = sx + sgn * R * ln[0], sy + sgn * R * ln[1]
        a0 = math.atan2(sy - cy, sx - cx)
        na = max(2, int(math.pi * R / step))
        for i in range(1, na + 1):
            a = a0 + sgn * math.pi * i / na
            pts.append((cx + R * math.cos(a), cy + R * math.sin(a)))
        self.arc_end = len(pts) - 1
        ex, ey = pts[-1]
        th1 = th0 + math.pi
        for i in range(1, int(3.0 / step) + 1):          # lead-in straight back toward the rows
            pts.append((ex + math.cos(th1) * step * i, ey + math.sin(th1) * step * i))
        self.path, self.pi = pts, 0
        self.state = "HEADLAND_TURN"
        self.open_cnt = self.wall_cnt = 0
        self.offset = self.offset_t = 0.0
        self.detour = None

    def _turn(self, x, y, yaw, d):
        P = self.path
        while self.pi < len(P) - 2 and math.hypot(P[self.pi + 1][0] - x, P[self.pi + 1][1] - y) < \
                math.hypot(P[self.pi][0] - x, P[self.pi][1] - y):
            self.pi += 1
        j, Ld = self.pi, 0.75
        while j < len(P) - 1 and math.hypot(P[j][0] - x, P[j][1] - y) < Ld:
            j += 1
        al = wrap(math.atan2(P[j][1] - y, P[j][0] - x) - yaw)
        v = self.V_ARC
        w = 2.0 * v * math.sin(al) / Ld
        # back at the rows? walls visible on both sides -> hand over to wall following
        both_wall = d["left_front"] < self.OPEN and d["right_front"] < self.OPEN
        self.wall_cnt = self.wall_cnt + 1 if (both_wall and self.pi > self.arc_end) else 0
        if self.wall_cnt >= 3:
            self.dir *= -1
            self.state = "IN_ROW_TRACKING"
            self.open_cnt = 0
        return v, w

    # ------------------------------------------------------------------ main step
    def update(self, x, y, yaw, d, dt):
        if self._last is not None: self.odo += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)
        if self.state == "IN_ROW_TRACKING": v, w = self._in_row(x, y, yaw, d, dt)
        elif self.state == "HEADLAND_TURN": v, w = self._turn(x, y, yaw, d)
        else:                                               # ENDING: glide out of the field and stop
            v, w = 0.0, 0.0
            if self.odo - self.end_odo < 1.0:
                v = self.V_ARC
                w = -1.5 * wrap(yaw - (0.0 if self.dir > 0 else math.pi))
            elif abs(self.v) < 0.01:
                return 0.0, 0.0, "MISSION_COMPLETE"
        # smooth the commands
        self.w_f += 0.25 * (w - self.w_f)
        w = clamp(self.w_f, -self.W_MAX, self.W_MAX)
        self.w += clamp(w - self.w, -self.ALPHA_MAX * dt, self.ALPHA_MAX * dt)
        self.v += clamp(v - self.v, -self.ACC_MAX * dt, self.ACC_MAX * dt)
        name = "OBSTACLE_AVOIDANCE" if (self.detour and self.state == "IN_ROW_TRACKING") else self.state
        return self.v, self.w, name
